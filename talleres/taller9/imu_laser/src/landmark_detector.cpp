#include <vector>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <robmovil_msgs/msg/landmark_array.hpp>
#include <robmovil_msgs/msg/landmark.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "landmark_detector.h"

#define LANDMARK_DIAMETER 0.1 // metros (0.1 = 10cm)

robmovil_ekf::LandmarkDetector::LandmarkDetector() :
    Node("landmark_detector_node"), tf_buffer(this->get_clock()), tf_listener(tf_buffer), transform_received(false)
{
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>( "/scan", rclcpp::QoS(10), std::bind(&LandmarkDetector::on_laser_scan, this, std::placeholders::_1));
  landmark_pub = this->create_publisher<robmovil_msgs::msg::LandmarkArray>("/landmarks", rclcpp::QoS(10));
  pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud>("/landmarks_pointcloud", rclcpp::QoS(10));

  
  this->declare_parameter("robot_frame", std::string("base_link"));
  this->declare_parameter("publish_robot_frame", std::string("base_link"));
  this->declare_parameter("laser_frame", std::string("laser"));

  this->get_parameter("robot_frame", robot_frame);
  this->get_parameter("publish_robot_frame", publish_robot_frame);
  this->get_parameter("laser_frame", laser_frame);

  RCLCPP_INFO(this->get_logger(), "Publishing to frame  %s", publish_robot_frame.c_str());
}

float dist(tf2::Vector3 a, tf2::Vector3 b){
  return (a - b).length();
}

void robmovil_ekf::LandmarkDetector::on_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if(!update_laser_tf(msg->header.stamp)){
    RCLCPP_WARN(this->get_logger(), "%s -> %s transform not yet received, not publishing landmarks", laser_frame.c_str(), robot_frame.c_str());
    return;
  }

  /* COMPLETAR: 
   * Convertir range, bearing a puntos cartesianos x,y,0. (ojo: descarta puntos en el "infinito")
   * Descartando aquellas mediciones por fuera de los rangos validos 
  */
  std::vector<tf2::Vector3> cartesian;
  
  int n = msg->ranges.size();
  for (int i = 0; i < n; i++)
  {
    float range = msg->ranges[i];
    
    // Filtro mediciones inválidas
    if (range < msg->range_min || range > msg->range_max || range < 0.1)
      continue;

    // Calculo el ángulo correspondiente a este rayo
    float angle = msg->angle_min + i * msg->angle_increment;

    // Convierto de polares a cartesianas
    tf2::Vector3 p(
        range * std::cos(angle),
        range * std::sin(angle),
        0.0f
    );

    // Convierto el marco del laser al marco del robot
    p = laser_transform * p;
    cartesian.push_back(p);
  }

  /* Mensaje del arreglo de landmarks detectados */
  robmovil_msgs::msg::LandmarkArray landmark_array;
  landmark_array.header.stamp = msg->header.stamp;
  landmark_array.header.frame_id = publish_robot_frame;
  
  /* VECTORES AUXILIARES: Pueden utilizar landmark_points para ir acumulando
   * mediciones cercanas */
  std::vector<tf2::Vector3> landmark_points;
  
  // centroides estimados de los postes en coordenadas cartesianas
  std::vector<tf2::Vector3> centroids;
  
  float agrupacion_threshold = LANDMARK_DIAMETER;
  size_t i = 0;
  while (i < cartesian.size())
  {
    // Grupo del punto actual
    landmark_points.clear();
    landmark_points.push_back(cartesian[i]);

    // Agrupo puntos consecutivos que esten cerca del anterior
    size_t j = i + 1;
    while (j < cartesian.size() && dist(cartesian[j-1], cartesian[j]) < agrupacion_threshold) {
      landmark_points.push_back(cartesian[j]);
      j++;
    }

    // Avanzo i al siguiente grupo
    i = j;

    // Filtro grupos chicos (ruido) - un poste debería tener varios puntos
    if (landmark_points.size() < 2) {
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "landmark con %zu puntos", landmark_points.size());

    /* COMPLETAR: calcular el centroide de los puntos acumulados */
    // El centroide es el promedio de las posiciones de los puntos
    tf2::Vector3 centroid(0, 0, 0);
    for (const auto& p : landmark_points) {
      centroid += p;
    }
    centroid /= static_cast<double>(landmark_points.size());
    
    RCLCPP_INFO(this->get_logger(), "landmark detectado (cartesianas): %f %f %f", centroid.getX(), centroid.getY(), centroid.getZ());
    centroids.push_back(centroid);

    /* Convertir el centroide a coordenadas polares, construyendo el mensaje requerido */
    robmovil_msgs::msg::Landmark landmark;
    
    // distancia desde el robot al centroide
    float r = sqrt(pow(centroid.getX(), 2) + pow(centroid.getY(),2)); 
    landmark.range = r;

    // angulo de la recta que conecta al robot con el centroide
    float a = atan2(centroid.getY(), centroid.getX()); 
    landmark.bearing = a;

    /* Fin Completar */

    /* se agrega el landmark en coordenadas polares */
    landmark_array.landmarks.push_back(landmark);
    RCLCPP_INFO(this->get_logger(), "landmark detectado (polares): %f %f", landmark.range, landmark.bearing);
  }

  /* Publicamos el mensaje de los landmarks encontrados */
  if (!landmark_array.landmarks.empty()){
    landmark_pub->publish(landmark_array);
    publish_pointcloud(landmark_array.header, centroids);
  }
}

bool robmovil_ekf::LandmarkDetector::update_laser_tf(const rclcpp::Time& required_time)
{
  try
  {
    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer.lookupTransform(robot_frame, laser_frame, required_time, tf2::durationFromSec(0.1));
    tf2::fromMsg(transformStamped.transform, laser_transform);
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "No se pudo transformar %s a %s: %s",
              robot_frame.c_str(), laser_frame.c_str(), ex.what());
    return false;
  }
}

void robmovil_ekf::LandmarkDetector::publish_pointcloud(const std_msgs::msg::Header& header, const std::vector<tf2::Vector3>& landmark_positions)
{
  sensor_msgs::msg::PointCloud pointcloud;
  pointcloud.header.stamp = header.stamp;
  pointcloud.header.frame_id = header.frame_id;

  for (int i = 0; i < landmark_positions.size(); i++)
  {
    geometry_msgs::msg::Point32 point;
    point.x = landmark_positions[i].getX();
    point.y = landmark_positions[i].getY();
    point.z = landmark_positions[i].getZ();
    pointcloud.points.push_back(point);
  }
  pointcloud_pub->publish(pointcloud);
}
