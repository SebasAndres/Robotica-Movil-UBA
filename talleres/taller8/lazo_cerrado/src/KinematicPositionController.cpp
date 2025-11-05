#include <angles/angles.h>
#include "KinematicPositionController.h"

#define LOOKAHEAD_DISTANCE 0.5

KinematicPositionController::KinematicPositionController() :
  TrajectoryFollower(), tfBuffer_(this->get_clock()),transform_listener_( tfBuffer_ )
{
    rclcpp::QoS qos_profile(rclcpp::KeepLast(50));
    qos_profile.reliable();
    qos_profile.durability_volatile();

    expected_position_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(10));

    current_pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10), std::bind(&KinematicPositionController::getCurrentPoseFromOdometry, this, std::placeholders::_1));
          
    std::string goal_selection = this->declare_parameter("goal_selection", "TIME_BASED");
    fixed_goal_x_ = this->declare_parameter("fixed_goal_x", 3.0);
    fixed_goal_y_ = this->declare_parameter("fixed_goal_y", 0.0);
    fixed_goal_a_ = this->declare_parameter("fixed_goal_a", -M_PI_2);
    
    if(goal_selection == "TIME_BASED")
      goal_selection_ = TIME_BASED;
    else if(goal_selection == "PURSUIT_BASED")
      goal_selection_ = PURSUIT_BASED;
    else if(goal_selection == "FIXED_GOAL")
      goal_selection_ = FIXED_GOAL;
    else
      goal_selection_ = TIME_BASED; // default
}

double lineal_interp(const rclcpp::Time& t0, const rclcpp::Time& t1, double y0, double y1, const rclcpp::Time& t)
{
  return y0 + (t - t0).seconds() * (y1 - y0) / (t1 - t0).seconds();
}

void KinematicPositionController::getCurrentPoseFromOdometry(const nav_msgs::msg::Odometry& odometry_msg)
{
  x = odometry_msg.pose.pose.position.x;
  y = odometry_msg.pose.pose.position.y;
  tf2::Quaternion q(odometry_msg.pose.pose.orientation.x,
                    odometry_msg.pose.pose.orientation.y,
                    odometry_msg.pose.pose.orientation.z,
                    odometry_msg.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  a = yaw;
}

/**
 * NOTA: Para un sistema estable mantener:
 * - K_BETA < 0 < K_RHO < K_ALPHA
 */
#define K_RHO 1 
#define K_ALPHA 1.5  // + Rotación sobre el eje
#define K_BETA -0.5

bool KinematicPositionController::control(
  const rclcpp::Time& t, 
  double& v, 
  double& w
)
{
  // Se obtiene la pose actual publicada por la odometria
  double current_x, current_y, current_a;
  current_x = this->x; current_y = this->y; current_a = this->a;

  // Se obtiene la pose objetivo actual a seguir
  double goal_x, goal_y, goal_a;
  if( not getCurrentGoal(t, goal_x, goal_y, goal_a) )
    return false;
  publishCurrentGoal(t, goal_x, goal_y, goal_a); // publicación de la pose objetivo para visualizar en RViz

  /** EJERCICIO 1: COMPLETAR: Aqui deberan realizar las cuentas necesarias para determinar:
   *             - la velocidad lineal: asignando la variable v
   *             - la velocidad angular: asignando la variable w 
   *  
   *  RECORDAR: cambiar el marco de referencia en que se encuentran dx, dy y theta */

  double dx = goal_x - current_x;
  double dy = goal_y - current_y;
  double theta = goal_a - current_a;

  double theta_I = -theta;
  dx = cos(theta_I)*dx + sin(theta_I)*dy;
  dy = -sin(theta_I)*dx + cos(theta_I)*dy;

  // Computar variables del sistema de control
  double rho = sqrt(pow(dx,2)+pow(dy,2));
  double alpha = angles::normalize_angle(
    atan2(dy, dx) - theta_I
  ); // Normalizes the angle to be -M_PI circle to +M_PI circle It takes and returns radians. 
  double beta =  angles::normalize_angle(
    - theta_I - alpha
  ); // Realizar el calculo dentro del metodo de normalizacion

  /* dx
  Calcular velocidad lineal y angular* 
   * Existen constantes definidas al comienzo del archivo para
   * K_RHO, K_ALPHA, K_BETA */
  v = K_RHO * rho;
  w = K_ALPHA * alpha + K_BETA * beta;  

  RCLCPP_INFO(
    this->get_logger(), "atan2: %.2f, theta siegwart: %.2f, expected_atheta: %.2f, rho: %.2f, alpha: %.2f, beta: %.2f, v: %.2f, w: %.2f",
    atan2(dy, dx), theta, current_a, rho, alpha, beta, v, w);

  RCLCPP_INFO(this->get_logger(), "goal_x: %.2f, goal_y: %.2f, goal_a: %.2f, current_x: %.2f, current_y: %.2f, current_a: %.2f",
            goal_x, goal_y, goal_a, current_x, current_y, current_a);

  return true;
}

/* Funcion auxiliar para calcular la distancia euclidea */
double dist2(double x0, double y0, double x1, double y1)
{ return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));}

bool KinematicPositionController::getPursuitBasedGoal(
  const rclcpp::Time& t, 
  double& x, 
  double& y, 
  double& a
){
  // Los obtienen los valores de la posicion y orientacion actual.
  double current_x, current_y, current_a;
  current_x = this->x; current_y = this->y; current_a = this->a;
    
  // Se obtiene la trayectoria requerida.
  const robmovil_msgs::msg::Trajectory& trajectory = getTrajectory();
  
  /** EJERCICIO 3:
   * Se recomienda encontrar el waypoint de la trayectoria más cercano al robot en términos de x,y
   * y luego buscar el primer waypoint que se encuentre a una distancia predefinida de lookahead en x,y */
  
  /* NOTA: De esta manera les es posible recorrer la trayectoria requerida */  

  double min_dist = float('inf');
  double closest_wx, closest_wy;
  int nearest_wavepoint_index;
  int num_wavepoints_in_trajectory = trajectory.points.size();

  // Encuentro el punto de la trayectoria más cercano a la posición actual
  for(unsigned int i = 0; i < num_wavepoints_in_trajectory; i++)
  {
    const robmovil_msgs::msg::TrajectoryPoint& wpoint = trajectory.points[i];
    double wpoint_x = wpoint.transform.translation.x;
    double wpoint_y = wpoint.transform.translation.y;
    double wpoint_a = tf2::getYaw(wpoint.transform.rotation);
    
    double curr_wavepoint_dist = dist2(current_x, current_y, wpoint_x, wpoint_y);
    if (curr_wavepoint_dist < min_dist){
      closest_wx = wpoint_x;
      closest_wy = wpoint_y;
      min_dist = curr_wavepoint_dist;
      nearest_wavepoint_index = i;
    }

    // Podría haber una lógica de corte si siguen alejandose los puntos siguientes
  }

  // Obtenemos el goal_point como el siguiente de el más cercano de atrás
  int index_goal_point = nearest_wavepoint_index+1;
  double goal_x = trajectory.points[index_goal_point].transform.translation.x;
  double goal_y = trajectory.points[index_goal_point].transform.translation.y;
  double goal_a = tf2::getYaw(trajectory.points[index_goal_point].transform.rotation);
  while (
    index_goal_point < num_wavepoints_in_trajectory
    && dist2(current_x, current_y, goal_x, goal_y) < LOOKAHEAD_DISTANCE
  ){
    index_goal_point++;
    goal_x = trajectory.points[index_goal_point].transform.translation.x;
    goal_y = trajectory.points[index_goal_point].transform.translation.y;
    goal_a = tf2::getYaw(trajectory.points[index_goal_point].transform.rotation);
  }

  // Si no hay un punto adelante en la trayectoria posterior al lookahead 
  // --> Voy al último punto
  if (index_goal_point == num_wavepoints_in_trajectory){
    goal_x = trajectory.points[index_goal_point-1].transform.translation.x;
    goal_y = trajectory.points[index_goal_point-1].transform.translation.y;
    goal_a = tf2::getYaw(trajectory.points[index_goal_point].transform.rotation);
  }

  // Devuelvo en los parámetros pasados por referencia los GOAL x, GOAL y, A
  x = goal_x;
  y = goal_y;
  a = goal_a;

  /* retorna true si es posible definir un goal, false si se termino la trayectoria y no quedan goals. */
  return true;
}

bool KinematicPositionController::getTimeBasedGoal(const rclcpp::Time& t, double& x, double& y, double& a)
{
  size_t next_point_idx;

  if( not nextPointIndex(t, next_point_idx ) )
    return false;
    
  RCLCPP_INFO(this->get_logger(), "processing index: %zu", next_point_idx);

  const robmovil_msgs::msg::TrajectoryPoint& prev_point = getTrajectory().points[ next_point_idx-1 ];
  const robmovil_msgs::msg::TrajectoryPoint& next_point = getTrajectory().points[ next_point_idx ];

  const rclcpp::Time& t0 = getInitialTime() + prev_point.time_from_start;
  const rclcpp::Time& t1 = getInitialTime() + next_point.time_from_start;

  assert(t0 <= t);
  assert(t < t1);

  double x0 = prev_point.transform.translation.x;
  double x1 = next_point.transform.translation.x;

  double y0 = prev_point.transform.translation.y;
  double y1 = next_point.transform.translation.y;

  double a0 = tf2::getYaw(prev_point.transform.rotation);
  double a1 = tf2::getYaw(next_point.transform.rotation);

  x = lineal_interp(t0, t1, x0, x1, t);
  y = lineal_interp(t0, t1, y0, y1, t);
  a = lineal_interp(t0, t1, a0, a1, t);

  return true;
}
