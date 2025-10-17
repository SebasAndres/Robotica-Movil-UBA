## 1A: 
~~~
ros2 topic list
ros2 topic info 
<topicInfo>
~~~

Al ejecutar `ros2 topic list`, buscarás el tópico de comandos de velocidad del robot. Por convención en ROS, este suele ser:

/cmd_vel: El tópico al que debes publicar para controlar el robot.

## 1B
Velocidad lineal hacia adelante:
~~~
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
~~~

## 1C
Comando directo:
~~~
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
~~~

### Visualizar el Comportamiento y los Nodos

* [cite_start]**Comportamiento del Robot:** Cuando la ventana del nodo `keyboard` está activa y presionas las teclas (que tu nodo `keys_to_twist` está esperando), el robot **Pioneer debería moverse** en CoppeliaSim según la lógica que implementaste en el inciso **a)**[cite: 126].
* **Nodos tomando parte del sistema:** En este momento, hay al menos **tres nodos** clave operando en el sistema de ROS2:
    1.  [cite_start]**`keyboard`**: Publica eventos del teclado (`/keyup`, `/keydown`)[cite: 126].
    2.  [cite_start]**`keys_to_twist`** (Tu nodo): Se suscribe a `/keyup` y `/keydown`, e interpreta las teclas para publicar comandos de velocidad (`/cmd_vel`)[cite: 116, 126, 129].
    3.  [cite_start]**El nodo de CoppeliaSim (o su interfaz ROS2)**: Se suscribe al tópico `/cmd_vel` y traduce estos comandos en movimientos físicos para el robot simulado[cite: 89, 126].

***

## c) Componentes y Diferencias

### ¿Qué nodo está operando los motores de las ruedas?

El nodo que finalmente opera los motores de las ruedas es la **interfaz de ROS2 dentro de CoppeliaSim** (el simulador mismo).

* Tu nodo (`keys_to_twist`) solo genera y publica la **intención de movimiento** (`geometry_msgs/msg/Twist`) en el tópico `/cmd_vel`.
* El **simulador CoppeliaSim** está suscrito a ese tópico y tiene el *driver* interno que toma el mensaje `/cmd_vel` (velocidad lineal y angular) y lo traduce en las **señales de torque o velocidad individuales** que controlan los modelos virtuales de los motores del robot Pioneer.

### ¿Cuál es la diferencia en la forma de controlar el robot si la comparamos con el taller de control de motores con Arduino?

La diferencia fundamental está en el **nivel de abstracción y comunicación**:

| Característica | Taller de Control de Motores (Arduino) | Ejercicio de Teleoperación (ROS2/CoppeliaSim) |
| :--- | :--- | :--- |
| **Nivel de Abstracción** | **Bajo nivel (Hardware)**. [cite_start]Controlas directamente el hardware: ajustando señales PWM o voltajes que determinan la potencia/velocidad de cada motor[cite: 134]. | **Alto nivel (Comportamiento)**. [cite_start]Publicas la **velocidad deseada** del *robot* (m/s y rad/s)[cite: 130]. |
| **Cálculo de Inversa** | Necesitas calcular la **Cinemática Inversa** tú mismo (en el código de Arduino) para convertir la velocidad del robot en las velocidades de las ruedas derecha e izquierda. | El **nodo de CoppeliaSim** (o un *driver* interno) se encarga de aplicar la Cinemática Inversa para traducir la velocidad del robot (`/cmd_vel`) a las velocidades individuales de los motores de las ruedas. |
| **Comunicación** | Comunicación serial directa o por cable con el hardware (Arduino). | [cite_start]Comunicación por **tópicos** (el sistema de mensajes de ROS2), desacoplando el control del motor de la lógica de movimiento[cite: 89]. |