1. ¿Qué es el EKF?

El EKF es un algoritmo que estima el estado de un sistema (en este caso, la posición del robot: x, y, θ) combinando:
- Predicciones basadas en un modelo de movimiento (odometría)
- Mediciones de sensores (detección de landmarks con LiDAR)

Fase prediccion:
$x_t = f(x_{t-1}, u_t) + \text{ ruido}$
- $u_t=[v,w] \text{ son la velocidad lineal y angular}$

Fase prediccion:
$x_t = f(x_{t-1}, u_t) + \text{ruido}$