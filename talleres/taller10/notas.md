 1. Funcionamiento del Filtro de Kalman Extendido

  La idea central

  El EKF mantiene dos cosas:
  - x = estado estimado (dónde creo que estoy)
  - P = matriz de covarianza (qué tan seguro estoy de esa estimación)

  Las dos fases

  FASE 1: PREDICCIÓN ("creo que voy a estar acá")

  Usa el modelo de movimiento para predecir el nuevo estado:

  Estado:      x⁻ = f(x, u)           → "si estaba en x y me moví con u, ahora estoy en x⁻"
  Covarianza:  P⁻ = A·P·Aᵀ + W·Q·Wᵀ   → "mi incertidumbre creció"

  FASE 2: CORRECCIÓN ("el sensor me dice que estoy acá")

  Usa la medición para corregir la predicción:

  Ganancia:    K = P⁻·Hᵀ·(H·P⁻·Hᵀ + V·R·Vᵀ)⁻¹   → "cuánto confío en el sensor vs la predicción"
  Estado:      x = x⁻ + K·(z_real - z_predicha)   → "corrijo mi estimación"
  Covarianza:  P = (I - K·H)·P⁻                   → "mi incertidumbre bajó"

  ---
  ¿De dónde sale cada matriz?

  | Matriz | Qué es                           | De dónde sale                      | Tamaño |
  |--------|----------------------------------|------------------------------------|--------|
  | x      | Estado estimado [x, y, θ]        | Se inicializa y luego se actualiza | 3×1    |
  | P      | Covarianza del estado            | Se inicializa y luego se actualiza | 3×3    |
  | u      | Entrada de control [v, ω]        | Sensores (odometría, IMU)          | 2×1    |
  | A      | Jacobiano ∂f/∂x                  | Lo calculamos en makeA()           | 3×3    |
  | W      | Jacobiano ∂f/∂w                  | Lo calculamos en makeBaseW()       | 3×3    |
  | Q      | Covarianza del ruido de proceso  | La definimos en makeBaseQ()        | 3×3    |
  | z      | Medición [r, φ]                  | Sensor (LiDAR detecta landmark)    | 2×1    |
  | H      | Jacobiano ∂h/∂x                  | Se calcula en makeH()              | 2×3    |
  | V      | Jacobiano ∂h/∂v                  | Se calcula en makeBaseV()          | 2×2    |
  | R      | Covarianza del ruido de medición | La definimos en makeBaseR()        | 2×2    |

  ---
  ¿Por qué necesitamos los Jacobianos (A, W, H, V)?

  El Kalman "normal" funciona con sistemas lineales. Pero nuestro modelo es no lineal:

  f(x,u) = | x + v·Δt·cos(θ) |    ← no lineal por el cos/sin
           | y + v·Δt·sin(θ) |
           | θ + ω·Δt        |

  El EKF linealiza estas funciones usando la expansión de Taylor de primer orden. El Jacobiano es esa linealización:

  f(x) ≈ f(x₀) + A·(x - x₀)    donde A = ∂f/∂x evaluado en x₀

  ---
  Flujo completo con las ecuaciones

  ┌─────────────────────────────────────────────────────────────────────────┐
  │  INICIALIZACIÓN                                                          │
  │                                                                          │
  │  x = [0, 0, 0]ᵀ        ← estado inicial (posición y orientación)        │
  │  P = diag(0.1, 0.1, 0.1) ← incertidumbre inicial                        │
  └─────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
           ┌──────────────────────────────────────────────────┐
           │  PREDICCIÓN (cada vez que pasa tiempo)           │
           │                                                  │
           │  1. Leer u = [v, ω] de sensores                 │
           │                                                  │
           │  2. Calcular Jacobianos:                        │
           │     A = ∂f/∂x    (cómo f depende del estado)    │
           │     W = ∂f/∂w    (cómo f depende del ruido)     │
           │                                                  │
           │  3. Predecir estado:                            │
           │     x⁻ = f(x, u)                                │
           │     x⁻ = | x + v·Δt·cos(θ) |                    │
           │          | y + v·Δt·sin(θ) |                    │
           │          | θ + ω·Δt        |                    │
           │                                                  │
           │  4. Predecir covarianza:                        │
           │     P⁻ = A·P·Aᵀ + W·Q·Wᵀ                        │
           │          ─────── ────────                        │
           │          propaga   agrega                        │
           │          incert.   ruido                         │
           │          anterior  nuevo                         │
           │                                                  │
           │  → P CRECE (más incertidumbre)                  │
           └──────────────────────────────────────────────────┘
                                      │
                                      ▼
           ┌──────────────────────────────────────────────────┐
           │  CORRECCIÓN (cuando detecto un landmark)         │
           │                                                  │
           │  1. Recibir medición z_real = [r, φ]            │
           │                                                  │
           │  2. Calcular Jacobianos:                        │
           │     H = ∂h/∂x    (cómo h depende del estado)    │
           │     V = ∂h/∂v    (cómo h depende del ruido)     │
           │                                                  │
           │  3. Predecir medición esperada:                 │
           │     z_pred = h(x⁻, landmark)                    │
           │                                                  │
           │  4. Calcular innovación (error):                │
           │     y = z_real - z_pred                         │
           │                                                  │
           │  5. Calcular ganancia de Kalman:                │
           │     S = H·P⁻·Hᵀ + V·R·Vᵀ                        │
           │     K = P⁻·Hᵀ·S⁻¹                               │
           │                                                  │
           │  6. Corregir estado:                            │
           │     x = x⁻ + K·y                                │
           │                                                  │
           │  7. Corregir covarianza:                        │
           │     P = (I - K·H)·P⁻                            │
           │                                                  │
           │  → P DECRECE (más certeza)                      │
           └──────────────────────────────────────────────────┘
                                      │
                                      ▼
                              (volver a predicción)

  ---
  Intuición de la Ganancia de Kalman (K)

  K decide cuánto "creerle" a la medición vs la predicción:

  - Si el sensor es muy preciso (R chico) → K grande → confío más en la medición
  - Si el sensor es ruidoso (R grande) → K chico → confío más en la predicción
  - Si estoy muy perdido (P grande) → K grande → cualquier medición ayuda mucho
  - Si estoy muy seguro (P chico) → K chico → no necesito tanto la medición

  ---
  2. Las Elipses en RViz

  ¿Qué representan?

  La matriz de covarianza P en 2D (ignorando θ) se puede visualizar como una elipse:

  P = | σ²_x    σ_xy  |
      | σ_xy    σ²_y  |

  La elipse muestra la región de incertidumbre de la posición estimada.

  Interpretación geométrica

          y
          │      ╱╲
          │     ╱  ╲
          │    │ x̂  │   ← el centro es la posición estimada (x, y)
          │     ╲  ╱
          │      ╲╱
          └───────────── x

     - El TAMAÑO de la elipse indica cuánta incertidumbre hay
     - La ORIENTACIÓN indica si hay correlación entre x e y
     - Los EJES de la elipse son los eigenvectores de P
     - El LARGO de los ejes son los eigenvalores (desviaciones estándar)

  ¿Qué deberías ver?

  Solo predicción (only_prediction: True):

  Tiempo 0:        Tiempo 1:        Tiempo 2:        Tiempo 3:
      ·               ○               ◯                 ⬭
    (punto)       (pequeña)       (mediana)         (grande)

  La elipse CRECE porque solo sumamos incertidumbre (Q) sin corregir

  Con corrección (only_prediction: False):

  Predicción:      Corrección:      Predicción:      Corrección:
      ◯          →      ○       →       ◯        →       ○
   (grande)         (pequeña)        (grande)        (pequeña)

  La elipse OSCILA: crece en predicción, se achica en corrección

  Ejemplo visual

                      SOLO PREDICCIÓN
      ┌─────────────────────────────────────────┐
      │                                         │
      │   t=0      t=1       t=2       t=3      │
      │    ·    →   o    →   O    →   ( )      │
      │                                         │
      │   La elipse crece indefinidamente       │
      └─────────────────────────────────────────┘

                    CON CORRECCIÓN
      ┌─────────────────────────────────────────┐
      │                                         │
      │   pred    corr     pred     corr       │
      │    O   →   o   →    O   →    o         │
      │                                         │
      │   La elipse se mantiene acotada        │
      └─────────────────────────────────────────┘

  En tu caso específico

  La configuración en ekf.rviz muestra una elipse que representa 1 desviación estándar (≈68% de probabilidad de que el robot esté dentro).

  Cuando ejecutes:
  - Solo predicción: verás la elipse crecer y crecer mientras el robot se mueve
  - Con landmarks: verás la elipse achicarse cada vez que detecta un poste
