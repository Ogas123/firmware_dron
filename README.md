# Firmware de Control Óptimo (LQG) para Cuadricóptero

**Autor:** Agustín

**Proyecto:** Tesis de Ingeniería Electrónica

**Arquitectura:** Linear Quadratic Gaussian (LQR + Kalman) en Tiempo Discreto

**Frecuencia de Lazo:** 250 Hz ($T_s$ = 4 ms)

---

## 🎯 Objetivos del Proyecto

El objetivo principal de este firmware es estabilizar y controlar un cuadricóptero utilizando la teoría moderna de control óptimo, superando las limitaciones empíricas de los controladores PID clásicos. La arquitectura se fundamenta en el **Principio de Separación** (Åström & Wittenmark), dividiendo el problema en dos etapas independientes pero complementarias ejecutadas en un entorno de tiempo real (FreeRTOS):

1. **Filtro de Kalman Dinámico (LQE):** Para la estimación estocástica de estados y fusión sensorial.
2. **Regulador Cuadrático Lineal (LQR):** Para el cálculo del esfuerzo óptimo de control mediante una matriz de ganancias calculada analíticamente.

---

## ⚙️ Arquitectura de Hardware

El sistema está diseñado para maximizar el rendimiento computacional sin introducir retrasos de fase en la física del vehículo.

| Componente | Especificación y Configuración |
| --- | --- |
| **Procesador** | ESP32-S3 (Dual-Core Xtensa LX7). Ejecución asimétrica con FreeRTOS. |
| **IMU** | MPU6050 (Acelerómetro + Giroscopio). Bus I2C a 400 kHz. DLPF configurado a 98 Hz. |
| **Telemetría Z** | Sensor Láser ToF VL53L1X para fusión sensorial de altitud. |
| **Actuadores** | Motores Brushed 720 coreless. |
| **Potencia** | MOSFETs de alta velocidad. Control LEDC PWM a 5 kHz (Resolución de 12 bits: 0 - 4095). |

---

## 🧮 Fundamentación Teórica: LQG Completo de 4 Canales

El vehículo ha sido modelado en espacio de estados continuo y discretizado (ZOH) en cuatro canales desacoplados: **Roll**, **Pitch**, **Yaw** y **Altura**.

### 1. Estimación Óptima: Filtro de Kalman (LQE)

La IMU fue calibrada en una etapa *offline* mediante el algoritmo no lineal de **Levenberg-Marquardt** en Python, mitigando los sesgos deterministas. El firmware ejecuta en tiempo real las ecuaciones recursivas de predicción y corrección para estimar el estado $\hat{x}$:

$$\hat{x}(k+1\vert{}k) = \Phi \hat{x}(k\vert{}k-1) + \Gamma u(k) + K \left[ y(k) - C \hat{x}(k\vert{}k-1) \right]$$

### 2. Control Óptimo: LQR

Para evitar saturar la CPU, la matriz de realimentación $L$ se precalcula en estado estacionario resolviendo la Ecuación Algebraica de Riccati (DARE). El ESP32 simplemente ejecuta la ley de control lineal:

$$u(k) = -L \hat{x}(k)$$

---

## 📂 Estructura de Archivos

El código está modularizado para garantizar la máxima eficiencia del compilador de C++ y la separación limpia de responsabilidades.

| Archivo | Responsabilidad Principal |
| --- | --- |
| `firmware_dron.ino` | Orquestador principal. Configura los *Hardware Timers*, semáforos y lanza las tareas de FreeRTOS. |
| `Config.h` | Definición global de pines, constantes físicas del dron y parámetros estáticos. |
| `IMU.cpp` / `.h` | Lectura veloz I2C del MPU6050 y aplicación de la matriz de calibración Levenberg-Marquardt. |
| `ToF.cpp` / `.h` | Adquisición de distancia en el eje Z mediante el sensor láser VL53L1X. |
| `Kalman.cpp` / `.h` | Ejecución matricial desenrollada (*unrolled*) de las 5 ecuaciones del LQE. |
| `LQR.cpp` / `.h` | Multiplicación de la matriz de ganancia estática $L$ por los estados estimados para generar la señal $u(k)$. |
| `Motores.cpp` / `.h` | Mixer dinámico y API LEDC del ESP32 para inyectar la señal a los MOSFETs. |
| `Comunicaciones.cpp` / `.h` | Gestión del puerto serie, WiFi y recepción de comandos del piloto. |
| `README.md` | Documentación arquitectónica del proyecto. |

---

## 🚀 Despliegue en FreeRTOS (Dual-Core)

Para garantizar un determinismo matemático estricto y evitar el *jitter* de los RTOS convencionales, las cargas de trabajo se dividen físicamente en los dos núcleos del ESP32-S3:

* **Core 1 (Lazo de Control de Vuelo):** Un *Hardware Timer* dispara un semáforo binario exactamente cada 4000 µs (250 Hz). Este núcleo ejecuta estrictamente la adquisición, el Filtro de Kalman, la ley LQR y la escritura PWM en los motores.
* **Core 0 (Comunicaciones Asíncronas):** Ejecuta una tarea dedicada a escupir la telemetría (vía Serie/WiFi) a 50 Hz y procesar la radio, asegurando que el *buffer* de I/O jamás bloquee el lazo de control matemático.

---

## 🛠️ Notas de Seguridad y Operación

* **Límite de Nyquist:** Configurado en 125 Hz. El hardware DLPF de la IMU atenúa el ruido de los motores *brushed* garantizando que no se introduzca *aliasing* en el algoritmo estocástico.
* **Saturación (Clamping):** Protecciones por software integradas en el mezclador de motores para evitar desbordamientos en la resolución de 12 bits del PWM (0 - 4095).