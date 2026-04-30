# 🚁 LiteWing ESP32-S3 Flight Controller

Firmware de control de vuelo desarrollado en C++ *bare-metal* para el hardware LiteWing basado en el microcontrolador ESP32-S3. Este proyecto reemplaza los monolitos de código tradicionales por una arquitectura de software altamente modular, enfocada en la robustez matemática, el determinismo temporal y la eficiencia computacional.

## ⚙️ Características Principales

* **Control PID Discreto Riguroso:** Implementación basada en la teoría de control digital de Åström y Wittenmark. Incluye aproximación de Euler (Forward) para la integral, filtro pasa-bajos para el término derivativo, algoritmo de posición y protección Anti-Windup.
* **Lazo de Control Determinista:** Ejecución estricta a 250 Hz (4000 µs) utilizando temporizadores de hardware (`micros()`) sin bloqueos ni funciones `delay()`, garantizando estabilidad en la derivada y un comportamiento lineal.
* **Comunicaciones UDP Inalámbricas:** Eliminación del hardware de radio RC tradicional. El ESP32-S3 opera como un SoftAP Wi-Fi, recibiendo comandos de vuelo y enviando telemetría a través de paquetes UDP de alta velocidad.
* **I2C Dual Asíncrono:** Aislamiento de hardware para sensores críticos. La IMU (MPU6050) opera en `I2C0` a 400kHz, mientras que el sensor láser ToF (VL53L1X) opera de forma paralela y no bloqueante en `I2C1`, previniendo que un fallo en el láser congele la estabilización del dron.
* **Control de Potencia PWM de Alta Resolución:** Uso del periférico LEDC nativo del ESP32-S3 a 5000Hz con 12 bits de resolución (0-4095) y un mezclador (Mixer) con clamping de seguridad por software.

## 📂 Arquitectura Modular

El proyecto está dividido en módulos independientes para facilitar la mantenibilidad y escalabilidad:

* `Config.h`: Matriz central de definiciones, credenciales de red, pines y calibraciones estáticas.
* `Comunicaciones.cpp`: Gestión del stack Wi-Fi y parseo de paquetes UDP.
* `IMU.cpp`: Inicialización, calibración dinámica de offsets y lectura matemática de la MPU6050.
* `ToF.cpp`: Lectura no bloqueante de altitud mediante el sensor VL53L1X de STMicroelectronics.
* `Control.cpp`: Cerebro matemático. Calcula el error y computa los esfuerzos de control en los ejes Roll, Pitch y Yaw.
* `Motores.cpp`: Mixer cinemático que traduce las salidas del PID a comandos físicos PWM para los 4 motores con escobillas.
* `Leds.cpp`: Feedback visual del estado del sistema replicando el comportamiento de puertos UTP sin bloquear el microprocesador.

## 🛠️ Hardware Utilizado
* **Placa Controladora:** LiteWing (Basada en XIAO ESP32-S3)
* **IMU:** MPU6050 (Acelerómetro y Giroscopio de 6 ejes)
* **Sensor de Altitud (ToF):** VL53L1X
* **Actuadores:** 4x Motores Brushed (DC) conectados a transistores on-board.

## 👨‍💻 Autor
**Agustín Schwerdt**
Estudiante de Ingeniería Electrónica (UNCOMA). 
Proyecto desarrollado con un enfoque profundo en la aplicación práctica de la teoría de control, diseño de hardware y robótica.