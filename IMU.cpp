#include <Arduino.h>
#include <Wire.h>
#include "IMU.h"
#include "LEDs.h"

// Variables de estado (Velocidad y Aceleración)
float RateRoll, RatePitch, RateYaw;
float offsetRoll = 0, offsetPitch = 0, offsetYaw = 0;
float AccX, AccY, AccZ;

// Variables para los ángulos brutos del acelerómetro
float AngleRoll_Acc, AnglePitch_Acc;


// --- VARIABLES DEL FILTRO DE KALMAN (Notación Åström) ---
// x_hat: Estimación del estado (Ángulo)
// P: Matriz de covarianza del error de estimación (Incertidumbre)
float x_hat_Roll = 0, P_Roll = 2 * 2;
float x_hat_Pitch = 0, P_Pitch = 2 * 2;

// --- PARÁMETROS DEL SISTEMA (Notación Åström) ---
const float h = 0.004;       // Periodo de muestreo en segundos (Gamma)
const float Q = 4 * 4;       // Matriz de covarianza del ruido del proceso (R1)
const float R = 3 * 3;       // Matriz de covarianza del ruido de medición (R2)

// --- FUNCIÓN DEL FILTRO DE KALMAN 1D (Notación Åström) ---
// Parámetros:
// x_hat: Estado estimado (pasado por referencia)
// P: Covarianza del error (pasada por referencia)
// u: Señal de control/entrada (Lectura del giroscopio)
// y: Salida medida del proceso (Lectura del acelerómetro)
void kalman_1d(float &x_hat, float &P, float u, float y) {
  
  // ==========================================
  // 1. ACTUALIZACIÓN DE TIEMPO (PREDICCIÓN)
  // Ecuaciones:
  // x_hat(k|k-1) = Phi * x_hat(k-1|k-1) + Gamma * u(k-1)
  // P(k|k-1) = Phi * P(k-1|k-1) * Phi^T + R1
  // ==========================================
  
  // En nuestro modelo 1D, Phi (matriz de transición) es 1.
  // Gamma (matriz de entrada) es el tiempo de muestreo 'h'.
  x_hat = x_hat + h * u; 
  
  // R1 corresponde a la varianza del ruido del proceso (Q).
  // La propagación del error incluye la varianza escalada por el tiempo.
  P = P + (h * h * Q); 

  // ==========================================
  // 2. ACTUALIZACIÓN DE MEDICIÓN (CORRECCIÓN)
  // Ecuaciones:
  // K(k) = P(k|k-1) * C^T * [C * P(k|k-1) * C^T + R2]^-1
  // x_hat(k|k) = x_hat(k|k-1) + K(k) * [y(k) - C * x_hat(k|k-1)]
  // P(k|k) = [I - K(k) * C] * P(k|k-1)
  // ==========================================

  // Matriz de observación C es 1 (medimos directamente el ángulo).
  // R2 corresponde a la varianza del ruido de medición (R).
  // Cálculo de la Ganancia de Kalman (K)
  float K = P / (P + R); 
  
  // Corrección de la estimación del estado con la "innovación" [y - x_hat]
  x_hat = x_hat + K * (y - x_hat);
  
  // Actualización de la matriz de covarianza (P). I (Identidad) es 1.0.
  P = (1.0f - K) * P;
}



void initIMU() {
  // 1. Iniciar bus I2C en los pines D4(SDA) y D5(SCL) del ESP32-S3
  Wire.begin();
  Wire.setClock(400000);  // Reloj I2C al máximo (Fast Mode)
  delay(250);   // Tiempo para que la IMU estabilice su energía

  // 2. Despertar a la MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Registro PWR_MGMT_1
  Wire.write(0x00); // 0x00 = Despierto
  Wire.endTransmission();

  // 3. Configurar el Filtro Pasa Bajos (DLPF)
  Wire.beginTransmission(0x68);  
  Wire.write(0x1A); // Registro CONFIG
  Wire.write(0x05); // Filtro a ~10Hz
  Wire.endTransmission();

  // 4. Configurar la escala del Giroscopio
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Registro GYRO_CONFIG
  Wire.write(0x08); // Escala a ±500 °/s
  Wire.endTransmission();

  // 5. Configurar la escala del Acelerometro
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Registro ACCEL_CONFIG
  Wire.write(0x10); // Escala a ±8g
  Wire.endTransmission();

  // 6. Offset del Giroscopio
  // Tomo las primeras 2000 muestras y las promedio 
  Serial.println("Calibrando giroscopio... NO MOVER");
  
  float sumRoll = 0, sumPitch = 0, sumYaw = 0;

  for(int i = 0; i < 2000; i++){
    leerIMU();
    sumRoll += RateRoll;
    sumPitch += RatePitch;
    sumYaw += RateYaw;

    // Modulo matemático para que titile el LED cada 100 lecturas
    if(i % 100 == 0) setLedSys(HIGH);
    if(i % 100 == 50) setLedSys(LOW);

    delay(1);
  }
  offsetRoll = sumRoll / 2000.0;
  offsetPitch = sumPitch / 2000.0;
  offsetYaw = sumYaw / 2000.0;
  //offsets calculados
  setLedSys(HIGH); // Dejamos el LED prendido fijo: "¡Listo para volar!"

  Serial.println("Calibración completada!");
}



void leerIMU() {
  // --- GIROSCOPIO ---
  Wire.beginTransmission(0x68);
  Wire.write(0x43); 
  Wire.endTransmission(false); 
  Wire.requestFrom(0x68, 6); 
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  RateRoll -= offsetRoll;
  RatePitch -= offsetPitch;
  RateYaw -= offsetYaw;

  // --- ACELERÓMETRO ---
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); 
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6); 
  
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  AccX = (float)AccXLSB / 4096.0;
  AccY = (float)AccYLSB / 4096.0;
  AccZ = (float)AccZLSB / 4096.0;

  AngleRoll_Acc = atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) * RAD_TO_DEG;
  AnglePitch_Acc = -atan(AccX / sqrt(AccY*AccY + AccZ*AccZ)) * RAD_TO_DEG;

  // 2. LA MAGIA: Pasamos los datos crudos por el Filtro de Kalman
  // Invocamos la función para Roll (usando la nueva notación)
  kalman_1d(x_hat_Roll, P_Roll, RateRoll, AngleRoll_Acc);
  
  // Invocamos la función para Pitch (usando la nueva notación)
  kalman_1d(x_hat_Pitch, P_Pitch, RatePitch, AnglePitch_Acc);
}