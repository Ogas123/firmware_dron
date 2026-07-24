#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "IMU.h"

// Variables de estado (Velocidad y Aceleración)
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;

float offsetRoll = 0, offsetPitch = 0, offsetYaw = 0;
float offset_gravedad_ms2 = 9.80665f; 

// Variables para los ángulos brutos del acelerómetro
float AngleRoll_Acc, AnglePitch_Acc;

void initIMU() {
  // 1. Iniciar bus I2C en los pines D4(SDA) y D5(SCL) del ESP32-S3
  Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL); 
  Wire.setClock(400000);  // Reloj I2C al máximo (Fast Mode)
  delay(250);   // Tiempo para que la IMU estabilice su energía

  // 2. Despertar a la MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Registro PWR_MGMT_1
  Wire.write(0x00); // 0x00 = Despierto
  Wire.endTransmission();

  // --------------------------------------------------------------
  // 3. Configurar el FILTRO PASA BAJOS (DLPF) - ANTIALIASING
  // --------------------------------------------------------------
  Wire.beginTransmission(0x68);  
  Wire.write(0x1A); // Registro CONFIG
  Wire.write(0x02); // Filtro a ~98 Hz (retraso de 2.8 ms)
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

  // ----------------------------------------------------
  // 6. CALIBRACION Offset del Giroscopio
  // ----------------------------------------------------
  // Tomo las primeras 2000 muestras y las promedio
  Serial.println("Calibrando giroscopio... NO MOVER");
  
  float sumRoll = 0, sumPitch = 0, sumYaw = 0;

  for(int i = 0; i < 2000; i++){
    leerIMU();
    sumRoll += RateRoll;
    sumPitch += RatePitch;
    sumYaw += RateYaw;
  }
  offsetRoll = sumRoll / 2000.0;
  offsetPitch = sumPitch / 2000.0;
  offsetYaw = sumYaw / 2000.0;

  digitalWrite(PIN_LED_BLUE, HIGH); 
  Serial.println("Calibración completada!");
}

void leerIMU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Registro inicial ACCEL_XOUT_H
  Wire.endTransmission(false); 
  Wire.requestFrom(0x68, 14); // Pide los 14 bytes de corrido

  int16_t AccXLSB  = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB  = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB  = Wire.read() << 8 | Wire.read();
  int16_t TempLSB  = Wire.read() << 8 | Wire.read(); // No lo uso
  int16_t GyroXLSB = Wire.read() << 8 | Wire.read();
  int16_t GyroYLSB = Wire.read() << 8 | Wire.read();
  int16_t GyroZLSB = Wire.read() << 8 | Wire.read();

  // ---------------------------------
  // --- GIROSCOPIO ---
  // ---------------------------------
  RatePitch = (float)GyroXLSB / 65.5f;
  RateRoll  = (float)GyroYLSB / 65.5f;
  RateYaw   = (float)GyroZLSB / 65.5f;

  RatePitch -= offsetPitch;
  RateRoll  -= offsetRoll;
  RateYaw   -= offsetYaw;

  // ---------------------------------
  // --- ACELERÓMETRO ---
  // ---------------------------------
  // 1. Lectura directa en m/s^2
  float g_real = 9.80665f;
  float AccX_crudo = ((float)AccXLSB / 4096.0f) * g_real;
  float AccY_crudo = ((float)AccYLSB / 4096.0f) * g_real;
  float AccZ_crudo = ((float)AccZLSB / 4096.0f) * g_real;

  // 2. Restar el Offset (Vector 'b' calculado por tu Python)
  float a_x_1 = AccX_crudo - B_X;
  float a_y_1 = AccY_crudo - B_Y;
  float a_z_1 = AccZ_crudo - B_Z;

  // 3. Multiplicar por matriz de Escala (Matriz 'K')
  float a_x_2 = a_x_1 * S_X;
  float a_y_2 = a_y_1 * S_Y;
  float a_z_2 = a_z_1 * S_Z;

  // 4. Multiplicar por la matriz de Desalineación (Tu modelo exacto)
  AccX = a_x_2;
  AccY = (ALFA_YX * a_x_2) + a_y_2;
  AccZ = (ALFA_ZX * a_x_2) + (ALFA_ZY * a_y_2) + a_z_2;

  // 5. Cálculo de ángulos (Tus fórmulas originales)
  AnglePitch_Acc = atan2(AccY, sqrt(AccX*AccX + AccZ*AccZ)) * RAD_TO_DEG;
  AngleRoll_Acc  = -atan2(AccX, sqrt(AccY*AccY + AccZ*AccZ)) * RAD_TO_DEG;
}