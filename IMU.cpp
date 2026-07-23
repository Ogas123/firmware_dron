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
  // ---------------------------------
  // --- GIROSCOPIO ---
  // ---------------------------------
  Wire.beginTransmission(0x68);
  Wire.write(0x43); 
  Wire.endTransmission(false); 
  Wire.requestFrom(0x68, 6); 
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // El giroscopio crudo: (Cuidado con los ejes rotados físicamente aquí también si es necesario)
  RatePitch = (float)GyroX / 65.5;
  RateRoll = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  RatePitch -= offsetPitch;
  RateRoll -= offsetRoll;
  RateYaw -= offsetYaw;

  // ---------------------------------
  // --- ACELERÓMETRO ---
  // ---------------------------------
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); 
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6); 
  
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // 1. Lectura en m/s^2 (Escala ±8g -> 4096 LSB/g)
  float g_real = 9.80665f;
  float AccX_crudo = ((float)AccXLSB / 4096.0) * g_real;
  float AccY_crudo = ((float)AccYLSB / 4096.0) * g_real;
  float AccZ_crudo = ((float)AccZLSB / 4096.0) * g_real;

  // 2. Restar el Offset (Vector 'b')
  float a_x_1 = AccX_crudo - B_X;
  float a_y_1 = AccY_crudo - B_Y;
  float a_z_1 = AccZ_crudo - B_Z;

  // 3. Multiplicar por matriz de Escala (Matriz 'K')
  float a_x_2 = a_x_1 * S_X;
  float a_y_2 = a_y_1 * S_Y;
  float a_z_2 = a_z_1 * S_Z;

  // 4. Multiplicar por matriz de Misalignment (Matriz T^a)
  float AccX_cal = a_x_2 - (ALFA_YX * a_y_2) + (ALFA_ZX * a_z_2);
  float AccY_cal = a_y_2 - (ALFA_ZY * a_z_2);
  float AccZ_cal = a_z_2;

  // 5. MAPEO FÍSICO DE EJES (IMU Rotada 90 grados)
  // Como la IMU está rotada, el sensor X ahora apunta a un ala (Y del dron)
  // Y el sensor Y apunta a la nariz (X del dron).
  AccX = AccY_cal; 
  AccY = -AccX_cal; // El negativo depende de si rotaste horario o antihorario
  AccZ = AccZ_cal;

  // 6. Cálculo de ángulos para Kalman (Ejes corregidos a estándar Aeronáutico)
  AngleRoll_Acc = atan2(AccY, sqrt(AccX*AccX + AccZ*AccZ)) * RAD_TO_DEG;
  AnglePitch_Acc = -atan2(AccX, sqrt(AccY*AccY + AccZ*AccZ)) * RAD_TO_DEG;
}