#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "IMU.h"
#include "LEDs.h"
#include "SensorFusion.h"

// Variables de estado (Velocidad y Aceleración)
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;

float offsetRoll = 0, offsetPitch = 0, offsetYaw = 0;
float offsetAccelZ = 0;

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
  
  float sumRoll = 0, sumPitch = 0, sumYaw = 0, sumAccZ = 0;

  for(int i = 0; i < 2000; i++){
    leerIMU();
    sumRoll += RateRoll;
    sumPitch += RatePitch;
    sumYaw += RateYaw;

    // Calculamos el EarthAccZ "crudo" durante la calibración
    // Para esto, necesitamos que la función leerIMU no use el offset todavía
    float roll_rad = x_hat_Roll * DEG_TO_RAD;
    float pitch_rad = x_hat_Pitch * DEG_TO_RAD;
    float currentAccZ = AccX * sin(pitch_rad) 
                      - AccY * sin(roll_rad) * cos(pitch_rad) 
                      + AccZ * cos(roll_rad) * cos(pitch_rad);
    
    sumAccZ += (currentAccZ - 1.0f) * 9810.0f; // El error respecto a 1g
    
    //LED parpadeando
    if(i % 100 == 0) setLedSys(HIGH);
    if(i % 100 == 50) setLedSys(LOW);
    delay(1);
  }
  offsetRoll = sumRoll / 2000.0;
  offsetPitch = sumPitch / 2000.0;
  offsetYaw = sumYaw / 2000.0;
  offsetAccelZ = sumAccZ / 2000.0;
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

  // 1. Calculamos el ángulo BRUTO con atan2 para evitar la división por cero (Anti-NaN)
  AngleRoll_Acc = atan2(AccY, sqrt(AccX*AccX + AccZ*AccZ)) * RAD_TO_DEG;
  AnglePitch_Acc = -atan2(AccX, sqrt(AccY*AccY + AccZ*AccZ)) * RAD_TO_DEG;

  // 2. LA MAGIA: Pasamos los datos crudos por el Filtro de Kalman
  // Invocamos la función para Roll (usando la nueva notación)
  kalman(x_hat_Roll, P_Roll, RateRoll, AngleRoll_Acc);
  
  // Invocamos la función para Pitch (usando la nueva notación)
  kalman(x_hat_Pitch, P_Pitch, RatePitch, AnglePitch_Acc);


}