#include <Arduino.h>
#include <Wire.h>
#include "IMU.h"
#include "LEDs.h"

// Definimos las variables reales acá
float RateRoll, RatePitch, RateYaw;
float offsetRoll = 0, offsetPitch = 0, offsetYaw = 0;

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

  // 5. Offset del Giroscopio
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
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Apuntamos al primer registro del giroscopio (Gyro_XOUT_H)
  Wire.endTransmission(); 
  
  Wire.requestFrom(0x68, 6); // Pedimos los 6 bytes de un tirón
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Convertimos a grados por segundo (Escala de ±500 °/s)
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // Aplicamos la resta del offset calculado
  RateRoll -= offsetRoll;
  RatePitch -= offsetPitch;
  RateYaw -= offsetYaw;
}