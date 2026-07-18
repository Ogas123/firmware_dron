#include <Arduino.h>
#include "IMU.h"
#include "ToF.h"
#include "SensorFusion.h"
#include "Control.h"
#include "Motores.h"
#include "Config.h"

uint32_t LoopTimer;

void setup() {
  // Usamos una velocidad alta para que los prints no frenen el lazo de control
  Serial.begin(115200); 
  
  // Inicializamos los módulos en orden
  initSensorFusion();
  initIMU();
  initToF();
  initMotores();
  
  // Guardamos la marca de tiempo inicial
  LoopTimer = micros();
}

void loop() {
  // ==========================================================
  // 1. ADQUISICIÓN DE DATOS Y ESTIMACIÓN DE ESTADOS
  // ==========================================================
  
  // Lazo Rápido (250 Hz): Lee I2C, trigonometría y Predicción de Kalman
  leerIMU();

  // Lazo Lento (~30 Hz): Verifica si hay dato nuevo
  leerToF();


  // ==========================================================
  // 2. TELEMETRÍA (Para el Serial Plotter)
  // ==========================================================
  
  // --- ACTITUD (Ángulos) ---
  Serial.print("Roll_acc:"); Serial.print(AngleRoll_Acc); Serial.print(",");
  Serial.print("Roll_gyr:"); Serial.print(RateRoll); Serial.print(",");
  Serial.print("Roll_Kalman:"); Serial.print(x_hat_Roll); Serial.print(",");
  
  Serial.print("Pitch_acc:"); Serial.print(AnglePitch_Acc); Serial.print(",");
  Serial.print("Pitch_gyr:"); Serial.print(RatePitch); Serial.print(",");
  Serial.print("Pitch_Kalman:"); Serial.println(x_hat_Pitch); 

  // --- ALTITUD (Eje Z) ---
  Serial.print("Alt_ToF_Raw:"); Serial.print(distanciaAlturaMM); Serial.print(",");


  // ==========================================================
  // 3. LAZOS PID Y MOTORES
  // ==========================================================
  if (true) {
    calcularPID(); // Calcula PID_Roll, PID_Pitch, PID_Yaw e InputThrottle
    
    actualizarMotores(true, (int)InputThrottle, PID_Roll, PID_Pitch, PID_Yaw);
  } else {
    actualizarMotores(false, 0, 0, 0, 0); // Motores apagados
  }


  // ==========================================================
  // 4. TEMPORIZACIÓN ESTRICTA (250 Hz = 4000 microsegundos)
  // ==========================================================
  // Esperamos hasta que se cumplan exactamente 4ms desde el último ciclo
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}