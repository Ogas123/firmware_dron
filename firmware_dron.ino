#include <Arduino.h>
#include "IMU.h"
#include "ToF.h"
#include "SensorFusion.h"
// #include "Control.h" // Descomentar cuando sumes los motores

uint32_t LoopTimer;

void setup() {
  // Usamos una velocidad alta para que los prints no frenen el lazo de control
  Serial.begin(115200); 
  
  // Inicializamos los módulos en orden
  initSensorFusion();
  initIMU();
  initToF();
  
  // Guardamos la marca de tiempo inicial
  LoopTimer = micros();
}

void loop() {
  // ==========================================================
  // 1. ADQUISICIÓN DE DATOS Y ESTIMACIÓN DE ESTADOS
  // ==========================================================
  
  // Lazo Rápido (250 Hz): Lee I2C, trigonometría y Predicción de Kalman
  leerIMU();

  // Lazo Lento (~30 Hz): Verifica si hay dato nuevo y hace la Corrección de Kalman
  leerToF();


  // ==========================================================
  // 2. TELEMETRÍA (Para el Serial Plotter)
  // ==========================================================
  
  // --- ACTITUD (Ángulos) ---
  Serial.print("Roll_Raw:"); Serial.print(AngleRoll_Acc); Serial.print(",");
  Serial.print("Roll_Kalman:"); Serial.print(x_hat_Roll); Serial.print(",");
  
  Serial.print("Pitch_Raw:"); Serial.print(AnglePitch_Acc); Serial.print(",");
  Serial.print("Pitch_Kalman:"); Serial.print(x_hat_Pitch); Serial.print(",");

  // --- ALTITUD (Eje Z) ---
  //Imprimimos la lectura del ToF en mm y la estimación fusionada de Kalman
  Serial.print("Alt_ToF_Raw:"); Serial.print(distanciaAlturaMM); Serial.print(",");
  Serial.print("Alt_Kalman:"); Serial.println(AltitudeKalman);


  // ==========================================================
  // 3. LAZOS PID Y MOTORES
  // ==========================================================



  // ==========================================================
  // 4. TEMPORIZACIÓN ESTRICTA (250 Hz = 4000 microsegundos)
  // ==========================================================
  // Esperamos hasta que se cumplan exactamente 4ms desde el último ciclo
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}