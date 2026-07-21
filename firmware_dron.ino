#include <Arduino.h>
#include "IMU.h"
#include "ToF.h"
#include "Motores.h"
#include "Config.h"
#include "Comunicaciones.h"
#include "Kalman.h"
#include "LQR.h"

uint32_t LoopTimer;

EstadoDron estadoActual = APAGADO;

void setup() {
  pinMode(PIN_LED_GREEN, OUTPUT); //LED SYS
  pinMode(PIN_LED_RED, OUTPUT);   //LED ERR
  pinMode(PIN_LED_BLUE, OUTPUT);  //LED LINK

  digitalWrite(PIN_LED_GREEN, HIGH);

  // Usamos una velocidad alta para que los prints no frenen el lazo de control
  Serial.begin(115200); 
  
  // Inicializamos los módulos en orden
  initIMU();
  initToF();
  initMotores();
  initComunicaciones();

  
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
  Serial.print("AccX:"); Serial.print(AccX); Serial.print(",");
  Serial.print("AccY:"); Serial.print(AccY); Serial.print(",");
  Serial.print("AccZ:"); Serial.print(AccZ); Serial.print(",");

  // --- ACTITUD (Ángulos) ---
  //Serial.print("Roll_acc:"); Serial.print(AngleRoll_Acc); Serial.print(",");
  //Serial.print("Roll_gyr:"); Serial.print(RateRoll); Serial.print(",");
  //Serial.print("Roll_Kalman:"); Serial.print(x_hat_Roll); Serial.print(",");
  
  //Serial.print("Pitch_acc:"); Serial.print(AnglePitch_Acc); Serial.print(",");
  //Serial.print("Pitch_gyr:"); Serial.print(RatePitch); Serial.print(",");
  //Serial.print("Pitch_Kalman:"); Serial.println(x_hat_Pitch); 

  // --- ALTITUD (Eje Z) ---
  //Serial.print("Alt_ToF_Raw:"); Serial.print(distanciaAlturaMM); Serial.print(",");


  // ==========================================================
  // 3. MAQUINA DE ESTADOS, LAZOS PID Y MOTORES
  // ==========================================================
  
  recibirComandosUDP(); // Chequeamos si llegó algo por red



  // ==========================================================
  // 4. TEMPORIZACIÓN ESTRICTA (250 Hz = 4000 microsegundos)
  // ==========================================================
  // Esperamos hasta que se cumplan exactamente 4ms desde el último ciclo
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}