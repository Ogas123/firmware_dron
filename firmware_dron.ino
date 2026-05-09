#include "Config.h"
#include "IMU.h"
#include "Comunicaciones.h"
#include "Motores.h"
#include "Control.h"
#include "ToF.h"

unsigned long tiempoBucle;

void setup() {
  Serial.begin(115200);
  
  //initComunicaciones();
  initIMU();
  initToF();
  //initMotores();
  //initControl();
  
}

void loop() {
  // 1. Refrescar los datos del sensor y correr el Filtro de Kalman
  leerIMU(); 
  //leerToF();

  //Serial.print("Roll rate [°/s] = ");
  //Serial.print(RateRoll);
  //Serial.print("\tPitch rate [°/s] = ");
  //Serial.print(RatePitch);
  //Serial.print("\tYaw rate [°/s] = ");
  //Serial.println(RateYaw);

  //Serial.println(distanciaAlturaMM);
  
  //Serial.print("Aceleracion X [g] = ");
  //Serial.print(AccX);
  //Serial.print("Aceleracion Y [g] = ");
  //Serial.print(AccY);
  //Serial.print("Aceleracion Z [g] = ");
  //Serial.println(AccZ);

  // 2. Imprimir los ángulos YA FILTRADOS (Notación Åström)
  Serial.print("Roll Estimado: "); 
  Serial.print(x_hat_Roll);
  Serial.print(" | Pitch Estimado: "); 
  Serial.println(x_hat_Pitch);

  // 3. Garantizar el tiempo de muestreo h = 0.004 (250Hz)
  delay(4); }