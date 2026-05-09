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

  Serial.print("Roll [] = ");
  Serial.print(AngleRoll);
  Serial.print("Pitch [] = ");
  Serial.println(AnglePitch);

  delay(50); // Simulación temporal del tiempo de ciclo
}