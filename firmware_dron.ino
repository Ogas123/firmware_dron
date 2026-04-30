#include "Config.h"
#include "IMU.h"
#include "Comunicaciones.h"
#include "Motores.h"
#include "Control.h"
// #include "ToF.h" // Descomentar cuando lo conectes

unsigned long tiempoBucle;

void setup() {
  Serial.begin(115200);
  
  initComunicaciones();
  initIMU();
  //initMotores();
  //initControl();
  
}

void loop() {
  leerIMU();

}