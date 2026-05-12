#include <Arduino.h>
#include "Config.h"
#include "LEDs.h"

void initLeds() {
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
}

void setLedSys(bool estado) {
  // LED Verde para sistema (System)
  digitalWrite(PIN_LED_GREEN, estado ? HIGH : LOW);
}

void setLedLink(bool estado) {
  // LED Azul para conexión (Link)
  digitalWrite(PIN_LED_BLUE, estado ? HIGH : LOW);
}

void setLedErr(bool estado) {
  // LED Rojo para errores o batería baja
  digitalWrite(PIN_LED_RED, estado ? HIGH : LOW);
}