#include <Arduino.h>
#include "Config.h"
#include "Leds.h"

void initLeds() {
  pinMode(PIN_LED_SYS, OUTPUT);
  pinMode(PIN_LED_LINK, OUTPUT);
  pinMode(PIN_LED_ERR, OUTPUT);
}

void setLedSys(bool estado) {
  digitalWrite(PIN_LED_SYS, estado ? HIGH : LOW);
}

void setLedLink(bool estado) {
  digitalWrite(PIN_LED_LINK, estado ? HIGH : LOW);
}

void setLedErr(bool estado) {
  digitalWrite(PIN_LED_ERR, estado ? HIGH : LOW);
}

