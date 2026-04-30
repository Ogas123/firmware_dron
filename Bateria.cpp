#include <Arduino.h>
#include "Config.h"
#include "Bateria.h"

void initBateria() {
  analogReadResolution(12); // Mejorar la precisión del ADC (0 a 4095)
}

float leerVoltajeBateria() {
  // 1. Leemos el valor crudo del ADC (0 a 4095)
  int valorCrudo = analogRead(PIN_BATERIA);

  // 2. Convertimos el valor crudo a voltaje en el pin (Ref de 3.3V)
  float voltajePin = (valorCrudo / 4095.0) * 3.3;

  // 3. Multiplicamos por 2 para compensar el divisor de tensión del esquemático
  float voltajeBateriaReal = voltajePin * 2.0;

  return voltajeBateriaReal;
}

