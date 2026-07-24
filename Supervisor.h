#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <Arduino.h>

// Variables del supervisor de vuelo
extern float AlturaObjetivoFinal;
extern float TasaAscenso;
extern float baseThrottleDinamico;

// Prototipo de la función
void ejecutarSupervisorVuelo();

#endif