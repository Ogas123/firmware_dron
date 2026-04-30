#ifndef TOF_H
#define TOF_H

// Variable global para que el PID de altura la pueda leer
extern int distanciaAlturaMM; 

void initToF();
void leerToF();

#endif