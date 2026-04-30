#ifndef IMU_H
#define IMU_H

// Usamos extern para que estas variables existan en todo el proyecto
extern float RateRoll, RatePitch, RateYaw;

void initIMU();
void leerIMU();

#endif