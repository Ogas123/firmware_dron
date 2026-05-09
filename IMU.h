#ifndef IMU_H
#define IMU_H

// Usamos extern para que estas variables existan en todo el proyecto
extern float RateRoll, RatePitch, RateYaw;
extern float AccX, AccY, AccZ;
extern float AngleRoll, AnglePitch;

void initIMU();
void leerIMU();

#endif