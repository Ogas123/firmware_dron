#ifndef IMU_H
#define IMU_H

// Variables en crudo de la IMU
extern float RateRoll, RatePitch, RateYaw;
extern float AccX, AccY, AccZ;

// Variables del Filtro de Kalman 1D (Actitud)
extern float x_hat_Roll;
extern float x_hat_Pitch;

// Ángulos brutos del acelerómetro (opcionales para graficar)
extern float AngleRoll_Acc;
extern float AnglePitch_Acc;

// Nuevas variables de Cinemática Vertical (para Filtro 2D ToF)
extern float EarthAccZ;

void initIMU();
void leerIMU();

#endif