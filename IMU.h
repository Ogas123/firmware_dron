#ifndef IMU_H
#define IMU_H

// Usamos extern para que estas variables existan en todo el proyecto
extern float RateRoll, RatePitch, RateYaw;
extern float AccX, AccY, AccZ;

// Nuevas variables del Filtro de Kalman (Notación Åström)
extern float x_hat_Roll;
extern float x_hat_Pitch;

// Opcional: Ángulos brutos del acelerómetro (útiles para graficar y comparar)
extern float AngleRoll_Acc;
extern float AnglePitch_Acc;

void initIMU();
void leerIMU();

#endif