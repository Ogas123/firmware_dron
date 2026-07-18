#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

// --- Estados Estimados de Actitud (1D) ---
extern float x_hat_Roll;
extern float x_hat_Pitch;
extern float P_Roll;
extern float P_Pitch;


// --- Funciones del Módulo ---
void initSensorFusion();

// Filtro 1D (Giroscopio + Acelerómetro)
void kalman(float &x_hat, float &P, float u, float y);


#endif