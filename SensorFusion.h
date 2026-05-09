#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

// --- Estados Estimados de Actitud (1D) ---
extern float x_hat_Roll;
extern float x_hat_Pitch;
extern float P_Roll;
extern float P_Pitch;

// --- Estados Estimados de Altitud (2D) ---
extern float AltitudeKalman;         // z estimado (mm)
extern float VelocityVerticalKalman; // v_z estimado (mm/s)

// --- Funciones del Módulo ---
void initSensorFusion();

// Filtro 1D (Giroscopio + Acelerómetro)
void kalman_1d(float &x_hat, float &P, float u, float y);

// Filtro 2D (Cinemática Z + Láser ToF)
void kalman_2d_predict(float earth_acc_z);
void kalman_2d_update(float medicion_ToF_mm);

#endif