#include <Arduino.h>
#include "Control.h"
#include "IMU.h" 

// --- Salidas del PID ---
float PID_Roll = 0, PID_Pitch = 0, PID_Yaw = 0;
float DesiredRateRoll = 0, DesiredRatePitch = 0, DesiredRateYaw = 0;

// --- Parámetros de Diseño PID (Notación Åström) ---
// En lugar de Kp, Ki, Kd de formato empírico, usamos Ganancia (K), Tiempo Integral (Ti) y Tiempo Derivativo (Td)
float K_Roll = 1.2, Ti_Roll = 2.0, Td_Roll = 0.05;
float K_Pitch = 1.2, Ti_Pitch = 2.0, Td_Pitch = 0.05;
float K_Yaw = 2.0, Ti_Yaw = 2.0, Td_Yaw = 0.0;

// Parámetros de muestreo y filtrado
const float h = 0.004; // Periodo de muestreo en segundos (250 Hz)
const float N = 10.0;  // Factor limitador de ganancia derivativa en alta frecuencia

// --- Variables de Estado del Controlador ---
// El término integral se guarda de una iteración a la otra
float ItermRoll = 0, ItermPitch = 0, ItermYaw = 0;

// El término derivativo filtrado necesita memoria de su estado anterior
float DtermRoll = 0, DtermPitch = 0, DtermYaw = 0;

// Para evitar el "Derivative Kick" se guarda la lectura anterior del sensor (y), no el error
float PrevRateRoll = 0, PrevRatePitch = 0, PrevRateYaw = 0;

// Límite Anti-Windup
const float MAX_I_TERM = 400.0; 

void initControl() {
  ItermRoll = 0; ItermPitch = 0; ItermYaw = 0;
  DtermRoll = 0; DtermPitch = 0; DtermYaw = 0;
  PrevRateRoll = 0; PrevRatePitch = 0; PrevRateYaw = 0;
}

void calcularPID() {
  // Coeficientes precalculados para el filtro derivativo (Backward difference)
  // Ecuaciones: ad = Td / (Td + N*h)  |  bd = (K * Td * N) / (Td + N*h)
  
  float ad_Roll = Td_Roll / (Td_Roll + N * h);
  float bd_Roll = (K_Roll * Td_Roll * N) / (Td_Roll + N * h);
  
  float ad_Pitch = Td_Pitch / (Td_Pitch + N * h);
  float bd_Pitch = (K_Pitch * Td_Pitch * N) / (Td_Pitch + N * h);
  
  float ad_Yaw = Td_Yaw / (Td_Yaw + N * h);
  float bd_Yaw = (K_Yaw * Td_Yaw * N) / (Td_Yaw + N * h);

  // 1. Cálculo del error actual (Setpoint - Medición)
  float ErrorRoll = DesiredRateRoll - RateRoll;
  float ErrorPitch = DesiredRatePitch - RatePitch;
  float ErrorYaw = DesiredRateYaw - RateYaw;

  // 2. Términos Proporcionales (P)
  float PtermRoll = K_Roll * ErrorRoll;
  float PtermPitch = K_Pitch * ErrorPitch;
  float PtermYaw = K_Yaw * ErrorYaw;

  // 3. Términos Derivativos (D) con filtro pasa-bajos y operando sobre la medición (y)
  DtermRoll = ad_Roll * DtermRoll - bd_Roll * (RateRoll - PrevRateRoll);
  DtermPitch = ad_Pitch * DtermPitch - bd_Pitch * (RatePitch - PrevRatePitch);
  DtermYaw = ad_Yaw * DtermYaw - bd_Yaw * (RateYaw - PrevRateYaw);

  // 4. Salida PID Total (Algoritmo de Posición)
  PID_Roll = PtermRoll + ItermRoll + DtermRoll;
  PID_Pitch = PtermPitch + ItermPitch + DtermPitch;
  PID_Yaw = PtermYaw + ItermYaw + DtermYaw;

  // 5. Actualización de los Términos Integrales (I) por Aproximación Forward (Euler) para el próximo ciclo
  ItermRoll += (K_Roll * h / Ti_Roll) * ErrorRoll;
  ItermPitch += (K_Pitch * h / Ti_Pitch) * ErrorPitch;
  ItermYaw += (K_Yaw * h / Ti_Yaw) * ErrorYaw;

  // 6. Anti-Windup: Detener la acumulación si excede el límite saturando el valor
  if (ItermRoll > MAX_I_TERM) ItermRoll = MAX_I_TERM;
  else if (ItermRoll < -MAX_I_TERM) ItermRoll = -MAX_I_TERM;
  
  if (ItermPitch > MAX_I_TERM) ItermPitch = MAX_I_TERM;
  else if (ItermPitch < -MAX_I_TERM) ItermPitch = -MAX_I_TERM;

  if (ItermYaw > MAX_I_TERM) ItermYaw = MAX_I_TERM;
  else if (ItermYaw < -MAX_I_TERM) ItermYaw = -MAX_I_TERM;

  // 7. Almacenar variables para la siguiente iteración
  PrevRateRoll = RateRoll;
  PrevRatePitch = RatePitch;
  PrevRateYaw = RateYaw;
}