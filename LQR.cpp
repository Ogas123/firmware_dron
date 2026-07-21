#include <Arduino.h>
#include "LQR.h" 

// --- IMPORTACIÓN DE VARIABLES GLOBALES EXTERNAS ---
extern float x_hat_roll[2];
extern float x_hat_pitch[2];
extern float x_hat_yaw[1];
extern float x_hat_alt[2];

// --- Salidas Finales a los Motores ---
float u_roll = 0, u_pitch = 0, u_yaw = 0, u_alt = 0;

// ====================================================================
// SETPOINTS DEL SISTEMA
// ====================================================================
float DesiredAngleRoll  = 0.0f;
float DesiredAnglePitch = 0.0f;
float DesiredRateYaw    = 0.0f;
float DesiredAltitude   = 1.0f; // Asegurar consistencia de unidades (ej. metros) con el modelo Python

// ====================================================================
// GANANCIAS LQR ÓPTIMAS (Calculadas en Python)
// Reemplazar con las salidas reales de la consola de tu Notebook
// ====================================================================
const float L_roll[2]  = {14.09f, 15.38f}; 
const float L_pitch[2] = {14.09f, 15.38f}; 
const float L_yaw[1]   = {3.16f};          
const float L_alt[2]   = {15.49f, 5.01f};  

void initControl() {
  u_roll = 0; u_pitch = 0; u_yaw = 0; u_alt = 0;
}

// ==================================================================================
// CONTROLADOR LQR - ESTADO ESTACIONARIO
// Ecuación: u(k) = -L * (x_hat(k) - x_ref)
// ==================================================================================
void calcularControl() {
  
  // 1. Canal Roll (Actitud)
  // El error se forma restando el setpoint al estado actual estimado.
  // El setpoint de la velocidad angular (índice 1) es siempre 0 en hovering.
  float err_roll_0 = x_hat_roll[0] - DesiredAngleRoll;
  float err_roll_1 = x_hat_roll[1] - 0.0f; 
  u_roll = -(L_roll[0] * err_roll_0 + L_roll[1] * err_roll_1);

  // 2. Canal Pitch (Actitud)
  float err_pitch_0 = x_hat_pitch[0] - DesiredAnglePitch;
  float err_pitch_1 = x_hat_pitch[1] - 0.0f;
  u_pitch = -(L_pitch[0] * err_pitch_0 + L_pitch[1] * err_pitch_1);

  // 3. Canal Yaw (Guiñada - 1D)
  float err_yaw_0 = x_hat_yaw[0] - DesiredRateYaw;
  u_yaw = -(L_yaw[0] * err_yaw_0);

  // 4. Canal Altitud (Posición Vertical Z)
  float err_alt_0 = x_hat_alt[0] - DesiredAltitude;
  float err_alt_1 = x_hat_alt[1] - 0.0f; // Queremos velocidad vertical 0 al llegar a la meta
  u_alt = -(L_alt[0] * err_alt_0 + L_alt[1] * err_alt_1);
  
  // Nota: Estas salidas u_* se sumarán luego a tu Throttle Base (HoverThrottle) 
  // dentro de tu función del Mixer de motores.
}