#include <Arduino.h>
#include "LQR.h" 
#include "Config.h"
#include "Supervisor.h" // Para leer el estadoActual y apagar los motores
#include "Telemetria.h"

// --- IMPORTACIÓN DE VARIABLES GLOBALES EXTERNAS ---
extern float x_hat_roll[2];
extern float x_hat_pitch[2];
extern float x_hat_yaw[1];
extern float x_hat_alt[2];
extern EstadoDron estadoActual;

// --- Salidas Finales a los Motores ---
float u_roll = 0, u_pitch = 0, u_yaw = 0, u_alt = 0;

// ====================================================================
// SETPOINTS DEL SISTEMA
// ====================================================================
float DesiredAngleRoll  = 0.0f;
float DesiredAnglePitch = 0.0f;
float DesiredRateYaw    = 0.0f;
float DesiredAltitude   = 0.0f;

void initControl() {
  u_roll = 0.0f; u_pitch = 0.0f; u_yaw = 0.0f; u_alt = 0.0f;
}

// ==================================================================================
// CONTROLADOR LQR ÓPTIMO EN ESTADO ESTACIONARIO (2x2)
// Ecuación: u(k) = -L * (x_hat(k) - x_ref)
// ==================================================================================
void calcularControl() {
  
  // 0. Seguridad: Si el dron está apagado, cortamos las salidas de control
  if (estadoActual == APAGADO) {
    u_roll = 0.0f; u_pitch = 0.0f; u_yaw = 0.0f; u_alt = 0.0f;
    return;
  }

  // 1. Canal Roll (LQR 2x2 - Inclinación Lateral phi y Tasa p)
  float err_roll_0 = x_hat_roll[0] - (DesiredAngleRoll + TRIM_ROLL);
  float err_roll_1 = x_hat_roll[1] - 0.0f; 
  u_roll = -(L_roll[0] * err_roll_0 + L_roll[1] * err_roll_1);

  // 2. Canal Pitch (LQR 2x2 - Inclinación Longitudinal theta y Tasa q)
  float err_pitch_0 = x_hat_pitch[0] - (DesiredAnglePitch + TRIM_PITCH);
  float err_pitch_1 = x_hat_pitch[1] - 0.0f;
  u_pitch = -(L_pitch[0] * err_pitch_0 + L_pitch[1] * err_pitch_1);

  // 3. Canal Yaw (LQR 1D - Tasa de Guiñada r)
  float err_yaw_0 = x_hat_yaw[0] - DesiredRateYaw;
  u_yaw = -(L_yaw[0] * err_yaw_0);

  // 4. Canal Altitud (LQR 2D - Posición z y Velocidad vz)
  float err_alt_0 = x_hat_alt[0] - DesiredAltitude;
  float err_alt_1 = x_hat_alt[1] - 0.0f;
  u_alt = -(L_alt[0] * err_alt_0 + L_alt[1] * err_alt_1);

  // Clamping de seguridad para empuje de altura (+/- 300 PWM)
  if (u_alt > 300.0f)  u_alt = 300.0f;
  if (u_alt < -300.0f) u_alt = -300.0f;
}