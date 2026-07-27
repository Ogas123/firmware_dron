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

// ====================================================================
// ESTADOS INTEGRALES (LQI)
// ====================================================================
float integral_roll = 0.0f;
float integral_pitch = 0.0f;
const float MAX_WINDUP = 15.0f; // Límite de seguridad para que la integral no explote

void initControl() {
  u_roll = 0; u_pitch = 0; u_yaw = 0; u_alt = 0;
  integral_roll = 0.0f; 
  integral_pitch = 0.0f;
}

// ==================================================================================
// CONTROLADOR LQI - ESTADO ESTACIONARIO
// Ecuación: u(k) = -L * (x_hat(k) - x_ref) - Li * integral(error)
// ==================================================================================
void calcularControl() {
  
  // 0. Seguridad: Si el dron está apagado, reseteamos las integrales a cero
  // y cortamos las salidas para evitar acumular errores estando en el piso.
  if (estadoActual == APAGADO) {
    integral_roll = 0.0f;
    integral_pitch = 0.0f;
    u_roll = 0.0f; u_pitch = 0.0f; u_yaw = 0.0f; u_alt = 0.0f;
    return;
  }

  // 1. Canal Roll (Actitud LQI)
  float err_roll_0 = x_hat_roll[0] - DesiredAngleRoll;
  float err_roll_1 = x_hat_roll[1] - 0.0f; 
  
  integral_roll += err_roll_0 * h; // 'h' viene de Config.h (0.004s)
  integral_roll = constrain(integral_roll, -MAX_WINDUP, MAX_WINDUP); // Anti-windup
  
  // u = -K_p * error_pos - K_d * error_vel - K_i * error_integral
  u_roll = -(L_roll[0] * err_roll_0 + L_roll[1] * err_roll_1 + L_roll[2] * integral_roll);

  // 2. Canal Pitch (Actitud LQI)
  float err_pitch_0 = x_hat_pitch[0] - DesiredAnglePitch;
  float err_pitch_1 = x_hat_pitch[1] - 0.0f;
  
  integral_pitch += err_pitch_0 * h;
  integral_pitch = constrain(integral_pitch, -MAX_WINDUP, MAX_WINDUP); // Anti-windup
  
  u_pitch = -(L_pitch[0] * err_pitch_0 + L_pitch[1] * err_pitch_1 + L_pitch[2] * integral_pitch);

  // 3. Canal Yaw (Guiñada - 1D LQR Clásico)
  float err_yaw_0 = x_hat_yaw[0] - DesiredRateYaw;
  u_yaw = -(L_yaw[0] * err_yaw_0);

  // 4. Canal Altitud (Posición Vertical Z LQR Clásico)
  float err_alt_0 = x_hat_alt[0] - DesiredAltitude;
  float err_alt_1 = x_hat_alt[1] - 0.0f; // Queremos velocidad vertical 0 al llegar a la meta
  u_alt = -(L_alt[0] * err_alt_0 + L_alt[1] * err_alt_1);
  
  // Clamping de seguridad para la salida de control de altura
  if (u_alt > 500.0f)  u_alt = 500.0f;
  if (u_alt < -500.0f) u_alt = -500.0f;
}