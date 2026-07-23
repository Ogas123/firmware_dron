#include <Arduino.h>
#include "Kalman.h"
#include "Config.h"

// ====================================================================
// VARIABLES DE ESTADO Y COVARIANZA
// ====================================================================
float x_hat_roll[2]  = {0.0f,  
                        0.0f};
float P_roll[2][2]   = {{10.0f, 0.0f}, 
                        {0.0f, 10.0f}}; 

float x_hat_pitch[2] = {0.0f, 0.0f};
float P_pitch[2][2]  = {{10.0f, 0.0f}, 
                        {0.0f, 10.0f}};

float x_hat_alt[2]   = {0.0f, 
                        0.0f};
float P_alt[2][2]    = {{10.0f, 0.0f}, 
                        {0.0f, 10.0f}};

float x_hat_yaw[1]   = {0.0f};
float P_yaw[1]       = {10.0f};

// ====================================================================
// FUNCIÓN RECURSIVA: FILTRO DE KALMAN 2x2 (C = Identidad)
// ====================================================================
void updateKalmanRecursive2x2(float x[2], float P[2][2], float u, const float y[2], 
                              const float Gamma[2], const float Q[2][2], const float R[2][2]) {
  
  float x_pred[2];
  float P_pred[2][2];
  float S[2][2];
  float K[2][2];

  // 1. PREDICCIÓN (Cambio: Usamos Phi_2x2)
  x_pred[0] = Phi_2x2[0][0]*x[0] + Phi_2x2[0][1]*x[1] + Gamma[0]*u;
  x_pred[1] = Phi_2x2[1][0]*x[0] + Phi_2x2[1][1]*x[1] + Gamma[1]*u;

  // P^- = Phi * P_{k-1} * Phi^T + Q (Cambio: Usamos Ts en vez de h)
  P_pred[0][0] = P[0][0] + h*P[1][0] + h*(P[0][1] + h*P[1][1]) + Q[0][0];
  P_pred[0][1] = P[0][1] + h*P[1][1] + Q[0][1];
  P_pred[1][0] = P[1][0] + h*P[1][1] + Q[1][0];
  P_pred[1][1] = P[1][1] + Q[1][1];

  // 2. GANANCIA DE KALMAN
  S[0][0] = P_pred[0][0] + R[0][0];
  S[0][1] = P_pred[0][1] + R[0][1];
  S[1][0] = P_pred[1][0] + R[1][0];
  S[1][1] = P_pred[1][1] + R[1][1];

  float detS = S[0][0]*S[1][1] - S[0][1]*S[1][0];
  float invS[2][2];
  invS[0][0] =  S[1][1] / detS;
  invS[0][1] = -S[0][1] / detS;
  invS[1][0] = -S[1][0] / detS;
  invS[1][1] =  S[0][0] / detS;

  K[0][0] = P_pred[0][0]*invS[0][0] + P_pred[0][1]*invS[1][0];
  K[0][1] = P_pred[0][0]*invS[0][1] + P_pred[0][1]*invS[1][1];
  K[1][0] = P_pred[1][0]*invS[0][0] + P_pred[1][1]*invS[1][0];
  K[1][1] = P_pred[1][0]*invS[0][1] + P_pred[1][1]*invS[1][1];

  // 3. CORRECCIÓN
  float err_0 = y[0] - x_pred[0];
  float err_1 = y[1] - x_pred[1];

  x[0] = x_pred[0] + K[0][0]*err_0 + K[0][1]*err_1;
  x[1] = x_pred[1] + K[1][0]*err_0 + K[1][1]*err_1;

  P[0][0] = P_pred[0][0] - (K[0][0]*P_pred[0][0] + K[0][1]*P_pred[1][0]);
  P[0][1] = P_pred[0][1] - (K[0][0]*P_pred[0][1] + K[0][1]*P_pred[1][1]);
  P[1][0] = P_pred[1][0] - (K[1][0]*P_pred[0][0] + K[1][1]*P_pred[1][0]);
  P[1][1] = P_pred[1][1] - (K[1][0]*P_pred[0][1] + K[1][1]*P_pred[1][1]);
}

// ====================================================================
// FILTRO DE KALMAN PARA ALTURA (C = [1, 0])
// ====================================================================
void updateKalmanAltura(float x[2], float P[2][2], float acc_z_ms2, float dist_tof_m) {
  
  // Como la lectura ya está en m/s^2, la aceleración neta es una simple resta.
  float a_net = acc_z_ms2 - 9.80665;

  float x_pred[2];
  // Predicción cinemática usando 'h' (definido en Config.h)
  x_pred[0] = x[0] + h * x[1] + 0.5f * h * h * a_net;
  x_pred[1] = x[1] + h * a_net;

  float P_pred[2][2];
  P_pred[0][0] = P[0][0] + h*P[1][0] + h*(P[0][1] + h*P[1][1]) + Q_alt[0][0];
  P_pred[0][1] = P[0][1] + h*P[1][1] + Q_alt[0][1];
  P_pred[1][0] = P[1][0] + h*P[1][1] + Q_alt[1][0];
  P_pred[1][1] = P[1][1] + Q_alt[1][1];

  float S = P_pred[0][0] + R_alt_scalar; 
  
  float K[2];
  K[0] = P_pred[0][0] / S;
  K[1] = P_pred[1][0] / S;

  float err = dist_tof_m - x_pred[0];

  x[0] = x_pred[0] + K[0] * err;
  x[1] = x_pred[1] + K[1] * err;

  P[0][0] = P_pred[0][0] - K[0] * P_pred[0][0];
  P[0][1] = P_pred[0][1] - K[0] * P_pred[0][1];
  P[1][0] = P_pred[1][0] - K[1] * P_pred[0][0];
  P[1][1] = P_pred[1][1] - K[1] * P_pred[0][1];
}

// ====================================================================
// BUCLE PRINCIPAL DE ESTIMACIÓN (LQG)
// ====================================================================
void actualizarFiltrosLQG(float u_roll, float u_pitch, float u_yaw, float u_alt,
                          float y_roll[2], float y_pitch[2], float y_yaw, 
                          float dist_tof_m, float acc_z_ms2) { 
  
  updateKalmanRecursive2x2(x_hat_roll, P_roll, u_roll, y_roll, Gamma_roll_pitch, Q_roll_pitch, R_roll_pitch);
  updateKalmanRecursive2x2(x_hat_pitch, P_pitch, u_pitch, y_pitch, Gamma_roll_pitch, Q_roll_pitch, R_roll_pitch);
  
  float x_pred_yaw = x_hat_yaw[0] + Gamma_yaw * u_yaw; 
  float P_pred_yaw = P_yaw[0] + Q_yaw;
  float K_yaw_gain = P_pred_yaw / (P_pred_yaw + R_yaw);
  
  x_hat_yaw[0] = x_pred_yaw + K_yaw_gain * (y_yaw - x_pred_yaw);
  P_yaw[0] = (1.0f - K_yaw_gain) * P_pred_yaw;

  updateKalmanAltura(x_hat_alt, P_alt, acc_z_ms2, dist_tof_m);
}