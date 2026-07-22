#include <Arduino.h>
#include "Kalman.h"

// ====================================================================
// VARIABLES DE ESTADO Y COVARIANZA (Dinámicas)
// ====================================================================
float x_hat_roll[2]  = {0.0f, 
                        0.0f};
float P_roll[2][2]   = {{10.0f, 0.0f}, 
                        {0.0f, 10.0f}}; // Incertidumbre inicial alta

float x_hat_pitch[2] = {0.0f, 
                        0.0f};
float P_pitch[2][2]  = {{10.0f, 0.0f}, 
                        {0.0f, 10.0f}};

float x_hat_alt[2]   = {0.0f, 
                        0.0f};
float P_alt[2][2]    = {{10.0f, 0.0f}, 
                        {0.0f, 10.0f}};

// Yaw (1x1)
float x_hat_yaw[1]   = {0.0f};
float P_yaw[1]       = {10.0f};

// ====================================================================
// MATRICES DEL SISTEMA LTI
// ====================================================================
const float h = 0.004f; 
const float Phi[2][2] = {{1.0f, h}, 
                        {0.0f, 1.0f}}; 

// Gamma (Reemplazar valores con los de Python)
const float Gamma_roll_pitch[2] = {0.0f, 
                                  0.012f}; 
const float Gamma_alt[2]        = {0.0f, 
                                  0.008f}; 
const float Gamma_yaw = 0.004f;

// Matrices de Covarianza de Ruido (Q y R)
const float Q_roll_pitch[2][2] = {{0.001f, 0.0f}, 
                                  {0.0f, 0.01f}};
const float R_roll_pitch[2][2] = {{0.05f, 0.0f}, 
                                  {0.0f, 0.002f}};
const float Q_yaw = 0.01f;

const float Q_alt[2][2] = {{0.0001f, 0.0f}, 
                          {0.0f, 0.001f}};
const float R_alt[2][2] = {{0.005f, 0.0f}, 
                          {0.0f, 0.01f}};
const float R_yaw = 0.002f;

// ====================================================================
// FILTRO DE KALMAN PARA ALTURA (C = [1, 0])
// ====================================================================
void updateKalmanAltura(float x[2], float P[2][2], float acc_z_g, float dist_tof_m) {
  
  // 1. Quitar la gravedad y convertir a m/s^2 (Aceleración neta)
  // Si el dron está en reposo, AccZ es ~1.0G. Si sube, AccZ > 1.0G.
  float a_net = (acc_z_g - 1.0f);

  // 2. PREDICCIÓN (Modelo Cinemático)
  float x_pred[2];
  // z = z + v*h + 0.5*a*h^2
  x_pred[0] = x[0] + h * x[1] + 0.5f * h * h * a_net;
  // v = v + a*h
  x_pred[1] = x[1] + h * a_net;

  // P^- = Phi * P_{k-1} * Phi^T + Q_alt
  float P_pred[2][2];
  P_pred[0][0] = P[0][0] + h*P[1][0] + h*(P[0][1] + h*P[1][1]) + Q_alt[0][0];
  P_pred[0][1] = P[0][1] + h*P[1][1] + Q_alt[0][1];
  P_pred[1][0] = P[1][0] + h*P[1][1] + Q_alt[1][0];
  P_pred[1][1] = P[1][1] + Q_alt[1][1];

  // 3. CÁLCULO DE GANANCIA (Como C = [1, 0], la innovación S es un escalar)
  // R_alt_scalar es el ruido de medición del ToF (R_alt[0][0] = 0.005f)
  float R_alt_scalar = 0.005f; 
  float S = P_pred[0][0] + R_alt_scalar;
  
  float K[2];
  K[0] = P_pred[0][0] / S;
  K[1] = P_pred[1][0] / S;

  // 4. CORRECCIÓN
  // El error se calcula ÚNICAMENTE con la medición de posición del ToF
  float err = dist_tof_m - x_pred[0];

  x[0] = x_pred[0] + K[0] * err;
  x[1] = x_pred[1] + K[1] * err;

  // P^+ = (I - K*C) * P^-
  P[0][0] = P_pred[0][0] - K[0] * P_pred[0][0];
  P[0][1] = P_pred[0][1] - K[0] * P_pred[0][1];
  P[1][0] = P_pred[1][0] - K[1] * P_pred[0][0];
  P[1][1] = P_pred[1][1] - K[1] * P_pred[0][1];
}

// ====================================================================
// FUNCIÓN RECURSIVA: FILTRO DE KALMAN 2x2 (Asumiendo C = Identidad)
// ====================================================================
void updateKalmanRecursive2x2(float x[2], float P[2][2], float u, const float y[2], 
                              const float Gamma[2], const float Q[2][2], const float R[2][2]) {
  
  // VARIABLES TEMPORALES
  float x_pred[2];
  float P_pred[2][2];
  float S[2][2];
  float K[2][2];

  // ---------------------------------------------------------
  // 1. PREDICCIÓN (Update de Tiempo)
  // ---------------------------------------------------------
  // \hat{x}^- = \Phi \hat{x}_{k-1} + \Gamma u
  x_pred[0] = Phi[0][0]*x[0] + Phi[0][1]*x[1] + Gamma[0]*u;
  x_pred[1] = Phi[1][0]*x[0] + Phi[1][1]*x[1] + Gamma[1]*u;

  // P^- = \Phi P_{k-1} \Phi^T + Q
  P_pred[0][0] = P[0][0] + h*P[1][0] + h*(P[0][1] + h*P[1][1]) + Q[0][0];
  P_pred[0][1] = P[0][1] + h*P[1][1] + Q[0][1];
  P_pred[1][0] = P[1][0] + h*P[1][1] + Q[1][0];
  P_pred[1][1] = P[1][1] + Q[1][1];

  // ---------------------------------------------------------
  // 2. CÁLCULO DE LA GANANCIA DE KALMAN DINÁMICA
  // ---------------------------------------------------------
  // Innovación de Covarianza: S = C P^- C^T + R  (Como C=I, S = P^- + R)
  S[0][0] = P_pred[0][0] + R[0][0];
  S[0][1] = P_pred[0][1] + R[0][1];
  S[1][0] = P_pred[1][0] + R[1][0];
  S[1][1] = P_pred[1][1] + R[1][1];

  // Inversa de S (Matriz 2x2)
  float detS = S[0][0]*S[1][1] - S[0][1]*S[1][0];
  float invS[2][2];
  invS[0][0] =  S[1][1] / detS;
  invS[0][1] = -S[0][1] / detS;
  invS[1][0] = -S[1][0] / detS;
  invS[1][1] =  S[0][0] / detS;

  // Ganancia K = P^- C^T S^{-1} (Como C=I, K = P^- S^{-1})
  K[0][0] = P_pred[0][0]*invS[0][0] + P_pred[0][1]*invS[1][0];
  K[0][1] = P_pred[0][0]*invS[0][1] + P_pred[0][1]*invS[1][1];
  K[1][0] = P_pred[1][0]*invS[0][0] + P_pred[1][1]*invS[1][0];
  K[1][1] = P_pred[1][0]*invS[0][1] + P_pred[1][1]*invS[1][1];

  // ---------------------------------------------------------
  // 3. CORRECCIÓN (Update de Medición)
  // ---------------------------------------------------------
  // Error (Innovación) = y - C \hat{x}^-
  float err_0 = y[0] - x_pred[0];
  float err_1 = y[1] - x_pred[1];

  // \hat{x}^+ = \hat{x}^- + K * Error
  x[0] = x_pred[0] + K[0][0]*err_0 + K[0][1]*err_1;
  x[1] = x_pred[1] + K[1][0]*err_0 + K[1][1]*err_1;

  // P^+ = (I - K C) P^- (Como C=I, P^+ = P^- - K P^-)
  P[0][0] = P_pred[0][0] - (K[0][0]*P_pred[0][0] + K[0][1]*P_pred[1][0]);
  P[0][1] = P_pred[0][1] - (K[0][0]*P_pred[0][1] + K[0][1]*P_pred[1][1]);
  P[1][0] = P_pred[1][0] - (K[1][0]*P_pred[0][0] + K[1][1]*P_pred[1][0]);
  P[1][1] = P_pred[1][1] - (K[1][0]*P_pred[0][1] + K[1][1]*P_pred[1][1]);
}

void actualizarFiltrosLQG(float u_roll, float u_pitch, float u_yaw, float u_alt,
                          float y_roll[2], float y_pitch[2], float y_yaw, 
                          float dist_tof_m, float acc_z_g) {
  
  // 1. Filtros 2x2 para Actitud (C = I)
  updateKalmanRecursive2x2(x_hat_roll, P_roll, u_roll, y_roll, Gamma_roll_pitch, Q_roll_pitch, R_roll_pitch);
  updateKalmanRecursive2x2(x_hat_pitch, P_pitch, u_pitch, y_pitch, Gamma_roll_pitch, Q_roll_pitch, R_roll_pitch);
  
  // 2. Filtro 1x1 para Yaw (Escalar)
  float x_pred_yaw = x_hat_yaw[0] + Gamma_yaw * u_yaw; 
  float P_pred_yaw = P_yaw[0] + Q_yaw;
  float K_yaw_gain = P_pred_yaw / (P_pred_yaw + R_yaw);
  
  x_hat_yaw[0] = x_pred_yaw + K_yaw_gain * (y_yaw - x_pred_yaw);
  P_yaw[0] = (1.0f - K_yaw_gain) * P_pred_yaw;

  // 3. NUEVO: Filtro Cinemático para Altura (C = [1, 0])
  updateKalmanAltura(x_hat_alt, P_alt, acc_z_g, dist_tof_m);
}