#include <Arduino.h>
#include "Kalman.h"

// ====================================================================
// VARIABLES DE ESTADO Y COVARIANZA (Dinámicas)
// ====================================================================
float x_hat_roll[2]  = {0.0f, 0.0f};
float P_roll[2][2]   = {{10.0f, 0.0f}, {0.0f, 10.0f}}; // Incertidumbre inicial alta

float x_hat_pitch[2] = {0.0f, 0.0f};
float P_pitch[2][2]  = {{10.0f, 0.0f}, {0.0f, 10.0f}};

float x_hat_alt[2]   = {0.0f, 0.0f};
float P_alt[2][2]    = {{10.0f, 0.0f}, {0.0f, 10.0f}};

// Yaw (1x1)
float x_hat_yaw[1]   = {0.0f};
float P_yaw[1]       = {10.0f};

// ====================================================================
// MATRICES DEL SISTEMA LTI
// ====================================================================
const float h = 0.004f; 
const float Phi[2][2] = {{1.0f, h}, {0.0f, 1.0f}}; 

// Gamma (Reemplazar valores con los de Python)
const float Gamma_roll_pitch[2] = {0.0f, 0.012f}; 
const float Gamma_alt[2]        = {0.0f, 0.008f}; 

// Matrices de Covarianza de Ruido (Q y R)
const float Q_roll_pitch[2][2] = {{0.001f, 0.0f}, {0.0f, 0.01f}};
const float R_roll_pitch[2][2] = {{0.05f, 0.0f}, {0.0f, 0.002f}};

const float Q_alt[2][2] = {{0.0001f, 0.0f}, {0.0f, 0.001f}};
const float R_alt[2][2] = {{0.005f, 0.0f}, {0.0f, 0.01f}};

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
                          float y_roll[2], float y_pitch[2], float y_yaw, float y_alt[2]) {
  
  // Los filtros corren de manera recursiva actualizando su propia covarianza P
  updateKalmanRecursive2x2(x_hat_roll, P_roll, u_roll, y_roll, Gamma_roll_pitch, Q_roll_pitch, R_roll_pitch);
  updateKalmanRecursive2x2(x_hat_pitch, P_pitch, u_pitch, y_pitch, Gamma_roll_pitch, Q_roll_pitch, R_roll_pitch);
  updateKalmanRecursive2x2(x_hat_alt, P_alt, u_alt, y_alt, Gamma_alt, Q_alt, R_alt);
  
  // (El de Yaw se haría análogo pero 1x1, resolviendo matemática escalar simple)
}