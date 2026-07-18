#include <Arduino.h>
#include "SensorFusion.h"

// ====================================================================
// VARIABLES DE ESTADO Y COVARIANZA
// ====================================================================

// --- KALMAN  (Actitud: Roll y Pitch) ---
// Notación: x_hat es la predicción con anticipación de un paso (\hat{x}_{k|k-1})
// P es la covarianza del error de predicción P_k
float x_hat_Roll = 0.0f;
float P_Roll = 4.0f;

float x_hat_Pitch = 0.0f;
float P_Pitch = 4.0f;

// ====================================================================
// PARÁMETROS DEL SISTEMA (Diseño en Espacio de Estados)
// ====================================================================
const float h = 0.004f; // Periodo de muestreo (250 Hz)

// Matrices del sistema: x_{k+1} = \Phi x_k + \Gamma u_k + v_k
const float Phi = 1.0f;   // \Phi: Matriz de transición de estado
const float Gamma = h;    // \Gamma: Matriz de control (integra la velocidad angular)

// Matriz de medición: y_k = C x_k + e_k
const float C_mat = 1.0f; // C: Matriz de observación

// Modelos de Ruido (Matrices de covarianza R1, R2 y R12)
const float R1 = 16.0f;   // R_1: Covarianza del ruido del proceso v_k (Giroscopio)
const float R2 = 9.0f;    // R_2: Covarianza del ruido de medición e_k (Acelerómetro)
const float R12 = 0.0f;   // R_{12}: Covarianza cruzada nula

void initSensorFusion() {
  x_hat_Roll = 0.0f;
  P_Roll = 4.0f;
  
  x_hat_Pitch = 0.0f;
  P_Pitch = 4.0f;
}

// ====================================================================
// FILTRO DE KALMAN (Problema de predicción con anticipación de un paso)
// ====================================================================

// u: Señal de control u_k (Tasa del giroscopio)
// y: Observación y_k (Ángulo medido por el acelerómetro)
void kalman(float &x_hat, float &P, float u, float y) {
  
  // 1. Cálculo de la ganancia óptima K_k
  // K_k = (\Phi * P_k * C^T + R_{12}) * (R_2 + C * P_k * C^T)^{-1}
  float K_k = (Phi * P * C_mat + R12) / (R2 + C_mat * P * C_mat); 
  
  // 2. Estimador de estados (Predicción)
  // \hat{x}_{k+1|k} = \Phi * \hat{x}_{k|k-1} + \Gamma * u_k + K_k * (y_k - C * \hat{x}_{k|k-1})
  float x_hat_next = (Phi * x_hat) + (Gamma * u) + K_k * (y - (C_mat * x_hat));
  
  // 3. Actualización de la covarianza del error
  // P_{k+1} = \Phi * P_k * \Phi^T + R_1 - K_k * (R_2 + C * P_k * C^T) * K_k^T
  float P_next = (Phi * P * Phi) + R1 - K_k * (R2 + C_mat * P * C_mat) * K_k;
  
  // 4. Desplazamiento temporal (Memoria de estado para el ciclo k+1)
  x_hat = x_hat_next;
  P = P_next;
}