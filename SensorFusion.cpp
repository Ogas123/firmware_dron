#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "SensorFusion.h"

using namespace BLA;

// ====================================================================
// VARIABLES DE ESTADO Y COVARIANZA (Notación Åström)
// ====================================================================

// --- 1. KALMAN 1D (Actitud: Roll y Pitch) ---
// x_hat: Ángulo estimado (Scalar) | P: Covarianza del error
float x_hat_Roll = 0, P_Roll = 4.0f;
float x_hat_Pitch = 0, P_Pitch = 4.0f;

// --- 2. KALMAN 2D (Altitud: z y vz) ---
// x_hat_2d: Vector de estado [Altitud (z), Velocidad Vertical (v_z)]^T
BLA::Matrix<2,1> x_hat_2d = {0.0f, 0.0f};
// P_2d: Matriz de covarianza (2x2)
BLA::Matrix<2,2> P_2d = {100.0f, 0.0f, 
                         0.0f, 100.0f}; 

// Variables de salida para el control
float AltitudeKalman = 0;
float VelocityVerticalKalman = 0;

// ====================================================================
// MATRICES DE DISEÑO DEL SISTEMA (Notación Åström)
// ====================================================================
const float h = 0.004f; // Ts: Periodo de muestreo (250 Hz)

// --- Parámetros Filtro 1D ---
const float Q_1D = 16.0f;  // R1: Ruido del proceso (Giroscopio)
const float R_1D = 9.0f;   // R2: Ruido de medición (Acelerómetro)

// --- Matrices Filtro 2D (Espacio de Estados Discreto) ---
BLA::Matrix<2,2> Phi;       // F: Transición de estado
BLA::Matrix<2,1> Gamma_mat; // G: Matriz de entrada (Control)
BLA::Matrix<1,2> C_mat;     // H: Matriz de observación
BLA::Matrix<2,2> I_mat;     // I: Identidad

BLA::Matrix<2,2> R1_2D;     // Q: Ruido del proceso (Acelerómetro)
BLA::Matrix<1,1> R2_2D;     // R: Ruido de medición (ToF)

void initSensorFusion() {
  // x(k+1) = Phi * x(k) + Gamma * u(k)
  Phi = {1.0f, h, 
         0.0f, 1.0f};

  Gamma_mat = {0.5f * h * h, 
               h};

  C_mat = {1.0f, 0.0f}; // Solo medimos posición z

  I_mat = {1.0f, 0.0f, 
           0.0f, 1.0f};

  // --- TUNING (Ajuste de Pesos) ---
  // Subimos drásticamente sigma_acc para que el filtro "desconfíe" de la inercia
  // Esto reduce la pendiente del "serrucho"
  float sigma_acc = 400.0f; 
  R1_2D = Gamma_mat * ~Gamma_mat * (sigma_acc * sigma_acc);

  // Bajamos R2 para que el ToF tenga mucha más autoridad de corrección
  R2_2D = {25.0f}; 
}

// ====================================================================
// IMPLEMENTACIÓN DE FILTROS
// ====================================================================

// Kalman 1D para Roll y Pitch (Sincrónico)
void kalman_1d(float &x_hat, float &P, float u, float y) {
  x_hat = x_hat + h * u; 
  P = P + (h * h * Q_1D); 
  float K = P / (P + R_1D); 
  x_hat = x_hat + K * (y - x_hat);
  P = (1.0f - K) * P;
}

// Kalman 2D: Etapa de Predicción (Inercial a 250 Hz)
void kalman_2d_predict(float earth_acc_z) {
  BLA::Matrix<1,1> u = {earth_acc_z};

  // Predicción del estado y la incertidumbre
  x_hat_2d = Phi * x_hat_2d + Gamma_mat * u;
  P_2d = Phi * P_2d * ~Phi + R1_2D;
  
  AltitudeKalman = x_hat_2d(0,0);
  VelocityVerticalKalman = x_hat_2d(1,0);
}

// Kalman 2D: Etapa de Corrección (Óptica a ~30 Hz)
void kalman_2d_update(float medicion_ToF_mm) {
  // Lógica de "Hard Reset" si el error es absurdo (como en tu captura)
  if (abs(x_hat_2d(0,0) - medicion_ToF_mm) > 200.0f) {
      x_hat_2d(0,0) = medicion_ToF_mm;
      x_hat_2d(1,0) = 0.0f; 
      P_2d = {100.0f, 0.0f, 0.0f, 100.0f}; 
  }

  BLA::Matrix<1,1> y = {medicion_ToF_mm};

  // Ganancia de Kalman: K = P * H^T * (H * P * H^T + R)^-1
  BLA::Matrix<1,1> S = C_mat * P_2d * ~C_mat + R2_2D;
  BLA::Matrix<2,1> K = P_2d * ~C_mat * Inverse(S);
  
  // Actualización: x = x + K * (y - H*x)
  x_hat_2d = x_hat_2d + K * (y - C_mat * x_hat_2d);
  
  // Covarianza: P = (I - K*H) * P
  P_2d = (I_mat - K * C_mat) * P_2d;
  
  AltitudeKalman = x_hat_2d(0,0);
  VelocityVerticalKalman = x_hat_2d(1,0);
}