#include <Arduino.h>
#include "Control.h"
#include "IMU.h" 
#include "SensorFusion.h" 

// --- IMPORTACIÓN DE VARIABLES GLOBALES EXTERNAS ---
// Le decimos al compilador que esta variable se define en otro archivo
extern float distanciaAlturaMM;

// --- Salidas Finales a los Motores ---
float PID_Roll = 0, PID_Pitch = 0, PID_Yaw = 0;
float InputThrottle = 0; 

// ====================================================================
// 1. PARÁMETROS DEL LAZO EXTERNO: CONTROL DE ÁNGULO (Pitch & Roll)
// ====================================================================
// Setpoints para vuelo estacionario (Hovering)
float DesiredAngleRoll = 0.0;
float DesiredAnglePitch = 0.0;

// Salidas del lazo externo (se convierten en el Setpoint del lazo interno)
float DesiredRateRoll = 0.0;
float DesiredRatePitch = 0.0;

// Parámetros de Diseño PID Ángulo (Notación Åström)
// Típicamente el lazo externo es solo Proporcional. Si Ti o Td son 0, se anula su acción.
float K_Angle_Roll = 0.0, Ti_Angle_Roll = 0.0, Td_Angle_Roll = 0.0;
float K_Angle_Pitch = 0.0, Ti_Angle_Pitch = 0.0, Td_Angle_Pitch = 0.0;

float ItermAngleRoll = 0, ItermAnglePitch = 0;
float DtermAngleRoll = 0, DtermAnglePitch = 0;
float PrevAngleRoll = 0, PrevAnglePitch = 0; // Para el término D sobre la medición (y)


// ====================================================================
// 2. PARÁMETROS DEL LAZO INTERNO: CONTROL DE VELOCIDAD (Rate)
// ====================================================================
// El Yaw solo tiene control de velocidad (Rate Control)
float DesiredRateYaw = 0.0;

// Parámetros de Diseño PID Rate (Notación Åström)
float K_Rate_Roll = 1.5, Ti_Rate_Roll = 0.0, Td_Rate_Roll = 0.0;
float K_Rate_Pitch = 1.5, Ti_Rate_Pitch = 0.0, Td_Rate_Pitch = 0.0;
float K_Rate_Yaw = 1.5, Ti_Rate_Yaw = 0.0, Td_Rate_Yaw = 0.0;

float ItermRateRoll = 0, ItermRatePitch = 0, ItermRateYaw = 0;
float DtermRateRoll = 0, DtermRatePitch = 0, DtermRateYaw = 0;
float PrevRateRoll = 0, PrevRatePitch = 0, PrevRateYaw = 0;


// ====================================================================
// 3. PARÁMETROS DE CONTROL DE ALTITUD (Lazo Único Directo)
// ====================================================================
float alturaDeseada = 1000.0;     // Setpoint autónomo absoluto en milímetros (1 metro)
float HoverThrottle = 1500.0;     // PWM base (Feedforward) para equilibrar el MTOW de 66g

// Parámetros de Diseño PID Altitud (Notación Åström)
// Td_Alt inicia en 0. Como la medición del ToF (distanciaAlturaMM) tiene forma 
// de "escalera" cada 33ms, una acción derivativa alta generará picos de PWM.
// Se recomienda sintonizar primero como un controlador P o PI puro.
float K_Alt = 1.5, Ti_Alt = 5.0, Td_Alt = 0.0; 

float ItermAlt = 0;
float DtermAlt = 0;
float PrevAlt = 0; // Memoria del estado anterior (z)


// --- Constantes del Sistema Discreto ---
const float h = 0.004; // Periodo de muestreo (250 Hz)
const float N = 10.0;  // Filtro derivativo
const float MAX_I_TERM = 400.0; // Límite Anti-Windup

void initControl() {
  ItermAngleRoll = 0; ItermAnglePitch = 0;
  DtermAngleRoll = 0; DtermAnglePitch = 0;
  PrevAngleRoll = 0; PrevAnglePitch = 0;

  ItermRateRoll = 0; ItermRatePitch = 0; ItermRateYaw = 0;
  DtermRateRoll = 0; DtermRatePitch = 0; DtermRateYaw = 0;
  PrevRateRoll = 0; PrevRatePitch = 0; PrevRateYaw = 0;

  // Limpieza del lazo de Altitud
  ItermAlt = 0;
  DtermAlt = 0;
  PrevAlt = 0;
}

// ==================================================================================
// CONTROLADOR PID DISCRETO - ALGORITMO DE POSICIÓN
// Basado en la formulación canónica de Karl J. Åström & Tore Hägglund
// ==================================================================================
float calcularLazoPID(float Error, float MedidaActual, float &MedidaAnterior, 
                      float K, float Ti, float Td, 
                      float &Iterm, float &Dterm) {
  
  // 1. TÉRMINO PROPORCIONAL (P)
  // Responde al instante presente. 
  // P(k) = K * e(k)
  float Pterm = K * Error;

  // 2. TÉRMINO DERIVATIVO (D) CON FILTRO PASA-BAJOS
  // Discretización: Diferencia hacia atrás (Backward Euler).
  // Justificación de Åström: Evita el "ringing" (oscilaciones numéricas) que causaría Tustin.
  // Además, se deriva la salida 'y' (MedidaActual) y no el error 'e', para evitar impulsos 
  // infinitos (Derivative Kick) ante cambios bruscos de setpoint.
  if (Td > 0) {
    float ad = Td / (Td + N * h);   // Polo discreto del filtro derivativo
    float bd = K * ad * N;          // Ganancia derivativa filtrada (Notación exacta de Åström)
    
    // Ecuación en diferencias: D(k) = ad * D(k-1) - bd * (y(k) - y(k-1))
    Dterm = ad * Dterm - bd * (MedidaActual - MedidaAnterior);
  } else {
    Dterm = 0;
  }

  // 3. LEY DE CONTROL TOTAL
  // Se calcula la señal de control u(k) antes de actualizar la integral.
  // u(k) = P(k) + I(k) + D(k)
  float SalidaPID = Pterm + Iterm + Dterm;

  // 4. TÉRMINO INTEGRAL (I)
  // Discretización: Rectangular hacia adelante (Forward Euler).
  // Se actualiza el integrador para ser utilizado en el PRÓXIMO instante de muestreo (k+1).
  if (Ti > 0) {
    float bi = (K * h) / Ti; // Ganancia integral discreta (Notación bi de Åström)
    
    // I(k+1) = I(k) + bi * e(k)
    Iterm = Iterm + bi * Error;
    
    // Saturación Condicional (Anti-Windup por Clamping).
    // Nota académica: Åström prefiere la técnica de "Back-Calculation" (Seguimiento), 
    // pero requiere ingresar los límites físicos de saturación del motor dentro de esta función.
    if (Iterm > MAX_I_TERM) Iterm = MAX_I_TERM;
    else if (Iterm < -MAX_I_TERM) Iterm = -MAX_I_TERM;
  }

  // 5. ACTUALIZACIÓN DE MEMORIA DE ESTADO
  // Se guarda y(k) para el cálculo de la derivada en el próximo ciclo.
  MedidaAnterior = MedidaActual;

  return SalidaPID;
}

void calcularPID() {
  
  // ====================================================================
  // PASO 1: LAZO EXTERNO (Calcula qué tan rápido debemos rotar)
  // Entrada: Ángulos Deseados vs. Ángulos Estimados por Kalman (x_hat)
  // Salida: DesiredRate
  // ====================================================================
  
  float ErrorAngleRoll = DesiredAngleRoll - x_hat_Roll;
  DesiredRateRoll = calcularLazoPID(ErrorAngleRoll, x_hat_Roll, PrevAngleRoll, 
                                    K_Angle_Roll, Ti_Angle_Roll, Td_Angle_Roll, 
                                    ItermAngleRoll, DtermAngleRoll);

  float ErrorAnglePitch = DesiredAnglePitch - x_hat_Pitch;
  DesiredRatePitch = calcularLazoPID(ErrorAnglePitch, x_hat_Pitch, PrevAnglePitch, 
                                     K_Angle_Pitch, Ti_Angle_Pitch, Td_Angle_Pitch, 
                                     ItermAnglePitch, DtermAnglePitch);


  // ====================================================================
  // PASO 2: LAZO INTERNO (Calcula la fuerza a los motores para cumplir la rotación)
  // Entrada: DesiredRate (del lazo externo) vs. Rate (del giroscopio)
  // Salida: Señal de control final (PID)
  // ====================================================================

  float ErrorRateRoll = DesiredRateRoll - RateRoll;
  PID_Roll = calcularLazoPID(ErrorRateRoll, RateRoll, PrevRateRoll, 
                             K_Rate_Roll, Ti_Rate_Roll, Td_Rate_Roll, 
                             ItermRateRoll, DtermRateRoll);

  float ErrorRatePitch = DesiredRatePitch - RatePitch;
  PID_Pitch = calcularLazoPID(ErrorRatePitch, RatePitch, PrevRatePitch, 
                              K_Rate_Pitch, Ti_Rate_Pitch, Td_Rate_Pitch, 
                              ItermRatePitch, DtermRatePitch);

  // El eje YAW no tiene lazo externo, usa directamente el Setpoint de velocidad del piloto
  float ErrorRateYaw = DesiredRateYaw - RateYaw;
  PID_Yaw = calcularLazoPID(ErrorRateYaw, RateYaw, PrevRateYaw, 
                            K_Rate_Yaw, Ti_Rate_Yaw, Td_Rate_Yaw, 
                            ItermRateYaw, DtermRateYaw);


  // ====================================================================
  // PASO 3: CONTROL DE ALTITUD (Autónomo: Lazo Único de Posición)
  // Entrada: alturaDeseada vs. distanciaAlturaMM (Lectura del ToF VL53L1X)
  // Salida: InputThrottle
  // ====================================================================
  
  // 1. Calculamos el Error de Posición Z
  float ErrorAltitude = alturaDeseada - distanciaAlturaMM;

  // 2. Evaluamos el lazo PID directo sobre la posición
  //float PID_Altitude = calcularLazoPID(ErrorAltitude, distanciaAlturaMM, PrevAlt, 
  //                                     K_Alt, Ti_Alt, Td_Alt, 
  //                                     ItermAlt, DtermAlt);

  // 3. Salida final al acelerador (Feedforward de Hover + Esfuerzo de Control)
  //InputThrottle = HoverThrottle + PID_Altitude;
}