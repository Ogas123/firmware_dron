#include <Arduino.h>
#include "Control.h"
#include "IMU.h" // De aquí tomamos x_hat_Roll, x_hat_Pitch y los Rate del giroscopio
#include "SensorFusion.h" // De aquí tomamos AltitudeKalman y VelocityVerticalKalman

// --- Salidas Finales a los Motores ---
float PID_Roll = 0, PID_Pitch = 0, PID_Yaw = 0;
float InputThrottle = 1000; 

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
float K_Angle_Roll = 2.0, Ti_Angle_Roll = 0.0, Td_Angle_Roll = 0.0;
float K_Angle_Pitch = 2.0, Ti_Angle_Pitch = 0.0, Td_Angle_Pitch = 0.0;

float ItermAngleRoll = 0, ItermAnglePitch = 0;
float DtermAngleRoll = 0, DtermAnglePitch = 0;
float PrevAngleRoll = 0, PrevAnglePitch = 0; // Para el término D sobre la medición (y)


// ====================================================================
// 2. PARÁMETROS DEL LAZO INTERNO: CONTROL DE VELOCIDAD (Rate)
// ====================================================================
// El Yaw solo tiene control de velocidad (Rate Control)
float DesiredRateYaw = 0.0;

// Parámetros de Diseño PID Rate (Notación Åström)
float K_Rate_Roll = 1.2, Ti_Rate_Roll = 2.0, Td_Rate_Roll = 0.05;
float K_Rate_Pitch = 1.2, Ti_Rate_Pitch = 2.0, Td_Rate_Pitch = 0.05;
float K_Rate_Yaw = 2.0, Ti_Rate_Yaw = 2.0, Td_Rate_Yaw = 0.0;

float ItermRateRoll = 0, ItermRatePitch = 0, ItermRateYaw = 0;
float DtermRateRoll = 0, DtermRatePitch = 0, DtermRateYaw = 0;
float PrevRateRoll = 0, PrevRatePitch = 0, PrevRateYaw = 0;


// ====================================================================
// 3. PARÁMETROS DE CONTROL DE ALTITUD (Cascada Posición -> Velocidad)
// ====================================================================
float TargetAltitude_mm = 1000.0; // Setpoint autónomo absoluto (1 metro)
float HoverThrottle = 1500.0;     // PWM base para vencer la gravedad

// Lazo Externo: Posición Z (Proporcional)
float K_Pos_Z = 1.2;
float MaxVelocity_Z = 600.0;      // Límite de seguridad: subida/bajada máx a 60cm/s
float DesiredVelocityVertical = 0.0;

// Lazo Interno: Velocidad Vertical Z (PID Notación Åström)
float K_Vel_Z = 3.5, Ti_Vel_Z = 9.33, Td_Vel_Z = 0.71;

float ItermVelZ = 0;
float DtermVelZ = 0;
float PrevVelZ = 0;


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

  ItermVelZ = 0;
  DtermVelZ = 0;
  PrevVelZ = 0;
}

// Función auxiliar para evitar código repetido en el cálculo del PID (Estructura Åström)
// Resuelve el Algoritmo de Posición con Euler Forward para el integrador y Backward para el derivativo
float calcularLazoPID(float Error, float MedidaActual, float &MedidaAnterior, 
                      float K, float Ti, float Td, 
                      float &Iterm, float &Dterm) {
  
  // 1. Término Proporcional
  float Pterm = K * Error;

  // 2. Término Derivativo (Filtro pasa-bajos sobre la medición 'y' para evitar el Derivative Kick)
  if (Td > 0) {
    float ad = Td / (Td + N * h);
    float bd = (K * Td * N) / (Td + N * h);
    Dterm = ad * Dterm - bd * (MedidaActual - MedidaAnterior);
  } else {
    Dterm = 0;
  }

  // 3. Salida PID Total
  float SalidaPID = Pterm + Iterm + Dterm;

  // 4. Actualización del Integrador para el próximo ciclo (Euler Forward)
  if (Ti > 0) {
    Iterm += (K * h / Ti) * Error;
    // Anti-Windup
    if (Iterm > MAX_I_TERM) Iterm = MAX_I_TERM;
    else if (Iterm < -MAX_I_TERM) Iterm = -MAX_I_TERM;
  }

  // 5. Guardar estado
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
  // PASO 3: CONTROL DE ALTITUD (Autónomo: Posición -> Velocidad Vertical)
  // Entrada: TargetAltitude_mm vs. AltitudeKalman y VelocityVerticalKalman
  // Salida: InputThrottle
  // ====================================================================
  
  // 1. Lazo Externo: Control de Posición Z (Genera la velocidad objetivo)
  float ErrorAltitude = TargetAltitude_mm - AltitudeKalman;
  DesiredVelocityVertical = K_Pos_Z * ErrorAltitude;

  // Saturación de la velocidad de ascenso/descenso
  if (DesiredVelocityVertical > MaxVelocity_Z) DesiredVelocityVertical = MaxVelocity_Z;
  if (DesiredVelocityVertical < -MaxVelocity_Z) DesiredVelocityVertical = -MaxVelocity_Z;

  // 2. Lazo Interno: Control de Velocidad Z (Mantiene la velocidad objetivo)
  float ErrorVelocityVertical = DesiredVelocityVertical - VelocityVerticalKalman;
  float PID_VerticalVelocity = calcularLazoPID(ErrorVelocityVertical, VelocityVerticalKalman, PrevVelZ, 
                                               K_Vel_Z, Ti_Vel_Z, Td_Vel_Z, 
                                               ItermVelZ, DtermVelZ);

  // 3. Salida final al acelerador (Feedforward de Hovering + Control PID)
  InputThrottle = HoverThrottle + PID_VerticalVelocity;

  // 4. Límite de seguridad del PWM
  if (InputThrottle > 1800) InputThrottle = 1800;
  if (InputThrottle < 1100) InputThrottle = 1100;
}