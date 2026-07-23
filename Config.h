#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ====================================================================
// ====================================================================
// PARÁMETROS MATEMÁTICOS: LQG (LQR + KALMAN)
// ====================================================================
// ====================================================================

// --- Parámetros de Tiempo ---
constexpr float h = 0.004f; // 250 Hz (4 ms)

// ==========================================================
// 1. MATRICES DE ESTIMACIÓN (FILTRO DE KALMAN - LQE)
// ==========================================================

// Matriz de Transición de Estados (Phi) - Cinemática 2x2 para Roll, Pitch y Altura
constexpr float Phi_2x2[2][2] = {
    {1.0000f, h}, 
    {0.0000f, 1.0000f}
};

// Matrices de Entrada Estocástica (Gamma)
constexpr float Gamma_roll_pitch[2] = {0.000001f, 0.000736f}; 
constexpr float Gamma_yaw = 0.000007f;
constexpr float Gamma_alt_lqr[2] = {0.000131f, 0.065574f};
constexpr float Gamma_alt_kf[2] = {0.000008f, 0.004000f};

// Matrices de Covarianza de Ruido de Proceso (Q)
constexpr float Q_roll_pitch[2][2] = {
    {0.0100f, 0.0000f}, 
    {0.0000f, 0.0500f}
};
constexpr float Q_yaw = 0.0200f;
constexpr float Q_alt[2][2] = {
    {0.0010f, 0.0000f}, 
    {0.0000f, 0.0100f}
};

// Matrices de Covarianza de Ruido de Medición (R)
constexpr float R_roll_pitch[2][2] = {
    {0.0200f, 0.0000f}, 
    {0.0000f, 0.0010f}
};
constexpr float R_yaw = 0.0010f;
constexpr float R_alt_scalar = 0.0050f; // Ruido del sensor láser ToF VL53L1X



// ==========================================================
// 2. MATRICES DE CONTROL ÓPTIMO (LQR)
// ==========================================================

// Ganancias de realimentación (L) precalculadas en estado estacionario (Python)
// u(k) = -L * x_hat(k)
constexpr float L_roll[2]  = {4.46f, 7.11f};
constexpr float L_pitch[2] = {4.46f, 7.11f};
constexpr float L_yaw[1]   = {22.36f};
constexpr float L_alt[2]   = {1715.94f, 853.27f};

// ==========================================================
// 3. MATRICES INICIALES DE INCERTIDUMBRE (P0)
// ==========================================================
constexpr float P0_2x2[2][2] = {
    {10.0f, 0.0f}, 
    {0.0f, 10.0f}
};
constexpr float P0_1x1 = 10.0f;



// ==========================================================
// PARÁMETROS CALIBRADOS LEVENBERG-MARQUARDT
// ==========================================================
#define ALFA_YX    0.000278
#define ALFA_ZX    0.001603
#define ALFA_ZY    0.000864
#define S_X        1.005936
#define S_Y        0.997343
#define S_Z        0.991658
#define B_X        0.313151
#define B_Y        0.016393
#define B_Z        0.223452
// ==========================================================



// ==========================================
// CONFIGURACIÓN DE HARDWARE 
// ==========================================

// --- I2C Principal (IMU MPU6050, MS5611, HMC5883) ---
#define PIN_IMU_SDA 11
#define PIN_IMU_SCL 10

// --- I2C Secundario (Sensor ToF VL53L1X) ---
//NO LO USO, CONECTO EL TOF AL PRINCIPAL
#define PIN_TOF_SDA 40
#define PIN_TOF_SCL 41

// --- SPI (Flujo Óptico PMW3901) ---
#define PIN_OPT_MISO 37
#define PIN_OPT_MOSI 35
#define PIN_OPT_CLK  36
#define PIN_OPT_CS   42

// --- Alertas Acústicas ---
#define PIN_BUZZER_PLUS  39
#define PIN_BUZZER_MINUS 38

// --- ADC Batería y LEDs ---
#define PIN_BATERIA   2

#define PIN_LED_GREEN 9
#define PIN_LED_RED   8
#define PIN_LED_BLUE  7

// ====================================================================
// DISPOSICIÓN FÍSICA DE LOS MOTORES (Configuración en 'X')
// ====================================================================
// Motor 1 (M1): Frontal Derecho   / Front-Right (FR)  -> Pin 5
// Motor 2 (M2): Trasero Derecho   / Rear-Right  (RR)  -> Pin 6
// Motor 3 (M3): Trasero Izquierdo / Rear-Left   (RL)  -> Pin 3
// Motor 4 (M4): Frontal Izquierdo / Front-Left  (FL)  -> Pin 4
// ====================================================================
#define PIN_MOTOR_1 5 
#define PIN_MOTOR_2 6
#define PIN_MOTOR_3 3
#define PIN_MOTOR_4 4

// Offsets de ecualización de potencia por motor (compensación de desgaste/diferencia mecánica)
#define OFFSET_MOTOR_1 500  // Front-Right (FR) +40 PWM extra para compensar falta de fuerza
#define OFFSET_MOTOR_2   0  // Rear-Right (RR)
#define OFFSET_MOTOR_3   0  // Rear-Left (RL)
#define OFFSET_MOTOR_4 110  // Front-Left (FL)



// ==========================================
// RED Y COMUNICACIONES
// ==========================================

#define WIFI_SSID "LiteWing_Agus"
#define WIFI_PASS "12345678"
#define UDP_PORT  4210

// --- IP Estática ---
const IPAddress DRON_IP(192, 168, 4, 1);
const IPAddress DRON_GATEWAY(192, 168, 4, 1);
const IPAddress DRON_SUBNET(255, 255, 255, 0);

#endif