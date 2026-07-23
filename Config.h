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
    {1.0f, h}, 
    {0.0f, 1.0f}
};

// Matrices de Entrada Estocástica (Gamma)
// NOTA: Reemplazar los valores de Roll y Pitch con la salida Gamma de tu script de Python
constexpr float Gamma_roll_pitch[2] = {0.0f, 0.012f}; 
constexpr float Gamma_yaw = h;

// Matrices de Covarianza de Ruido de Proceso (Q)
constexpr float Q_roll_pitch[2][2] = {
    {0.001f, 0.0f}, 
    {0.0f, 0.01f}
};
constexpr float Q_yaw = 0.01f;
constexpr float Q_alt[2][2] = {
    {0.0001f, 0.0f}, 
    {0.0f, 0.001f}
};

// Matrices de Covarianza de Ruido de Medición (R)
constexpr float R_roll_pitch[2][2] = {
    {0.05f, 0.0f}, 
    {0.0f, 0.002f}
};
constexpr float R_yaw = 0.002f;
constexpr float R_alt_scalar = 0.005f; // Ruido del sensor láser ToF VL53L1X

// ==========================================================
// 2. MATRICES DE CONTROL ÓPTIMO (LQR)
// ==========================================================

// Ganancias de realimentación (L) precalculadas en estado estacionario (Python)
// u(k) = -L * x_hat(k)
constexpr float L_roll[2]  = {14.09f, 15.38f};
constexpr float L_pitch[2] = {14.09f, 15.38f};
constexpr float L_yaw[1]   = {3.16f};
constexpr float L_alt[2]   = {15.49f, 5.01f};

// ==========================================================
// 3. MATRICES INICIALES DE INCERTIDUMBRE (P0)
// ==========================================================
// Se inician con valores altos para que el filtro confíe en las mediciones
// durante los primeros milisegundos de arranque.
constexpr float P0_2x2[2][2] = {
    {10.0f, 0.0f}, 
    {0.0f, 10.0f}
};
constexpr float P0_1x1 = 10.0f;



// ==========================================================
// PARÁMETROS CALIBRADOS LEVENBERG-MARQUARDT
// ==========================================================
#define ALFA_YX   -0.000245
#define ALFA_ZX   -0.001074
#define ALFA_ZY    0.002110
#define S_X        1.005516
#define S_Y        0.997124
#define S_Z        0.989201
#define B_X        0.307720
#define B_Y        0.030755
#define B_Z        0.188368
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