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

// --------------------------------------------------------------------
// CONSTANTES EN PRECISIÓN SIMPLE (FPU del ESP32-S3)
// --------------------------------------------------------------------
// El Xtensa LX7 tiene FPU de 32 bits por hardware; toda operacion en double
// se emula por software. Las macros DEG_TO_RAD / RAD_TO_DEG de Arduino son
// literales double, asi que multiplicar por ellas promueve la expresion a
// double y dispara la emulacion. Estas versiones 'f' mantienen el lazo de
// 4 ms integramente en la FPU.
// --------------------------------------------------------------------
constexpr float DEG2RAD_F  = 0.01745329252f;
constexpr float RAD2DEG_F  = 57.29577951f;
constexpr float GRAVEDAD_F = 9.80665f;      // [m/s^2]

// ==========================================================
// 1. MATRICES DE ESTIMACIÓN (FILTRO DE KALMAN - LQE)
// ==========================================================

// Matriz de Transición de Estados (Phi) - Cinemática 2x2 para Roll, Pitch y Altura
constexpr float Phi_2x2[2][2] = {
    {1.0000f, h}, 
    {0.0000f, 1.0000f}
};

// --------------------------------------------------------------------
// CONVENIO DE UNIDADES (coherente entre el cuaderno y el firmware)
// --------------------------------------------------------------------
// Actitud  : angulo [grados], velocidad angular [grados/s], u [cuentas PWM]
// Guiñada  : velocidad angular [grados/s], u [cuentas PWM]
// Altura   : posicion [m], velocidad [m/s], u [cuentas PWM]
//
// Los modelos continuos se derivan de Newton-Euler en rad/s^2 y se convierten
// a grados/s^2 ANTES de discretizar por ZOH, de modo que Phi, Gamma y L viven
// todos en el mismo sistema de unidades que los estados del filtro:
//   b_actitud = (K_tau / I_xx) * 180/pi = 12.145638 grados/s^2 por cuenta PWM
//   b_guiñada = (K_kappa / I_zz) * 180/pi = 0.132781 grados/s^2 por cuenta PWM
//   b_altura  = K_thrust / m              = 0.005448 m/s^2 por cuenta PWM
// --------------------------------------------------------------------

// Matrices de Entrada Estocástica (Gamma) con Alta Precisión Notación Científica
constexpr float Gamma_roll_pitch[2] = {9.716510e-05f, 4.858255e-02f}; // [grados, grados/s] por cuenta PWM
constexpr float Gamma_yaw           = 5.311252e-04f;                  // [grados/s] por cuenta PWM
constexpr float Gamma_alt_lqr[2]    = {4.358511e-08f, 2.179256e-05f}; // [m, m/s] por cuenta PWM
constexpr float Gamma_alt_kf[2]     = {8.000000e-06f, 4.000000e-03f}; // [m, m/s] por (m/s^2) de AccZ

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
// 2. GANANCIAS DE CONTROL ÓPTIMO (LQR PURO DE ESTADO, SIN ACCIÓN INTEGRAL)
// ==========================================================

// Ganancias de realimentación (L) precalculadas en estado estacionario (LQR Óptimo)
// u(k) = -L0 * (pos - pos_ref) - L1 * vel
// Todas son solucion exacta de la DARE sobre el modelo en unidades de firmware:
//   Roll/Pitch : Q = diag(30.3683, 76.0562), R = 1       -> [4.4600, 7.1100]
//   Yaw        : Q = 20.0464,                R = 1       -> 4.4720
//   Altura     : Q = diag(300, 10),          R = 1e-4    -> [1715.9370, 853.2671]
constexpr float L_roll[2]  = {4.4600f, 7.1100f}; // L0 (Ángulo) y L1 (Velocidad Angular)
constexpr float L_pitch[2] = {4.4600f, 7.1100f}; // Idéntico por simetría estructural
constexpr float L_yaw[1]   = {4.4720f};          // Tasa de Guiñada r
constexpr float L_alt[2]   = {1715.9370f, 853.2671f};

// Autoridad máxima del canal de altura, en cuentas PWM. Debe superar el empuje
// extra que aporta el efecto suelo (~318 PWM medidos apoyado) o el dron no
// puede completar el descenso.
constexpr float U_ALT_MAX  = 450.0f;

// Constante de tiempo del modo lento del canal de altura.
// El LQR de altura es proporcional puro, sin acción integral, de modo que en
// régimen la velocidad vertical vale  Vz = (adelanto de la referencia)/TAU_ALT.
// De ahí salen tanto el límite de seguimiento del ascenso como el del descenso.
constexpr float TAU_ALT    = L_alt[1] / L_alt[0]; // 0.4973 s

// ==========================================================
// TRIMS DE ACTITUD PARA ELIMINAR DERIVA LATERAL Y LONGITUDINAL
// ==========================================================
// TRIM_ROLL = -1.5f compensa la deriva a la derecha desplazando el setpoint a la izquierda
constexpr float TRIM_ROLL  = -1.5f; // [grados]
constexpr float TRIM_PITCH = 0.0f;  // [grados]

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
constexpr float ALFA_YX = 0.000278f;
constexpr float ALFA_ZX = 0.001603f;
constexpr float ALFA_ZY = 0.000864f;
constexpr float S_X     = 1.005936f;
constexpr float S_Y     = 0.997343f;
constexpr float S_Z     = 0.991658f;
constexpr float B_X     = 0.313151f;
constexpr float B_Y     = 0.016393f;
constexpr float B_Z     = 0.223452f;
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

// --- SPI (cabecera libre en la PCB, sin sensor asignado) ---
// El diseño final no incorpora sensor de flujo óptico: no hay estados de
// velocidad ni de posición horizontal (ver §1.1 del cuaderno). Los pines
// quedan documentados porque la PCB los rutea, pero el firmware no los usa.
#define PIN_SPI_MISO 37
#define PIN_SPI_MOSI 35
#define PIN_SPI_CLK  36
#define PIN_SPI_CS   42

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