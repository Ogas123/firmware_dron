#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

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

// --- ADC Batería y LEDs ---
#define PIN_BATERIA   2

#define PIN_LED_GREEN 9
#define PIN_LED_RED   8
#define PIN_LED_BLUE  7

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

// ==========================================
// PARÁMETROS DE VUELO Y SEGURIDAD
// ==========================================
#define BATERIA_CRITICA 3.3

#endif