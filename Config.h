#ifndef CONFIG_H
#define CONFIG_H

// --- Pines ---
#define PIN_MOTOR_FL 0
#define PIN_MOTOR_FR 1
#define PIN_MOTOR_BL 2
#define PIN_MOTOR_BR 3
#define PIN_BATERIA  6

#define PIN_LED_SYS  7 // LED Verde (System)
#define PIN_LED_LINK 8 // LED Azul (UDP)
#define PIN_LED_ERR  9 // LED Rojo (Batería)

// --- I2C Secundario (Sensor ToF VL53L1X) ---
//#define PIN_TOF_SDA 
//#define PIN_TOF_SCL  

// --- Comunicaciones ---
#define WIFI_SSID "LiteWing_Agus"
#define WIFI_PASS "12345678"
#define UDP_PORT  4210

// --- IP ESTÁTICA ---
const IPAddress DRON_IP(192, 168, 4, 1);
const IPAddress DRON_GATEWAY(192, 168, 4, 1);
const IPAddress DRON_SUBNET(255, 255, 255, 0);

// --- Seguridad ---
#define BATERIA_CRITICA 3.3

#endif