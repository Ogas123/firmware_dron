#include <Arduino.h>
#include "Config.h"
#include "Motores.h"

// ====================================================================
// CONFIGURACIÓN DE BATERÍA (Compensación de Empuje)
// ====================================================================
float FactorCompensacionBateria = 1.0f;
float VoltajeBateriaReal = 4.2f;

// Factor = Voltaje medido con multímetro / Lectura ADC cruda
const float FACTOR_ADC_A_VOLTIOS = 4.2f / 4095.0f; 
const float VOLTAJE_NOMINAL_HOVER = 3.7f; // El voltaje al que afinaste el LQR

void actualizarBateria() {
  float v_crudo = analogRead(PIN_BATERIA) * FACTOR_ADC_A_VOLTIOS;
  
  // Filtro Pasa-Bajos EMA (Suaviza el ruido eléctrico de los motores)
  VoltajeBateriaReal = (0.95f * VoltajeBateriaReal) + (0.05f * v_crudo);
  
  // Saturación segura (Evita que el PWM se multiplique por cero o infinito)
  float v_seguro = constrain(VoltajeBateriaReal, 3.0f, 4.3f);
  FactorCompensacionBateria = VOLTAJE_NOMINAL_HOVER / v_seguro;
}

// ====================================================================
// DISPOSICIÓN FÍSICA DE LOS MOTORES (Configuración en 'X')
// ====================================================================
void initMotores() {
  // 1. Inicializar Hardware de Motores (1 KHz, 12 bits)
  // Utilizamos la API LEDC con el periférico PWM por hardware
  ledcAttach(PIN_MOTOR_1, 1000, 12);
  ledcAttach(PIN_MOTOR_2, 1000, 12);
  ledcAttach(PIN_MOTOR_3, 1000, 12);
  ledcAttach(PIN_MOTOR_4, 1000, 12);
  
  apagarMotores(); // Seguridad al inicio

  // 2. Inicializar Hardware de Batería
  pinMode(PIN_BATERIA, INPUT);
  VoltajeBateriaReal = analogRead(PIN_BATERIA) * FACTOR_ADC_A_VOLTIOS; // Lectura inicial
}

void actualizarMotores(bool armado, int throttleBase, float controlRoll, float controlPitch, float controlYaw) {
  
  if (!armado) {
    apagarMotores();
    return; 
  }

  // 1. EL MIXER (Cálculo matemático en float)
  float m1_float = throttleBase + controlRoll - controlPitch - controlYaw + OFFSET_MOTOR_1;
  float m2_float = throttleBase + controlRoll + controlPitch + controlYaw + OFFSET_MOTOR_2;
  float m3_float = throttleBase - controlRoll + controlPitch - controlYaw + OFFSET_MOTOR_3;
  float m4_float = throttleBase - controlRoll - controlPitch + controlYaw + OFFSET_MOTOR_4;

  // 2. COMPENSACIÓN POR BATERÍA (Feedforward dinámico)
  m1_float *= FactorCompensacionBateria;
  m2_float *= FactorCompensacionBateria;
  m3_float *= FactorCompensacionBateria;
  m4_float *= FactorCompensacionBateria;

  // Casteamos el resultado a enteros
  int pwmMotor1 = (int)m1_float;
  int pwmMotor2 = (int)m2_float;
  int pwmMotor3 = (int)m3_float;
  int pwmMotor4 = (int)m4_float;

  // 3. SATURACIÓN (0 a 4095)
  if(pwmMotor1 > 4095) pwmMotor1 = 4095; if(pwmMotor1 < 0) pwmMotor1 = 0;
  if(pwmMotor2 > 4095) pwmMotor2 = 4095; if(pwmMotor2 < 0) pwmMotor2 = 0;
  if(pwmMotor3 > 4095) pwmMotor3 = 4095; if(pwmMotor3 < 0) pwmMotor3 = 0;
  if(pwmMotor4 > 4095) pwmMotor4 = 4095; if(pwmMotor4 < 0) pwmMotor4 = 0;

  // 4. ESCRITURA EN HARDWARE
  ledcWrite(PIN_MOTOR_1, pwmMotor1); 
  ledcWrite(PIN_MOTOR_2, pwmMotor2); 
  ledcWrite(PIN_MOTOR_3, pwmMotor3); 
  ledcWrite(PIN_MOTOR_4, pwmMotor4); 
}

void apagarMotores() {
  ledcWrite(PIN_MOTOR_1, 0);
  ledcWrite(PIN_MOTOR_2, 0);
  ledcWrite(PIN_MOTOR_3, 0);
  ledcWrite(PIN_MOTOR_4, 0);
}