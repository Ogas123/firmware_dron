#include <Arduino.h>
#include "Config.h"
#include "Motores.h"

void initMotores() {
  // Utilizamos la API LEDC con el periférico PWM por hardware
  // Frecuencia: 5000 Hz, Resolución: 12 bits (0 a 4095)
  ledcAttach(PIN_MOTOR_FL, 5000, 12);
  ledcAttach(PIN_MOTOR_FR, 5000, 12);
  ledcAttach(PIN_MOTOR_BL, 5000, 12);
  ledcAttach(PIN_MOTOR_BR, 5000, 12);
  
  apagarMotores(); // Seguridad al inicio
}

// Agregamos el parámetro 'armado'
void actualizarMotores(bool armado, int throttleBase, float controlRoll, float controlPitch, float controlYaw) {
  
  // SEGURIDAD 1: Si no está armado, cortamos todo. Ignoramos el PID.
  if (!armado) {
    apagarMotores();
    return; 
  }

  // 1. EL MIXER
  int pwmMotor1 = throttleBase - controlRoll - controlPitch - controlYaw;
  int pwmMotor2 = throttleBase - controlRoll + controlPitch + controlYaw;
  int pwmMotor3 = throttleBase + controlRoll + controlPitch - controlYaw;
  int pwmMotor4 = throttleBase + controlRoll - controlPitch + controlYaw;

  // SEGURIDAD 2: Saturación (Clamping) para no desbordar los 12 bits (0 a 4095)
  // Motor 1
  if(pwmMotor1 > 4095) pwmMotor1 = 4095;
  if(pwmMotor1 < 0) pwmMotor1 = 0;
  
  // Motor 2
  if(pwmMotor2 > 4095) pwmMotor2 = 4095;
  if(pwmMotor2 < 0) pwmMotor2 = 0;
  
  // Motor 3
  if(pwmMotor3 > 4095) pwmMotor3 = 4095;
  if(pwmMotor3 < 0) pwmMotor3 = 0;
  
  // Motor 4
  if(pwmMotor4 > 4095) pwmMotor4 = 4095;
  if(pwmMotor4 < 0) pwmMotor4 = 0;

  // 3. Escribimos la potencia en los transistores
  ledcWrite(PIN_MOTOR_FR, pwmMotor1); 
  ledcWrite(PIN_MOTOR_BR, pwmMotor2); 
  ledcWrite(PIN_MOTOR_BL, pwmMotor3); 
  ledcWrite(PIN_MOTOR_FL, pwmMotor4); 
}

void apagarMotores() {
  ledcWrite(PIN_MOTOR_FL, 0);
  ledcWrite(PIN_MOTOR_FR, 0);
  ledcWrite(PIN_MOTOR_BL, 0);
  ledcWrite(PIN_MOTOR_BR, 0);
}