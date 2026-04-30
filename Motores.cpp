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

void actualizarMotores(int throttleBase, float controlRoll, float controlPitch, float controlYaw) {
  
  // SEGURIDAD 1: Si el acelerador es 0, ignoramos el PID y apagamos todo.
  if (throttleBase == 0) {
    apagarMotores();
    return; // Cortamos la ejecución de la función acá mismo
  }

  // 1. EL MIXER
  // Nota: Dependiendo de cómo conectes los motores físicamente, 
  // vas a tener que asignar el 1, 2, 3 y 4 al pin correcto (FL, FR, BL, BR)
  int pwmMotor1 = throttleBase - controlRoll - controlPitch - controlYaw;
  int pwmMotor2 = throttleBase - controlRoll + controlPitch + controlYaw;
  int pwmMotor3 = throttleBase + controlRoll + controlPitch - controlYaw;
  int pwmMotor4 = throttleBase + controlRoll - controlPitch + controlYaw;

  // SEGURIDAD 2: Saturación (Clamping) para no desbordar los 12 bits
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
  // Ajustá qué motor es cuál según el sentido de giro de tus hélices
  ledcWrite(PIN_MOTOR_FR, pwmMotor1); // Ejemplo: Motor 1 al Front-Right
  ledcWrite(PIN_MOTOR_BR, pwmMotor2); // Ejemplo: Motor 2 al Back-Right
  ledcWrite(PIN_MOTOR_BL, pwmMotor3); // Ejemplo: Motor 3 al Back-Left
  ledcWrite(PIN_MOTOR_FL, pwmMotor4); // Ejemplo: Motor 4 al Front-Left
}

void apagarMotores() {
  ledcWrite(PIN_MOTOR_FL, 0);
  ledcWrite(PIN_MOTOR_FR, 0);
  ledcWrite(PIN_MOTOR_BL, 0);
  ledcWrite(PIN_MOTOR_BR, 0);
}