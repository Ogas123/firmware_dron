#include "Supervisor.h"
#include "Config.h"
#include "LQR.h"
#include "Motores.h"
#include "Telemetria.h"

// ====================================================================
// CONFIGURACIÓN DE VUELO
// ====================================================================
// Variable global fácil de modificar para ajustar el peso del dron
int THROTTLE_HOVER = 1500; 

// Definición de las variables del supervisor
float AlturaObjetivoFinal = 0.5f;     
float TasaAscenso = 0.001f;           
float baseThrottleDinamico = 0.0f;    

// Importamos los estados y variables calculadas en otros módulos
extern float u_roll, u_pitch, u_yaw, u_alt;
extern float x_hat_alt[2];
extern float DesiredAltitude;
extern EstadoDron estadoActual; 

void ejecutarSupervisorVuelo() {
  switch (estadoActual) {
      
    case DESPEGANDO:
      calcularControl();
      
      if (baseThrottleDinamico < (float)THROTTLE_HOVER) {
        // 1. Rampa rápida de despegue (+8.0 PWM por ciclo) para desapegarse del suelo limpiamente
        baseThrottleDinamico += 8.0f;   
        DesiredAltitude = x_hat_alt[0]; 
        u_alt = 0.0f;                    
        
        // Mientras el empuje sea menor al 70% de hover (apoyado en suelo):
        // Se desactiva u_yaw para evitar que gire sobre sus patitas antes de despegar
        if (baseThrottleDinamico < (0.70f * THROTTLE_HOVER)) {
          u_yaw = 0.0f;
        }
      } else {
        // 2. Ya en el aire: habilitar control de altura y transición a VOLANDO
        if ((DesiredAltitude - x_hat_alt[0]) < 0.05f) {
          DesiredAltitude += TasaAscenso;
        }
        
        if (DesiredAltitude >= AlturaObjetivoFinal) {
          DesiredAltitude = AlturaObjetivoFinal;
          estadoActual = VOLANDO; 
          Serial.println("INFO: Meta de altura alcanzada. Transición a VOLANDO.");
        }
      }
      actualizarMotores(true, (int)baseThrottleDinamico + (int)u_alt, u_roll, u_pitch, u_yaw);
      break;

    case VOLANDO:
      DesiredAltitude = AlturaObjetivoFinal; 
      calcularControl();
      actualizarMotores(true, THROTTLE_HOVER + (int)u_alt, u_roll, u_pitch, u_yaw);
      break;

    case ATERRIZANDO:
      DesiredAltitude -= TasaAscenso; 
      if (DesiredAltitude <= 0.05f || x_hat_alt[0] <= 0.03f) {
        estadoActual = APAGADO; 
        Serial.println("INFO: Touchdown detectado. Transición a APAGADO.");
      } else {
        calcularControl();
        actualizarMotores(true, THROTTLE_HOVER + (int)u_alt, u_roll, u_pitch, u_yaw);
      }
      break;

    case APAGADO:
    default:
      u_roll = 0.0f; 
      u_pitch = 0.0f; 
      u_yaw = 0.0f; 
      u_alt = 0.0f;
      baseThrottleDinamico = 0.0f;
      DesiredAltitude = 0.0f; 
      actualizarMotores(false, 0, 0, 0, 0);
      break;
  }
}