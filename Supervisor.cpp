#include "Supervisor.h"
#include "Config.h"
#include "LQR.h"
#include "Motores.h"
#include "Comunicaciones.h"

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
      if (baseThrottleDinamico < 1750.0f) {
        baseThrottleDinamico += 2.0f;   
        DesiredAltitude = x_hat_alt[0]; 
      } else {
        DesiredAltitude += TasaAscenso;
        if (DesiredAltitude >= AlturaObjetivoFinal) {
          DesiredAltitude = AlturaObjetivoFinal;
          estadoActual = VOLANDO; 
          Serial.println("INFO: Meta de altura alcanzada. Transición a VOLANDO.");
        }
      }
      calcularControl();
      actualizarMotores(true, (int)baseThrottleDinamico + (int)u_alt, u_roll, u_pitch, u_yaw);
      break;

    case VOLANDO:
      DesiredAltitude = AlturaObjetivoFinal; 
      calcularControl();
      actualizarMotores(true, 1750 + (int)u_alt, u_roll, u_pitch, u_yaw);
      break;

    case ATERRIZANDO:
      DesiredAltitude -= TasaAscenso; 
      if (DesiredAltitude <= 0.05f || x_hat_alt[0] <= 0.03f) {
        estadoActual = APAGADO; 
        Serial.println("INFO: Touchdown detectado. Transición a APAGADO.");
      } else {
        calcularControl();
        actualizarMotores(true, 1750 + (int)u_alt, u_roll, u_pitch, u_yaw);
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