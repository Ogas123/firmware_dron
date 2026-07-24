#ifndef COMUNICACIONES_H
#define COMUNICACIONES_H

enum EstadoDron { APAGADO, DESPEGANDO, VOLANDO, ATERRIZANDO };

extern EstadoDron estadoActual;

void initComunicaciones();
void enviarMensajeUDP(const char* mensaje);
void recibirComandosUDP();

// Nota: usamos uint8_t* para enviar bytes puros y size_t para el tamaño
void enviarTelemetriaBinaria(const uint8_t* datos, size_t longitud);
void tareaTelemetria(void *pvParameters);

// Le decimos al compilador que empaquete esto byte por byte sin espacios vacíos
struct __attribute__((packed)) TelemetriaDron {
  uint32_t timestamp; // Útil para saber si se perdieron paquetes (micros o millis)
  
  // Crudos (3 floats = 12 bytes)
  float accX; float accY; float accZ;
  
  // Roll (4 floats = 16 bytes)
  float rollAcc; float rollGyr; float rollKalman; float rollRateKalman;
  
  // Pitch (4 floats = 16 bytes)
  float pitchAcc; float pitchGyr; float pitchKalman; float pitchRateKalman;
  
  // Yaw (2 floats = 8 bytes)
  float yawRateGyr; float yawRateKalman;
  
  // Altura (3 floats = 12 bytes)
  float altToF; float altKalman; float vzKalman;
}; // TOTAL: 68 bytes

#endif


//[Byte Offset]
//      0       1       2       3
//    +-------+-------+-------+-------+  <-- Inicio del paquete (Byte 0)
// 00 |           timestamp           |  (uint32_t)  [Tiempo]
//    +-------+-------+-------+-------+
// 04 |             accX              |  (float)     [IMU Cruda]
//    +-------+-------+-------+-------+
// 08 |             accY              |  (float)
//    +-------+-------+-------+-------+
// 12 |             accZ              |  (float)
//    +-------+-------+-------+-------+
// 16 |           rollAcc             |  (float)     [Canal Roll]
//    +-------+-------+-------+-------+
// 20 |           rollGyr             |  (float)
//    +-------+-------+-------+-------+
// 24 |         rollKalman            |  (float)
//    +-------+-------+-------+-------+
// 28 |       rollRateKalman          |  (float)
//    +-------+-------+-------+-------+
// 32 |          pitchAcc             |  (float)     [Canal Pitch]
//    +-------+-------+-------+-------+
// 36 |          pitchGyr             |  (float)
//    +-------+-------+-------+-------+
// 40 |        pitchKalman            |  (float)
//    +-------+-------+-------+-------+
// 44 |      pitchRateKalman          |  (float)
//    +-------+-------+-------+-------+
// 48 |         yawRateGyr            |  (float)     [Canal Yaw]
//    +-------+-------+-------+-------+
// 52 |       yawRateKalman           |  (float)
//    +-------+-------+-------+-------+
// 56 |           altToF              |  (float)     [Canal Altura]
//    +-------+-------+-------+-------+
// 60 |         altKalman             |  (float)
//    +-------+-------+-------+-------+
// 64 |          vzKalman             |  (float)
//    +-------+-------+-------+-------+  <-- Fin del paquete (Byte 68)
//
//    TAMAÑO TOTAL: 68 Bytes (17 variables x 4 bytes c/u)


