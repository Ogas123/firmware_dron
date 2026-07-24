#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Config.h"
#include "Comunicaciones.h"
#include "Motores.h"

WiFiUDP udp;

// Instanciamos la variable de estado inicializándola en APAGADO
EstadoDron estadoActual = APAGADO;

// ==========================================================
// VARIABLES GLOBALES
// ==========================================================
// Importamos las variables crudas de la IMU
extern float AccX, AccY, AccZ;
extern float AngleRoll_Acc, AnglePitch_Acc;
extern float RateRoll, RatePitch, RateYaw;

// Importamos la lectura del ToF 
extern float dist_tof_m; 

extern float x_hat_roll[2];
extern float x_hat_pitch[2];
extern float x_hat_yaw[1];
extern float x_hat_alt[2];


void initComunicaciones() {
  // 1. Configurar ESP32 como Access Point
  Serial.println("\nIniciando red Wi-Fi SoftAP...");
  WiFi.mode(WIFI_AP);

  // Obligamos al ESP32 a usar la IP, Gateway y Subnet que definimos en Config.h
  WiFi.softAPConfig(DRON_IP, DRON_GATEWAY, DRON_SUBNET);

  // Levantamos la red
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.println("Wi-Fi listo. IP del Dron: " + IP.toString());

  // 2. Iniciar el servidor UDP
  udp.begin(UDP_PORT);
  Serial.println("Escuchando por UDP en el puerto " + String(UDP_PORT));
}

void enviarMensajeUDP(const char* mensaje) {
  // Enviamos el texto a la dirección de Broadcast
  udp.beginPacket("255.255.255.255", UDP_PORT);
  udp.println(mensaje); // Manda el array de caracteres directo y agrega el salto de línea
  udp.endPacket();
}

void recibirComandosUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = '\0';
    }
    
    String comando = String(incomingPacket);
    comando.trim(); 
    comando.toUpperCase(); 

    Serial.printf("Comando UDP recibido: %s\n", comando.c_str());

    // ---------------------------------------------------------
    // LÓGICA DE TRANSICIONES POR WI-FI
    // ---------------------------------------------------------
    if (comando == "1" && estadoActual == APAGADO) {
      estadoActual = DESPEGANDO; // Dispara la rampa de despegue autónomo
      Serial.println("OK: Iniciando secuencia de DESPEGUE...");
    } 
    else if (comando == "2" && (estadoActual == VOLANDO || estadoActual == DESPEGANDO)) {
      estadoActual = ATERRIZANDO; // Dispara la rampa de descenso autónomo
      Serial.println("OK: Iniciando secuencia de ATERRIZAJE...");
    }
    else if (comando == "0") { 
      estadoActual = APAGADO; // BOTÓN DE PÁNICO VITAL
      actualizarMotores(false, 0, 0, 0, 0); // <-- CORTA EL HARDWARE AQUÍ MISMO
      Serial.println("EMERGENCIA: MOTORES CORTADOS INSTANTÁNEAMENTE");
    }
  }
}


// ==========================================================
// TAREA DE TELEMETRÍA (CORE 0) - ASÍNCRONA
// ==========================================================
void tareaTelemetria(void *pvParameters) {
  // Creamos un buffer en memoria estática lo suficientemente grande para alojar todo el texto de la telemetría (256 o 512 bytes).
  char buffer_telemetria[512];

  for(;;) {
    // 1. Revisar si llegó un comando de armado/desarmado (UDP RX)
    recibirComandosUDP();

    // Aceleraciones crudas
    //Serial.print("AccX:"); Serial.print(AccX); Serial.print(",");
    //Serial.print("AccY:"); Serial.print(AccY); Serial.print(",");
    //Serial.print("AccZ:"); Serial.print(AccZ); Serial.print(",");

    // Actitud ROLL (Acelerómetro vs Giroscopio vs Estimación Óptima)
    //Serial.print("Roll_acc:"); Serial.print(AngleRoll_Acc); Serial.print(",");
    //Serial.print("Roll_gyr:"); Serial.print(RateRoll); Serial.print(",");
    //Serial.print("Roll_Kalman:"); Serial.print(x_hat_roll[0]); Serial.print(",");

    // Actitud PITCH
    //Serial.print("Pitch_acc:"); Serial.print(AnglePitch_Acc); Serial.print(",");
    //Serial.print("Pitch_gyr:"); Serial.print(RatePitch); Serial.print(",");
    //Serial.print("Pitch_Kalman:"); Serial.print(x_hat_pitch[0]); Serial.print(",");

    // Actitud YAW
    //Serial.print("Yaw_gyr:"); Serial.print(RateYaw); Serial.print(",");
    //Serial.print("Yaw_Kalman:"); Serial.print(x_hat_yaw[0]); Serial.print(",");

    // Altitud
    //Serial.print("Alt_ToF_Raw:"); Serial.print(dist_tof_m); Serial.print(",");
    //Serial.print("Alt_Kalman:"); Serial.print(x_hat_alt[0]);

    //Serial.println();


    // Empaquetamos agrupando por Canal y sumando el Giroscopio en Roll y Pitch
    snprintf(buffer_telemetria, sizeof(buffer_telemetria),
             
             // --- 0. Sensores Crudos (Para calibrar LM) ---
             "AccX:%.4f,AccY:%.4f,AccZ:%.4f,"
             
             // --- 1. Canal Roll ---
             "Roll_Acc:%.2f,Roll_Gyr:%.2f,Roll_Kalman:%.2f,RollRate_Kalman:%.2f,"
             
             // --- 2. Canal Pitch ---
             "Pitch_Acc:%.2f,Pitch_Gyr:%.2f,Pitch_Kalman:%.2f,PitchRate_Kalman:%.2f,"
             
             // --- 3. Canal Yaw ---
             "YawRate_Gyr:%.2f,YawRate_Kalman:%.2f,"
             
             // --- 4. Canal Altura ---
             "Alt_ToF:%.3f,Alt_Kalman:%.3f,Vz_Kalman:%.3f",
             
             // ==========================================
             // VARIABLES MAPEADAS AL TEXTO SUPERIOR
             // ==========================================
             
             // 0. Crudos
             AccX, AccY, AccZ,
             
             // 1. Roll: Acelerómetro vs Giroscopio vs Posición (0) vs Velocidad (1)
             AngleRoll_Acc, RateRoll, x_hat_roll[0], x_hat_roll[1],
             
             // 2. Pitch: Acelerómetro vs Giroscopio vs Posición (0) vs Velocidad (1)
             AnglePitch_Acc, RatePitch, x_hat_pitch[0], x_hat_pitch[1],
             
             // 3. Yaw: Giroscopio vs Velocidad (0)
             RateYaw, x_hat_yaw[0],
             
             // 4. Altura: Medición ToF vs Posición (0) vs Velocidad (1)
             dist_tof_m, x_hat_alt[0], x_hat_alt[1]
             );

    // 3. Enviar el paquete completo por UDP Broadcast(Convertimos el char array a String)
    enviarMensajeUDP(buffer_telemetria);
    //enviarMensajeUDP(String(AccX) + "," + String(AccY) + "," + String(AccZ));
    
    // 4. Relajamos la tarea para no saturar el Wi-Fi ni el procesador.
    // 20 ms = 50 Hz de tasa de refresco.
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}
