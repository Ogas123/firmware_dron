#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Config.h"
#include "Comunicaciones.h"
#include "LEDs.h"


WiFiUDP udp;

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

void enviarMensajeUDP(String mensaje) {
  // Enviamos el texto a la dirección de Broadcast
  udp.beginPacket("255.255.255.255", UDP_PORT);
  udp.println(mensaje); // Usamos println para que agregue el salto de línea al final
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
    
    // Convertimos a String para facilitar la manipulación
    String comando = String(incomingPacket);
    comando.trim(); // Limpiamos espacios y saltos de línea ocultos
    comando.toUpperCase(); // Hacemos que no sea sensible a mayúsculas/minúsculas

    Serial.printf("Comando recibido: %s\n", comando.c_str());

    // Transiciones de estado permitidas
    if (comando == "DESPEGAR" && estadoActual == APAGADO) {
      estadoActual = VOLANDO;
      Serial.println("DESPEGANDO");
    } 
    else if (comando == "CORTAR") { // BOTÓN DE PÁNICO VITAL
      estadoActual = APAGADO;
      Serial.println("EMERGENCIA: MOTORES APAGADOS INSTANTÁNEAMENTE");
    }
  }
}




