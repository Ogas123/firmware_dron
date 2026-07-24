#ifndef COMUNICACIONES_H
#define COMUNICACIONES_H

enum EstadoDron { APAGADO, DESPEGANDO, VOLANDO, ATERRIZANDO };

extern EstadoDron estadoActual;

void initComunicaciones();
void enviarMensajeUDP(const char* mensaje);
void recibirComandosUDP();
void tareaTelemetria(void *pvParameters);

#endif