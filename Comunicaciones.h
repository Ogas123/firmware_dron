#ifndef COMUNICACIONES_H
#define COMUNICACIONES_H

enum EstadoDron {
  APAGADO,
  VOLANDO,
};

extern EstadoDron estadoActual;

void initComunicaciones();
void enviarMensajeUDP(String mensaje);
void recibirComandosUDP();

#endif