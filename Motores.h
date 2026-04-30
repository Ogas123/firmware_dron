#ifndef MOTORES_H
#define MOTORES_H

void initMotores();
void actualizarMotores(int throttleBase, float controlRoll, float controlPitch, float controlYaw);
void apagarMotores();

#endif