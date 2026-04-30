#ifndef CONTROL_H
#define CONTROL_H

// Variables que le pasaremos al Mixer
extern float PID_Roll, PID_Pitch, PID_Yaw;

// Setpoints (Lo que queremos que haga el dron, en °/s)
extern float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;

void initControl();
void calcularPID();

#endif