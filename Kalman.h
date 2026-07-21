#ifndef KALMAN_H
#define KALMAN_H

// ====================================================================
// VARIABLES DE ESTADO ESTIMADO (\hat{x})
// Expuestas globalmente para que el LQR (y la telemetría) puedan leerlas.
// Formato: [Posición/Ángulo, Velocidad/Rate]
// ====================================================================
extern float x_hat_roll[2];
extern float x_hat_pitch[2];
extern float x_hat_alt[2];

// Yaw es de orden 1 (solo Rate)
extern float x_hat_yaw[1];

// ====================================================================
// FUNCIONES PÚBLICAS DEL OBSERVADOR
// ====================================================================

/**
 * @brief Inicializa las matrices de covarianza P y los estados estimados en 0.
 * Se debe llamar una sola vez en el setup() del ESP32.
 */
void initSensorFusion();

/**
 * @brief Ejecuta la predicción y corrección del Filtro de Kalman Dinámico para los 4 canales.
 * Se debe llamar en el loop() a 250 Hz (cada 4ms).
 * 
 * @param u_roll   Esfuerzo de control actual en Roll (delta PWM)
 * @param u_pitch  Esfuerzo de control actual en Pitch (delta PWM)
 * @param u_yaw    Esfuerzo de control actual en Yaw (delta PWM)
 * @param u_alt    Esfuerzo de control actual en Altitud (delta PWM)
 * @param y_roll   Vector [2] con mediciones crudas: [Ángulo Accel, Rate Gyro]
 * @param y_pitch  Vector [2] con mediciones crudas: [Ángulo Accel, Rate Gyro]
 * @param y_yaw    Medición cruda escalar: Rate Gyro Z
 * @param y_alt    Vector [2] con mediciones crudas: [Distancia ToF, Accel Z (sin g)]
 */
void actualizarFiltrosLQG(float u_roll, float u_pitch, float u_yaw, float u_alt,
                          float y_roll[2], float y_pitch[2], float y_yaw, float y_alt[2]);

#endif // KALMAN_H