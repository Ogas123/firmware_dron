#include <Arduino.h>
#include "Config.h"
#include "IMU.h"
#include "Kalman.h"
#include "LQR.h"
#include "Motores.h"

// ==========================================================
// VARIABLES GLOBALES Y SEMÁFOROS
// ==========================================================
hw_timer_t * controlTimer = NULL;
SemaphoreHandle_t timerSemaphore;

// Importamos las variables crudas de la IMU (Asumiendo que son globales en IMU.cpp)
extern float AccX, AccY, AccZ;
extern float AngleRoll_Acc, AnglePitch_Acc;
extern float RateRoll, RatePitch, RateYaw;

// Importamos la lectura del ToF (Global en tu código)
extern float distanciaAlturaMM; 

// Prototipo de la tarea de telemetría
void tareaTelemetria(void *pvParameters);

// ==========================================================
// RUTINA DE INTERRUPCIÓN (ISR) - HARDWARE TIMER (250 Hz)
// ==========================================================
void IRAM_ATTR onTimer() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(timerSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR(); 
  }
}

// ==========================================================
// SETUP
// ==========================================================
void setup() {
  // Aumentamos los baudios para no saturar el buffer con tanta data
  Serial.begin(500000); 

  // 1. Inicialización de todos los módulos
  initIMU();
  initControl();
  initMotores();

  // 2. Crear Tarea de Telemetría anclada al Core 0
  xTaskCreatePinnedToCore(
    tareaTelemetria,   // Función que ejecuta la tarea
    "TaskTelemetria",  // Nombre (para debugging interno de FreeRTOS)
    4096,              // Tamaño del Stack en bytes
    NULL,              // Parámetros que le pasamos a la función
    1,                 // Prioridad (1 es baja, el control del dron manda)
    NULL,              // Handle de la tarea
    0                  // ¡Núcleo 0! (El Core 1 queda libre para el loop)
  );

  // 3. Crear el semáforo y configurar el Timer de Hardware (Core 3.x)
  timerSemaphore = xSemaphoreCreateBinary();
  
  // Seteamos la frecuencia del timer a 1 MHz (1 tick = 1 microsegundo)
  controlTimer = timerBegin(1000000); 
  
  // Atamos la interrupción (sin el tercer parámetro de edge)
  timerAttachInterrupt(controlTimer, &onTimer);
  
  // Nueva API timerAlarm: (timer, valor de alarma, auto-recarga, conteo de recarga)
  // Alarma a los 4000 us (250 Hz), autoreload true, reload count 0
  timerAlarm(controlTimer, 4000, true, 0); 
  
  Serial.println("SISTEMA ARMADO. LQG EN CORE 1 (250Hz) / TELEMETRÍA EN CORE 0.");
  
}

// ==========================================================
// LOOP PRINCIPAL (CORE 1) - CONTROL LQG ESTRICTO
// ==========================================================
void loop() {
  // Se bloquea aquí sin gastar CPU hasta que el Timer le da luz verde (cada 4 ms exactos)
  if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {
    
    // 1. Leer Sensores
    leerIMU();
    float y_roll[2]  = {AngleRoll_Acc, RateRoll};
    float y_pitch[2] = {AnglePitch_Acc, RatePitch};
    float y_yaw      = RateYaw;
    float y_alt[2]   = {distanciaAlturaMM, AccZ};

    // 2. Filtro de Kalman Dinámico
    actualizarFiltrosLQG(u_roll, u_pitch, u_yaw, u_alt, 
                         y_roll, y_pitch, y_yaw, y_alt);

    // 3. Ley de Control (LQR)
    calcularControl();

    // 4. Actuación (Mezclador a MOSFETs)
    actualizarMotores(true, 1500, u_roll, u_pitch, u_yaw);
  }
}

// ==========================================================
// TAREA DE TELEMETRÍA (CORE 0) - ASÍNCRONA
// ==========================================================
void tareaTelemetria(void *pvParameters) {
  // Bucle infinito propio de la tarea de FreeRTOS
  for(;;) {
    // Aceleraciones crudas
    Serial.print("AccX:"); Serial.print(AccX); Serial.print(",");
    Serial.print("AccY:"); Serial.print(AccY); Serial.print(",");
    Serial.print("AccZ:"); Serial.print(AccZ); Serial.print(",");

    // Actitud ROLL (Acelerómetro vs Giroscopio vs Estimación Óptima)
    Serial.print("Roll_acc:"); Serial.print(AngleRoll_Acc); Serial.print(",");
    Serial.print("Roll_gyr:"); Serial.print(RateRoll); Serial.print(",");
    Serial.print("Roll_Kalman:"); Serial.print(x_hat_roll[0]); Serial.print(",");
    
    // Actitud PITCH
    Serial.print("Pitch_acc:"); Serial.print(AnglePitch_Acc); Serial.print(",");
    Serial.print("Pitch_gyr:"); Serial.print(RatePitch); Serial.print(",");
    Serial.print("Pitch_Kalman:"); Serial.print(x_hat_pitch[0]); Serial.print(","); 

    // Altitud
    Serial.print("Alt_ToF_Raw:"); Serial.print(distanciaAlturaMM);

    // Importante: El salto de línea final para el Serial Plotter
    Serial.println(); 

    // Relajamos la tarea para no saturar el bus UART. 
    // 20 ms = 50 Hz de tasa de refresco para el plotter (Súper fluido para el ojo humano)
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}