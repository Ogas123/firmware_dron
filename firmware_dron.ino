#include <Arduino.h>
#include "Config.h"
#include "IMU.h"
#include "ToF.h"
#include "Kalman.h"
#include "LQR.h"
#include "Motores.h"
#include "Comunicaciones.h"

// ==========================================================
// VARIABLES GLOBALES
// ==========================================================
hw_timer_t * controlTimer = NULL;
SemaphoreHandle_t timerSemaphore;

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
  initToF();
  initControl();
  initMotores();
  initComunicaciones();

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

  // 3. Crear el semáforo y configurar el Timer de Hardware
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
    leerToF();

    float y_roll[2]  = {AngleRoll_Acc, RateRoll};
    float y_pitch[2] = {AnglePitch_Acc, RatePitch};
    float y_yaw      = RateYaw;
    
    // 2. Filtro de Kalman Dinámico
    actualizarFiltrosLQG(u_roll, u_pitch, u_yaw, u_alt, 
                         y_roll, y_pitch, y_yaw, 
                         dist_tof_m, AccZ);

    // 3. MÁQUINA DE ESTADOS
    if (estadoActual == VOLANDO) {
      // Dron Armado: Calculamos la ley de control óptimo
      calcularControl();
      
      // Enviamos señales PWM a los ESCs
      actualizarMotores(true, 0, u_roll, u_pitch, u_yaw); 
    } 
    else {
      // Dron Apagado o en Pánico: Forzamos control a cero
      u_roll = 0.0f;
      u_pitch = 0.0f;
      u_yaw = 0.0f;
      u_alt = 0.0f;
      
      // Apagado seguro de los motores
      actualizarMotores(false, 0, 0, 0, 0);
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
    Serial.print("Roll_acc:"); Serial.print(AngleRoll_Acc); Serial.print(",");
    Serial.print("Roll_gyr:"); Serial.print(RateRoll); Serial.print(",");
    Serial.print("Roll_Kalman:"); Serial.print(x_hat_roll[0]); Serial.print(",");

    // Actitud PITCH
    Serial.print("Pitch_acc:"); Serial.print(AnglePitch_Acc); Serial.print(",");
    Serial.print("Pitch_gyr:"); Serial.print(RatePitch); Serial.print(",");
    Serial.print("Pitch_Kalman:"); Serial.print(x_hat_pitch[0]); Serial.print(",");

    // Actitud YAW
    //Serial.print("Yaw_gyr:"); Serial.print(RateYaw); Serial.print(",");
    //Serial.print("Yaw_Kalman:"); Serial.print(x_hat_yaw[0]); Serial.print(",");

    // Altitud
    Serial.print("Alt_ToF_Raw:"); Serial.print(dist_tof_m); Serial.print(",");
    Serial.print("Alt_Kalman:"); Serial.print(x_hat_alt[0]);

    Serial.println();


    // 2. Empaquetar todas las variables en un solo string de texto.
    // Usamos "%.2f" para redondear a 2 decimales y no enviar bytes innecesarios.
    // Para la altura (ToF) usamos "%.3f" para tener precisión milimétrica.
    snprintf(buffer_telemetria, sizeof(buffer_telemetria),
             "AccX:%.2f,AccY:%.2f,AccZ:%.2f,"
             "Roll_acc:%.2f,Roll_gyr:%.2f,Roll_Kalman:%.2f,"
             "Pitch_acc:%.2f,Pitch_gyr:%.2f,Pitch_Kalman:%.2f,"
             "Yaw_gyr:%.2f,Yaw_Kalman:%.2f,"
             "Alt_ToF_Raw:%.3f,Alt_Kalman:%.3f",
             AccX, AccY, AccZ,
             AngleRoll_Acc, RateRoll, x_hat_roll[0],
             AnglePitch_Acc, RatePitch, x_hat_pitch[0],
             RateYaw, x_hat_yaw[0],
             dist_tof_m, x_hat_alt[0]);

    // 3. Enviar el paquete completo por UDP Broadcast(Convertimos el char array a String)
    //enviarMensajeUDP(String(buffer_telemetria));
    //enviarMensajeUDP(String(AccX) + "," + String(AccY) + "," + String(AccZ));
    
    // 4. Relajamos la tarea para no saturar el Wi-Fi ni el procesador.
    // 20 ms = 50 Hz de tasa de refresco.
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}