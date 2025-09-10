// src/pid_controller.c
#include "pid_controller.h"
#include <stdio.h>
#include <math.h> // Para la función de valor absoluto fabs()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pulse_counter.h" // Para leer la posición del encoder
#include "pwm_generator.h" // Para la función que mueve el motor

/************************************************************************************
 *                                                                                  *
 *                   CONFIGURACIÓN DE PARÁMETROS DEL CONTROLADOR                    *
 *                                                                                  *
 ************************************************************************************/

// --- PARÁMETROS DE TEMPORIZACIÓN ---
// Frecuencia a la que se ejecutará el bucle de control. 50 Hz es un buen punto
// de partida para sistemas mecánicos. Un valor más bajo es menos reactivo, uno
// más alto consume más CPU y puede ser más sensible al ruido.
#define PID_LOOP_PERIOD_MS 20       // Frecuencia del bucle: 1000ms / 20ms = 50 Hz

// --- PARÁMETROS DE CONTROL ---
// El punto de equilibrio deseado. Gracias a la señal Z del encoder que resetea
// el contador, nuestro punto de equilibrio es siempre CERO.
#define SETPOINT           0

// Banda muerta (Dead Band). Si el error (en cuentas del encoder) es menor que
// este valor, lo consideramos cero. Esto es CRUCIAL para evitar que el motor
// vibre o "tiemble" constantemente tratando de corregir errores minúsculos.
#define DEAD_BAND_PULSES   50

// --- PARÁMETROS DEL ACTUADOR (MOTOR) ---
// Límite máximo de pulsos que el PID puede ordenar en una sola corrección.
// Sirve como medida de seguridad para evitar que una reacción brusca del PID
// genere un movimiento demasiado violento.
#define MAX_OUTPUT_PULSES  500

// Frecuencia base (velocidad mínima) para los movimientos de corrección.
#define BASE_FREQUENCY     3000

// Factor de escalado de velocidad. Hace que la corrección sea más rápida
// para errores grandes. La frecuencia final será:
// Frecuencia = BASE_FREQUENCY + (Error * FREQ_PER_ERROR_PULSE)
#define FREQ_PER_ERROR_PULSE 50.0f

/************************************************************************************
 *                        FIN DE LA CONFIGURACIÓN DE PARÁMETROS                     *
 ************************************************************************************/

static const char *TAG = "PID_CONTROLLER";

// Variables de estado globales para el controlador
static volatile bool g_pid_enabled = false;
static float g_kp = 1.0;  // Ganancia Proporcional: El "presente". Reacciona al error actual.
static float g_ki = 0.0;  // Ganancia Integral: El "pasado". Corrige errores acumulados. (DESHABILITADA)
static float g_kd = 10.0;  // Ganancia Derivativa: El "futuro". Predice y amortigua. (DESHABILITADA)

static float g_integral = 0.0;
static float g_last_error = 0.0;


// --- Implementación de funciones públicas ---

void pid_toggle_enable(void) {
    g_pid_enabled = !g_pid_enabled;
    if (g_pid_enabled) {
        g_integral = 0.0;
        g_last_error = 0.0;
        ESP_LOGW(TAG, "Control PID HABILITADO");
    } else {
        ESP_LOGW(TAG, "Control PID DESHABILITADO");
    }
}

void pid_set_kp(float kp) { g_kp = kp; ESP_LOGI(TAG, "Kp actualizado a: %f", g_kp); }
void pid_set_ki(float ki) { g_ki = ki; ESP_LOGI(TAG, "Ki actualizado a: %f", g_ki); }
void pid_set_kd(float kd) { g_kd = kd; ESP_LOGI(TAG, "Kd actualizado a: %f", g_kd); }


// --- Tarea Principal del Controlador ---

void pid_controller_task(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PID_LOOP_PERIOD_MS));

        if (!g_pid_enabled) {
            continue;
        }

        // 1. MEDIR: Leer la posición actual del péndulo
        int16_t current_position = pulse_counter_get_value();

        // 2. CALCULAR ERROR: La diferencia entre donde queremos estar (0) y donde estamos
        float error = SETPOINT - current_position;

        // 3. APLICAR BANDA MUERTA
        if (fabs(error) < DEAD_BAND_PULSES) {
            error = 0;
        }

        // 4. CALCULAR SALIDA DEL CONTROLADOR (Solo Proporcional por ahora)
        float p_term = g_kp * error;
        float output = p_term; // En el futuro será: p_term + i_term + d_term

        // 5. SATURAR LA SALIDA
        if (output > MAX_OUTPUT_PULSES) output = MAX_OUTPUT_PULSES;
        if (output < -MAX_OUTPUT_PULSES) output = -MAX_OUTPUT_PULSES;

        // 6. ACTUAR: Si la salida no es cero, mover el motor
        if (fabs(output) > 0.1) {
            int num_pulses = (int)fabs(output);
            
            // Si output es positivo, el péndulo cayó a la izquierda (posición negativa, error positivo).
            // Necesitamos movernos a la izquierda para corregir. Asumimos dir=0 es izquierda.
            // Si output es negativo, el péndulo cayó a la derecha (posición positiva, error negativo).
            // Necesitamos movernos a la derecha. Asumimos dir=1 es derecha.
            // NOTA: Si el motor se mueve en sentido contrario, invierte la lógica aquí (0 : 1)
            int direction = (output > 0) ? 0 : 1; 
            
            int frequency = BASE_FREQUENCY + (int)(fabs(output) * FREQ_PER_ERROR_PULSE);

            execute_movement(num_pulses, frequency, direction);
        }
        
        g_last_error = error; // Guardamos para el futuro término Derivativo
    }
}