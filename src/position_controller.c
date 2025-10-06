// src/position_controller.c
#include "position_controller.h"
#include <math.h>
#include "freertos/task.h"
#include "esp_log.h"
#include "pid_controller.h" // Incluimos para acceder a g_car_position_pulses

// --- PARÁMETROS DEL PID DE POSICIÓN ---
#define POS_PID_LOOP_PERIOD_MS 100 // Bucle lento, 10 Hz
#define POS_SETPOINT           0   // El centro del riel
// Límite máximo para la velocidad de corrección (en Hz). ¡Este es el secreto!
// Un valor bajo asegura que la corrección de posición sea siempre suave.
#define MAX_CORRECTION_VELOCITY_HZ 5000.0f 

static const char *TAG = "POS_CONTROLLER";

// --- Constantes y variables de estado para el PID de posición ---
static float g_kpx = 0.1; // Ganancia Proporcional para la posición (empezar bajo)
static float g_kix = 0.005; // Ganancia Integral para la posición (muy bajo)
static float g_kdx = 0.2; // Ganancia Derivativa para la posición
static float pos_integral = 0.0;
static float pos_last_error = 0.0;
volatile float g_position_correction_velocity = 0.0f;

// Funciones de sintonización
void pos_pid_set_kp(float kp) { g_kpx = kp; ESP_LOGI(TAG, "Position Kp set to: %f", g_kpx); }
void pos_pid_set_ki(float ki) { g_kix = ki; ESP_LOGI(TAG, "Position Ki set to: %f", g_kix); }
void pos_pid_set_kd(float kd) { g_kdx = kd; ESP_LOGI(TAG, "Position Kd set to: %f", g_kdx); }

void position_controller_task(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const float dt = POS_PID_LOOP_PERIOD_MS / 1000.0f;

    while(1) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(POS_PID_LOOP_PERIOD_MS));

        // Solo calculamos si el PID principal está activo
        if (pid_is_enabled()) {
            float error = POS_SETPOINT - g_car_position_pulses;
            
            pos_integral += error * dt;
            // Anti-windup simple
            if (pos_integral > (MAX_CORRECTION_VELOCITY_HZ / g_kix)) pos_integral = (MAX_CORRECTION_VELOCITY_HZ / g_kix);
            if (pos_integral < -(MAX_CORRECTION_VELOCITY_HZ / g_kix)) pos_integral = -(MAX_CORRECTION_VELOCITY_HZ / g_kix);

            float derivative = (error - pos_last_error) / dt;
            pos_last_error = error;

            float p_term = g_kpx * error;
            float i_term = g_kix * pos_integral;
            float d_term = g_kdx * derivative;

            float output_velocity = p_term + i_term + d_term;

            // --- SATURACIÓN (CRUCIAL) ---
            if (output_velocity > MAX_CORRECTION_VELOCITY_HZ) output_velocity = MAX_CORRECTION_VELOCITY_HZ;
            if (output_velocity < -MAX_CORRECTION_VELOCITY_HZ) output_velocity = -MAX_CORRECTION_VELOCITY_HZ;
            
            g_position_correction_velocity = output_velocity;
        } else {
            // Si el PID principal está apagado, reseteamos nuestro estado
            pos_integral = 0.0;
            pos_last_error = 0.0;
            g_position_correction_velocity = 0.0f;
        }
    }
}