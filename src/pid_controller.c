/*A pesar de llamarse PID_controller,
este modelo no se basa en un PID sino en ecuaciones de estado*/

// src/pid_controller.c
#include "pid_controller.h"
#include <stdio.h>
#include <math.h> // Para la función de valor absoluto fabs()
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pulse_counter.h" // Para leer la posición del encoder
#include "pwm_generator.h" // Para la función que mueve el motor
#include "uart_echo.h"

static const char *TAG = "STATE_EC";

// --- PARÁMETROS FÍSICOS (reemplaza con los tuyos) ---
#define M_CART 0.5f  // kg
#define M_POLE 0.2f  // kg
#define POLE_LENGTH_M 0.3f // metros (l)
#define PID_LOOP_PERIOD_MS 20

// --- GANANCIAS LQR (copia aquí los resultados de tu script de Python) ---
static const float K1 = -3.16f;  // Ganancia para x
static const float K2 = -5.94f;  // Ganancia para x_dot
static const float K3 = 42.53f;  // Ganancia para theta
static const float K4 = 4.91f;   // Ganancia para theta_dot

// --- LÍMITES Y CONVERSIONES ---
#define MAX_CONTROL_FORCE 10.0f // Límite de la fuerza U (en Newtons) - A sintonizar
#define FORCE_TO_ACCEL_SCALE (1.0f / (M_CART + M_POLE))
// Necesitas la constante de tu modelo que convierte v_car a f_pulsos
#define VELOCITY_TO_FREQ_SCALE (10000000.0f / (14.0f * 3.14159f))

static const float LOOP_PERIOD_S = PID_LOOP_PERIOD_MS / 1000.0f;
// --- VARIABLES DE ESTADO Y SETPOINTS ---
static volatile bool g_controller_enabled = false;
volatile int32_t g_car_position_pulses = 0; // Odometría del carro
static float setpoint_x_m = 0.0f;           // Objetivo de posición del carro
static float setpoint_theta_rad = 0.0f;     // Objetivo de ángulo del péndulo

// --- IMPLEMENTACIÓN DE FUNCIONES PÚBLICAS ---

// FUNCIÓN DE PARADA DE EMERGENCIA
void state_controller_force_disable(void) {
    if (g_controller_enabled) { // Solo actúa si el control estaba activo
        g_controller_enabled = false;
        
        ESP_LOGE("STATE_CTRL", "¡PARADA DE EMERGENCIA! Controlador LQR DESHABILITADO.");

        // Enviar un comando de parada explícito a la cola del motor.
        // Usamos xQueueOverwrite para que anule cualquier comando de movimiento pendiente.
        motor_command_t stop_cmd = { .num_pulses = 0, .frequency = 0, .direction = 0 };
        xQueueOverwrite(motor_command_queue, &stop_cmd);
    }
}

void state_controller_toggle_enable(void) {
    g_controller_enabled = !g_controller_enabled;
    ESP_LOGW("STATE_CTRL", "Control LQR %s", g_controller_enabled ? "HABILITADO" : "DESHABILITADO");
}

bool state_controller_is_enabled(void) {
    return g_controller_enabled;
}

void state_controller_set_theta_setpoint(float setpoint_rad) {
    setpoint_theta_rad = setpoint_rad;
    ESP_LOGW("STATE_CTRL", "Setpoint de ángulo actualizado a: %.2f rad", setpoint_theta_rad);
}

void state_controller_task(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // Variables para estimar el estado
    float last_theta_rad = 0.0f;
    float current_velocity_m_s = 0.0f;
    float setpoint_x_m = 0.0f; // Queremos que el carro esté en 0 metros

    if (g_controller_enabled) state_controller_toggle_enable();

    while(1) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PID_LOOP_PERIOD_MS));
        if (!g_controller_enabled) continue;

        // --- 1. ESTIMACIÓN DE ESTADO ---
        // Ángulo (theta)
        int16_t angle_counts = pulse_counter_get_value();
        float theta_rad = (float)angle_counts * (2.0f * 3.14159f / 4096.0f);

        // Velocidad Angular (theta_dot)
        float theta_dot_rad_s = (theta_rad - last_theta_rad) / LOOP_PERIOD_S;
        last_theta_rad = theta_rad;

        // Posición del Carro (x)
        // Necesitas tu constante de pulsos por metro (136400 pulsos / 0.59m)
        const float PULSES_PER_METER = 136400.0f / 0.59f;
        float x_m = (float)g_car_position_pulses / PULSES_PER_METER;
        
        // Velocidad del Carro (x_dot)
        // x_dot es la velocidad que *comandamos* en el ciclo anterior. Es una buena estimación.
        float x_dot_m_s = current_velocity_m_s;

        // --- 2. CÁLCULO DE LA LEY DE CONTROL ---
        float error_x = x_m - setpoint_x_m;
        float error_theta = theta_rad - setpoint_theta_rad;

        float U = -(K1 * error_x + K2 * x_dot_m_s + K3 * error_theta + K4 * theta_dot_rad_s);

        // Saturar la fuerza de control
        if (U > MAX_CONTROL_FORCE) U = MAX_CONTROL_FORCE;
        if (U < -MAX_CONTROL_FORCE) U = -MAX_CONTROL_FORCE;

        // --- 3. CONVERTIR FUERZA A COMANDO DE MOTOR ---
        float target_acceleration = U * FORCE_TO_ACCEL_SCALE;
        float target_velocity = x_dot_m_s + (target_acceleration * LOOP_PERIOD_S);
        current_velocity_m_s = target_velocity; // Guardar para el próximo ciclo

        int target_frequency = (int)fabs(target_velocity * VELOCITY_TO_FREQ_SCALE);
        int direction = (target_velocity >= 0) ? 1 : 0;
        
        // El número de pulsos se calcula para un movimiento continuo
        int num_pulses = (uint32_t)(target_frequency * PID_LOOP_PERIOD_MS) / 1000;
        if (num_pulses == 0 && target_frequency > 10) num_pulses = 1;

        // Enviar comando a la cola
        motor_command_t cmd = { num_pulses, target_frequency, direction };
        xQueueOverwrite(motor_command_queue, &cmd);
    }
}