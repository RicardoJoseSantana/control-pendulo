// src/state_controller.c
#include "state_controller.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pulse_counter.h"
#include "pwm_generator.h"
#include "uart_echo.h"

static const char *TAG = "STATE_CONTROLLER";

// --- PARÁMETROS FÍSICOS ---
#define M_CART 0.320f // Kg
#define M_POLE 0.030f // Kg
#define POLE_LENGTH 0.26f // m
#define CONTROLLER_PERIOD_MS 20 // 50 Hz, debe coincidir con el 'Ts' de tu simulación (0.02)
static const float LOOP_PERIOD_S = CONTROLLER_PERIOD_MS / 1000.0f;

// --- GANANCIAS CALCULADAS EN MATLAB ---
// U = Ki*v - K*x
// ¡OJO! Tu ley de control es U = Ki*v - K*x. El K de MATLAB ya incluye el signo negativo.
// Y tu Ki también es negativa. Así que la fórmula en código será:
// U = -KI_from_matlab * v_state - (K1*x1 + K2*x2 + K3*x3 + K4*x4)
static const float K1 = -30.5772f; // Ganancia para x1 (theta)
static const float K2 = -4.9774f;  // Ganancia para x2 (theta_dot)
static const float K3 = -29.9377f; // Ganancia para x3 (x)
static const float K4 = -11.3899f; // Ganancia para x4 (x_dot)
static const float KI = -0.7554f;  // Ganancia para x5 (v)

// --- LÍMITES Y CONVERSIONES ---
#define MAX_CONTROL_FORCE 20.0f // Límite de la fuerza U (en Newtons) - A sintonizar
#define PULSES_PER_METER (136400.0f / 0.59f) // Tu dato de calibración
#define RAD_PER_ENCODER_COUNT (2.0f * 3.14159f / 4096.0f)

// --- VARIABLES DE ESTADO Y CONTROL ---
static volatile bool g_controller_enabled = false;
volatile int32_t g_car_position_pulses = 0;
static float g_cart_setpoint_m = 0.0f; // Objetivo de posición del carro en metros
static float g_theta_vertical_rad = 0.0f; // Ángulo vertical en radianes
static float g_integrator_state = 0.0f; // El estado v(k) del integrador
static float g_last_theta_rad = 0.0f;

// --- IMPLEMENTACIÓN DE FUNCIONES PÚBLICAS ---
void state_controller_toggle_enable(void) {
    g_controller_enabled = !g_controller_enabled;
    if(g_controller_enabled) {
        // Al habilitar, reseteamos el integrador y el estado de la derivada
        g_integrator_state = 0.0f;
        g_last_theta_rad = (float)pulse_counter_get_value() * RAD_PER_ENCODER_COUNT;
    } else {
        // Enviar comando de parada al deshabilitar
        motor_command_t stop_cmd = { .target_frequency = 0, .direction = 0 };
        xQueueOverwrite(motor_command_queue, &stop_cmd);
    }
    ESP_LOGW(TAG, "Control de Estado %s", g_controller_enabled ? "HABILITADO" : "DESHABILITADO");
}
bool state_controller_is_enabled(void) { return g_controller_enabled; }
void state_controller_set_cart_setpoint(float position_m) { g_cart_setpoint_m = position_m; }
void state_controller_set_theta_setpoint(float setpoint_rad) { g_theta_vertical_rad = setpoint_rad; }
void state_controller_force_disable(void) { /* ...implementación anterior... */ }

// --- TAREA PRINCIPAL DEL CONTROLADOR ---
void state_controller_task(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();
    float current_cart_velocity_m_s = 0.0f;

    while(1) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROLLER_PERIOD_MS));
        if (!g_controller_enabled) continue;

        // --- 1. ESTIMACIÓN DEL ESTADO COMPLETO (x(k)) ---
        int16_t angle_counts = pulse_counter_get_value();
        float x1_theta = (float)angle_counts * RAD_PER_ENCODER_COUNT - g_theta_vertical_rad;
        float x2_theta_dot = (x1_theta - g_last_theta_rad) / LOOP_PERIOD_S;
        g_last_theta_rad = x1_theta;
        float x3_x = (float)g_car_position_pulses / PULSES_PER_METER;
        float x4_x_dot = current_cart_velocity_m_s;

        // --- 2. ACTUALIZAR EL INTEGRADOR (v(k)) ---
        // r(k) es la referencia de posición del carro
        float position_error = g_cart_setpoint_m - x3_x;
        g_integrator_state += position_error;

        // --- 3. APLICAR LA LEY DE CONTROL (u(k)) ---
        float feedback_sum = (K1 * x1_theta) + (K2 * x2_theta_dot) + (K3 * x3_x) + (K4 * x4_x_dot);
        float u = (KI * -g_integrator_state) - feedback_sum; // U = -KI*v - K*x

        // Saturar la fuerza de control
        if (u > MAX_CONTROL_FORCE) u = MAX_CONTROL_FORCE;
        if (u < -MAX_CONTROL_FORCE) u = -MAX_CONTROL_FORCE;

        // --- 4. CONVERTIR FUERZA `u` A COMANDO DE MOTOR ---
        // Usamos la física del carro: a = F/m_total
        float target_acceleration = u / (M_CART + M_POLE);
        float target_velocity = x4_x_dot + (target_acceleration * LOOP_PERIOD_S);
        current_cart_velocity_m_s = target_velocity; // Guardar para el próximo ciclo

        // Necesitas tu constante de conversión de velocidad a frecuencia
        const float VELOCITY_TO_FREQ_SCALE = (PULSES_PER_METER * 0.59) / 136400.0f; // Simplificado, necesitas calibrar esto
        
        int target_frequency = (int)fabs(target_velocity * (PULSES_PER_METER / (2 * 3.14159f * (59 / (2 * 3.14159f)))));
        int direction = (target_velocity >= 0) ? 1 : 0;
        
        // --- Lógica de Saturación de Frecuencia (Mínima y Máxima) ---
        if (target_frequency > 0 && target_frequency < 10) target_frequency = 10;
        if (target_frequency > 100000) target_frequency = 100000;

        int pulses_this_cycle = (int)(target_frequency * LOOP_PERIOD_S);
        if (direction == 1) {
            g_car_position_pulses += pulses_this_cycle;
        } else {
            g_car_position_pulses -= pulses_this_cycle;
        }
        
        // El movimiento es continuo, dura lo que el ciclo del controlador
        motor_command_t cmd = { .num_pulses = pulses_this_cycle, .target_frequency = target_frequency, .direction = direction };
        xQueueOverwrite(motor_command_queue, &cmd);
    }
}