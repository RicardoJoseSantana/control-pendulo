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

/************************************************************************************
 *                                                                                  *
 *                   CONFIGURACIÓN DE PARÁMETROS DEL CONTROLADOR                    *
 *                                                                                  *
 ************************************************************************************/

// --- PARÁMETROS DE TEMPORIZACIÓN ---
// Frecuencia a la que se ejecutará el bucle de control. 50 Hz es un buen punto
// de partida para sistemas mecánicos. Un valor más bajo es menos reactivo, uno
// más alto consume más CPU y puede ser más sensible al ruido.
#define PID_LOOP_PERIOD_MS 10 // Frecuencia del bucle: 1000ms / 20ms = 50 Hz
#define POSITION_CONTROL_DIVIDER 10 // Ejecutar el PID de posición 1 vez cada 10 ciclos (100ms)

// Banda muerta (Dead Band). Si el error (en cuentas del encoder) es menor que
// este valor, lo consideramos cero. Esto es CRUCIAL para evitar que el motor
// vibre o "tiemble" constantemente tratando de corregir errores minúsculos.
//#define DEAD_BAND_ANGLE   1.6f //con factor de crecimiento de 0.088° o 0.1° 1.76
#define ANGLE_DEAD_BAND_COUNTS 20
#define CAR_DEAD_BAND_PULSES   15000 

#define DEAD_BAND_X_CM 5

// Evita que el término integral crezca indefinidamente y desestabilice el sistema.
// Este valor debe ser menor o igual a MAX_OUTPUT_PULSES.
//#define MAX_INTEGRAL       1000.0f
#define MAX_ANGLE_INTEGRAL    1000.0f
#define MAX_POSITION_INTEGRAL 5000.0f

// --- PARÁMETROS DEL ACTUADOR (MOTOR) ---
// Límite máximo de pulsos que el PID puede ordenar en una sola corrección.
// Sirve como medida de seguridad para evitar que una reacción brusca del PID
// genere un movimiento demasiado violento.
//#define MAX_OUTPUT_PULSES  1700
#define MAX_ANGLE_OUTPUT 1700
#define MAX_POSITION_OUTPUT 300  // Límite para la contribución de la posición

// Frecuencia base (velocidad mínima) para los movimientos de corrección.
#define BASE_FREQUENCY 1000

// Factor de escalado de velocidad. Hace que la corrección sea más rápida
// para errores grandes. La frecuencia final será:
// Frecuencia = BASE_FREQUENCY + (Error * FREQ_PER_ERROR_PULSE)
#define FREQ_PER_ERROR_PULSE 80

// Ganancia Proporcional para la posición del carro. Convierte el error de posición
// en un pequeño ángulo de inclinación deseado (en cuentas del encoder).
// Este es tu nuevo "dial" para controlar qué tan rápido vuelve el carro al centro.
#define POSITION_CONTROL_GAIN 0.0005f//0.001//0.05f

// Límite máximo para el offset del setpoint. Evita que pida ángulos demasiado grandes.
#define MAX_SETPOINT_OFFSET 25 // Máximo offset de 50 cuentas

//máxima frecuencia
#define MAX_FRECUENCY_LIMIT 140000

static const float ANGLE_LOOP_PERIOD_S = PID_LOOP_PERIOD_MS / 1000.0f;
static const float POSITION_LOOP_PERIOD_S = (PID_LOOP_PERIOD_MS * POSITION_CONTROL_DIVIDER) / 1000.0f;
static const char *TAG = "PID_CASCADE_CONTROLLER";

/************************************************************************************
 *                        FIN DE LA CONFIGURACIÓN DE PARÁMETROS                     *
 ************************************************************************************/

// Calculamos el tiempo del ciclo en segundos (dt) una sola vez.
//static const float PID_LOOP_PERIOD_S = PID_LOOP_PERIOD_MS / 1000.0f;
//static float g_smoothed_output = 0.0;
//static const char *TAG = "PID_CONTROLLER";

// Variables de estado globales para el controlador
// para 3200pulse/rev kp=5, ki=1, kd=10, para 2000pulse/rev kp=70, ki=1, kd=10, para 10000pulse/rev kp=41, ki=0.4, kd=70
static volatile bool g_pid_enabled = false;
//static float g_kp = 41.0;  // Ganancia Proporcional: El "presente". Reacciona al error actual.
//static float g_ki = 0.4;  // Ganancia Integral: El "pasado". Corrige errores acumulados.
//static float g_kd = 70.0;  // Ganancia Derivativa: El "futuro". Predice y amortigua.

// --- Variable para el angulo del pendulo ---
//static volatile int16_t g_absolute_setpoint = 0; // Se inicializa en 0 por defecto
static volatile int16_t g_angle_setpoint_counts = 0;

// --- Variable para la posición del carro ---
volatile int32_t g_car_position_pulses = 0; //pwm_generator puede actualizarla.

//static float g_integral = 0.0;
//static float g_last_error = 0.0;

// --- PID DE ÁNGULO (PID_θ) ---
static float g_kp_angle = 41.0;
static float g_ki_angle = 0.4;
static float g_kd_angle = 70.0;
static float angle_integral = 0.0;
static float angle_last_error = 0.0;
static float g_angle_smoothed_output = 0.0;

// --- PID DE POSICIÓN (PID_x) ---
static float g_kpx_pos = 0;//0.1;
static float g_kix_pos = 0;//0.005;
static float g_kdx_pos = 0;//0.2;
static float pos_integral = 0.0;
static float pos_last_error = 0.0;
static float g_position_correction_output = 0.0f; // Salida del PID de posición

// --- AÑADIDO: Implementación de las nuevas funciones ---
float pid_get_kp(void) { return g_kp_angle; }
float pid_get_ki(void) { return g_ki_angle; }
float pid_get_kd(void) { return g_kd_angle; }

float pid_get_kpx(void) { return g_kpx_pos; }
float pid_get_kix(void) { return g_kix_pos; }
float pid_get_kdx(void) { return g_kdx_pos; }

// --- Implementación de funciones públicas ---

// --- establecer el setpoint externamente ---
void pid_set_absolute_setpoint(int16_t new_setpoint) {
    g_angle_setpoint_counts = new_setpoint;
    ESP_LOGW(TAG, "Setpoint absoluto establecido en: %d", g_angle_setpoint_counts);
}

void pid_toggle_enable(void)
{
    g_pid_enabled = !g_pid_enabled;
    if (g_pid_enabled)
    {
        // --- LÓGICA PARA ESTABLECER EL SETPOINT DINÁMICO ---
        // 1. Leer la posición actual del encoder en el momento de la habilitación.
        //int16_t current_position = pulse_counter_get_value();
        // 2. Establecer esa posición como nuestro nuevo punto de equilibrio.
        //g_absolute_setpoint = current_position;

        angle_integral= 0.0;
        angle_last_error = 0.0;
        ESP_LOGW(TAG, "Control PID HABILITADO");
    }
    else
    {
        ESP_LOGW(TAG, "Control PID DESHABILITADO");
    }
}

void pid_set_kp(float kp)
{
    g_kp_angle = kp;
    ESP_LOGI(TAG, "Kp actualizado a: %f", g_kp_angle);
}
void pid_set_ki(float ki)
{
    g_ki_angle = ki;
    ESP_LOGI(TAG, "Ki actualizado a: %f", g_ki_angle);
}
void pid_set_kd(float kd)
{
    g_kd_angle = kd;
    ESP_LOGI(TAG, "Kd actualizado a: %f", g_kd_angle);
}

void pid_set_kpx(float kpx)
{
    g_kpx_pos = kpx;
    ESP_LOGI(TAG, "Kp actualizado a: %f", g_kpx_pos);
}
void pid_set_kix(float kix)
{
    g_kix_pos = kix;
    ESP_LOGI(TAG, "Ki actualizado a: %f", g_kix_pos);
}
void pid_set_kdx(float kdx)
{
    g_kdx_pos = kdx;
    ESP_LOGI(TAG, "Kd actualizado a: %f", g_kdx_pos);
}

// --- AÑADIDO: Implementación de la nueva función 'getter' ---
int16_t pid_get_setpoint(void)
{
    return g_angle_setpoint_counts;
}

// --- AÑADIDO: Implementación de la función de deshabilitación forzada ---
void pid_force_disable(void)
{
    if (g_pid_enabled)
    { // Solo actúa y muestra el mensaje si estaba habilitado
        g_pid_enabled = false;
        // Reseteamos el estado para un futuro arranque limpio
        angle_integral = 0.0;
        angle_last_error = 0.0;
        ESP_LOGE(TAG, "¡PARADA DE EMERGENCIA! Control PID DESHABILITADO.");
    }
}

bool pid_is_enabled(void)
{
    return g_pid_enabled;
}

// --- Tarea Principal del Controlador ---

void pid_controller_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t cycle_counter = 0;
    //float dead_band_x = DEAD_BAND_X_CM*37200/12; //convierte lo cm a pulsos utiles
    //int DEAD_BAND_PULSES = DEAD_BAND_ANGLE*4096/360; //convierte el angulo a pulsos utiles

    // Reseteamos el error anterior al habilitar para evitar un pico inicial en D
    pid_toggle_enable(); // Habilita y resetea
    pid_toggle_enable(); // Deshabilita de nuevo, pero el estado está limpio

    while (1)
    {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PID_LOOP_PERIOD_MS));

        if (!g_pid_enabled)
        {
            angle_last_error = 0; // Mientras está deshabilitado, el error anterior es cero
            angle_integral = 0.0; // Reseteamos el término integral al deshabilitar
            continue;
        }



        // --- CÁLCULO DE BAJA FRECUENCIA: PID DE POSICIÓN (PID_x) ---
        // Se ejecuta solo una vez cada POSITION_CONTROL_DIVIDER ciclos
        if (cycle_counter % POSITION_CONTROL_DIVIDER == 0) {
            float pos_error = 0.0f - g_car_position_pulses;
            if (fabs(pos_error) < CAR_DEAD_BAND_PULSES) pos_error = 0;
            
            pos_integral += pos_error * POSITION_LOOP_PERIOD_S;
            if (pos_integral > MAX_POSITION_INTEGRAL) pos_integral = MAX_POSITION_INTEGRAL;
            if (pos_integral < -MAX_POSITION_INTEGRAL) pos_integral = -MAX_POSITION_INTEGRAL;
            if (pos_error == 0) pos_integral = 0;

            float pos_derivative = (pos_error - pos_last_error) / POSITION_LOOP_PERIOD_S;
            pos_last_error = pos_error;

            float p_term = g_kpx_pos * pos_error;
            float i_term = g_kix_pos * pos_integral;
            float d_term = g_kdx_pos * pos_derivative;

            float output = p_term + i_term + d_term;

            // Saturar la salida del PID de posición
            if (output > MAX_POSITION_OUTPUT) output = MAX_POSITION_OUTPUT;
            if (output < -MAX_POSITION_OUTPUT) output = -MAX_POSITION_OUTPUT;
            
            g_position_correction_output = output; // Guardar el resultado
        }
        cycle_counter++;



        // --- CÁLCULO DE ALTA FRECUENCIA: PID DE ÁNGULO (PID_θ) ---
        // Se ejecuta en cada ciclo de la tarea (cada 10ms)
        int16_t current_angle = pulse_counter_get_value();
        float angle_error = g_angle_setpoint_counts - current_angle;
        if (fabs(angle_error) < ANGLE_DEAD_BAND_COUNTS) angle_error = 0;
        
        angle_integral += angle_error * ANGLE_LOOP_PERIOD_S;
        if (angle_integral > MAX_ANGLE_INTEGRAL) angle_integral = MAX_ANGLE_INTEGRAL;
        if (angle_integral < -MAX_ANGLE_INTEGRAL) angle_integral = -MAX_ANGLE_INTEGRAL;
        if (angle_error == 0) angle_integral = 0;
        
        float p_term = g_kp_angle * angle_error;
        float i_term = g_ki_angle * angle_integral;
        float derivative = (angle_error - angle_last_error) / ANGLE_LOOP_PERIOD_S;
        float d_term = g_kd_angle * derivative;
        angle_last_error = angle_error;
        
        float balance_output_raw = p_term + i_term + d_term;
        
        g_angle_smoothed_output = (0.7f * g_angle_smoothed_output) + (0.3f * balance_output_raw);
        if (g_angle_smoothed_output > MAX_ANGLE_OUTPUT) g_angle_smoothed_output = MAX_ANGLE_OUTPUT;
        if (g_angle_smoothed_output < -MAX_ANGLE_OUTPUT) g_angle_smoothed_output = -MAX_ANGLE_OUTPUT;

        // --- MEZCLA DE SALIDAS ---
        float final_output = g_angle_smoothed_output + g_position_correction_output;

        // 6. ACTUAR: Si la salida no es cero, enviar comando al motor
        if (fabs(final_output) > 0.1) {
            int num_pulses = (int)fabs(final_output);
            int direction = (final_output > 0) ? 1 : 0; 
            int frequency = 1000 + (int)(fabs(final_output) * 80.0f); // Usando tus defines originales

            // --- OPTIMIZACIÓN DE CONTINUIDAD (SOLUCIÓN 1) ---

            // 1. Calcular la duración teórica del movimiento
            uint32_t duration_ms = (uint32_t)(num_pulses * 1000) / frequency;
            
            // 2. Solo actuar si el movimiento es significativo (dura al menos 1ms)
            if (duration_ms > 0) {
                
                // 3. Asegurar que el movimiento dure al menos un ciclo de PID
                // Si la duración calculada es más corta que nuestro ciclo de control...
                if (duration_ms < PID_LOOP_PERIOD_MS) {
                    // ... recalculamos el número de pulsos para que el movimiento 
                    // llene exactamente un ciclo de control. Esto crea la continuidad.
                    num_pulses = 2*(uint32_t)(frequency * PID_LOOP_PERIOD_MS) / 1000;
                }

                if (frequency > MAX_FRECUENCY_LIMIT) {
                    frequency = MAX_FRECUENCY_LIMIT;
                }

                // 4. Enviar el comando (posiblemente ajustado) a la cola del motor
                motor_command_t cmd = {
                    .num_pulses = num_pulses,
                    .frequency = frequency,
                    .direction = direction
                };
                xQueueOverwrite(motor_command_queue, &cmd);
            }
            // Si la duración era 0, simplemente no hacemos nada en este ciclo.

        } else {
            // Si la salida del PID es cero, enviamos un comando de parada explícito.
            motor_command_t stop_cmd = { .num_pulses = 0, .frequency = 1000, .direction = 0 };
            xQueueOverwrite(motor_command_queue, &stop_cmd);
        }

    }
}

