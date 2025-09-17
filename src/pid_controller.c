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

/************************************************************************************
 *                                                                                  *
 *                   CONFIGURACIÓN DE PARÁMETROS DEL CONTROLADOR                    *
 *                                                                                  *
 ************************************************************************************/

// --- PARÁMETROS DE TEMPORIZACIÓN ---
// Frecuencia a la que se ejecutará el bucle de control. 50 Hz es un buen punto
// de partida para sistemas mecánicos. Un valor más bajo es menos reactivo, uno
// más alto consume más CPU y puede ser más sensible al ruido.
#define PID_LOOP_PERIOD_MS 20 // Frecuencia del bucle: 1000ms / 20ms = 50 Hz

// --- PARÁMETROS DE CONTROL ---
// El punto de equilibrio deseado. Gracias a la señal Z del encoder que resetea
// el contador, nuestro punto de equilibrio es siempre CERO.
// #define SETPOINT //   0 Este es para colocar el péndulo en 0  grados para dejarlo colgado en este caso queremos que empiece en la posición colocada

// Banda muerta (Dead Band). Si el error (en cuentas del encoder) es menor que
// este valor, lo consideramos cero. Esto es CRUCIAL para evitar que el motor
// vibre o "tiemble" constantemente tratando de corregir errores minúsculos.
#define DEAD_BAND_PULSES 10

// Evita que el término integral crezca indefinidamente y desestabilice el sistema.
// Este valor debe ser menor o igual a MAX_OUTPUT_PULSES.
#define MAX_INTEGRAL 400.0f

// --- PARÁMETROS DEL ACTUADOR (MOTOR) ---
// Límite máximo de pulsos que el PID puede ordenar en una sola corrección.
// Sirve como medida de seguridad para evitar que una reacción brusca del PID
// genere un movimiento demasiado violento.
#define MAX_OUTPUT_PULSES 1000

// Frecuencia base (velocidad mínima) para los movimientos de corrección.
#define BASE_FREQUENCY 6000

// Factor de escalado de velocidad. Hace que la corrección sea más rápida
// para errores grandes. La frecuencia final será:
// Frecuencia = BASE_FREQUENCY + (Error * FREQ_PER_ERROR_PULSE)
#define FREQ_PER_ERROR_PULSE 80.0f

/************************************************************************************
 *                        FIN DE LA CONFIGURACIÓN DE PARÁMETROS                     *
 ************************************************************************************/

// Calculamos el tiempo del ciclo en segundos (dt) una sola vez.
static const float PID_LOOP_PERIOD_S = PID_LOOP_PERIOD_MS / 1000.0f;

static const char *TAG = "PID_CONTROLLER";

// Variables de estado globales para el controlador
// para 3200pulse/rev kp=5, ki=1, kd=10, para 2000pulse/rev kp=70, ki=1, kd=10
static volatile bool g_pid_enabled = false;
static float g_kp = 1.0; // Ganancia Proporcional: El "presente". Reacciona al error actual.
static float g_ki = 0.0; // Ganancia Integral: El "pasado". Corrige errores acumulados. (DESHABILITADA)
static float g_kd = 0.0; // Ganancia Derivativa: El "futuro". Predice y amortigua. (DESHABILITADA)

// --- AÑADIDO: 'g_setpoint' es ahora una variable que podemos cambiar ---
static volatile int16_t g_setpoint = 0; // Se inicializa en 0 por defecto

static float g_integral = 0.0;
static float g_last_error = 0.0;

// --- Implementación de funciones públicas ---

void pid_toggle_enable(void)
{
    g_pid_enabled = !g_pid_enabled;
    if (g_pid_enabled)
    {
        // --- LÓGICA PARA ESTABLECER EL SETPOINT DINÁMICO ---
        // 1. Leer la posición actual del encoder en el momento de la habilitación.
        int16_t current_position = pulse_counter_get_value();
        // 2. Establecer esa posición como nuestro nuevo punto de equilibrio.
        g_setpoint = current_position;

        g_integral = 0.0;
        g_last_error = 0.0;
        ESP_LOGW(TAG, "Control PID HABILITADO");
    }
    else
    {
        ESP_LOGW(TAG, "Control PID DESHABILITADO");
    }
}

void pid_set_kp(float kp)
{
    g_kp = kp;
    ESP_LOGI(TAG, "Kp actualizado a: %f", g_kp);
}
void pid_set_ki(float ki)
{
    g_ki = ki;
    ESP_LOGI(TAG, "Ki actualizado a: %f", g_ki);
}
void pid_set_kd(float kd)
{
    g_kd = kd;
    ESP_LOGI(TAG, "Kd actualizado a: %f", g_kd);
}

// --- AÑADIDO: Implementación de la nueva función 'getter' ---
int16_t pid_get_setpoint(void)
{
    return g_setpoint;
}

bool pid_is_enabled(void)
{
    return g_pid_enabled;
}

// --- Tarea Principal del Controlador ---

void pid_controller_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    // Reseteamos el error anterior al habilitar para evitar un pico inicial en D
    pid_toggle_enable(); // Habilita y resetea
    pid_toggle_enable(); // Deshabilita de nuevo, pero el estado está limpio

    while (1)
    {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PID_LOOP_PERIOD_MS));

        if (!g_pid_enabled)
        {
            g_last_error = 0; // Mientras está deshabilitado, el error anterior es cero
            g_integral = 0.0; // Reseteamos el término integral al deshabilitar
            continue;
        }

        // 1. MEDIR: Leer la posición actual del péndulo
        int16_t current_position = pulse_counter_get_value();

        // 2. CALCULAR ERROR: La diferencia entre donde queremos estar (0) y donde estamos
        // float error = SETPOINT - current_position;
        float error = g_setpoint - current_position;

        // 3. APLICAR BANDA MUERTA
        if (fabs(error) < DEAD_BAND_PULSES)
        {
            error = 0;
        }

        // Acumulamos el error en el término integral.
        // Se multiplica por (PID_LOOP_PERIOD_MS / 1000.0f) para que sea independiente de la frecuencia del bucle.
        g_integral += error * PID_LOOP_PERIOD_S;

        // Anti-Windup: Limitamos el término integral para que no crezca demasiado.
        if (g_integral > MAX_INTEGRAL)
            g_integral = MAX_INTEGRAL;
        if (g_integral < -MAX_INTEGRAL)
            g_integral = -MAX_INTEGRAL;

        // Si el error es cero, reseteamos el integral para evitar que siga actuando innecesariamente.
        if (error == 0)
            g_integral = 0;

        float i_term = g_ki * g_integral;

        // 4. CALCULAR SALIDA DEL CONTROLADOR (Solo Proporcional por ahora)
        float p_term = g_kp * error;

        // --- Término Derivativo (D) ---
        // Calcula la "velocidad" del error (cuánto cambió desde el último ciclo)
        float derivative = (error - g_last_error) / PID_LOOP_PERIOD_S;
        float d_term = g_kd * derivative;

        // Sumamos los términos para obtener la salida final
        float output = p_term + d_term + i_term;

        // 5. SATURAR LA SALIDA
        if (output > MAX_OUTPUT_PULSES)
            output = MAX_OUTPUT_PULSES;
        if (output < -MAX_OUTPUT_PULSES)
            output = -MAX_OUTPUT_PULSES;

        // 6. ACTUAR: Si la salida no es cero, mover el motor
        if (fabs(output) > 0.1)
        {
            int num_pulses = (int)fabs(output);

            // Si output es positivo, el péndulo cayó a la izquierda (posición negativa, error positivo).
            // Necesitamos movernos a la izquierda para corregir. Asumimos dir=0 es izquierda.
            // Si output es negativo, el péndulo cayó a la derecha (posición positiva, error negativo).
            // Necesitamos movernos a la derecha. Asumimos dir=1 es derecha.
            // NOTA: Si el motor se mueve en sentido contrario, invierte la lógica aquí (0 : 1)
            int direction = (output > 0) ? 1 : 0;

            int frequency = BASE_FREQUENCY + (int)(fabs(output) * FREQ_PER_ERROR_PULSE);

            execute_movement(num_pulses, frequency, direction);
        }

        g_last_error = error; // Guardamos para el futuro término Derivativo
    }
}