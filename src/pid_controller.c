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

// --- PARÁMETROS DE CONTROL ---
// El punto de equilibrio deseado. Gracias a la señal Z del encoder que resetea
// el contador, nuestro punto de equilibrio es siempre CERO.
// #define SETPOINT //   0 Este es para colocar el péndulo en 0  grados para dejarlo colgado en este caso queremos que empiece en la posición colocada

// Banda muerta (Dead Band). Si el error (en cuentas del encoder) es menor que
// este valor, lo consideramos cero. Esto es CRUCIAL para evitar que el motor
// vibre o "tiemble" constantemente tratando de corregir errores minúsculos.
#define DEAD_BAND_PULSES   20

// Evita que el término integral crezca indefinidamente y desestabilice el sistema.
// Este valor debe ser menor o igual a MAX_OUTPUT_PULSES.
#define MAX_INTEGRAL       1000.0f

// --- PARÁMETROS DEL ACTUADOR (MOTOR) ---
// Límite máximo de pulsos que el PID puede ordenar en una sola corrección.
// Sirve como medida de seguridad para evitar que una reacción brusca del PID
// genere un movimiento demasiado violento.
#define MAX_OUTPUT_PULSES  1600

// Frecuencia base (velocidad mínima) para los movimientos de corrección.
#define BASE_FREQUENCY 1000

// Factor de escalado de velocidad. Hace que la corrección sea más rápida
// para errores grandes. La frecuencia final será:
// Frecuencia = BASE_FREQUENCY + (Error * FREQ_PER_ERROR_PULSE)
#define FREQ_PER_ERROR_PULSE 90.0f

/************************************************************************************
 *                        FIN DE LA CONFIGURACIÓN DE PARÁMETROS                     *
 ************************************************************************************/

// Calculamos el tiempo del ciclo en segundos (dt) una sola vez.
//static const float PID_LOOP_PERIOD_S = PID_LOOP_PERIOD_MS / 1000.0f;
static float g_smoothed_output = 0.0;
static const char *TAG = "PID_CONTROLLER";

// Variables de estado globales para el controlador
// para 3200pulse/rev kp=5, ki=1, kd=10, para 2000pulse/rev kp=70, ki=1, kd=10, para 10000pulse/rev kp=41, ki=0.4, kd=70
static volatile bool g_pid_enabled = false;
static float g_kp = 41.0;  // Ganancia Proporcional: El "presente". Reacciona al error actual.
static float g_ki = 0.4;  // Ganancia Integral: El "pasado". Corrige errores acumulados.
static float g_kd = 70.0;  // Ganancia Derivativa: El "futuro". Predice y amortigua.

// --- AÑADIDO: 'g_setpoint' es ahora una variable que podemos cambiar ---
static volatile int16_t g_setpoint = 0; // Se inicializa en 0 por defecto

static float g_integral = 0.0;
static float g_last_error = 0.0;

// --- AÑADIDO: Implementación de las nuevas funciones ---
float pid_get_kp(void) { return g_kp; }
float pid_get_ki(void) { return g_ki; }
float pid_get_kd(void) { return g_kd; }

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

// --- AÑADIDO: Implementación de la función de deshabilitación forzada ---
void pid_force_disable(void)
{
    if (g_pid_enabled)
    { // Solo actúa y muestra el mensaje si estaba habilitado
        g_pid_enabled = false;
        // Reseteamos el estado para un futuro arranque limpio
        g_integral = 0.0;
        g_last_error = 0.0;
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
        float error = g_setpoint - current_position;

        // 3. APLICAR BANDA MUERTA
        if (fabs(error) < DEAD_BAND_PULSES)
        {
            error = 0;
        }

        // Acumulamos el error en el término integral.
        // Se multiplica por (PID_LOOP_PERIOD_MS / 1000.0f) para que sea independiente de la frecuencia del bucle.
        g_integral += error * PID_LOOP_PERIOD_MS;

        // Anti-Windup: Limitamos el término integral para que no crezca demasiado.
        if (g_integral > MAX_INTEGRAL)
            g_integral = MAX_INTEGRAL;
        if (g_integral < -MAX_INTEGRAL)
            g_integral = -MAX_INTEGRAL;

        // Si el error es cero, reseteamos el integral para evitar que siga actuando innecesariamente.
        if (error == 0)
            g_integral = 0;

        float i_term = g_ki * g_integral;

        // 4. CALCULAR SALIDA DEL CONTROLADOR
        float p_term = g_kp * error;

        // --- Término Derivativo (D) ---
        // Calcula la "velocidad" del error (cuánto cambió desde el último ciclo)
        float derivative = (error - g_last_error) / PID_LOOP_PERIOD_MS;
        float d_term = g_kd * derivative;

        // Sumamos los términos para obtener la salida final
        float output = p_term + d_term + i_term;

        // 5. SATURAR LA SALIDA
        //if (output > MAX_OUTPUT_PULSES) output = MAX_OUTPUT_PULSES;
        //if (output < -MAX_OUTPUT_PULSES) output = -MAX_OUTPUT_PULSES;

        // --- AÑADIDO: Filtro de Salida (Paso Bajo Simple) ---
        // La nueva salida es un 70% de la salida anterior más un 30% de la nueva calculada.
        // Esto "amortigua" los cambios bruscos. El 0.3 es el "factor de suavizado".
        g_smoothed_output = (0.7f * g_smoothed_output) + (0.3f * output);

        // 5. SATURAR LA SALIDA (ahora sobre la salida suavizada)
        if (g_smoothed_output > MAX_OUTPUT_PULSES) g_smoothed_output = MAX_OUTPUT_PULSES;
        if (g_smoothed_output < -MAX_OUTPUT_PULSES) g_smoothed_output = -MAX_OUTPUT_PULSES;

        // 6. ACTUAR: Si la salida no es cero, enviar comando al motor
        /*if (fabs(output) > 0.1) {
            motor_command_t cmd;
            cmd.num_pulses = (int)fabs(output);

            // Si output es positivo, el péndulo cayó a la izquierda (posición negativa, error positivo).
            // Necesitamos movernos a la izquierda para corregir. Asumimos dir=0 es izquierda.
            // Si output es negativo, el péndulo cayó a la derecha (posición positiva, error negativo).
            // Necesitamos movernos a la derecha. Asumimos dir=1 es derecha.
            // NOTA: Si el motor se mueve en sentido contrario, invierte la lógica aquí (0 : 1)
            cmd.direction = (output > 0) ? 1 : 0;

            cmd.frequency = BASE_FREQUENCY + (int)(fabs(output) * FREQ_PER_ERROR_PULSE);

            // --- CAMBIO CLAVE: Enviar comando a la cola ---
            // Usamos xQueueOverwrite para asegurarnos de que el comando más reciente
            // siempre esté disponible para la tarea del motor. Si el motor está
            // ejecutando un comando antiguo, este lo sobrescribirá.
            xQueueOverwrite(motor_command_queue, &cmd);
        }
        else
        {
            // Si la salida es cero (dentro de la banda muerta o saturada a cero),
            // podríamos enviar un comando para detener el motor si es necesario.
            // Por ejemplo, un comando con 0 pulsos.
            motor_command_t stop_cmd = {.num_pulses = 0, .frequency = 0, .direction = 0};
            xQueueOverwrite(motor_command_queue, &stop_cmd);
        }*/

        if (fabs(g_smoothed_output) > 0.1) {
            int num_pulses = (int)fabs(g_smoothed_output);
            int direction = (g_smoothed_output > 0) ? 1 : 0; 
            int frequency = BASE_FREQUENCY + (int)(fabs(g_smoothed_output) * FREQ_PER_ERROR_PULSE);

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
                    num_pulses = (uint32_t)(frequency * PID_LOOP_PERIOD_MS) / 1000;
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

        g_last_error = error; // Guardamos para el futuro término Derivativo
    }
}

