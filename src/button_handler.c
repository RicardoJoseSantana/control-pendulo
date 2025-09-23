// src/button_handler.c
#include "button_handler.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "uart_echo.h" // Incluimos para acceder a la cola y la estructura de comando
#include "pid_controller.h"
#include "pwm_generator.h"
#include "system_status.h" // Para manejar el estado del movimiento manual

// --- PINES DE LOS BOTONES ---
#define SEQUENCE_BUTTON_GPIO GPIO_NUM_0
#define ENABLE_PID_BUTTON_GPIO GPIO_NUM_23   // Botón para Habilitar/Deshabilitar PID
#define MANUAL_LEFT_BUTTON_GPIO GPIO_NUM_21  // Nuevo botón para mover a la izquierda
#define MANUAL_RIGHT_BUTTON_GPIO GPIO_NUM_22 // Nuevo botón para mover a la derecha
#define EMERGENCY_STOP_GPIO GPIO_NUM_34      // Botón de parada de emergencia

// --- PARÁMETROS DE MOVIMIENTO MANUAL ---
#define MANUAL_MOVE_SPEED_HZ 10000 // Velocidad constante para el movimiento manual (alta)
#define MANUAL_MOVE_PULSES 400     // Cantidad de pulsos por ciclo (movimiento suave)

// --- PARÁMETROS DE MOVIMIENTO secuencia ---
#define SEQUENCE_BASE_PULSES 16000
#define SEQUENCE_BASE_SPEED_HZ 5000

static const char *TAG = "BUTTON_HANDLER";
// Para saber si el PID está activo, llamaremos a una función.
// extern bool g_pid_enabled;

void button_handler_task(void *arg)
{
    ESP_LOGI(TAG, "Iniciando tarea de lectura de botones...");

    // Configuración de los 3 pines GPIO como entrada con pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = ((1ULL << ENABLE_PID_BUTTON_GPIO) |
                         (1ULL << MANUAL_LEFT_BUTTON_GPIO) |
                         (1ULL << MANUAL_RIGHT_BUTTON_GPIO) |
                         (1ULL << EMERGENCY_STOP_GPIO)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Tarea iniciada. BOOT:Habilitar PID. GPIO21:Izquierda. GPIO22:Derecha.");

    int last_button_state = 1; // 1 = no presionado
    int last_sequence_button_state = 1;
    int last_stop_button_state = 1;

    while (1)
    {
        // --- AÑADIDO: Lógica para la Parada de Emergencia (máxima prioridad) ---
        int current_stop_button_state = gpio_get_level(EMERGENCY_STOP_GPIO);
        if (last_stop_button_state == 1 && current_stop_button_state == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
            if (gpio_get_level(EMERGENCY_STOP_GPIO) == 0)
            {
                // Llama a la función que solo deshabilita
                pid_force_disable();
            }
        }
        last_stop_button_state = current_stop_button_state;

        int current_button_state = gpio_get_level(ENABLE_PID_BUTTON_GPIO);

        // Detectar el flanco de bajada (cuando se presiona el botón)
        if (last_button_state == 1 && current_button_state == 0)
        {
            // Anti-rebote: esperar un poco y confirmar
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(ENABLE_PID_BUTTON_GPIO) == 0)
            {

                pid_toggle_enable();
            }
        }
        last_button_state = current_button_state;

        // Solo permitimos el movimiento manual si el PID está explícitamente deshabilitado
        if (!pid_is_enabled())
        {

            int current_sequence_button_state = gpio_get_level(SEQUENCE_BUTTON_GPIO);
            // Detectar solo el momento en que se presiona el botón (flanco de bajada)
            if (last_sequence_button_state == 1 && current_sequence_button_state == 0)
            {
                vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
                if (gpio_get_level(SEQUENCE_BUTTON_GPIO) == 0)
                {

                    // ESP_LOGW(TAG, "¡Botón de secuencia presionado! Iniciando movimiento...");

                    // --- AÑADIDO: Bucle de la secuencia de movimiento ---
                    // Esta tarea se bloqueará aquí hasta que la secuencia termine.
                    for (int ii = 0; ii < 5; ii++)
                    {
                        int dirr = (ii % 2 == 0) ? 0 : 1; // Alterna la dirección 0, 1, 0, 1, 0
                        int pulses = SEQUENCE_BASE_PULSES - ii * 2000;
                        int frequency = SEQUENCE_BASE_SPEED_HZ + ii * 1000;

                        // ESP_LOGI(TAG, "Secuencia paso %d: Dir=%d, Pulsos=%d, Freq=%d Hz", ii + 1, dirr, pulses, frequency);
                        execute_movement(pulses, frequency, dirr);
                    }

                    // ESP_LOGW(TAG, "Secuencia de movimiento finalizada.");
                }
            }
            last_sequence_button_state = current_sequence_button_state;

            // Leemos el estado de los botones de movimiento manual
            int left_button_state = gpio_get_level(MANUAL_LEFT_BUTTON_GPIO);
            int right_button_state = gpio_get_level(MANUAL_RIGHT_BUTTON_GPIO);

            // Si se presiona el botón izquierdo Y no el derecho
            if (left_button_state == 1 && right_button_state == 0)
            {
                status_set_manual_move_state(MANUAL_MOVE_RIGHT); // Reportar estado
                // Por seguridad, podríamos comprobar si el PID está deshabilitado aquí
                ESP_LOGD(TAG, "Moviendo a la izquierda...");
                // Asumimos que la dirección 0 es izquierda
                execute_movement(MANUAL_MOVE_PULSES, MANUAL_MOVE_SPEED_HZ, 0);
            }
            // Si se presiona el botón derecho Y no el izquierdo
            else if (right_button_state == 1 && left_button_state == 0)
            {
                status_set_manual_move_state(MANUAL_MOVE_LEFT); // Reportar estado
                // Por seguridad, podríamos comprobar si el PID está deshabilitado aquí
                ESP_LOGD(TAG, "Moviendo a la derecha...");
                // Asumimos que la dirección 1 es derecha
                execute_movement(MANUAL_MOVE_PULSES, MANUAL_MOVE_SPEED_HZ, 1);
            }
            else
            {
                // Si no se presiona ningún botón o ambos, detenemos el movimiento
                status_set_manual_move_state(MANUAL_MOVE_NONE); // Reportar estado
                // No hacemos nada. El motor se detendrá al completar los pulsos.
            }
        }
        else
        {
            // Si el PID está activado, nos aseguramos de que el estado manual esté limpio
            status_set_manual_move_state(MANUAL_MOVE_NONE);
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // Sondeo cada 5ms
    }
}