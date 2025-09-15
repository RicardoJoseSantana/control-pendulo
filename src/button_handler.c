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

// --- PINES DE LOS BOTONES ---
#define ENABLE_PID_BUTTON_GPIO  GPIO_NUM_23  // Botón BOOT para Habilitar/Deshabilitar PID
#define MANUAL_LEFT_BUTTON_GPIO GPIO_NUM_21 // Nuevo botón para mover a la izquierda
#define MANUAL_RIGHT_BUTTON_GPIO GPIO_NUM_22 // Nuevo botón para mover a la derecha

// --- PARÁMETROS DE MOVIMIENTO MANUAL ---
#define MANUAL_MOVE_SPEED_HZ 5000 // Velocidad constante para el movimiento manual (alta)
#define MANUAL_MOVE_PULSES   200   // Cantidad de pulsos por ciclo (movimiento suave)

static const char *TAG = "BUTTON_HANDLER";
// Para saber si el PID está activo, llamaremos a una función.
//extern bool g_pid_enabled;

void button_handler_task(void *arg) {
    ESP_LOGI(TAG, "Iniciando tarea de lectura de botones...");

    // Configuración de los 3 pines GPIO como entrada con pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = ((1ULL << ENABLE_PID_BUTTON_GPIO) | 
                         (1ULL << MANUAL_LEFT_BUTTON_GPIO) | 
                         (1ULL << MANUAL_RIGHT_BUTTON_GPIO)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Tarea iniciada. BOOT:Habilitar PID. GPIO21:Izquierda. GPIO22:Derecha.");

    int last_button_state = 1; // 1 = no presionado

    while (1) {
        int current_button_state = gpio_get_level(ENABLE_PID_BUTTON_GPIO);
        
        // Detectar el flanco de bajada (cuando se presiona el botón)
        if (last_button_state == 1 && current_button_state == 0) {
            // Anti-rebote: esperar un poco y confirmar
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(ENABLE_PID_BUTTON_GPIO) == 0) {
                
               pid_toggle_enable();
            }
        }
        last_button_state = current_button_state;


        // Solo permitimos el movimiento manual si el PID está explícitamente deshabilitado
        if (!pid_is_enabled()) {
        // Leemos el estado de los botones de movimiento manual
        int left_button_state = gpio_get_level(MANUAL_LEFT_BUTTON_GPIO);
        int right_button_state = gpio_get_level(MANUAL_RIGHT_BUTTON_GPIO);

        // Si se presiona el botón izquierdo Y no el derecho
        if (left_button_state == 0 && right_button_state == 1) {
             // Por seguridad, podríamos comprobar si el PID está deshabilitado aquí
            ESP_LOGD(TAG, "Moviendo a la izquierda...");
            // Asumimos que la dirección 0 es izquierda
            execute_movement(MANUAL_MOVE_PULSES, MANUAL_MOVE_SPEED_HZ, 0);
        }
        // Si se presiona el botón derecho Y no el izquierdo
        else if (right_button_state == 0 && left_button_state == 1) {
            // Por seguridad, podríamos comprobar si el PID está deshabilitado aquí
            ESP_LOGD(TAG, "Moviendo a la derecha...");
            // Asumimos que la dirección 1 es derecha
            execute_movement(MANUAL_MOVE_PULSES, MANUAL_MOVE_SPEED_HZ, 1);
        }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Sondeo cada 20ms
    }
}