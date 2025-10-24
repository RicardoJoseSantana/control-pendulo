// src/button_handler.c
#include "button_handler.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "system_status.h" // Para manejar el estado del movimiento manual

// --- PINES DE LOS BOTONES ---
#define VIEW_CYCLE_BUTTON_GPIO GPIO_NUM_19 // Cambio de vistas en la pantalla LCD
#define ENABLE_PID_BUTTON_GPIO GPIO_NUM_18    // Botón para Habilitar/Deshabilitar PID
#define MANUAL_LEFT_BUTTON_GPIO GPIO_NUM_16   // Nuevo botón para mover a la izquierda
#define MANUAL_RIGHT_BUTTON_GPIO GPIO_NUM_17  // Nuevo botón para mover a la derecha

static const char *TAG = "BUTTON_HANDLER";

// --- ¡SIMULACIÓN DEL ESTADO DEL PID! ---
// Esta variable reemplaza la necesidad del módulo PID completo.
static volatile bool g_is_pid_active = false;

// Implementación de la función pública
bool button_handler_is_pid_active(void) {
    return g_is_pid_active;
}

void button_handler_task(void *arg)
{
    ESP_LOGI(TAG, "Iniciando tarea de lectura de botones...");

    // Configuración de los 3 pines GPIO como entrada con pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = ((1ULL << ENABLE_PID_BUTTON_GPIO) |
                         (1ULL << MANUAL_LEFT_BUTTON_GPIO) |
                         (1ULL << MANUAL_RIGHT_BUTTON_GPIO) |
                         (1ULL << VIEW_CYCLE_BUTTON_GPIO)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Tarea iniciada. BOOT:Habilitar PID. GPIO21:Izquierda. GPIO22:Derecha.");

    int last_button_state = 1; // 1 = no presionado
    int last_view_button_state = 1;

    while (1)
    {
        // --- Lógica para el botón de cambio de vista ---
        int current_view_button_state = gpio_get_level(VIEW_CYCLE_BUTTON_GPIO);
        if (last_view_button_state == 1 && current_view_button_state == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
            if (gpio_get_level(VIEW_CYCLE_BUTTON_GPIO) == 0)
            {
                ESP_LOGI(TAG, "Botón de cambio de vista presionado.");
                status_cycle_lcd_view(); // ¡Llamamos a nuestra nueva función!
            }
        }
        last_view_button_state = current_view_button_state;

        // --- Lógica para el botón de habilitar/deshabilitar PID (Simulada)---
        int current_button_state = gpio_get_level(ENABLE_PID_BUTTON_GPIO);

        // Detectar el flanco de bajada (cuando se presiona el botón)
        if (last_button_state == 1 && current_button_state == 0)
        {
            // Anti-rebote: esperar un poco y confirmar
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(ENABLE_PID_BUTTON_GPIO) == 0)
            {
                g_is_pid_active = !g_is_pid_active; // Cambiar el estado simulado del PID
                ESP_LOGI(TAG, "Botón de Habilitar/Deshabilitar PID presionado. Nuevo estado PID: %s",
                         g_is_pid_active ? "HABILITADO" : "DESHABILITADO");
            }
        }
        last_button_state = current_button_state;

        // Leemos el estado de los botones de movimiento manual
            int left_button_state = gpio_get_level(MANUAL_LEFT_BUTTON_GPIO);
            int right_button_state = gpio_get_level(MANUAL_RIGHT_BUTTON_GPIO);

        // Solo permitimos el movimiento manual si el PID está explícitamente deshabilitado
        if (!g_is_pid_active)
        {


            

            // Si se presiona el botón izquierdo Y no el derecho
            if (left_button_state == 1 && right_button_state == 0)
            {
                status_set_manual_move_state(MANUAL_MOVE_RIGHT); // Reportar estado
                ESP_LOGD(TAG, "Moviendo a la izquierda...");
                // Asumimos que la dirección 0 es izquierda
            }
            // Si se presiona el botón derecho Y no el izquierdo
            else if (right_button_state == 1 && left_button_state == 0)
            {
                status_set_manual_move_state(MANUAL_MOVE_LEFT); // Reportar estado
                ESP_LOGD(TAG, "Moviendo a la derecha...");
                // Asumimos que la dirección 1 es derecha
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

        vTaskDelay(pdMS_TO_TICKS(20)); // Sondeo mayor o igual a 10ms 10ms
    }
}