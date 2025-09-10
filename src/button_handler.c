// src/button_handler.c
#include "button_handler.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "uart_echo.h" // Incluimos para acceder a la cola y la estructura de comando
#include "pid_controller.h"

#define BOOT_BUTTON_GPIO GPIO_NUM_0
static const char *TAG = "BUTTON_HANDLER";

void button_handler_task(void *arg) {
    ESP_LOGI(TAG, "Iniciando tarea de lectura del botón BOOT (GPIO 0)...");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Tarea iniciada. Presiona BOOT para repetir el último comando.");

    int last_button_state = 1; // 1 = no presionado

    while (1) {
        int current_button_state = gpio_get_level(BOOT_BUTTON_GPIO);
        
        // Detectar el flanco de bajada (cuando se presiona el botón)
        if (last_button_state == 1 && current_button_state == 0) {
            // Anti-rebote: esperar un poco y confirmar
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
                /*ESP_LOGI(TAG, "¡Botón BOOT presionado! Enviando comando de repetición...");

                // --- AÑADIDO: Lógica de envío de comando especial ---
                // Creamos un comando especial donde 'num_pulses' es -1.
                // Esto le indicará a la tarea PWM que debe repetir el último comando.
                pwm_command_t repeat_command;
                repeat_command.num_pulses = -1; // Valor centinela
                
                if (xQueueSend(pwm_command_queue, &repeat_command, pdMS_TO_TICKS(100)) != pdPASS) {
                    ESP_LOGE(TAG, "No se pudo enviar el comando de repetición a la cola.");
                } else {
                    ESP_LOGI(TAG, "Comando de repetición enviado.");
                }*/
               pid_toggle_enable();
            }
        }
        
        last_button_state = current_button_state;
        vTaskDelay(pdMS_TO_TICKS(20)); // Sondeo cada 20ms
    }
}