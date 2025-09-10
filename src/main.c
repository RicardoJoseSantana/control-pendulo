// src/main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 1. Inclusión de todas las cabeceras de los módulos
#include "nvs_flash.h"      // Para la memoria no volátil
#include "uart_echo.h"      // Para la tarea de comandos UART
#include "pwm_generator.h"  // Para la tarea de control del motor
#include "pulse_counter.h"  // Para la tarea de lectura del encoder
#include "button_handler.h" // Para la tarea de lectura del botón
#include "pid_controller.h"

void app_main(void) {
    // 2. Inicialización de servicios globales (primero que nada)
    // Es crucial inicializar la partición de la NVS antes de que cualquier tarea intente usarla.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Configuración del temporizador
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        //.freq_hz          = LEDC_FREQUENCY, // Frecuencia de 1 kHz
        .freq_hz          = 1000, // Una frecuencia inicial por defecto
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configuración del canal PWM
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Inicia con el PWM apagado
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // 3. Configuración del pin de dirección como salida
    gpio_config_t dir_gpio_config = {
        .pin_bit_mask = (1ULL << LEDC_DIRECTION_IO), // <-- Usa la definición del .h (GPIO 33)
        .mode = GPIO_MODE_OUTPUT
        //.pull_up_en = GPIO_PULLUP_DISABLE,
        //.pull_down_en = GPIO_PULLDOWN_DISABLE,
        //.intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&dir_gpio_config));

    // 3. Creación de todas las tareas de la aplicación
    // Cada tarea se ejecutará de forma independiente y concurrente.

    // Le damos una prioridad más alta (6) para asegurar que responda rápidamente.
    xTaskCreate(pid_controller_task, "pid_controller_task", configMINIMAL_STACK_SIZE * 4, NULL, 6, NULL);

    // Tarea para manejar los comandos recibidos por el puerto serie
    xTaskCreate(uart_echo_task, "uart_echo_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    // Tarea que espera comandos en la cola y mueve el motor
    //xTaskCreate(pwm_generator_task, "pwm_generator_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    // Tarea que inicializa el PCNT y reporta la posición del encoder para depuración
    xTaskCreate(pulse_counter_task, "pulse_counter_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    // Tarea que monitorea el botón BOOT y envía comandos de "repetir"
    xTaskCreate(button_handler_task, "button_handler_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}