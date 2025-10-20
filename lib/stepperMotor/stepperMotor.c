#include "stepperMotor.h"
#include "driver/gpio.h" // Necesario para controlar el pin de dirección
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PWM_GENERATOR";

// Configuración del LEDC (PWM)
void ledc_init(void) {
    // Configuración del temporizador
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        //.freq_hz          = LEDC_FREQUENCY, // Frecuencia de 1 kHz
        .freq_hz          = 100, // Una frecuencia inicial por defecto
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

    //ESP_LOGI(TAG, "LEDC configurado en GPIO %d a %d Hz", LEDC_OUTPUT_IO, LEDC_FREQUENCY);
    //ESP_LOGI(TAG, "LEDC configurado: Pin Pulsos GPIO %d, Pin Dir GPIO %d, a %d Hz", LEDC_OUTPUT_IO, LEDC_DIRECTION_IO, LEDC_FREQUENCY);
    ESP_LOGI(TAG, "Módulo PWM y Dirección inicializado.");
}

// --- FUNCIÓN DE ACCIÓN NUEVA Y CENTRAL ---
void execute_movement(int num_pulses, int frequency, int direction) {
    ESP_LOGI(TAG, "Ejecutando movimiento: %d pulsos a %d Hz, Dir: %d", num_pulses, frequency, direction);

    // 1. Establecer la dirección
    gpio_set_level(LEDC_DIRECTION_IO, direction);

    // 2. Ajustar la frecuencia del PWM dinámicamente
    ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, LEDC_TIMER, frequency));

    // 3. Calcular la duración del movimiento en milisegundos
    // duration_ms = (num_pulses / frequency_Hz) * 1000_ms_per_s
    uint32_t duration_ms = (uint32_t)(num_pulses * 1000) / frequency;
    ESP_LOGI(TAG, "Duración calculada: %lu ms", duration_ms);
    
    // 4. Iniciar la generación de pulsos (50% duty cycle)
    int duty_cycle = (1 << LEDC_DUTY_RES) / 2;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    
    // 5. Esperar el tiempo calculado
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    //if (duration_ms > 0) esp_rom_delay_us(duration_ms * 1000);

    // 6. Detener la generación de pulsos
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    gpio_set_level(LEDC_DIRECTION_IO, 0);
    
    ESP_LOGI(TAG, "Movimiento finalizado.");
}