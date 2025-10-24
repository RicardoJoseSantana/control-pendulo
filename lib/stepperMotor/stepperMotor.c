// lib/stepperMotor/stepperMotor.c

#include "stepperMotor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"

// --- Definiciones privadas de la librería ---
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_1_BIT // Óptimo para alta frecuencia

static const char *TAG = "STEPPER_MOTOR";
static stepper_config_t motor_config; // Variable global para guardar los pines

esp_err_t stepper_init(const stepper_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "La configuración no puede ser NULL");
        return ESP_ERR_INVALID_ARG;
    }
    motor_config = *config; // Copiar la configuración del usuario

    ESP_LOGI(TAG, "Inicializando motor en STEP pin %d, DIR pin %d", motor_config.step_pin, motor_config.dir_pin);

    // Configuración del temporizador LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = 10000, // Frecuencia inicial minima
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al configurar el temporizador LEDC: %s", esp_err_to_name(err));
        return err;
    }

    // Configuración del canal PWM (STEP pin)
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = motor_config.step_pin,
        .duty           = 0,
        .hpoint         = 0
    };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al configurar el canal LEDC: %s", esp_err_to_name(err));
        return err;
    }

    // Configuración del pin de dirección (DIR pin)
    gpio_config_t dir_gpio_config = {
        .pin_bit_mask = (1ULL << motor_config.dir_pin),
        .mode = GPIO_MODE_OUTPUT
    };
    err = gpio_config(&dir_gpio_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al configurar el pin de dirección: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Librería Stepper Motor inicializada correctamente.");
    return ESP_OK;
}

esp_err_t stepper_move(int steps, int speed_hz, int direction) {
    if (steps <= 0 || speed_hz <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 1. Establecer la dirección
    gpio_set_level(motor_config.dir_pin, direction);
    esp_rom_delay_us(10); // Pequeña espera para que el driver asiente la dirección

    // 2. Ajustar la frecuencia del PWM
    esp_err_t err = ledc_set_freq(LEDC_MODE, LEDC_TIMER, speed_hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al establecer la frecuencia a %d Hz: %s", speed_hz, esp_err_to_name(err));
        return err;
    }

    // 3. Calcular la duración
    uint32_t duration_ms = (uint32_t)(steps * 1000) / speed_hz;
    if (duration_ms == 0) duration_ms = 1; // Mover al menos por un tick

    // 4. Iniciar la generación de pulsos (50% duty cycle)
    const uint32_t duty = (1 << LEDC_DUTY_RES) / 2;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    
    // 5. Esperar el tiempo calculado
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // 6. Detener la generación de pulsos
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    
    // Opcional: poner la dirección en un estado por defecto
    gpio_set_level(motor_config.dir_pin, 0);

    return ESP_OK;
}