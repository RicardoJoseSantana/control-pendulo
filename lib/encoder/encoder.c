// lib/encoder/encoder.c
#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include "esp_err.h"

#define PCNT_UNIT PCNT_UNIT_0
static const char *TAG = "ENCODER_LIB";

// Variables estáticas de la librería
static encoder_config_t g_config;
static float g_degrees_per_count;
static volatile bool g_z_pulse_flag = false;

// ISR para la señal Z
static void IRAM_ATTR encoder_z_isr_handler(void* arg) {
    pcnt_counter_clear(PCNT_UNIT);
    g_z_pulse_flag = true;
}

esp_err_t encoder_init(const encoder_config_t *config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    g_config = *config;

    // Calcular el factor de conversión una sola vez
    g_degrees_per_count = 360.0f / (float)(g_config.pulses_per_rev * 4);

    // Configuración del Canal 0 (A y B)
    pcnt_config_t pcnt_config_ch0 = {
        .pulse_gpio_num = g_config.pin_a,
        .ctrl_gpio_num = g_config.pin_b,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
    };
    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config_ch0));

    // Configuración del Canal 1 (B y A)
    pcnt_config_t pcnt_config_ch1 = {
        .pulse_gpio_num = g_config.pin_b,
        .ctrl_gpio_num = g_config.pin_a,
        .channel = PCNT_CHANNEL_1,
        .unit = PCNT_UNIT,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
    };
    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config_ch1));

    gpio_set_pull_mode(g_config.pin_a, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(g_config.pin_b, GPIO_PULLUP_ONLY);

    pcnt_set_filter_value(PCNT_UNIT, 100);
    pcnt_filter_enable(PCNT_UNIT);
    
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);

    // Configurar la interrupción Z solo si se ha proporcionado un pin válido
    if (g_config.pin_z != GPIO_NUM_NC) {
        gpio_config_t z_pin_config = {
            .pin_bit_mask = (1ULL << g_config.pin_z),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .intr_type = GPIO_INTR_POSEDGE
        };
        gpio_config(&z_pin_config);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(g_config.pin_z, encoder_z_isr_handler, NULL);
    }

    ESP_LOGI(TAG, "Librería Encoder inicializada en pines A:%d, B:%d, Z:%d", 
             g_config.pin_a, g_config.pin_b, g_config.pin_z);
    return ESP_OK;
}

int16_t encoder_get_counts(void) {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT, &count);
    return count;
}

float encoder_get_angle_degrees(void) {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT, &count);
    return (float)count * g_degrees_per_count;
}

bool encoder_was_z_pulse_detected(void) {
    if (g_z_pulse_flag) {
        g_z_pulse_flag = false; // Resetear el flag al leerlo
        return true;
    }
    return false;
}