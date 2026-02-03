// src/pulse_counter.c
#include "pulse_counter.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "esp_log.h"

// --- CONFIGURACIÓN DE PINES (Tus pines específicos) ---
#define PCNT_INPUT_A_PIN    14 // Fase A del encoder
#define PCNT_INPUT_B_PIN    12 // Fase B del encoder
#define ENCODER_INDEX_Z_PIN 13 // Fase Z (índice) del encoder

#define PCNT_UNIT           PCNT_UNIT_0
//static const char *TAG = "PULSE_COUNTER_PCNT";

/**
 * @brief ISR para la señal Z. Resetea el contador del PCNT a cero.
 */
static void IRAM_ATTR encoder_index_z_isr_handler(void* arg) {
    pcnt_counter_clear(PCNT_UNIT);
}

/**
 * @brief Inicializa el PCNT en modo cuadratura y la interrupción de la señal Z.
 */
void pulse_counter_init(void) {
    // Configuración para el Canal 0 (Fase A como pulso, Fase B como dirección)
    pcnt_config_t pcnt_config_ch0 = {
        .pulse_gpio_num = PCNT_INPUT_A_PIN,
        .ctrl_gpio_num = PCNT_INPUT_B_PIN,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
    };
    pcnt_unit_config(&pcnt_config_ch0);

    // Configuración para el Canal 1 (Fase B como pulso, Fase A como dirección)
    pcnt_config_t pcnt_config_ch1 = {
        .pulse_gpio_num = PCNT_INPUT_B_PIN,
        .ctrl_gpio_num = PCNT_INPUT_A_PIN,
        .channel = PCNT_CHANNEL_1,
        .unit = PCNT_UNIT,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
    };
    pcnt_unit_config(&pcnt_config_ch1);

    // Configurar pull-up para los pines de entrada
    gpio_set_pull_mode(PCNT_INPUT_A_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PCNT_INPUT_B_PIN, GPIO_PULLUP_ONLY);

    // Habilitar filtro de ruido de hardware
    pcnt_set_filter_value(PCNT_UNIT, 100);
    pcnt_filter_enable(PCNT_UNIT);
    
    // Iniciar el contador
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);

    // Configurar la interrupción para la señal Z
    gpio_config_t z_pin_config = {
        .pin_bit_mask = (1ULL << ENCODER_INDEX_Z_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&z_pin_config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_INDEX_Z_PIN, encoder_index_z_isr_handler, NULL);

    //ESP_LOGI(TAG, "PCNT en modo cuadratura (pines A:%d, B:%d) y señal Z (pin:%d) inicializados.", 
    //         PCNT_INPUT_A_PIN, PCNT_INPUT_B_PIN, ENCODER_INDEX_Z_PIN);
}

/**
 * @brief Obtiene el valor actual del contador del PCNT.
 */
int16_t pulse_counter_get_value(void) {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT, &count);
    return count;
}
