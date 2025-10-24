// src/main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "encoder.h" // ¡Solo incluimos nuestra nueva librería!

// --- CONFIGURACIÓN DEL USUARIO ---
#define ENCODER_PIN_A 27
#define ENCODER_PIN_B 14
#define ENCODER_PIN_Z 12
#define ENCODER_PULSES_PER_REV 1024

static const char *TAG = "MAIN_APP";

void app_main(void) {
    // 1. Rellenar la estructura de configuración
    encoder_config_t encoder_cfg = {
        .pin_a = ENCODER_PIN_A,
        .pin_b = ENCODER_PIN_B,
        .pin_z = ENCODER_PIN_Z,
        .pulses_per_rev = ENCODER_PULSES_PER_REV
    };

    // 2. Inicializar la librería
    esp_err_t ret = encoder_init(&encoder_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al inicializar el encoder, deteniendo.");
        return;
    }

    // 3. Bucle principal para mostrar los datos
    while (1) {
        // Obtener el ángulo actual usando la función de la librería
        float angle = encoder_get_angle_degrees();
        
        // Imprimir el ángulo
        ESP_LOGI(TAG, "Ángulo del Encoder: %.2f grados", angle);

        // Comprobar si se ha detectado el pulso Z
        if (encoder_was_z_pulse_detected()) {
            ESP_LOGW(TAG, "¡Pulso de ÍNDICE (Z) detectado! El contador se ha reseteado.");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Mostrar datos 1 vez por segundo
    }
}