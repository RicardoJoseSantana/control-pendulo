// lib/encoder/encoder.h
#ifndef ENCODER_H
#define ENCODER_H

#include "driver/gpio.h"
#include <stdbool.h>

/**
 * @brief Estructura de configuración para inicializar el encoder.
 */
typedef struct {
    gpio_num_t pin_a;             // Pin GPIO para la Fase A
    gpio_num_t pin_b;             // Pin GPIO para la Fase B
    gpio_num_t pin_z;             // Pin GPIO para la Fase Z (índice), usar GPIO_NUM_NC si no se usa
    uint16_t pulses_per_rev;    // Pulsos por revolución del encoder (ej. 1024)
} encoder_config_t;

/**
 * @brief Inicializa el controlador del encoder.
 * 
 * Configura el periférico PCNT y las interrupciones de GPIO.
 * Debe ser llamada una sola vez.
 * 
 * @param config Puntero a la estructura de configuración.
 * @return esp_err_t ESP_OK si la inicialización fue exitosa.
 */
esp_err_t encoder_init(const encoder_config_t *config);

/**
 * @brief Obtiene el valor crudo actual del contador del encoder.
 * 
 * El rango dependerá del contador de 16 bits del PCNT.
 * 
 * @return El valor actual del contador.
 */
int16_t encoder_get_counts(void);

/**
 * @brief Obtiene el ángulo actual del encoder en grados.
 * 
 * Calcula el ángulo basado en la resolución y el conteo actual.
 * 
 * @return El ángulo en grados (-180.0 a 180.0).
 */
float encoder_get_angle_degrees(void);

/**
 * @brief Comprueba si se ha detectado un pulso de índice (Z) desde la última vez que se llamó.
 * 
 * Esta función es no bloqueante y resetea el flag al ser leída.
 * 
 * @return true si se detectó un pulso Z, false si no.
 */
bool encoder_was_z_pulse_detected(void);

#endif // ENCODER_H