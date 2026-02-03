// lib/stepperMotor/stepperMotor.h

#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "driver/gpio.h"

/**
 * @brief Estructura de configuración para inicializar el motor a pasos.
 * El usuario debe rellenar esta estructura con los pines que va a utilizar.
 */
typedef struct {
    gpio_num_t step_pin;      // Pin GPIO para los pulsos (STEP/PUL)
    gpio_num_t dir_pin;       // Pin GPIO para la dirección (DIR)
} stepper_config_t;

/**
 * @brief Inicializa el controlador del motor a pasos.
 * 
 * Configura los periféricos de hardware (LEDC/MCPWM) y los pines GPIO necesarios.
 * Debe ser llamada una sola vez antes de cualquier otra función de la librería.
 * 
 * @param config Puntero a la estructura de configuración con los pines a utilizar.
 * @return esp_err_t ESP_OK si la inicialización fue exitosa, o un código de error.
 */
esp_err_t stepper_init(const stepper_config_t *config);

/**
 * @brief Mueve el motor un número específico de pasos a una velocidad dada.
 * 
 * Esta es una función bloqueante. La ejecución no continuará hasta que el
 * movimiento haya terminado.
 * 
 * @param steps El número de micropasos a mover.
 * @param speed_hz La velocidad del movimiento en Hertz (pasos por segundo).
 * @param direction La dirección del movimiento (0 o 1).
 * @return esp_err_t ESP_OK si el comando es válido.
 */
esp_err_t stepper_move(int steps, int speed_hz, int direction);

#endif // STEPPER_MOTOR_H