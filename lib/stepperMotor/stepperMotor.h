#ifndef PWM_GENERATOR_H
#define PWM_GENERATOR_H

#include "driver/ledc.h" // Módulo LEDC para PWM
#include "esp_log.h"

// Definiciones para el PWM
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (GPIO_NUM_32) // Elige el GPIO que quieras para la salida PWM
#define LEDC_DIRECTION_IO       (GPIO_NUM_33) // Pin de salida para la dirección
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_1_BIT

void ledc_init(void);

// Declaración de la nueva función de acción. No es una tarea, sino una función normal.
void execute_movement(int num_pulses, int frequency, int direction);

#endif // PWM_GENERATOR_H