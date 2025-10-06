#ifndef PWM_GENERATOR_H
#define PWM_GENERATOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "uart_echo.h" // Para incluir la estructura pwm_command_t y la cola
//#include "pwm_generator.h"

// Definiciones para el PWM
#define LEDC_OUTPUT_IO          (GPIO_NUM_32) // Elige el GPIO que quieras para la salida PWM
#define LEDC_DIRECTION_IO       (GPIO_NUM_33) // Pin de salida para la dirección
#define MCPWM_UNIT              MCPWM_UNIT_0
#define MCPWM_TIMER             MCPWM_TIMER_0

typedef struct
{
  int num_pulses;
  int frequency;
  int direction;
} motor_command_t;

extern QueueHandle_t motor_command_queue;

// Declaración de la nueva función de acción. No es una tarea, sino una función normal.
int execute_movement(int num_pulses, int frequency, int direction);

void pwm_init(void);

void motor_control_task(void *arg);

#endif // PWM_GENERATOR_H