#ifndef PWM_GENERATOR_H
#define PWM_GENERATOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h" // libreria para control de motor
#include "esp_log.h"
#include "uart_echo.h" // Para incluir la estructura motor_command_t y la cola
//#include "motor_generator.h"

// Definiciones para el control del motor

#define STEP_PIN                (GPIO_NUM_32) // Elige el GPIO que quieras para la salida PWM
#define DIR_PIN                 (GPIO_NUM_33) // Pin de salida para la dirección
#define MCPWM_UNIT              MCPWM_UNIT_0  // unidad
#define MCPWM_TIMER             MCPWM_TIMER_0 // temporizador

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