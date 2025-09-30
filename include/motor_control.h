#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Define la estructura del comando UNA SOLA VEZ aquí.
typedef struct
{
    int num_pulses;
    int frequency;
    int direction;
} motor_command_t;

// Declara la cola como 'extern' para que todos los módulos sepan que existe.
extern QueueHandle_t motor_command_queue;

#endif // MOTOR_CONTROL_H