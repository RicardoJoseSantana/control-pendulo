// src/position_controller.h
#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "freertos/FreeRTOS.h"

// Variable externa para la velocidad de corrección de posición. El PID de ángulo la leerá.
extern volatile float g_position_correction_velocity;

// Tarea del controlador de posición
void position_controller_task(void *arg);

// Funciones para sintonizar el PID de posición desde el UART
void pos_pid_set_kp(float kp);
void pos_pid_set_ki(float ki);
void pos_pid_set_kd(float kd);

#endif // POSITION_CONTROLLER_H