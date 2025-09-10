// src/pid_controller.h
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>

/**
 * @brief Tarea principal del controlador PID.
 * Se ejecuta a una frecuencia fija para leer el sensor y controlar el motor.
 */
void pid_controller_task(void *arg);

/**
 * @brief Habilita o deshabilita el bucle de control del PID.
 * Función segura para ser llamada desde otras tareas (ej. el manejador del botón).
 */
void pid_toggle_enable(void);

/**
 * @brief Establece el valor de la ganancia Proporcional (Kp).
 */
void pid_set_kp(float kp);

/**
 * @brief Establece el valor de la ganancia Integral (Ki). (Para uso futuro)
 */
void pid_set_ki(float ki);

/**
 * @brief Establece el valor de la ganancia Derivativa (Kd). (Para uso futuro)
 */
void pid_set_kd(float kd);

#endif // PID_CONTROLLER_H