// src/pid_controller.h
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

// --- Declaración externa de la posición del carro ---
extern volatile int32_t g_car_position_pulses;

/**
 * @brief Tarea principal del controlador de ecuaciones de estado.
 * Se ejecuta a una frecuencia fija para leer el sensor y controlar el motor.
 */
void state_controller_task(void *arg);
// Habilita/Deshabilita el control
void state_controller_toggle_enable(void);
// Devuelve si el control está activo
bool state_controller_is_enabled(void);
// Establece el setpoint del ángulo (calculado externamente)
void state_controller_set_theta_setpoint(float setpoint_rad);

/**
 * @brief Deshabilita forzosamente el controlador de estado y detiene el motor.
 */
void state_controller_force_disable(void);

#endif // PID_CONTROLLER_H