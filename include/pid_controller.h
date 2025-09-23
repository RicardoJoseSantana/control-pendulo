// src/pid_controller.h
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Tarea principal del controlador PID.
 * Se ejecuta a una frecuencia fija para leer el sensor y controlar el motor.
 */
void pid_controller_task(void *arg);
void motor_control_task(void *arg);

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

/**
 * @brief Devuelve el punto de consigna (setpoint) actual del controlador.
 * @return El setpoint actual en cuentas del encoder.
 */
int16_t pid_get_setpoint(void);

/**
 * @brief Devuelve si el bucle de control del PID está actualmente habilitado.
 * @return true si está habilitado, false si no.
 */
bool pid_is_enabled(void);

// para parada de emergencia.

void pid_force_disable(void);

// --- AÑADIDO: Funciones para obtener los valores de las ganancias ---
float pid_get_kp(void);
float pid_get_ki(void);
float pid_get_kd(void);

#endif // PID_CONTROLLER_H