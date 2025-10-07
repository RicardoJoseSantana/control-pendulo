// src/state_controller.h
#ifndef STATE_CONTROLLER_H
#define STATE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

extern volatile int32_t g_car_position_pulses;

void state_controller_task(void *arg);
void state_controller_toggle_enable(void);
bool state_controller_is_enabled(void);
void state_controller_force_disable(void);

// Nueva función para establecer el objetivo de posición del carro en metros
void state_controller_set_cart_setpoint(float position_m);
// Función para establecer el setpoint del ángulo (necesaria para la calibración)
void state_controller_set_theta_setpoint(float setpoint_rad);

#endif // STATE_CONTROLLER_H