// src/pulse_counter.h
#ifndef PULSE_COUNTER_H
#define PULSE_COUNTER_H

#include <stdint.h> // Para usar int16_t

#define ENCODER_RESOLUTION 4096 // 1024 pulsos/rev * 4

/**
 * @brief Obtiene el valor actual del contador del encoder.
 * 
 * Esta función es segura para ser llamada desde cualquier tarea y proporciona la
 * posición angular actual con una resolución de 4096 cuentas por revolución.
 * 
 * @return El valor actual del contador.
 */
int16_t pulse_counter_get_value(void);

/**
 * @brief Tarea que inicializa y gestiona el periférico PCNT para el encoder.
 * 
 * Realiza toda la configuración del hardware. Después se usa para depuración.
 * 
 * @param arg Argumentos pasados a la tarea (no se usa).
 
void pulse_counter_task(void *arg);*/

/**
 * @brief Inicializa el hardware del contador de pulsos (PCNT y GPIOs).
 * 
 * Debe ser llamada una sola vez al inicio del programa.
 */
void pulse_counter_init(void);

#endif // PULSE_COUNTER_H