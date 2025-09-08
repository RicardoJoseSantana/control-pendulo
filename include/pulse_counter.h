// src/pulse_counter.h

#ifndef PULSE_COUNTER_H
#define PULSE_COUNTER_H

#include "freertos/FreeRTOS.h"
#include <stdbool.h> // --- AÑADIDO --- Para poder usar el tipo 'bool'

/**
 * @brief Variable global para almacenar el conteo de pulsos.
 *
 * Se declara como 'extern' para que otros archivos puedan acceder a ella.
 * Se declara como 'volatile' porque es modificada por una rutina de interrupción (ISR)
 * y leída por otra tarea, evitando que el compilador haga optimizaciones incorrectas.
 * Ahora representa la posición: se incrementa en una dirección y decrementa en la otra.
 */
extern volatile int g_pulse_count;

/**
 * @brief Tarea de FreeRTOS que inicializa y gestiona el conteo de pulsos.
 *
 * Esta tarea configura el GPIO y las interrupciones, y luego entra en un bucle
 * para procesar o mostrar el conteo de pulsos periódicamente.
 *
 * @param arg Argumentos pasados a la tarea (no se usa en este caso).
 */

extern volatile bool g_z_pulse_detected;

/**
 * @brief --- AÑADIDO --- Flag para indicar que el pulso Z (índice) ha sido detectado.
 *
 * Es 'volatile' porque se escribe en una ISR y se lee en la tarea principal.
 */

void pulse_counter_task(void *arg);

#endif // PULSE_COUNTER_H