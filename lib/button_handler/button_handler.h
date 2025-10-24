// src/button_handler.h
#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

/**
 * @brief Tarea que inicializa y monitorea el botón BOOT (GPIO 0).
 * 
 * Al detectar una pulsación, envía un comando para que se repita
 * la última acción de movimiento.
 * @param arg Argumentos de la tarea (no se usan).
 */
void button_handler_task(void *arg);

#endif // BUTTON_HANDLER_H