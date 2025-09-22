// include/system_status.h

#ifndef SYSTEM_STATUS_H
#define SYSTEM_STATUS_H

#include <stdint.h>

// Definimos los posibles estados de movimiento manual
typedef enum {
    MANUAL_MOVE_NONE,
    MANUAL_MOVE_LEFT,
    MANUAL_MOVE_RIGHT
} manual_move_state_t;

/**
 * @brief Establece el estado actual del movimiento manual.
 * 
 * Esta función es segura para ser llamada desde cualquier tarea (es "thread-safe"
 * porque la operación es atómica en una variable volátil).
 * 
 * @param state El nuevo estado de movimiento (NONE, LEFT, o RIGHT).
 */
void status_set_manual_move_state(manual_move_state_t state);

/**
 * @brief Obtiene el estado actual del movimiento manual.
 * 
 * @return El estado de movimiento actual.
 */
manual_move_state_t status_get_manual_move_state(void);

#endif // SYSTEM_STATUS_H