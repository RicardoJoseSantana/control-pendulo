// src/system_status.c

// src/system_status.c

#include "system_status.h"

// La variable de estado. 'volatile' es importante para que el compilador
// siempre la lea de la memoria, ya que puede ser modificada por otra tarea.
static volatile manual_move_state_t g_manual_move_state = MANUAL_MOVE_NONE;

void status_set_manual_move_state(manual_move_state_t state)
{
    g_manual_move_state = state;
}

manual_move_state_t status_get_manual_move_state(void)
{
    return g_manual_move_state;
}