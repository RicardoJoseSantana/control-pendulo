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

// --- Lógica para el estado de la vista de la pantalla ---
volatile lcd_view_state_t g_lcd_view_state = VIEW_MAIN_STATUS;

void status_cycle_lcd_view(void)
{
    // Convertimos el enum a un entero para hacer la suma
    int next_view_int = (int)g_lcd_view_state + 1;

    // Si hemos pasado la última vista, volvemos a la primera (0)
    if (next_view_int >= VIEW_COUNT)
    {
        next_view_int = 0;
    }

    // Convertimos de nuevo al tipo enum y lo guardamos
    g_lcd_view_state = (lcd_view_state_t)next_view_int;
}

lcd_view_state_t status_get_lcd_view(void)
{
    return g_lcd_view_state;
}