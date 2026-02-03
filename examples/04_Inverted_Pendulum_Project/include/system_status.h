// include/system_status.h

#ifndef SYSTEM_STATUS_H
#define SYSTEM_STATUS_H

#include <stdint.h>

// Definimos los posibles estados de movimiento manual
typedef enum
{
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

// --- AÑADIDO: ESTADO DE LA VISTA DE LA PANTALLA ---
typedef enum
{
    VIEW_MAIN_STATUS,    // Vista principal: Estado PID y Posición en Grados
    VIEW_POSITION, // Vista secundaria: Posición del carro en cm
    VIEW_PID_GAINS,      // Vista terciaria: Valores de las ganancias Kp, Ki, Kd
    VIEW_COUNT,           // ¡Importante! Siempre al final, para saber cuántas vistas hay
    VIEW_CALIBRATION //++
} lcd_view_state_t;

extern volatile lcd_view_state_t g_lcd_view_state;

/**
 * @brief Cambia a la siguiente vista de la pantalla.
 *
 * Rota cíclicamente entre las vistas disponibles.
 */
void status_cycle_lcd_view(void);

/**
 * @brief Obtiene la vista actual de la pantalla.
 *
 * @return La vista que debe ser mostrada.
 */
lcd_view_state_t status_get_lcd_view(void);

#endif // SYSTEM_STATUS_H