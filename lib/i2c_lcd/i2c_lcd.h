#ifndef I2C_LCD_H
#define I2C_LCD_H

#include <stdint.h> // Para usar tipos como uint8_t

/**
 * @brief Inicializa el hardware y la librería de la pantalla LCD.
 *
 * Debe ser llamada una vez al inicio del programa.
 */
void lcd_display_task(void *arg);

void lcd_init(void);

/**
 * @brief Limpia toda la pantalla y posiciona el cursor en (0, 0).
 */
void lcd_clear(void);

/**
 * @brief Posiciona el cursor en una columna y fila específicas.
 *
 * @param col La columna (0-19).
 * @param row La fila (0-3).
 */
void lcd_set_cursor(uint8_t col, uint8_t row);

/**
 * @brief Escribe una cadena de texto en la posición actual del cursor.
 *
 * @param str La cadena de texto a escribir.
 */
void lcd_write_string(const char *str);

/**
 * @brief Escribe una cadena formateada en una línea completa, rellenando con espacios.
 *
 * Se comporta como printf.
 * @param row La fila donde escribir (0-3).
 * @param format La cadena de formato (ej. "Contador: %d").
 * @param ... Los argumentos para la cadena de formato.
 */
void lcd_printf_line(uint8_t row, const char *format, ...);

// Declaración de la tarea de la visualizacion en pantalla
void lcd_display_task(void *pvParameters);

#endif 