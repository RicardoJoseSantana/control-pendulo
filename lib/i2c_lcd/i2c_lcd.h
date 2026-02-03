#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "esp_err.h"
#include <stdint.h>

// --- CONFIGURACIÓN DEL USUARIO (Hardware) ---
#define LCD_ADDR 0x27             // Dirección I2C (comúnmente 0x27 o 0x3F)
#define I2C_MASTER_SCL_IO 22      // GPIO SCL
#define I2C_MASTER_SDA_IO 21      // GPIO SDA
#define I2C_MASTER_NUM I2C_NUM_0  // Puerto I2C
#define I2C_MASTER_FREQ_HZ 100000 // Frecuencia

// --- Funciones Públicas ---

/**
 * @brief Inicializa el controlador I2C del ESP32 con los pines definidos.
 * @return esp_err_t ESP_OK si fue exitoso.
 */
esp_err_t lcd_i2c_controller_init(void);

/**
 * @brief Inicializa el LCD (secuencia de arranque y configuración 4-bit).
 */
void lcd_init(void);

/**
 * @brief Envía una cadena de texto al LCD.
 * @param str Cadena de caracteres a mostrar.
 */
void lcd_send_string(const char *str);

/**
 * @brief Mueve el cursor a una posición específica.
 * @param row Fila (0 o 1).
 * @param col Columna (0 a 15).
 */
void lcd_set_cursor(uint8_t row, uint8_t col);

/**
 * @brief Limpia la pantalla completa.
 */
void lcd_clear(void);

#endif // I2C_LCD_H