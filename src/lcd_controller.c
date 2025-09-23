#include "lcd_controller.h"
#include <stdio.h>   // Para vsnprintf
#include <stdarg.h>  // Para va_list, va_start, va_end
#include <string.h>  // Necesario para memset
#include "hd44780.h" // La nueva librería tiene el mismo nombre de cabecera
#include "esp_log.h"

#include "system_status.h"
#include "pid_controller.h"
#include "pulse_counter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- CONFIGURACIÓN PRIVADA DEL MÓDULO (Pines sin cambios) ---
#define LCD_RS_PIN GPIO_NUM_19
#define LCD_E_PIN GPIO_NUM_18
#define LCD_D4_PIN GPIO_NUM_5
#define LCD_D5_PIN GPIO_NUM_17
#define LCD_D6_PIN GPIO_NUM_16
#define LCD_D7_PIN GPIO_NUM_4
#define LCD_BL_PIN HD44780_NOT_USED // No controlamos el backlight por GPIO

// Etiqueta para los logs
static const char *TAG = "LCD_CONTROLLER";

// Variable estática para el descriptor del dispositivo LCD
static hd44780_t lcd_handle;

// --- IMPLEMENTACIÓN DE LAS FUNCIONES PÚBLICAS (Adaptadas a la nueva librería) ---

void lcd_init(void)
{
    // Limpiar la estructura de configuración por seguridad
    memset(&lcd_handle, 0, sizeof(hd44780_t));

    // --- CORREGIDO: Configuración para conexión directa a GPIO ---

    // 1. Asignar los pines GPIO
    lcd_handle.pins.rs = LCD_RS_PIN;
    lcd_handle.pins.e = LCD_E_PIN;
    lcd_handle.pins.d4 = LCD_D4_PIN;
    lcd_handle.pins.d5 = LCD_D5_PIN;
    lcd_handle.pins.d6 = LCD_D6_PIN;
    lcd_handle.pins.d7 = LCD_D7_PIN;
    lcd_handle.pins.bl = LCD_BL_PIN;

    // 2. Configurar las propiedades de la pantalla
    lcd_handle.lines = 2; // Para una pantalla 20x4 o 16x2
    lcd_handle.font = HD44780_FONT_5X8;

    // 3. ¡IMPORTANTE! Dejar el callback a NULL.
    // Al ser NULL, la librería sabrá que debe configurar y controlar los pines GPIO ella misma.
    lcd_handle.write_cb = NULL;
    // Opcional: El estado del backlight. Como no lo controlamos, lo dejamos en false.
    lcd_handle.backlight = false;

    ESP_LOGI(TAG, "Initializing LCD (Direct GPIO)...");
    hd44780_init(&lcd_handle);
    ESP_LOGI(TAG, "LCD Initialized.");
}

void lcd_clear(void)
{
    // La función para limpiar la pantalla ahora se llama hd44780_clear()
    hd44780_clear(&lcd_handle);
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    /* // La función para posicionar el cursor es la misma
    hd44780_set_cursor(&lcd_handle, col, row);
    */
    // --- CORREGIDO: La función se llama hd44780_gotoxy ---
    hd44780_gotoxy(&lcd_handle, col, row);
}

void lcd_write_string(const char *str)
{
    // La función para escribir una cadena ahora se llama hd44780_puts()
    hd44780_puts(&lcd_handle, str);
}
// --- NUEVA FUNCIÓN ---
void lcd_printf_line(uint8_t row, const char *format, ...)
{
    // Buffer para la línea completa (20 chars + nulo)
    char buffer[21];

    // Procesar los argumentos variables (como en printf)
    va_list args;
    va_start(args, format);
    // Usamos vsnprintf para escribir de forma segura en el buffer
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Rellenar el resto del buffer con espacios
    for (int i = len; i < 20; i++)
    {
        buffer[i] = ' ';
    }
    buffer[20] = '\0'; // Asegurar el terminador nulo

    // Mover el cursor y escribir la línea completa
    lcd_set_cursor(0, row);
    lcd_write_string(buffer);
}

// --- Tarea dedicada para actualizar la pantalla (es la pieza clave que faltaba iniciar) ---
void lcd_display_task(void *pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(500)); // Dar tiempo a que otros módulos se inicien
  ESP_LOGI("LCD_TASK", "Tarea de visualización iniciada.");

  while (1)
  {
    bool is_pid_on = pid_is_enabled();
    lcd_printf_line(0, "PID: %s", is_pid_on ? "ACTIVO" : "INACTIVO");

    if (is_pid_on)
    {
      int16_t position = pulse_counter_get_value();
      float degrees = (float)position* 360.0f / 4096.0f;
      lcd_printf_line(1, "Gra: %.1f", degrees);
    }
    else
    {
      manual_move_state_t move_state = status_get_manual_move_state();
      switch (move_state)
      {
      case MANUAL_MOVE_LEFT:
        lcd_printf_line(1, "Derecha -->");
        break;
      case MANUAL_MOVE_RIGHT:
        lcd_printf_line(1, "<-- Izquierda");
        break;
      case MANUAL_MOVE_NONE:
      default:
        int16_t position = pulse_counter_get_value();
        //lcd_printf_line(1, "Pos: %d", position);
        float degrees = (float)position* 360.0f / 4096.0f;
        //lcd_printf_line(0, "Pos: %d", position);
        lcd_printf_line(1, "Gra: %.1f", degrees);
        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}