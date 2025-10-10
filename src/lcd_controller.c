#include "lcd_controller.h"
#include <stdio.h>   // Para vsnprintf
#include <stdarg.h>  // Para va_list y macros relacionadas
#include <string.h>  // Necesario para memset
//#include "hd44780.h" // La nueva librería tiene el mismo nombre de cabecera
#include <LiquidCrystal_I2C.h>
#include "driver/i2c.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pid_controller.h"
#include "pulse_counter.h"
#include "system_status.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

// --- CONFIGURACIÓN PARA EL MÓDULO I2C ---
#define I2C_MASTER_SCL_IO 22        // GPIO pin for SCL
#define I2C_MASTER_SDA_IO 21        // GPIO pin for SDA
#define I2C_MASTER_NUM    I2C_NUM_0 // I2C port number
#define I2C_MASTER_FREQ_HZ 100000   // I2C master clock frequency
#define LCD_DEVICE_ADDRESS 0x27     // Dirección I2C del LCD (la más común)

// Ancho de la pantalla
#define LCD_LINE_WIDTH 16

// Etiqueta para los logs
static const char *TAG = "LCD_CONTROLLER_I2C";

// Variable estática para el descriptor del dispositivo LCD de la nueva librería
static LiquidCrystal_I2C lcd(LCD_DEVICE_ADDRESS, LCD_LINE_WIDTH, 2);

// --- IMPLEMENTACIÓN DE LAS FUNCIONES PÚBLICAS (Adaptadas a la nueva librería) ---

void lcd_init(void)
{
    ESP_LOGI(TAG, "Inicializando bus I2C para pantalla LCD...");
    
    // --- CAMBIO: Inicialización del bus I2C del ESP32 ---
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // ¡IMPORTANTE! Habilitamos los pull-ups internos.
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Son seguros con 5V y necesarios para I2C.
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    ESP_LOGI(TAG, "Bus I2C inicializado. Inicializando LCD en dirección 0x%X...", LCD_DEVICE_ADDRESS);
    
    // --- CAMBIO: Inicialización de la librería LiquidCrystal_I2C ---
    lcd.init();
    lcd.backlight(); // Encender la luz de fondo

    ESP_LOGI(TAG, "LCD inicializada correctamente.");
}

void lcd_clear(void)
{
    // La función para limpiar la pantalla
    lcd.clear();
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    /* // La función para posicionar el cursor
    */
    lcd.setCursor(col, row);
}

void lcd_write_string(const char *str)
{
    // La función para escribir una cadena
    lcd.print(str);
}
void lcd_printf_line(uint8_t row, const char *format, ...)
{
    char buffer[LCD_LINE_WIDTH + 1];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len >= LCD_LINE_WIDTH) {
        len = LCD_LINE_WIDTH;
    }

    for (int i = len; i < LCD_LINE_WIDTH; i++) {
        buffer[i] = ' ';
    }
    buffer[LCD_LINE_WIDTH] = '\0';

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
        // 1. Obtener la vista que debemos mostrar
        lcd_view_state_t current_view = status_get_lcd_view();

        // 2. Usar un 'switch' para decidir qué dibujar
        switch (current_view)
        {
        case VIEW_MAIN_STATUS:
        {
            bool is_pid_on = pid_is_enabled();
            manual_move_state_t move_state = status_get_manual_move_state();

            // Línea 1: Estado prioritario
            if (move_state == MANUAL_MOVE_LEFT)
            {
                lcd_printf_line(0, "<-- Izquierda");
            }
            else if (move_state == MANUAL_MOVE_RIGHT)
            {
                lcd_printf_line(0, "Derecha -->");
            }
            else
            {
                lcd_printf_line(0, "PID: %s", is_pid_on ? "ACTIVO" : "INACTIVO");
            }

            // Línea 2: Posición en grados
            int16_t position = pulse_counter_get_value();
            float degrees = (float)position * 360.0f / 4096.0f;
            lcd_printf_line(1, "Grados: %.1f", degrees-180);
            break;
        }

        case VIEW_ENCODER_COUNTS:
        {
            lcd_printf_line(0, "Encoder Crudo");
            int16_t position = pulse_counter_get_value();
            lcd_printf_line(1, "Pulsos: %d", position);
            break;
        }

        case VIEW_PID_GAINS:
        {
            // Obtenemos los valores actuales del módulo PID
            float kp = pid_get_kp();
            float ki = pid_get_ki();
            // Mostramos Kp y Ki en la misma línea
            lcd_printf_line(0, "Kp:%.2f Ki:%.2f", kp, ki);
            float kd = pid_get_kd();
            lcd_printf_line(1, "Kd: %.2f", kd);
            break;
        }

        default:
            // Vista por defecto en caso de error
            lcd_printf_line(0, "Vista Invalida");
            lcd_printf_line(1, "");
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}