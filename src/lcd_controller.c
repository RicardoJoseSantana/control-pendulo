#include "lcd_controller.h"
#include <stdio.h>   // Para vsnprintf
#include <stdarg.h>  // Para va_list y macros relacionadas
#include <string.h>  // Necesario para memset
#include "hd44780.h" // La nueva librería tiene el mismo nombre de cabecera
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "state_controller.h"
#include "pulse_counter.h"
#include "system_status.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

// --- CONFIGURACIÓN PRIVADA DEL MÓDULO (Pines sin cambios) ---
#define LCD_RS_PIN GPIO_NUM_19
#define LCD_E_PIN GPIO_NUM_18
#define LCD_D4_PIN GPIO_NUM_5
#define LCD_D5_PIN GPIO_NUM_17
#define LCD_D6_PIN GPIO_NUM_16
#define LCD_D7_PIN GPIO_NUM_4
#define LCD_RW_PIN GPIO_NUM_2       // Pin para lectura y escritura
#define LCD_BL_PIN HD44780_NOT_USED // No controlamos el backlight por GPIO

// Etiqueta para los logs
static const char *TAG = "LCD_CONTROLLER";

// Variable estática para el descriptor del dispositivo LCD
static hd44780_t lcd_handle;

// --- ¡NUEVA FUNCIÓN! Intenta leer el Busy Flag para ver si la LCD responde ---
static bool is_lcd_responsive(void)
{
    // 1. Configurar temporalmente D7 como entrada y R/W como salida
    gpio_set_direction(LCD_RS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_RW_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_E_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_D7_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LCD_D7_PIN, GPIO_PULLUP_ONLY);

    // 2. Poner la pantalla en modo LECTURA de COMANDO
    gpio_set_level(LCD_RS_PIN, 0); // Comando
    gpio_set_level(LCD_RW_PIN, 1); // Lectura
    gpio_set_level(LCD_E_PIN, 0);

    // 3. Dar un pulso en el pin E para que la LCD ponga el Busy Flag en D7

    esp_rom_delay_us(1); // Pequeña espera
    gpio_set_level(LCD_E_PIN, 1);
    esp_rom_delay_us(1); // Pequeña espera

    // 4. Leer el estado de D7. Si la LCD está alimentada, debería tirar de él a 0.
    // Si está apagada, nuestro pull-up lo mantendrá en 1.
    int level = gpio_get_level(LCD_D7_PIN); // Leer mientras E está en alto

    // 5. Finalizar el pulso y volver al modo ESCRITURA
    gpio_set_level(LCD_E_PIN, 0);
    gpio_set_level(LCD_RW_PIN, 0); // Volver al modo Escritura por defecto
    gpio_set_level(LCD_RS_PIN, 0); // Volver al modo Escritura por defecto

    // Si leímos un nivel BAJO (0), significa que la pantalla respondió.
    return (level == 0);
}

// --- IMPLEMENTACIÓN DE LAS FUNCIONES PÚBLICAS (Adaptadas a la nueva librería) ---

void lcd_init(void)
{
    // Bucle de espera: Intentar conectar hasta que responda
    while (!is_lcd_responsive())
    {
        ESP_LOGW(TAG, "Pantalla no detectada. Reintentando en 1 segundo...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "¡Pantalla detectada! Inicializando librería hd44780...");
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
    /*
    // Antes de llamar a la librería, configuramos todos los pines nosotros.
    // El pin RW también se configura como salida, empezando en BAJO (modo escritura).
    gpio_config_t io_conf = {
        .pin_bit_mask = ((1ULL << LCD_RS_PIN) | (1ULL << LCD_E_PIN) | (1ULL << LCD_RW_PIN) |
                         (1ULL << LCD_D4_PIN) | (1ULL << LCD_D5_PIN) | (1ULL << LCD_D6_PIN) |
                         (1ULL << LCD_D7_PIN)),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(LCD_RW_PIN, 0);

    ESP_LOGI(TAG, "Esperando a que la pantalla LCD esté disponible...");

    */

    // Ahora que sabemos que está viva, dejamos que la librería la inicialice
    hd44780_init(&lcd_handle);
    ESP_LOGI(TAG, "LCD inicializada correctamente por la librería.");
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
void lcd_printf_line(uint8_t row, const char *format, ...)
{
// --- CORRECCIÓN 2: Ajustar al tamaño real de la pantalla ---
// Definimos el ancho de la pantalla para no usar "números mágicos"
#define LCD_LINE_WIDTH 16 // Cambia a 20 si usas una 20x4

    // El búfer debe ser 1 más grande que el ancho para el terminador nulo '\0'
    char buffer[LCD_LINE_WIDTH + 1];

    // Procesar los argumentos variables
    va_list args;
    va_start(args, format);
    // vsnprintf es seguro y no escribirá más allá del tamaño del búfer
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Si vsnprintf escribió más caracteres de los que caben, lo truncamos.
    if (len > LCD_LINE_WIDTH)
    {
        len = LCD_LINE_WIDTH;
    }

    // Rellenar el resto del buffer con espacios
    for (int i = len; i < LCD_LINE_WIDTH; i++)
    {
        buffer[i] = ' ';
    }
    // Asegurar el terminador nulo al final
    buffer[LCD_LINE_WIDTH] = '\0';

    // --- CORRECCIÓN 1: Posicionar el cursor correctamente ---
    // La función es lcd_set_cursor(columna, fila)
    lcd_set_cursor(0, row); // Siempre empezamos en la columna 0 de la fila deseada
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
            bool is_pid_on = state_controller_is_enabled();
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
                lcd_printf_line(0, "estado: %s", is_pid_on ? "ACTIVO" : "INACTIVO");
            }

            // Línea 2: Posición en grados
            int16_t position = pulse_counter_get_value();
            float degrees = (float)position * 360.0f / 4096.0f;
            lcd_printf_line(1, "Grados: %.1f", degrees);
            break;
        }

        case VIEW_ENCODER_COUNTS:
        {
            lcd_printf_line(0, "Encoder Crudo");
            int16_t position = pulse_counter_get_value();
            lcd_printf_line(1, "Pulsos: %d", position);
            break;
        }

        /*case VIEW_PID_GAINS:
        {
            // Obtenemos los valores actuales del módulo PID
            float kp = pid_get_kp();
            float ki = pid_get_ki();
            // Mostramos Kp y Ki en la misma línea
            lcd_printf_line(0, "Kp:%.2f Ki:%.2f", kp, ki);
            float kd = pid_get_kd();
            lcd_printf_line(1, "Kd: %.2f", kd);
            break;
        }*/

        default:
            // Vista por defecto en caso de error
            lcd_printf_line(0, "Vista Invalida");
            lcd_printf_line(1, "");
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}