#include "i2c_lcd.h"
#include <stdio.h>   // Para vsnprintf
#include <stdarg.h>  // Para va_list y macros relacionadas
#include <string.h>  // Necesario para memset
#include "hd44780.h" // La nueva librería tiene el mismo nombre de cabecera
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "system_status.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

// --- CONFIGURACIÓN DEL BUS I2C Y PANTALLA ---
#define I2C_MASTER_PORT I2C_NUM_0      // Puerto I2C a usar (0 o 1)
#define I2C_MASTER_SDA_PIN GPIO_NUM_21 // Pin GPIO para SDA
#define I2C_MASTER_SCL_PIN GPIO_NUM_22 // Pin GPIO para SCL
#define I2C_MASTER_FREQ_HZ 100000      // Frecuencia del bus (100kHz es estándar)
#define LCD_ADDR 0x27                  // Dirección I2C del módulo. O Puede ser 0x3F.

// Etiqueta para los logs
static const char *TAG = "LCD_CONTROLLER_I2C";

static volatile bool g_is_pid_active_sim = false;

// Implementación de la función pública
bool button_handler_is_pid_active_sim(void) {
    return g_is_pid_active_sim;
}

// Variable estática para el descriptor del dispositivo LCD
static hd44780_t lcd_handle;

// --- FUNCIÓN CALLBACK: El Puente entre hd44780 e I2C ---
// La librería hd44780 no sabe hablar I2C. Nosotros le "enseñamos" cómo
// enviar un byte usando esta función.
static esp_err_t write_lcd_data_i2c(const hd44780_t *lcd, uint8_t data)
{
    // Prepara una transacción I2C
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // Envía la dirección del esclavo (la pantalla) con el bit de escritura
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    // Envía el byte de datos. La librería hd44780 ya ha preparado este byte
    // con el formato correcto para el expansor PCF8574.
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    // Ejecuta la transacción y espera a que termine
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

// --- FUNCIÓN DE INICIALIZACIÓN (Ahora para I2C) ---
void lcd_init(void)
{
    // 1. Inicializar el hardware del bus I2C del ESP32
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Driver I2C inicializado. Buscando LCD en 0x%X...", LCD_ADDR);

    // 2. Limpiar y configurar la estructura de la librería para modo I2C
    memset(&lcd_handle, 0, sizeof(hd44780_t));

    // --- ¡LA MAGIA ESTÁ AQUÍ! ---
    // a. Le decimos a la librería que use nuestra función de escritura I2C
    lcd_handle.write_cb = write_lcd_data_i2c;
    // b. Mapeamos los pines lógicos del LCD a los bits del expansor PCF8574
    lcd_handle.pins.rs = 0; // El pin RS está conectado al bit P0 del PCF8574
    lcd_handle.pins.e = 2;  // El pin E está conectado al bit P2
    lcd_handle.pins.d4 = 4; // El pin D4 está conectado al bit P4
    lcd_handle.pins.d5 = 5;
    lcd_handle.pins.d6 = 6;
    lcd_handle.pins.d7 = 7;
    lcd_handle.pins.bl = 3; // El backlight está conectado al bit P3

    // c. Configuramos las propiedades de la pantalla
    lcd_handle.lines = 2; // Para una pantalla 16x2 o 4 para una 20x4
    lcd_handle.font = HD44780_FONT_5X8;
    lcd_handle.backlight = true; // Queremos el backlight encendido por defecto

    // 3. Dejamos que la librería se inicialice a sí misma
    // La librería ahora llamará a 'write_lcd_data_i2c' para enviar todos
    // los comandos de inicialización.
    ESP_ERROR_CHECK(hd44780_init(&lcd_handle));

    ESP_LOGI(TAG, "LCD inicializada correctamente vía I2C.");
}

// La belleza de esta arquitectura es que el resto de las funciones
// (clear, gotoxy, puts, printf_line, y la tarea) no necesitan saber
// si la comunicación es por GPIO directo o I2C. Simplemente llaman
// a las funciones de la librería, y la librería usa el 'write_cb'
// que le hemos configurado.

void lcd_clear(void)
{
    // La función para limpiar la pantalla
    hd44780_clear(&lcd_handle);
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    // La función para posicionar el cursor 
    hd44780_gotoxy(&lcd_handle, col, row);
}

void lcd_write_string(const char *str)
{
    // La función para escribir una cadena de texto
    hd44780_puts(&lcd_handle, str);
}

void lcd_printf_line(uint8_t row, const char *format, ...)
{
// Ajustar al tamaño real de la pantalla ---
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
    // vTaskDelay(pdMS_TO_TICKS(500)); // Dar tiempo a que otros módulos se inicien
    ESP_LOGI("LCD_TASK", "Tarea de visualización iniciada.");
    float g_car_position_cm;

    while (1)
    {
        // 1. Obtener la vista que debemos mostrar
        lcd_view_state_t current_view = status_get_lcd_view();

        // 2. Usar un 'switch' para decidir qué dibujar
        switch (current_view)
        {
        case VIEW_MAIN_STATUS:
        {
            bool is_pid_on = button_handler_is_pid_active_sim();
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
            int16_t position = 4592; // encoder_get_position_degrees();
            float degrees = (float)position * 360.0f / 4096.0f;
            lcd_printf_line(1, "Grados: %.1f", degrees - 180);
            break;
        }

        case VIEW_POSITION:
        {
            lcd_printf_line(0, "Posicion carro:");
            g_car_position_cm = 0.0f;
            lcd_printf_line(1, "%.1f cm", g_car_position_cm);
            break;
        }

        case VIEW_PID_GAINS:
        {
            // Obtenemos los valores actuales del módulo PID
            float kp = 40.0f; // pid_get_kp();
            float ki = 0.02f; // pid_get_ki();
            // Mostramos Kp y Ki en la misma línea
            lcd_printf_line(0, "Kp:%.2f Ki:%.2f", kp, ki);
            float kd = 1.0f; // pid_get_kd();
            lcd_printf_line(1, "Kd: %.2f", kd);
            break;
        }

        case VIEW_CALIBRATION:
        {
            lcd_printf_line(0, "Calibrando...");
            lcd_printf_line(1, "Espere");
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

