#include "i2c_lcd.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "I2C_LCD";

// --- Definiciones Privadas (Máscaras de bits y Comandos) ---
#define LCD_RS_BIT 0
#define LCD_RW_BIT 1
#define LCD_EN_BIT 2
#define LCD_BACKLIGHT_BIT 3
#define LCD_D4_BIT 4

#define LCD_CMD_CLEAR 0x01
#define LCD_CMD_HOME 0x02
#define LCD_CMD_ENTRY_MODE 0x06
#define LCD_CMD_DISPLAY_OFF 0x08
#define LCD_CMD_DISPLAY_ON 0x0C
#define LCD_CMD_FUNCTION_SET 0x28
#define LCD_CMD_SET_CURSOR 0x80

// Estado interno de la luz de fondo
static uint8_t backlight_state = (1 << LCD_BACKLIGHT_BIT);

// --- Funciones Privadas (Static) ---

/**
 * @brief Envía datos crudos por I2C al expansor.
 */
static esp_err_t lcd_send_i2c(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data | backlight_state, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void lcd_pulse_enable(uint8_t data)
{
    lcd_send_i2c(data | (1 << LCD_EN_BIT));
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_i2c(data & ~(1 << LCD_EN_BIT));
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void lcd_write_nibble(uint8_t nibble, bool is_data)
{
    uint8_t data = (nibble & 0x0F) << LCD_D4_BIT;
    if (is_data)
        data |= (1 << LCD_RS_BIT);
    lcd_pulse_enable(data);
}

static void lcd_send_byte(uint8_t byte, bool is_data)
{
    lcd_write_nibble(byte >> 4, is_data);   // Nibble alto
    lcd_write_nibble(byte & 0x0F, is_data); // Nibble bajo
}

// --- Implementación de Funciones Públicas ---

esp_err_t lcd_i2c_controller_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(50)); // Espera inicial

    // Secuencia mágica de inicialización
    lcd_write_nibble(0x03, false);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, false);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x03, false);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x02, false); // Modo 4-bits

    // Configuración
    lcd_send_byte(LCD_CMD_FUNCTION_SET, false);
    lcd_send_byte(LCD_CMD_DISPLAY_OFF, false);
    lcd_clear();
    lcd_send_byte(LCD_CMD_ENTRY_MODE, false);
    lcd_send_byte(LCD_CMD_DISPLAY_ON, false);

    ESP_LOGI(TAG, "LCD Inicializado");
}

void lcd_send_string(const char *str)
{
    while (*str)
    {
        lcd_send_byte((uint8_t)(*str), true);
        str++;
    }
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    // Aseguramos límites básicos
    if (row > 3)
        row = 0;
    lcd_send_byte(LCD_CMD_SET_CURSOR | (col + row_offsets[row]), false);
}

void lcd_clear(void)
{
    lcd_send_byte(LCD_CMD_CLEAR, false);
    vTaskDelay(pdMS_TO_TICKS(2)); // Comando Clear es lento
}