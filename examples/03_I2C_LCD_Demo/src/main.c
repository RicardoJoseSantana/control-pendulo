#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c_lcd.h" // Incluimos nuestra librería

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    // 1. Inicializar Hardware
    ESP_ERROR_CHECK(lcd_i2c_controller_init());
    ESP_LOGI(TAG, "Hardware I2C listo");

    // 2. Inicializar Pantalla
    lcd_init();

    // 3. Lógica de la aplicación
    lcd_clear();

    lcd_set_cursor(0, 0);
    lcd_send_string("Hola Mundo!");

    lcd_set_cursor(1, 2);
    lcd_send_string("Bienvenidos");

    ESP_LOGI(TAG, "Pantalla actualizada.");
}