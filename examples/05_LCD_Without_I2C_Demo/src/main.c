#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_controller.h" // ¡Solo incluimos nuestro módulo!

// La función principal de la aplicación
void app_main(void)
{
    lcd_init();

    // Usamos nuestra nueva y potente función
    lcd_printf_line(0, "Proyecto Modular!");
    /*
    lcd_printf_line(1, "ESP32 con ESP-IDF");
    lcd_printf_line(3, "www.ejemplo.com");
    */
    int count = 0;
    while (1)
    {
        // ¡Mira qué fácil es ahora!
        lcd_printf_line(1, "Contador: %d", count++);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}