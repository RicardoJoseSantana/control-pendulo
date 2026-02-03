// src/main.c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "button_handler.h" // Para la tarea de lectura del botón
#include "freertos/queue.h" // Para la cola de comandos si desea añadir varias tareas
#include "i2c_lcd.h"
#include "system_status.h"


QueueHandle_t motor_command_queue;

void app_main(void)
{
  lcd_init();           // Inicializar la pantalla

  // Mensaje de bienvenida en la pantalla
  lcd_clear();
  lcd_printf_line(0, "Bienvenidos...");
  lcd_printf_line(1, "Iniciando");
  vTaskDelay(pdMS_TO_TICKS(1000));

  // --------- Cada tarea se ejecutará de forma independiente y concurrente. ---------

  // Tarea que monitorea el botón BOOT y envía comandos de "repetir"
  xTaskCreate(button_handler_task, "button_handler_task", configMINIMAL_STACK_SIZE * 3, NULL, 4, NULL);

  // TAREA DE LA PANTALLA (Prioridad baja, no es crítica)
  xTaskCreate(lcd_display_task, "LCDDisplay", 3072, NULL, 3, NULL);
}
