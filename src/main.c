// src/main.c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 1. Inclusión de todas las cabeceras de los módulos
#include "nvs_flash.h"      // Para la memoria no volátil
#include "uart_echo.h"      // Para la tarea de comandos UART
#include "pwm_generator.h"  // Para la tarea de control del motor
#include "pulse_counter.h"  // Para la tarea de lectura del encoder
#include "button_handler.h" // Para la tarea de lectura del botón
#include "state_controller.h"
#include "freertos/queue.h"
#include "lcd_controller.h" // ¡Solo incluimos nuestro módulo!
#include "system_status.h"

QueueHandle_t motor_command_queue;

void app_main(void)
{

  lcd_init(); // Inicializar la pantalla
  pwm_init(); // inicializa y configura pines del driver
  pulse_counter_init(); // inicializa y configura pines del encoder

  // Mensaje de bienvenida en la pantalla
  lcd_clear();
  lcd_printf_line(0, "Bienvenidos...");
  lcd_printf_line(1, "Iniciando");
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  // Creacion de la cola de tareas
  motor_command_queue = xQueueCreate(1, sizeof(motor_command_t));
  if (motor_command_queue == NULL) {
      ESP_LOGE("MAIN", "Error al crear la cola del motor.");
      return;
  }

  // --------- Cada tarea se ejecutará de forma independiente y concurrente. ---------
  xTaskCreate(state_controller_task, "State_Controller", 4096, NULL, 6, NULL);   
  xTaskCreate(motor_control_task, "Motor_Control", 3072, NULL, 5, NULL);
  xTaskCreate(uart_echo_task, "uart_echo_task", 3072, NULL, 4, NULL);
  xTaskCreate(button_handler_task, "button_handler_task", 3072, NULL, 4, NULL);
  xTaskCreate(lcd_display_task, "lcd_display_task", 3072, NULL, 3, NULL);
}
