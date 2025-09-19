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
#include "pid_controller.h"
#include "lcd_controller.h" // ¡Solo incluimos nuestro módulo!
#include "system_status.h"

void lcd_display_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(500)); 
    ESP_LOGI("LCD_TASK", "Tarea de visualización iniciada.");

    while (1)
    {
        bool is_pid_on = pid_is_enabled();
        
        // Mostrar el estado del PID en la primera línea (siempre)
        lcd_printf_line(0, "PID: %s", is_pid_on ? "ACTIVO" : "INACTIVO");

        // --- LÓGICA DE VISUALIZACIÓN INTELIGENTE ---
        if (is_pid_on) {
            // Si el PID está ACTIVO, la prioridad es mostrar la posición
            int16_t position = pulse_counter_get_value();
            lcd_printf_line(1, "Pos: %d", position);
        } else {
            // Si el PID está INACTIVO, comprobamos si hay movimiento manual
            manual_move_state_t move_state = status_get_manual_move_state();
            
            switch (move_state) {
                case MANUAL_MOVE_LEFT:
                    lcd_printf_line(1, "<-- Izquierda");
                    break;
                case MANUAL_MOVE_RIGHT:
                    lcd_printf_line(1, "Derecha -->");
                    break;
                case MANUAL_MOVE_NONE:
                default:
                    // Si no hay movimiento manual, mostramos la posición
                    int16_t position = pulse_counter_get_value();
                    lcd_printf_line(1, "Pos: %d", position);
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{

  lcd_init();
  ledc_init();
  
  lcd_clear();

  // Usamos nuestra nueva y potente función
  lcd_printf_line(0, "Bienvenidos...");
  lcd_printf_line(1, "Iniciando");
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Tarea para manejar los comandos recibidos por el puerto serie
  xTaskCreate(uart_echo_task, "uart_echo_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

  pcnt_and_z_index_init();

  // 3. Creación de todas las tareas de la aplicación
  // Cada tarea se ejecutará de forma independiente y concurrente.

  // Le damos una prioridad más alta (6) para asegurar que responda rápidamente.
  xTaskCreate(pid_controller_task, "pid_controller_task", configMINIMAL_STACK_SIZE * 4, NULL, 6, NULL);

  // Tarea que espera comandos en la cola y mueve el motor
  // xTaskCreate(pwm_generator_task, "pwm_generator_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

  // Tarea que inicializa el PCNT y reporta la posición del encoder para depuración
  // xTaskCreate(pulse_counter_task, "pulse_counter_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

  // Tarea que monitorea el botón BOOT y envía comandos de "repetir"
  xTaskCreate(button_handler_task, "button_handler_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

  /*
  // 2. Inicialización de servicios globales (primero que nada)
  // Es crucial inicializar la partición de la NVS antes de que cualquier tarea intente usarla.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  */
}