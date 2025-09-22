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
#include "freertos/queue.h"
#include "lcd_controller.h" // ¡Solo incluimos nuestro módulo!
#include "system_status.h"

// #define ENABLE_PID_BUTTON_GPIO  GPIO_NUM_23
// pid_toggle_enable();
// #define boton_emergencia GPIO_NUM_22
// while (gpio_get_level(ENABLE_PID_BUTTON_GPIO) != 0) ;

typedef struct
{
  int num_pulses;
  int frequency;
  int direction;
} motor_command_t;

QueueHandle_t motor_command_queue;

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
      lcd_printf_line(1, "Pos: %d", position);
    }
    else
    {
      manual_move_state_t move_state = status_get_manual_move_state();
      switch (move_state)
      {
      case MANUAL_MOVE_LEFT:
        lcd_printf_line(1, "<-- Izquierda");
        break;
      case MANUAL_MOVE_RIGHT:
        lcd_printf_line(1, "Derecha -->");
        break;
      case MANUAL_MOVE_NONE:
      default:
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
  // 2. Inicialización de servicios globales (primero que nada)
  // Es crucial inicializar la partición de la NVS antes de que cualquier tarea intente usarla.
  /*
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  */
  lcd_init(); // Inicializar la pantalla
  pwm_init();
  pulse_counter_init();

  // Mensaje de bienvenida en la pantalla
  lcd_clear();
  lcd_printf_line(0, "Bienvenidos...");
  lcd_printf_line(1, "Iniciando");
  vTaskDelay(pdMS_TO_TICKS(1000));

  // 3. Creación de todas las tareas de la aplicación
  // Cada tarea se ejecutará de forma independiente y concurrente.
  // Le damos una prioridad más alta (6) para asegurar que responda rápidamente.
  motor_command_queue = xQueueCreate(1, sizeof(motor_command_t));
  if (motor_command_queue == NULL)
  {
    ESP_LOGE("MAIN", "Error al crear la cola del motor.");
    return;
  }

  // --- 3. CREACIÓN DE TAREAS (Sin duplicados y en orden lógico) ---
  ESP_LOGI("MAIN", "Creando tareas de la aplicación...");

  xTaskCreate(pid_controller_task, "pid_controller_task", configMINIMAL_STACK_SIZE * 4, NULL, 6, NULL);

  // Crear la tarea del control del motor
  xTaskCreate(motor_control_task, "Motor_Control", 4096, NULL, 4, NULL);

  // Tarea para manejar los comandos recibidos por el puerto serie
  xTaskCreate(uart_echo_task, "uart_echo_task", configMINIMAL_STACK_SIZE * 3, NULL, 4, NULL);

  // Tarea que espera comandos en la cola y mueve el motor
  // xTaskCreate(pwm_generator_task, "pwm_generator_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
  // xTaskCreate(pwm_generator_task, "pwm_generator_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

  // Tarea que inicializa el PCNT y reporta la posición del encoder para depuración
  // xTaskCreate(pulse_counter_task, "pulse_counter_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

  // Tarea que monitorea el botón BOOT y envía comandos de "repetir"
  xTaskCreate(button_handler_task, "button_handler_task", configMINIMAL_STACK_SIZE * 3, NULL, 4, NULL);

  // TAREA DE LA PANTALLA (Prioridad baja, no es crítica)
  // ¡ESTA ES LA LÍNEA QUE FALTABA!
  xTaskCreate(lcd_display_task, "lcd_display_task", 3072, NULL, 4, NULL);
}
