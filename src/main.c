// src/main.c
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

//#define ENABLE_PID_BUTTON_GPIO  GPIO_NUM_23
//#define boton_emergencia GPIO_NUM_22

typedef struct {
    int num_pulses;
    int frequency;
    int direction;
} motor_command_t;

QueueHandle_t motor_command_queue;

void app_main(void) {
    // 2. Inicialización de servicios globales (primero que nada)
    // Es crucial inicializar la partición de la NVS antes de que cualquier tarea intente usarla.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    pwm_init();
    pulse_counter_init();

    //while (gpio_get_level(ENABLE_PID_BUTTON_GPIO) != 0) ;
    //pid_toggle_enable();

    // 3. Creación de todas las tareas de la aplicación
    // Cada tarea se ejecutará de forma independiente y concurrente.

    // Le damos una prioridad más alta (6) para asegurar que responda rápidamente.
    xTaskCreate(pid_controller_task, "pid_controller_task", configMINIMAL_STACK_SIZE * 4, NULL, 6, NULL);
    motor_command_queue = xQueueCreate(1, sizeof(motor_command_t));
    if (motor_command_queue == NULL) {
        ESP_LOGE("MAIN", "Error al crear la cola del motor.");
        return;
    }

    // Crear la tarea del controlador PID
    xTaskCreate(pid_controller_task, "PID_Controller", 4096, NULL, 5, NULL); // Ajusta stack size y prioridad
    
    // Crear la tarea del control del motor
    xTaskCreate(motor_control_task, "Motor_Control", 4096, NULL, 4, NULL);

    // Tarea para manejar los comandos recibidos por el puerto serie
    xTaskCreate(uart_echo_task, "uart_echo_task", configMINIMAL_STACK_SIZE * 3, NULL, 4, NULL);

    // Tarea que espera comandos en la cola y mueve el motor
    //xTaskCreate(pwm_generator_task, "pwm_generator_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    //xTaskCreate(pwm_generator_task, "pwm_generator_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    // Tarea que inicializa el PCNT y reporta la posición del encoder para depuración
    //xTaskCreate(pulse_counter_task, "pulse_counter_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    // Tarea que monitorea el botón BOOT y envía comandos de "repetir"
    xTaskCreate(button_handler_task, "button_handler_task", configMINIMAL_STACK_SIZE * 3, NULL, 4, NULL);
}