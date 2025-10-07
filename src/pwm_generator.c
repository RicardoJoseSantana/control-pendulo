// src/pwm_generator.c
#include "pwm_generator.h"
#include "state_controller.h" // Incluimos para g_car_position_pulses
#include "uart_echo.h"

static const char *TAG = "PWM_GENERATOR_LEGACY";

void pwm_init(void) {
    ESP_LOGI(TAG, "Inicializando MCPWM (Legacy API) para motor a pasos...");

    // 1. Configurar los pines STEP y DIR
    mcpwm_gpio_init(MCPWM_UNIT, MCPWM0A, STEP_OUTPUT_IO);
    gpio_set_direction(DIR_OUTPUT_IO, GPIO_MODE_OUTPUT);

    // 2. Configurar la unidad MCPWM
    mcpwm_config_t pwm_config = {
        .frequency = 1000,
        .cmpr_a = 50.0, // Duty 50%
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_init(MCPWM_UNIT, MCPWM_TIMER, &pwm_config);
    
    // 3. Apagar el motor al inicio estableciendo el duty cycle a 0%
    // Este es el método correcto para asegurar una salida en BAJO.
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 0.0));

    ESP_LOGI(TAG, "Módulo MCPWM (Legacy API) y Dirección inicializado.");
}

void motor_control_task(void *arg) {
    motor_command_t received_cmd;
    bool motor_is_running = false;

    while (1) {
        // Espera un comando de la cola
        if (xQueueReceive(motor_command_queue, &received_cmd, portMAX_DELAY) == pdTRUE) {
            
            // Si el comando es para mover (num_pulses > 0, como en tu controlador de estado)
            if (received_cmd.num_pulses > 0) {
                // Actualizar la odometría del carro
                if (received_cmd.direction == 1) {
                    g_car_position_pulses += received_cmd.num_pulses;
                } else {
                    g_car_position_pulses -= received_cmd.num_pulses;
                }

                // Ejecutar el movimiento
                gpio_set_level(DIR_OUTPUT_IO, received_cmd.direction);
                esp_rom_delay_us(10);

                // --- CORRECCIÓN CLAVE: Garantizar 50% de Duty Cycle ---
                // 1. Establecer la nueva frecuencia
                mcpwm_set_frequency(MCPWM_UNIT, MCPWM_TIMER, received_cmd.target_frequency);
                // 2. Inmediatamente después, forzar el ciclo de trabajo al 50%
                mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 50.0);
                
                // Si el motor no estaba corriendo, lo arrancamos (solo si estaba apagado con duty 0)
                // En realidad, con esta lógica, el motor siempre está "corriendo", solo que con duty 0 o 50
                // por lo que no necesitamos start/stop.

                uint32_t duration_ms = (uint32_t)(received_cmd.num_pulses * 1000) / received_cmd.target_frequency;
                vTaskDelay(pdMS_TO_TICKS(duration_ms));

                // Detener al final del movimiento poniendo el duty a 0
                mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 0.0);
                gpio_set_level(DIR_OUTPUT_IO, 0);

            } 
            // Si el comando es una orden de parada explícita (num_pulses = 0)
            else { 
                mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 0.0);
                gpio_set_level(DIR_OUTPUT_IO, 0);
            }
        }
    }
}

int execute_movement(int num_pulses, int frequency, int direction) {
    if (num_pulses <= 0 || frequency <= 0) {
        mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 0.0);
        return 0;
    }

    gpio_set_level(DIR_OUTPUT_IO, direction);
    esp_rom_delay_us(10);

    mcpwm_set_frequency(MCPWM_UNIT, MCPWM_TIMER, frequency);
    mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 50.0);

    uint32_t duration_ms = (uint32_t)(num_pulses * 1000) / frequency;
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    
    mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 0.0);

    return num_pulses;
}