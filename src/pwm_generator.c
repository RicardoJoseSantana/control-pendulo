#include "pid_controller.h"
#include "pwm_generator.h"
#include "driver/gpio.h" // Necesario para controlar el pin de dirección
#include "nvs_flash.h" // Cabeceras para NVS
#include "nvs.h"

static const char *TAG = "PWM_GENERATOR";

// Configuración del LEDC (PWM)
void pwm_init(void) {
    ESP_LOGI(TAG, "Inicializando MCPWM (Legacy API) para motor a pasos...");

    // 1. Configurar los pines STEP y DIR
    mcpwm_gpio_init(MCPWM_UNIT, MCPWM0A, LEDC_OUTPUT_IO);
    gpio_set_direction(LEDC_DIRECTION_IO, GPIO_MODE_OUTPUT);

    // 2. Configurar la unidad MCPWM
    mcpwm_config_t pwm_config = {
        .frequency = 1000,
        .cmpr_a = 50.0, // Duty 50%
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_init(MCPWM_UNIT, MCPWM_TIMER, &pwm_config);
    
    // 3. Apagar el motor al inicio estableciendo el duty cycle a 0%
    mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 0.0);

    ESP_LOGI(TAG, "Módulo MCPWM y Dirección inicializado.");
}

// --- FUNCIÓN DE ACCIÓN NUEVA Y CENTRAL ---
int execute_movement(int num_pulses, int frequency, int direction) {
    if (num_pulses <= 0 || frequency <= 0) {
        mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 0.0);
        return 0;
    }

    gpio_set_level(LEDC_DIRECTION_IO, direction);
    esp_rom_delay_us(10);

    mcpwm_set_frequency(MCPWM_UNIT, MCPWM_TIMER, frequency);
    mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 50.0);

    uint32_t duration_ms = (uint32_t)(num_pulses * 1000) / frequency;
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    
    mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 0.0);
    gpio_set_level(LEDC_DIRECTION_IO, 0);

    return num_pulses;
}

void motor_control_task(void *arg)
{
    motor_command_t received_cmd;

    while (1)
    {
        // Espera indefinidamente por un comando en la cola.
        // xQueueReceive devuelve pdTRUE si un ítem fue copiado a received_cmd.
        if (xQueueReceive(motor_command_queue, &received_cmd, portMAX_DELAY) == pdTRUE)
        {

            // Antes de ejecutar el movimiento, calculamos cómo cambiará la posición
            if (received_cmd.direction == 1) { // Asumimos que 1 es derecha (positivo)
                g_car_position_pulses += received_cmd.num_pulses;
            } else { // Dirección 0 es izquierda (negativo)
                g_car_position_pulses -= received_cmd.num_pulses;
            }

            // --- Lógica de Actuación con MCPWM ---
            if (received_cmd.num_pulses == 0) {
                // Comando de parada: poner duty a 0%
                mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 0.0);
                gpio_set_level(LEDC_DIRECTION_IO, 0);
                continue;
            }

            gpio_set_level(LEDC_DIRECTION_IO, received_cmd.direction);
            esp_rom_delay_us(10);

            // Establecer frecuencia y forzar duty a 50%
            mcpwm_set_frequency(MCPWM_UNIT, MCPWM_TIMER, received_cmd.frequency);
            mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 50.0);

            uint32_t duration_ms = (uint32_t)(received_cmd.num_pulses * 1000) / received_cmd.frequency;
            vTaskDelay(pdMS_TO_TICKS(duration_ms));

            // Detener al final del movimiento
            mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, 0.0);
            gpio_set_level(LEDC_DIRECTION_IO, 0);
        }
    }
}
