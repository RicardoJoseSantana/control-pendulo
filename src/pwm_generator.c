#include"pid_controller.h"
#include "pwm_generator.h"
#include "uart_echo.h"
#include "lcd_controller.h"
#include "driver/gpio.h" // Necesario para controlar el pin de dirección

static const char *TAG = "PWM_GENERATOR";

// Configuración del LEDC (PWM)
void pwm_init(void) {
    // 1. Configurar los pines STEP y DIR
    mcpwm_gpio_init(MCPWM_UNIT, MCPWM0A, STEP_PIN); // Usaremos el generador 0A para la señal STEP
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);

    // 2. Configurar la unidad MCPWM
    mcpwm_config_t pwm_config = {
        .frequency = 1000,    // Frecuencia inicial por defecto
        .cmpr_a = 50.0,       // Ciclo de trabajo inicial al 50%
        .cmpr_b = 50.0,       // No usamos la salida B
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_init(MCPWM_UNIT, MCPWM_TIMER, &pwm_config);
    
    // Detenemos el motor al inicio
    mcpwm_stop(MCPWM_UNIT, MCPWM_TIMER);

    ESP_LOGI(TAG, "Módulo MCPWM y Dirección inicializado.");
}

// --- FUNCIÓN DE ACCIÓN NUEVA Y CENTRAL ---
int execute_movement(int num_pulses, int frequency, int direction) {
    if (num_pulses <= 0 || frequency <= 0) {
        mcpwm_stop(MCPWM_UNIT, MCPWM_TIMER); // Asegurarse de que el motor esté parado
        return 0;
    }

    // 1. Establecer la dirección
    gpio_set_level(DIR_PIN, direction);
    // Pequeño delay para asegurar que el driver procesa el cambio de dirección antes de los pulsos
    esp_rom_delay_us(10); 

    // 2. Establecer la frecuencia de los pulsos
    mcpwm_set_frequency(MCPWM_UNIT, MCPWM_TIMER, frequency);
    
    // 3. Calcular la duración del movimiento
    uint32_t duration_ms = (uint32_t)(num_pulses * 1000) / frequency;

    // 4. Iniciar la generación de pulsos
    mcpwm_start(MCPWM_UNIT, MCPWM_TIMER);

    // 5. Esperar el tiempo calculado
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // 6. Detener la generación de pulsos
    mcpwm_stop(MCPWM_UNIT, MCPWM_TIMER);

    return num_pulses;
}

void motor_control_task(void *arg) {
    motor_command_t received_cmd;
    while (1) {
        if (xQueueReceive(motor_command_queue, &received_cmd, portMAX_DELAY) == pdTRUE) {
            if (received_cmd.num_pulses > 0) {
                execute_movement(received_cmd.num_pulses, received_cmd.frequency, received_cmd.direction);
            } else {
                // Comando de parada explícito
                mcpwm_stop(MCPWM_UNIT, MCPWM_TIMER);
            }
        }
    }
}