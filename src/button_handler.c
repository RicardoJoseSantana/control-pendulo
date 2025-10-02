// src/button_handler.c
#include "button_handler.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "uart_echo.h" // Incluimos para acceder a la cola y la estructura de comando
#include "pid_controller.h"
#include "pwm_generator.h"
#include "pulse_counter.h"
#include "system_status.h" // Para manejar el estado del movimiento manual

// --- PINES DE LOS BOTONES ---
#define VIEW_CYCLE_BUTTON_GPIO GPIO_NUM_15 // ¡NUEVO BOTÓN!
#define CALIBRATION_BUTTON_GPIO GPIO_NUM_0
#define ENABLE_PID_BUTTON_GPIO GPIO_NUM_23   // Botón para Habilitar/Deshabilitar PID
#define MANUAL_LEFT_BUTTON_GPIO GPIO_NUM_21  // Nuevo botón para mover a la izquierda
#define MANUAL_RIGHT_BUTTON_GPIO GPIO_NUM_22 // Nuevo botón para mover a la derecha
#define EMERGENCY_STOP_GPIO_LEFT GPIO_NUM_34      // Botón de parada de emergencia izquierdo
#define EMERGENCY_STOP_GPIO_RIGHT GPIO_NUM_35      // Botón de parada de emergencia derecho

// --- PARÁMETROS DE MOVIMIENTO MANUAL ---
/*#define MANUAL_MOVE_SPEED_HZ 20000 // Velocidad constante para el movimiento manual (alta)
#define MANUAL_MOVE_PULSES   400   // Cantidad de pulsos por ciclo (movimiento suave)

// --- PARÁMETROS DE MOVIMIENTO secuencia ---
#define SEQUENCE_BASE_PULSES 16000
#define SEQUENCE_BASE_SPEED_HZ 5000*/

// --- PARÁMETROS DE MOVIMIENTO ---
#define JOG_SPEED_HZ          20000 // Velocidad para el movimiento manual (jog)
#define JOG_PULSES            400   // Pulsos por ciclo de jog
#define CALIBRATION_SPEED_HZ  20000  // Velocidad constante para la calibración

static const char *TAG = "BUTTON_HANDLER";
// Para saber si el PID está activo, llamaremos a una función.
// extern bool g_pid_enabled;

// --- Contador de posición del carro en micropasos ---
static int32_t g_car_position_pulses = 0;

// Función auxiliar para botones de comando (pulsar y soltar)
static bool is_command_button_pressed(int gpio_num) {
    if (gpio_get_level(gpio_num) == 0) {
        vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
        if (gpio_get_level(gpio_num) == 0) {
            while(gpio_get_level(gpio_num) == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            return true;
        }
    }
    return false;
}

void button_handler_task(void *arg)
{
    ESP_LOGI(TAG, "Iniciando tarea de lectura de botones...");

    // Configuración de los 3 pines GPIO como entrada con pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = ((1ULL << ENABLE_PID_BUTTON_GPIO) |
                         (1ULL << MANUAL_LEFT_BUTTON_GPIO) |
                         (1ULL << MANUAL_RIGHT_BUTTON_GPIO) |
                         (1ULL << EMERGENCY_STOP_GPIO_RIGHT) |
                         (1ULL << EMERGENCY_STOP_GPIO_LEFT) |
                         (1ULL << VIEW_CYCLE_BUTTON_GPIO)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Tarea iniciada. BOOT:Habilitar PID. GPIO21:Izquierda. GPIO22:Derecha.");

    int last_button_state = 1; // 1 = no presionado
    //int last_sequence_button_state = 1;
    int last_stop_button_state = 1;
    int last_view_button_state = 1;

    while (1)
    {
        // --- AÑADIDO: Lógica para el botón de cambio de vista ---
        int current_view_button_state = gpio_get_level(VIEW_CYCLE_BUTTON_GPIO);
        if (last_view_button_state == 1 && current_view_button_state == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
            if (gpio_get_level(VIEW_CYCLE_BUTTON_GPIO) == 0)
            {
                ESP_LOGI(TAG, "Botón de cambio de vista presionado.");
                status_cycle_lcd_view(); // ¡Llamamos a nuestra nueva función!
            }
        }
        last_view_button_state = current_view_button_state;
        // --- AÑADIDO: Lógica para la Parada de Emergencia (máxima prioridad) ---
        int current_stop_button_state = gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT);
        if (last_stop_button_state == 1 && current_stop_button_state == 0) {
            
            if (gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT) == 0) {
                // Llama a la función que solo deshabilita
                pid_force_disable();
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
        }
        last_stop_button_state = current_stop_button_state;
        
        int current_stop_button_state_new = gpio_get_level(EMERGENCY_STOP_GPIO_LEFT);
        if (last_stop_button_state == 1 && current_stop_button_state_new == 0) {
            
            if (gpio_get_level(EMERGENCY_STOP_GPIO_LEFT) == 0) {
                // Llama a la función que solo deshabilita
                pid_force_disable();
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
        }
        last_stop_button_state = current_stop_button_state_new;

        int current_button_state = gpio_get_level(ENABLE_PID_BUTTON_GPIO);

        // Detectar el flanco de bajada (cuando se presiona el botón)
        if (last_button_state == 1 && current_button_state == 0)
        {
            // Anti-rebote: esperar un poco y confirmar
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(ENABLE_PID_BUTTON_GPIO) == 0)
            {

                pid_toggle_enable();
            }
        }
        last_button_state = current_button_state;

        // Solo permitimos el movimiento manual si el PID está explícitamente deshabilitado
        if (!pid_is_enabled())
        {

            
            // --- LÓGICA DE CALIBRACIÓN (HOMING) ---
            if (is_command_button_pressed(CALIBRATION_BUTTON_GPIO)) {
                ESP_LOGW(TAG, "--- INICIANDO RUTINA DE CALIBRACIÓN DE LÍMITES ---");
                
                int32_t limit_left_pos, limit_right_pos;
                g_car_position_pulses = 0;

                // 1. Mover a la izquierda hasta que el final de carrera se active
                ESP_LOGI(TAG, "Buscando límite derecho (GPIO %d)...", EMERGENCY_STOP_GPIO_RIGHT);
                // Leemos el estado del pin directamente. El bucle continúa MIENTRAS el botón NO esté presionado.
                while (gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT) == 1) {
                    int pulses_moved = execute_movement(JOG_PULSES, CALIBRATION_SPEED_HZ, 0); // Dir 0 = Izquierda
                    g_car_position_pulses -= pulses_moved;
                }
                // --- El bucle se rompe en cuanto el pin se va a BAJO ---
                limit_left_pos = g_car_position_pulses;
                ESP_LOGW(TAG, "Límite 1 detectado en: %ld pulsos", limit_left_pos);
                vTaskDelay(pdMS_TO_TICKS(200)); // Pausa para estabilizar

                // --- AÑADIDO: Moverse un poco para liberar el interruptor ---
                ESP_LOGI(TAG, "Liberando el interruptor...");
                execute_movement(JOG_PULSES * 5, JOG_SPEED_HZ, 1); // Mover un poco a la derecha
                g_car_position_pulses += JOG_PULSES * 5;
                vTaskDelay(pdMS_TO_TICKS(200));


                // 2. Mover a la derecha hasta que el final de carrera se active
                ESP_LOGI(TAG, "Buscando límite derecho...");
                while (gpio_get_level(EMERGENCY_STOP_GPIO_LEFT) == 1) {
                    int pulses_moved = execute_movement(JOG_PULSES, CALIBRATION_SPEED_HZ, 1); // Dir 1 = Derecha
                    g_car_position_pulses += pulses_moved;
                }
                limit_right_pos = g_car_position_pulses;
                ESP_LOGW(TAG, "Límite 2 detectado en: %ld pulsos", limit_right_pos);
                vTaskDelay(pdMS_TO_TICKS(200));

                // 3. Calcular el centro y mover el carro
                int32_t travel_range = abs(limit_right_pos - limit_left_pos);
                int32_t center_pos = limit_left_pos + (travel_range / 2);
                ESP_LOGW(TAG, "Recorrido: %ld pulsos. Centro: %ld", travel_range, center_pos);
                
                ESP_LOGI(TAG, "Moviendo al centro...");
                
                int32_t pulses_to_center = abs(center_pos - g_car_position_pulses);
                int direction_to_center = (center_pos > g_car_position_pulses) ? 1 : 0;
                execute_movement(pulses_to_center, JOG_SPEED_HZ, direction_to_center);
                g_car_position_pulses = center_pos;
                
                ESP_LOGW(TAG, "--- CALIBRACIÓN FINALIZADA. Posición: %ld ---", g_car_position_pulses);
            }



            // Leemos el estado de los botones de movimiento manual
            int left_button_state = gpio_get_level(MANUAL_LEFT_BUTTON_GPIO);
            int right_button_state = gpio_get_level(MANUAL_RIGHT_BUTTON_GPIO);

            // Si se presiona el botón izquierdo Y no el derecho
            if (left_button_state == 1 && right_button_state == 0 && gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT) == 1)
            {
                status_set_manual_move_state(MANUAL_MOVE_RIGHT); // Reportar estado
                // Por seguridad, podríamos comprobar si el PID está deshabilitado aquí
                ESP_LOGD(TAG, "Moviendo a la izquierda...");
                // Asumimos que la dirección 0 es izquierda
                //execute_movement(MANUAL_MOVE_PULSES, MANUAL_MOVE_SPEED_HZ, 0);
                motor_command_t cmd = {
                    .num_pulses = JOG_PULSES,
                    .frequency = JOG_SPEED_HZ,
                    .direction = 0
                };
                xQueueOverwrite(motor_command_queue, &cmd);
            }
            // Si se presiona el botón derecho Y no el izquierdo
            else if (right_button_state == 1 && left_button_state == 0 && gpio_get_level(EMERGENCY_STOP_GPIO_LEFT) == 1)
            {
                status_set_manual_move_state(MANUAL_MOVE_LEFT); // Reportar estado
                // Por seguridad, podríamos comprobar si el PID está deshabilitado aquí
                ESP_LOGD(TAG, "Moviendo a la derecha...");
                // Asumimos que la dirección 1 es derecha
                // execute_movement(MANUAL_MOVE_PULSES, MANUAL_MOVE_SPEED_HZ, 1);
                motor_command_t cmd = {
                    .num_pulses = JOG_PULSES,
                    .frequency = JOG_SPEED_HZ,
                    .direction = 1
                };
                xQueueOverwrite(motor_command_queue, &cmd);
            }
            else
            {
                // Si no se presiona ningún botón o ambos, detenemos el movimiento
                status_set_manual_move_state(MANUAL_MOVE_NONE); // Reportar estado
                // No hacemos nada. El motor se detendrá al completar los pulsos.
            }
        }
        else
        {
            // Si el PID está activado, nos aseguramos de que el estado manual esté limpio
            status_set_manual_move_state(MANUAL_MOVE_NONE);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Sondeo mayor o igual a 10ms 10ms
    }
}