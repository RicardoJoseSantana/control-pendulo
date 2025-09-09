#include <stdio.h>
#include <string.h>
#include <stdlib.h> // Para atoi
#include "uart_echo.h"
#include "freertos/queue.h" // Necesario para crear la cola

static const char *TAG = "UART_ECHO";

// Definimos la cola de comandos
QueueHandle_t pwm_command_queue;

void uart_echo_task(void *arg) {
    // Creamos la cola aquí. ¡Es importante que se cree antes de que ambas tareas intenten usarla!
    // El tamaño de la cola determina cuántos comandos puede almacenar antes de bloquearse
    pwm_command_queue = xQueueCreate(10, sizeof(pwm_command_t));
    if (pwm_command_queue == NULL) {
        ESP_LOGE(TAG, "No se pudo crear la cola de comandos PWM");
        vTaskDelete(NULL);
    }

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "No se pudo asignar memoria para el buffer UART");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Tarea de eco UART iniciada.");
    ESP_LOGI(TAG, "Formato: PULSOS <#pulsos> <frec> <dir>. Ej: PULSOS 200 1000 1");

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recibido: %s", (char *) data);
            //uart_write_bytes(UART_PORT, (const char *) data, len); // Eco de lo recibido

            // --- LÓGICA DE PARSEO ---
            char *cmd_token = strtok((char *)data, " ");
            if (cmd_token != NULL && strcmp(cmd_token, "PULSOS") == 0) {
                
                char *pulses_token = strtok(NULL, " ");
                char *freq_token = strtok(NULL, " ");
                char *dir_token = strtok(NULL, " \r\n");

                if (pulses_token != NULL && freq_token != NULL && dir_token != NULL) {
                    int num_pulses = atoi(pulses_token);
                    int frequency = atoi(freq_token);
                    int direction = atoi(dir_token);

                    // Validar los datos recibidos
                    if (num_pulses > 0 && frequency > 0 && (direction == 0 || direction == 1)) {
                        pwm_command_t command;
                        command.num_pulses = num_pulses;
                        command.frequency = frequency;
                        command.direction = direction;
                        
                        if (xQueueSend(pwm_command_queue, &command, portMAX_DELAY) != pdPASS) {
                            ESP_LOGE(TAG, "No se pudo enviar el comando a la cola");
                        } else {
                            ESP_LOGI(TAG, "Comando enviado: %d pulsos, %d Hz, Dir: %d", command.num_pulses, command.frequency, command.direction);
                        }
                    } else {
                        uart_write_bytes(UART_PORT, "Error: Argumentos inválidos.\r\n", strlen("Error: Argumentos inválidos.\r\n"));
                    }
                } else {
                    uart_write_bytes(UART_PORT, "Error: Faltan argumentos. Formato: PULSOS <#pulsos> <frec> <dir>\r\n", strlen("Error: Faltan argumentos. Formato: PULSOS <#pulsos> <frec> <dir>\r\n"));
                }
            } else {
                 uart_write_bytes(UART_PORT, "Comando desconocido.\r\n", strlen("Comando desconocido.\r\n"));
            }
        }
    }
}