#include <stdio.h>
#include <string.h>
#include <stdlib.h> // Para atoi
#include "uart_echo.h"
#include "freertos/queue.h" // Necesario para crear la cola
#include "pid_controller.h"

static const char *TAG = "UART_ECHO";

// Definimos la cola de comandos
// QueueHandle_t pwm_command_queue;

void uart_echo_task(void *arg) {

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

    //ESP_LOGI(TAG, "Tarea de eco UART iniciada.");
    //ESP_LOGI(TAG, "Formato: PULSOS <#pulsos> <frec> <dir>. Ej: PULSOS 200 1000 1");
    ESP_LOGI(TAG, "Consola de Sintonización iniciada.");
    ESP_LOGI(TAG, "Comandos: SETKP <valor>, SETKI <valor>, SETKD <valor>");

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, (BUF_SIZE - 1), 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recibido: %s", (char *) data);
            //uart_write_bytes(UART_PORT, (const char *) data, len); // Eco de lo recibido

            // --- LÓGICA DE PARSEO ---
            char *cmd = strtok((char *)data, " ");
            char *val = strtok(NULL, " \r\n");

            if (cmd != NULL && val != NULL) {
                float value = atof(val); // Convertir el string a float
                if (strcmp(cmd, "SETKP") == 0) {
                    pid_set_kp(value);
                } else if (strcmp(cmd, "SETKI") == 0) {
                    pid_set_ki(value);
                } else if (strcmp(cmd, "SETKD") == 0) {
                    pid_set_kd(value);
                } else {
                    uart_write_bytes(UART_PORT, "Comando desconocido.\r\n", strlen("Comando desconocido.\r\n"));
                }
            } else {
                 uart_write_bytes(UART_PORT, "Formato incorrecto.\r\n", strlen("Formato incorrecto.\r\n"));
            }
        }
    }
}