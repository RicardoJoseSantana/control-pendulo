#include <stdio.h>
#include <string.h>
#include <stdlib.h> // Para atoi
#include "uart_echo.h"
#include "freertos/queue.h" // Necesario para crear la cola
#include "state_controller.h"

static const char *TAG = "UART_ECHO";

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

    ESP_LOGI(TAG, "Consola de Sintonización iniciada.");
    ESP_LOGI(TAG, "Comandos: SETPOS <metros>, TELEMETRY <0|1>");

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recibido: %s", (char *) data);

            // --- LÓGICA DE PARSEO MODIFICADA ---
            char *cmd = strtok((char *)data, " ");
            char *val_str = strtok(NULL, " \r\n");

            if (cmd != NULL && val_str != NULL) {
                if (strcmp(cmd, "SETPOS") == 0) {
                    float position_m = atof(val_str);
                    state_controller_set_cart_setpoint(position_m);
                    ESP_LOGI(TAG, "Nuevo setpoint de posición del carro: %.2f m", position_m);
                } 
                // Aquí podrías añadir los comandos SETK1, SETK2, etc. si los necesitas.
                // else if (strcmp(cmd, "TELEMETRY") == 0) { ... }
                else {
                    uart_write_bytes(UART_PORT, "Comando desconocido.\r\n", strlen("Comando desconocido.\r\n"));
                }
            } else {
                 uart_write_bytes(UART_PORT, "Formato incorrecto.\r\n", strlen("Formato incorrecto.\r\n"));
            }
        }
    }
}