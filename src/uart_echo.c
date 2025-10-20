#include <stdio.h>
#include <string.h>
#include <stdlib.h> // Para atoi
#include "uart_echo.h"
#include "freertos/queue.h" // Necesario para crear la cola

static const char *TAG = "UART_ECHO";

void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT, //UART_SCLK_APB para versiones antiguas
    };

    // Instalar el driver del UART
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    
    // Configurar los pines del UART (usando los por defecto)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "Driver del UART inicializado.");
}