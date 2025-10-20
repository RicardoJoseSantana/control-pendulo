#ifndef UART_ECHO_H
#define UART_ECHO_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/queue.h" // Incluimos FreeRTOS Queue

// Define el puerto UART y el tamaño del buffer
#define UART_PORT UART_NUM_0
#define BUF_SIZE (1024)

void uart_init(void);

#endif // UART_ECHO_H