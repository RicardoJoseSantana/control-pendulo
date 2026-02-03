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

// Estructura para los comandos que enviaremos a la tarea PWM
typedef struct {
    int num_pulses;
    int frequency;
    int direction; // 'I' para Izquierda, 'D' para Derecha
} pwm_command_t;

// Variable externa para la cola de comandos
extern QueueHandle_t pwm_command_queue;

// Declaración de la tarea UART echo
void uart_echo_task(void *arg);

#endif // UART_ECHO_H