// src/main.c
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uart_echo.h"      // Para la tarea de comandos UART
#include "stepperMotor.h"  // Para la tarea de control del motor

static const char *TAG = "MOTOR";

void app_main(void) {

  ledc_init(); // Inicializa el módulo LEDC
  uart_init(); // Inicializa el uart

  uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
  if (data == NULL) {
      ESP_LOGE(TAG, "No se pudo asignar memoria para el buffer UART");
      vTaskDelete(NULL);
  }
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
                        execute_movement(num_pulses, frequency, direction);
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