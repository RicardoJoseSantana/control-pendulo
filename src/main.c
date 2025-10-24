// src/main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stepperMotor.h" // ¡Solo incluimos nuestra nueva librería!
#include "esp_log.h"
#include "uart_echo.h"

#define STEP_PIN 12
#define DIR_PIN 13

void mov_continuo(void);
void mov_manual(void);

void app_main(void) {

    // 1. Configurar la librería con los pines que queremos usar
    stepper_config_t motor_cfg = {
        .step_pin = STEP_PIN,
        .dir_pin = DIR_PIN
    };
    
    // 2. Inicializar la librería
    esp_err_t ret = stepper_init(&motor_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Fallo al inicializar el motor, deteniendo.");
        return;
    }

    // 3. ¡Usar la librería!
    ESP_LOGI("MAIN", "Iniciando movimiento de prueba...");

    //-----Escoger una de las opciones, comentar la otra-----

    // OPCION A: movimiento continuo
    mov_continuo();

    // OPCION B: movimiento manual (por comandos UART)
    //mov_manual();
  
}

void mov_continuo(void) {
    while (1) {

        // Mover 10000 pasos (una vuelta) a 10kHz en dirección 1
        ESP_LOGI("MAIN", "Moviendo a la derecha...");
        stepper_move(10000, 10000, 1);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Pausa de 1 segundo

        // Mover 10000 pasos (dos vueltas) a 10kHz en dirección 0
        ESP_LOGI("MAIN", "Moviendo a la izquierda más rápido...");
        stepper_move(20000, 10000, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
    }
}

void mov_manual(void) {
    uart_init(); // Inicializa el uart

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE("MAIN", "No se pudo asignar memoria para el buffer UART");
        vTaskDelete(NULL);
    }
    ESP_LOGI("MAIN", "Formato: PULSOS <#pulsos> <frec> <dir>. Ej: PULSOS 200 1000 1");

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI("MAIN", "Ejecutando: %s", (char *) data);
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
                        stepper_move(num_pulses, frequency, direction);
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