// src/pulse_counter.c

#include "pulse_counter.h" // Incluimos nuestra propia cabecera
#include <stdio.h>
#include <stdbool.h> // --- AÑADIDO --- Para usar 'bool'
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"

// --- Definiciones privadas del módulo ---
// --- Definiciones privadas del módulo ---
#define PULSE_GPIO_A GPIO_NUM_27 // --- RENOMBRADO --- Pin para la señal A del encoder
#define PULSE_GPIO_B GPIO_NUM_14 // --- AÑADIDO --- Pin para la señal B del encoder
#define PULSE_GPIO_Z GPIO_NUM_12 // --- AÑADIDO --- Pin para la señal Z (índice) del encoder

static const char *TAG = "PULSE_COUNTER";

// --- Definición de la variableS globalES ---
// Esta es la definición real de la variable declarada como 'extern' en el .h
volatile int g_pulse_count = 0;
volatile bool g_z_pulse_detected = false; // --- AÑADIDO ---

// --- Funciones privadas del módulo ---

/**
 * @brief Rutina de servicio de interrupción (ISR).
 *  ISR para el canal A. Ahora determina la dirección.
 * Se ejecuta cada vez que se detecta un flanco de subida en el GPIO A.
 * ¡Debe ser lo más rápida posible!
 */
static void IRAM_ATTR gpio_isr_handler_A(void *arg)
{
    // Leemos el estado del canal B en el instante exacto del flanco de A
    if (gpio_get_level(PULSE_GPIO_B) == 0)
    {
        // Si B es BAJO, giramos en una dirección (ej. horario)
        g_pulse_count++;
    }
    else
    {
        // Si B es ALTO, giramos en la dirección opuesta (ej. anti-horario)
        g_pulse_count--;
    }
}

/**
 * @brief --- AÑADIDO --- ISR para el canal Z. Solo establece un flag.
 * Se ejecuta cada vez que se detecta un flanco de subida en el GPIO Z.
 */
static void IRAM_ATTR gpio_isr_handler_Z(void *arg)
{
    g_z_pulse_detected = true;
}

/**
 * @brief Función principal de la tarea de conteo de pulsos.
 */
void pulse_counter_task(void *arg)
{
    ESP_LOGI(TAG, "Iniciando tarea de contador de pulsos en GPIOs A:%d, B:%d, Z:%d", PULSE_GPIO_A, PULSE_GPIO_B, PULSE_GPIO_Z);

    // 1. Configuración de los 3 pines GPIO
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,                                                                    // Todos son pines de entrada
        .pin_bit_mask = ((1ULL << PULSE_GPIO_A) | (1ULL << PULSE_GPIO_B) | (1ULL << PULSE_GPIO_Z)), // Máscara para los 3 pines
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Habilitar pull-up interno para todos
        .intr_type = GPIO_INTR_POSEDGE,   // Interrupción en flanco de subida para A y Z
    };
    gpio_config(&io_conf);

    // El pin B no necesita interrupción, solo lo leemos. La deshabilitamos para él.
    gpio_set_intr_type(PULSE_GPIO_B, GPIO_INTR_DISABLE);

    // 2. Instalar el servicio de interrupción del GPIO (una sola vez por aplicación)
    // Usamos ESP_INTR_FLAG_IRAM para que la ISR se ejecute desde la RAM, siendo más rápida.
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    // 3. Agregar el manejador de interrupción para nuestroS pinES específicoS
    gpio_isr_handler_add(PULSE_GPIO_A, gpio_isr_handler_A, (void *)PULSE_GPIO_A);
    gpio_isr_handler_add(PULSE_GPIO_Z, gpio_isr_handler_Z, (void *)PULSE_GPIO_Z);

    ESP_LOGI(TAG, "Configuración completa. Esperando pulsos...");

    int last_pulse_count = 0; // Para detectar cambios

    // Bucle principal de la tarea
    while (1)
    {
        // --- MODIFICADO --- Lógica del bucle principal

        // Comprobamos si el pulso de índice ha sido detectado por la ISR
        if (g_z_pulse_detected)
        {
            ESP_LOGW(TAG, "¡Pulso de ÍNDICE (Z) detectado! Reseteando contador a CERO. Valor anterior: %d", g_pulse_count);
            g_pulse_count = 0;
            g_z_pulse_detected = false; // Importante: Limpiamos el flag después de procesarlo
            last_pulse_count = 0;
        }

        // Imprimimos la posición solo si ha cambiado, para no saturar la consola
        if (g_pulse_count != last_pulse_count)
        {
            ESP_LOGI(TAG, "Posición actual: %d", g_pulse_count);
            last_pulse_count = g_pulse_count;
        }

        // Dormimos la tarea por un breve periodo para ceder la CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}