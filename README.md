# Colección de Librerías de Control de Movimiento para ESP32-IDF

![ESP32](https://img.shields.io/badge/Plataforma-ESP32-blueviolet) ![Framework](https://img.shields.io/badge/Framework-ESP--IDF-orange) ![License](https://img.shields.io/badge/License-MIT-green)

Este repositorio contiene una colección de librerías modulares y reutilizables para el control de movimiento con el microcontrolador ESP32 y el framework ESP-IDF, desarrolladas en el entorno de PlatformIO, como parte de Pasantías en la **Escuela de Ingeniería Eléctrica** de la **Universidad Central de Venezuela (UCV)**.

El objetivo principal es proporcionar bloques de construcción de software limpios y bien documentados para proyectos que incorporen encoder, motores de paso y pantallas lcd.

Un agradecimiento especial al **Laboratorio de Investigación y Desarrollo Electrónico (LIDE)** por proporcionar los recursos y el espacio para la investigación y el desarrollo.

### Autores

- Br. Ricardo Santana
- Br. Axel Rivero

### Tutor Académico

- Ing. Prof. Alejandro Herrera

## Librerías Incluidas

Este paquete incluye las siguientes librerías:

- ### Encoder

  Una librería robusta para leer encoders incrementales en cuadratura utilizando el periférico de hardware PCNT del ESP32. Incluye detección de señal de índice (Z) y conversión a grados.

- ### StepperMotor

  Una librería de alto nivel para controlar drivers de motores de paso (como el DM542T). Abstrae la generación de pulsos (STEP/PUL) y la señal de dirección (DIR), permitiendo comandar movimientos precisos en pasos y velocidad.

- ### I2C_LCD

  Un controlador para pantallas de caracteres LCD con adaptador I2C. Proporciona funciones simples para inicializar, limpiar y escribir texto en la pantalla, ideal para la visualización de datos de telemetría.

- ### LCD_Native
  Un controlador para pantallas de caracteres LCD con interfaz paralela (GPIO). Permite la comunicación directa sin adaptadores externos, proporcionando funciones eficientes para inicializar y visualizar texto, ideal para implementaciones que requieren control nativo del hardware.

## Cómo Usar

Este repositorio está estructurado para ser usado con PlatformIO. La forma más fácil de utilizar estas librerías es:

1.  Clona o descarga este repositorio.
2.  Copia las subcarpetas deseadas de la carpeta `lib` a la carpeta `lib` de tu propio proyecto de PlatformIO.
3.  Incluye la cabecera correspondiente en tu código (ej. `#include "encoder.h"`).
4.  ¡Empieza a usar las funciones de la librería!

## Ejemplos de Uso

La carpeta `examples` contiene varios proyectos de PlatformIO completos que demuestran cómo usar cada librería de forma individual y en conjunto.

- **[01_Encoder_Demo](./examples/01_Encoder_Demo/)**: Un ejemplo simple que inicializa el encoder y muestra su ángulo en la consola serie.
- **[02_StepperMotor_Demo](./examples/02_StepperMotor_Demo/)**: Demuestra cómo mover el motor en diferentes direcciones y velocidades.
- **[03_I2C_LCD_Demo](./examples/03_I2C_LCD_Demo/)**: Muestra cómo escribir texto y datos en la pantalla LCD.
- **[04_Inverted_Pendulum_Project](./examples/04_Inverted_Pendulum_Project/)**: Un proyecto avanzado completo que integra las tres librerías para construir un péndulo invertido en desarrollo, incluyendo control PID, sintonización por UART, y visualización en LCD.
- **[05_LCD_Without_I2C_Demo](./examples/05_LCD_Without_I2C_Demo/)**: Muestra cómo escribir texto y datos en la pantalla LCD en conexión directa sin adaptador I2C.

Para ejecutar un ejemplo, abre su carpeta en VS Code con PlatformIO y sube el código a tu ESP32.

## Licencia

Este proyecto está bajo la licencia MIT. Ver el archivo [LICENSE](./LICENSE) para más detalles.
