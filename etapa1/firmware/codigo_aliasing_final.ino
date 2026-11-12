#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver/i2s.h"

// ============================================================
// PINES
// ============================================================
#define PIN_BOTON 5

#define I2S_MIC_BCLK 18
#define I2S_MIC_LRC  17
#define I2S_MIC_DIN  15

#define I2S_SPK_BCLK 3
#define I2S_SPK_LRC  8
#define I2S_SPK_DOUT 16
#define I2S_SPK_SD   46

// ============================================================
// CONFIGURACION
// ============================================================
const uint32_t TASA_MUESTREO = 16000;
const size_t MUESTRAS_POR_BLOQUE = 256;
const size_t FACTOR_DIEZMADO = 4;

// Buffers estáticos con tamaño fijo para evitar asignación dinámica en tiempo real.
// El micrófono entrega 32 bits, el altavoz recibe 16 bits.
int32_t buffer_lectura_mic[MUESTRAS_POR_BLOQUE];
int16_t buffer_escritura_spk[MUESTRAS_POR_BLOQUE];

// ============================================================
// FUNCIONES
// ============================================================
void configurar_pines();
void configurar_microfono_i2s();
void configurar_altavoz_i2s();
void procesar_audio_tiempo_real();

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);

    configurar_pines();
    configurar_microfono_i2s();
    configurar_altavoz_i2s();

    Serial.println("Sistema iniciado.");
    Serial.println("Boton: loopback de audio con aliasing.");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    // Modo normal: procesar audio y aplicar aliasing al presionar el botón
    if (digitalRead(PIN_BOTON) == LOW) {
        delay(50); // Antirrebote básico

        if (digitalRead(PIN_BOTON) == LOW) {
            procesar_audio_tiempo_real();
        }
    }

    delay(5);
}

// ============================================================
// PINES
// ============================================================
void configurar_pines() {
    pinMode(PIN_BOTON, INPUT_PULLUP);

    pinMode(I2S_SPK_SD, OUTPUT);
    digitalWrite(I2S_SPK_SD, LOW);
}

// ============================================================
// CONFIGURACION I2S DEL MICROFONO
// ============================================================
void configurar_microfono_i2s() {
    i2s_config_t configuracion_i2s_rx = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = TASA_MUESTREO,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, 
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,   
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = MUESTRAS_POR_BLOQUE
    };

    i2s_pin_config_t pines_rx = {
        .bck_io_num = I2S_MIC_BCLK,
        .ws_io_num = I2S_MIC_LRC,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_MIC_DIN
    };

    i2s_driver_install(I2S_NUM_0, &configuracion_i2s_rx, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pines_rx);
}

// ============================================================
// CONFIGURACION I2S DEL ALTAVOZ
// ============================================================
void configurar_altavoz_i2s() {
    i2s_config_t configuracion_i2s_tx = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = TASA_MUESTREO,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = MUESTRAS_POR_BLOQUE
    };

    i2s_pin_config_t pines_tx = {
        .bck_io_num = I2S_SPK_BCLK,
        .ws_io_num = I2S_SPK_LRC,
        .data_out_num = I2S_SPK_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_1, &configuracion_i2s_tx, 0, NULL);
    i2s_set_pin(I2S_NUM_1, &pines_tx);
}

// ============================================================
// PROCESAMIENTO: GENERACION DE ALIASING
// ============================================================
void procesar_audio_tiempo_real() {
    Serial.println("Loopback con Aliasing iniciado.");

    // Encender el amplificador MAX98357A
    digitalWrite(I2S_SPK_SD, HIGH);
    delay(20);

    size_t bytes_leidos = 0;
    size_t bytes_escritos = 0;

    // Calcular el tamaño estático en bytes para lectura (32 bits = 4 bytes por muestra)
    const size_t tamano_lectura_bytes = MUESTRAS_POR_BLOQUE * sizeof(int32_t);

    // Bucle mientras el botón se mantenga presionado
    while (digitalRead(PIN_BOTON) == LOW) {

        // 1. Lectura de muestras desde el micrófono
        i2s_read(
            I2S_NUM_0,
            buffer_lectura_mic,
            tamano_lectura_bytes,
            &bytes_leidos,
            portMAX_DELAY
        );

        if (bytes_leidos > 0) {
            
            // Determinar exactamente cuántas muestras de 32 bits ingresaron
            size_t muestras_leidas = bytes_leidos / sizeof(int32_t);

            // 2. Procesamiento de Señal: Diezmado para forzar Aliasing
            // El bucle tradicional avanza en saltos de 4 muestras.
            for (size_t indice = 0; indice < muestras_leidas; indice = indice + FACTOR_DIEZMADO) {
                
                // Extraer la muestra actual en crudo de 32 bits
                int32_t muestra_cruda = buffer_lectura_mic[indice];
                
                // El INMP441 justifica los datos a la izquierda. 
                // Se desplaza a la derecha 14 bits para adaptar el volumen y el rango
                // al formato de 16 bits que espera el MAX98357A.
                int16_t muestra_adaptada = (int16_t)(muestra_cruda >> 14);

                // Retenedor de Orden Cero: 
                // Se duplica la misma muestra adaptada en las siguientes 3 posiciones
                // para mantener el reloj de salida a 16kHz mientras la señal efectiva cae a 4kHz.
                buffer_escritura_spk[indice]     = muestra_adaptada;
                buffer_escritura_spk[indice + 1] = muestra_adaptada;
                buffer_escritura_spk[indice + 2] = muestra_adaptada;
                buffer_escritura_spk[indice + 3] = muestra_adaptada;
            }

            // Calcular el tamaño en bytes de los datos procesados (16 bits = 2 bytes por muestra)
            size_t tamano_escritura_bytes = muestras_leidas * sizeof(int16_t);

            // 3. Escritura hacia el amplificador
            i2s_write(
                I2S_NUM_1,
                buffer_escritura_spk,
                tamano_escritura_bytes,
                &bytes_escritos,
                portMAX_DELAY
            );
        }
    }

    // Limpiar el buffer de hardware y apagar el amplificador al soltar el botón
    i2s_zero_dma_buffer(I2S_NUM_1);
    delay(50);
    digitalWrite(I2S_SPK_SD, LOW);

    Serial.println("Loopback con Aliasing finalizado.");
}