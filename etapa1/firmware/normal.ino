#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include "driver/i2s.h"

// ============================================================================
// DEFINICIÓN DE PINES Y CONSTANTES
// ============================================================================
#define PIN_BOTON 5

// Pines I2S para el micrófono (INMP441)
#define I2S_MIC_BCLK 18
#define I2S_MIC_LRC  17
#define I2S_MIC_DIN  15

// Pines I2S para el altavoz (MAX98357A)
#define I2S_SPK_BCLK 3
#define I2S_SPK_LRC  8
#define I2S_SPK_DOUT 16
#define I2S_SPK_SD   46   // Pin de Shutdown para el amplificador

const uint32_t TASA_MUESTREO = 16000;
const uint32_t BITS_POR_MUESTRA = 16;

// Tamaño del bloque de procesamiento (chunk). 
// Un tamaño pequeño (ej. 512 muestras) reduce el retardo (latencia) entre 
// que hablas y escuchas, pero exige que el bucle procese más rápido.
const size_t MUESTRAS_POR_BLOQUE = 512;
const size_t TAMANO_BLOQUE_BYTES = MUESTRAS_POR_BLOQUE * (BITS_POR_MUESTRA / 8);

// Buffer estático asignado en la memoria RAM interna (SRAM).
// No usamos memoria dinámica (malloc) para garantizar estabilidad continua.
int16_t buffer_audio_tiempo_real[MUESTRAS_POR_BLOQUE];

// ============================================================================
// DECLARACIÓN DE FUNCIONES MODULARES
// ============================================================================
void configurar_pines();
void configurar_microfono_i2s();
void configurar_altavoz_i2s();
void procesar_audio_tiempo_real();

// ============================================================================
// CONFIGURACIÓN INICIAL (SETUP)
// ============================================================================
void setup() {
    Serial.begin(115200);

    configurar_pines();
    configurar_microfono_i2s();
    configurar_altavoz_i2s();

    Serial.println("Sistema de audio en tiempo real inicializado.");
    Serial.println("Mantenga presionado el boton para hablar...");
}

// ============================================================================
// BUCLE PRINCIPAL (LOOP)
// ============================================================================
void loop() {
    if (digitalRead(PIN_BOTON) == LOW) {
        delay(50); // Filtro simple de rebote mecánico (debounce)
        
        // Confirmar que fue una pulsación real y no ruido eléctrico
        if (digitalRead(PIN_BOTON) == LOW) {
            // Entrar a la rutina de procesamiento continuo
            procesar_audio_tiempo_real();
        }
    }
    
    // Un pequeño retardo para no saturar el procesador cuando está en reposo
    delay(10);
}

// ============================================================================
// IMPLEMENTACIÓN DE FUNCIONES
// ============================================================================

void configurar_pines() {
    pinMode(PIN_BOTON, INPUT_PULLUP);

    // Configurar el pin de apagado (Shutdown) del amplificador. LOW = Apagado.
    pinMode(I2S_SPK_SD, OUTPUT);
    digitalWrite(I2S_SPK_SD, LOW);
}

void configurar_microfono_i2s() {
    i2s_config_t configuracion_i2s_rx = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = TASA_MUESTREO,
        .bits_per_sample = (i2s_bits_per_sample_t)BITS_POR_MUESTRA,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 256 // Buffers DMA pequeños para baja latencia
    };

    i2s_pin_config_t configuracion_pines_rx = {
        .bck_io_num = I2S_MIC_BCLK,
        .ws_io_num = I2S_MIC_LRC,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_MIC_DIN
    };

    i2s_driver_install(I2S_NUM_0, &configuracion_i2s_rx, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &configuracion_pines_rx);
}

void configurar_altavoz_i2s() {
    i2s_config_t configuracion_i2s_tx = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = TASA_MUESTREO,
        .bits_per_sample = (i2s_bits_per_sample_t)BITS_POR_MUESTRA,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 256 // Buffers DMA pequeños para baja latencia
    };

    i2s_pin_config_t configuracion_pines_tx = {
        .bck_io_num = I2S_SPK_BCLK,
        .ws_io_num = I2S_SPK_LRC,
        .data_out_num = I2S_SPK_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_1, &configuracion_i2s_tx, 0, NULL);
    i2s_set_pin(I2S_NUM_1, &configuracion_pines_tx);
}

void procesar_audio_tiempo_real() {
    Serial.println("Transmision en vivo iniciada...");
    
    // 1. Encender el amplificador
    digitalWrite(I2S_SPK_SD, HIGH);
    delay(20); // Esperar estabilización de corriente para evitar chasquido inicial
    
    size_t bytes_leidos = 0;
    size_t bytes_escritos = 0;

    // 2. Bucle de procesamiento de señal (Signal Processing Loop)
    // Se ejecuta ininterrumpidamente mientras el botón siga presionado
    while (digitalRead(PIN_BOTON) == LOW) {
        
        // Paso A: Capturar un fragmento de sonido del bus DMA del micrófono
        i2s_read(I2S_NUM_0, buffer_audio_tiempo_real, TAMANO_BLOQUE_BYTES, &bytes_leidos, portMAX_DELAY);
        
        // Opcional: Aquí se podría aplicar DSP (filtros, ganancias, etc.) 
        // iterando sobre buffer_audio_tiempo_real mediante un bucle for.
        
        // Paso B: Inyectar el mismo fragmento al bus DMA del altavoz
        i2s_write(I2S_NUM_1, buffer_audio_tiempo_real, bytes_leidos, &bytes_escritos, portMAX_DELAY);
    }

    // 3. Secuencia de apagado limpio al soltar el botón
    Serial.println("Transmision finalizada.");
    
    // Limpiar el DMA del altavoz con ceros para evitar el zumbido residual (Offset DC)
    i2s_zero_dma_buffer(I2S_NUM_1);
    
    // Retardo necesario para que los ceros viajen hasta el chip amplificador
    delay(50);
    
    // Apagar el amplificador físicamente
    digitalWrite(I2S_SPK_SD, LOW);
}