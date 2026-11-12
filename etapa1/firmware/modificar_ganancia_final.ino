#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <Wire.h>
#include "driver/i2s.h"

// ============================================================
// ASIGNACION DE PINES GPIO
// ============================================================
#define PIN_BOTON 5

// Pines I2S para el Microfono INMP441 (RX - I2S_NUM_0)
#define I2S_MIC_BCLK 18
#define I2S_MIC_LRC  17
#define I2S_MIC_DIN  15

// Pines I2S para el Altavoz MAX98357A (TX - I2S_NUM_1)
#define I2S_SPK_BCLK 3
#define I2S_SPK_LRC  8
#define I2S_SPK_DOUT 16
#define I2S_SPK_SD   46

// Bus I2C para el conversor ADS1115
#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 10

// ============================================================
// CONFIGURACION GENERAL Y CONSTANTES
// ============================================================
const uint32_t TASA_MUESTREO = 16000;

// Cantidad de muestras de audio por bloque DMA
const size_t MUESTRAS_POR_BLOQUE = 512;
const size_t TAMANO_BUFFER_RX_BYTES = MUESTRAS_POR_BLOQUE * sizeof(int32_t);
const size_t TAMANO_BUFFER_TX_BYTES = MUESTRAS_POR_BLOQUE * sizeof(int16_t);

// Direccion I2C del ADS1115 (pin ADDR conectado a GND)
const uint8_t DIRECCION_ADS1115 = 0x48;

// Rango maximo de ganancia digital escalada (0.0 a 0.50 para evitar acople y recorte)
const float GANANCIA_MINIMA = 0.0f;
const float GANANCIA_MAXIMA = 0.50f;
static float factor_ganancia_actual = 0.10f;

// Buffers estaticos de transmision y recepcion
static int32_t buffer_rx_32bits[MUESTRAS_POR_BLOQUE];
static int16_t buffer_tx_16bits[MUESTRAS_POR_BLOQUE];

// ============================================================
// DECLARACION DE FUNCIONES MODULARES
// ============================================================
void configurar_pines();
void configurar_i2c_ads1115();
void configurar_microfono_i2s();
void configurar_altavoz_i2s();
uint16_t leer_ads1115_canal_0();
void actualizar_ganancia_en_vivo();
void procesar_audio_tiempo_real();

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);

    configurar_pines();
    configurar_i2c_ads1115();
    configurar_microfono_i2s();
    configurar_altavoz_i2s();

    Serial.println("Sistema iniciado con control de ganancia en vivo.");
    Serial.println("Mantenga presionado el boton para loopback y gire el potenciometro.");
}

// ============================================================
// LOOP PRINCIPAL
// ============================================================
void loop() {
    // Lectura del boton con antirrebote simple
    if (digitalRead(PIN_BOTON) == LOW) {
        delay(50);
        if (digitalRead(PIN_BOTON) == LOW) {
            procesar_audio_tiempo_real();
        }
    }

    delay(5);
}

// ============================================================
// CONFIGURACION DE PINES GPIO
// ============================================================
void configurar_pines() {
    pinMode(PIN_BOTON, INPUT_PULLUP);

    // Pin Shutdown del amplificador MAX98357A (LOW = mute/apagado)
    pinMode(I2S_SPK_SD, OUTPUT);
    digitalWrite(I2S_SPK_SD, LOW);
}

// ============================================================
// CONFIGURACION DEL BUS I2C Y MODULO ADS1115
// ============================================================
void configurar_i2c_ads1115() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);

    // Registro de configuracion del ADS1115 (0x01):
    // Canal simple A0 vs GND (MUX = 100)
    // Rango de escala completa FSR = +/- 4.096 V (PGA = 001)
    // Modo de conversion continua (MODE = 0)
    // Byte superior (MSB) = 0x42
    // Tasa de 128 SPS, comparador deshabilitado
    // Byte inferior (LSB) = 0x83
    Wire.beginTransmission(DIRECCION_ADS1115);
    Wire.write(0x01);
    Wire.write(0x42);
    Wire.write(0x83);
    Wire.endTransmission();

    // Establecer el puntero al registro de conversion (0x00) para lecturas directas
    Wire.beginTransmission(DIRECCION_ADS1115);
    Wire.write(0x00);
    Wire.endTransmission();
}

// ============================================================
// LECTURA DIRECTA DEL CANAL A0 (ADS1115)
// ============================================================
uint16_t leer_ads1115_canal_0() {
    Wire.requestFrom(DIRECCION_ADS1115, (uint8_t)2);

    if (Wire.available() >= 2) {
        uint8_t byte_alto = Wire.read();
        uint8_t byte_bajo = Wire.read();
        int16_t valor_bruto = (int16_t)((byte_alto << 8) | byte_bajo);

        // Asegurar rango no negativo por ruidos de referencia en 0 V
        if (valor_bruto < 0) {
            return 0;
        }
        return (uint16_t)valor_bruto;
    }

    return 0;
}

// ============================================================
// ACTUALIZACION DEL PARAMETRO DE GANANCIA
// ============================================================
void actualizar_ganancia_en_vivo() {
    uint16_t cuentas_adc = leer_ads1115_canal_0();

    // Con rango de +/-4.096 V, una tension de 3.3 V equivale aprox. a 26400 cuentas
    const float CUENTAS_MAXIMAS = 26400.0f;
    float cuentas_flotante = (float)cuentas_adc;

    if (cuentas_flotante > CUENTAS_MAXIMAS) {
        cuentas_flotante = CUENTAS_MAXIMAS;
    }

    // Normalizacion de 0.0 a 1.0 segun la posicion angular del potenciometro
    float posicion_normalizada = cuentas_flotante / CUENTAS_MAXIMAS;

    // Escalamiento lineal al rango de ganancia permitido
    factor_ganancia_actual = GANANCIA_MINIMA + (posicion_normalizada * (GANANCIA_MAXIMA - GANANCIA_MINIMA));
}

// ============================================================
// CONFIGURACION I2S DEL MICROFONO (RX - PUERTO 0)
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
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
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
// CONFIGURACION I2S DEL ALTAVOZ (TX - PUERTO 1)
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
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
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
// LOOPBACK CON CONTROL DE GANANCIA EN TIEMPO REAL
// ============================================================
void procesar_audio_tiempo_real() {
    Serial.println("Loopback iniciado.");

    // Habilitar la etapa de potencia del altavoz
    digitalWrite(I2S_SPK_SD, HIGH);
    delay(20);

    size_t bytes_leidos = 0;
    size_t bytes_escritos = 0;

    while (digitalRead(PIN_BOTON) == LOW) {

        // 1. Actualizar el valor de ganancia leyendo el ADS1115
        actualizar_ganancia_en_vivo();

        // 2. Capturar bloque de muestras desde el microfono I2S
        i2s_read(
            I2S_NUM_0,
            (void*)buffer_rx_32bits,
            TAMANO_BUFFER_RX_BYTES,
            &bytes_leidos,
            portMAX_DELAY
        );

        size_t muestras_leidas = bytes_leidos / sizeof(int32_t);

        // 3. Procesamiento aritmetico muestra por muestra
        for (size_t indice = 0; indice < muestras_leidas; indice++) {
            int32_t muestra_32 = buffer_rx_32bits[indice];

            // Alinear los 24 bits significativos del microfono a un rango de 16 bits
            int32_t muestra_base = muestra_32 >> 15;

            // Multiplicacion por la ganancia controlada por el potenciometro
            float muestra_escalada_f = (float)muestra_base * factor_ganancia_actual;
            int32_t muestra_salida = (int32_t)muestra_escalada_f;

            // Saturacion para evitar desbordamiento en int16_t
            if (muestra_salida > 32767) {
                muestra_salida = 32767;
            } else if (muestra_salida < -32768) {
                muestra_salida = -32768;
            }

            buffer_tx_16bits[indice] = (int16_t)muestra_salida;
        }

        // 4. Transmitir el bloque escalado al altavoz
        size_t bytes_a_enviar = muestras_leidas * sizeof(int16_t);
        if (bytes_a_enviar > 0) {
            i2s_write(
                I2S_NUM_1,
                (const void*)buffer_tx_16bits,
                bytes_a_enviar,
                &bytes_escritos,
                portMAX_DELAY
            );
        }
    }

    // Limpieza de buffers DMA y desactivacion del amplificador
    i2s_zero_dma_buffer(I2S_NUM_1);
    delay(50);
    digitalWrite(I2S_SPK_SD, LOW);

    Serial.println("Loopback finalizado.");
}