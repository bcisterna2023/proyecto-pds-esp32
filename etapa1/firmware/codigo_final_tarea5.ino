#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver/i2s.h"

// ============================================================
// ASIGNACION DE PINES
// ============================================================
#define PIN_BOTON 5

// Pines I2S para el Microfono (RX - I2S_NUM_0)
#define I2S_MIC_BCLK 18
#define I2S_MIC_LRC  17
#define I2S_MIC_DIN  15

// Pines I2S para el Altavoz / Amplificador (TX - I2S_NUM_1)
#define I2S_SPK_BCLK 3
#define I2S_SPK_LRC  8
#define I2S_SPK_DOUT 16
#define I2S_SPK_SD   46

// ============================================================
// CONFIGURACION GENERAL DE AUDIO
// ============================================================
const uint32_t TASA_MUESTREO = 16000;

// Cantidad de muestras de audio por cada ciclo de lectura/escritura DMA
const size_t MUESTRAS_POR_BLOQUE = 512;

// Tamano en bytes para lectura de 32 bits (4 bytes por muestra)
const size_t TAMANO_BUFFER_RX_BYTES = MUESTRAS_POR_BLOQUE * sizeof(int32_t);

// Tamano en bytes para escritura de 16 bits (2 bytes por muestra)
const size_t TAMANO_BUFFER_TX_BYTES = MUESTRAS_POR_BLOQUE * sizeof(int16_t);

// Arreglos estaticos de transferencia I2S
static int32_t buffer_rx_32bits[MUESTRAS_POR_BLOQUE];
static int16_t buffer_tx_16bits[MUESTRAS_POR_BLOQUE];

// ============================================================
// PARAMETROS Y BUFFER CIRCULAR PARA EL EFECTO LTI DE ECO (TAREA 5)
// ============================================================
// Capacidad maxima del buffer circular: 300 ms a 16 kHz = 4800 muestras (9.6 KB)
#define MAX_MUESTRAS_RETARDO 4800

// Parametros ajustables del efecto
static float TIEMPO_ECO_SEGUNDOS = 0.300f; // Retardo ajustable: 150 ms
static float GANANCIA_ECO_ALPHA   = 0.80f;  // Coeficiente alpha (atenuacion de la reflexion)

// Buffer circular estatico y puntero de escritura
static int16_t buffer_circular_retardo[MAX_MUESTRAS_RETARDO];
static size_t indice_escritura_circular = 0;
static size_t muestras_retardo_d = 0;

// ============================================================
// DECLARACION DE FUNCIONES
// ============================================================
void configurar_pines();
void configurar_microfono_i2s();
void configurar_altavoz_i2s();
void inicializar_buffer_eco();
void actualizar_parametros_eco(float tiempo_segundos, float ganancia_alpha);
void procesar_audio_con_eco();

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);

    configurar_pines();
    configurar_microfono_i2s();
    configurar_altavoz_i2s();
    inicializar_buffer_eco();

    Serial.println("==================================================");
    Serial.println("Sistema iniciado con efecto de Eco LTI (Tarea 5).");
    Serial.printf("Retardo inicial: %.0f ms | Ganancia alpha: %.2f\n", 
                  TIEMPO_ECO_SEGUNDOS * 1000.0f, GANANCIA_ECO_ALPHA);
    Serial.println("Presione el boton para procesar audio en tiempo real.");
    Serial.println("==================================================");
}

// ============================================================
// LOOP PRINCIPAL
// ============================================================
void loop() {
    // Si el boton esta presionado (activo en bajo)
    if (digitalRead(PIN_BOTON) == LOW) {
        delay(50); // Antirrebote simple
        if (digitalRead(PIN_BOTON) == LOW) {
            procesar_audio_con_eco();
        }
    }

    delay(5);
}

// ============================================================
// INICIALIZACION Y GESTION DEL BUFFER DE ECO
// ============================================================
void inicializar_buffer_eco() {
    size_t i;
    for (i = 0; i < MAX_MUESTRAS_RETARDO; i++) {
        buffer_circular_retardo[i] = 0;
    }
    indice_escritura_circular = 0;
    actualizar_parametros_eco(TIEMPO_ECO_SEGUNDOS, GANANCIA_ECO_ALPHA);
}

void actualizar_parametros_eco(float tiempo_segundos, float ganancia_alpha) {
    // D = round(Td * Fs)
    size_t d_calculado = (size_t)(tiempo_segundos * (float)TASA_MUESTREO);

    // Limitar al tamano maximo del buffer estatico
    if (d_calculado >= MAX_MUESTRAS_RETARDO) {
        d_calculado = MAX_MUESTRAS_RETARDO - 1;
    }

    muestras_retardo_d = d_calculado;
    GANANCIA_ECO_ALPHA = ganancia_alpha;
}

// ============================================================
// CONFIGURACION DE PINES GPIO
// ============================================================
void configurar_pines() {
    pinMode(PIN_BOTON, INPUT_PULLUP);

    // Pin de Shutdown del amplificador (LOW = apagado, HIGH = habilitado)
    pinMode(I2S_SPK_SD, OUTPUT);
    digitalWrite(I2S_SPK_SD, LOW);
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
// PROCESAMIENTO CON RETARDO/ECO LTI EN TIEMPO REAL
// ============================================================
void procesar_audio_con_eco() {
    Serial.println("Efecto de eco iniciado.");

    // Habilitar el amplificador (pin SD en HIGH)
    digitalWrite(I2S_SPK_SD, HIGH);
    delay(20);

    size_t bytes_leidos = 0;
    size_t bytes_escritos = 0;
    const float FACTOR_GANANCIA_ENTRADA = 0.40f;

    while (digitalRead(PIN_BOTON) == LOW) {

        // 1. Lectura de muestras crudas de 32 bits desde el microfono
        i2s_read(
            I2S_NUM_0,
            (void*)buffer_rx_32bits,
            TAMANO_BUFFER_RX_BYTES,
            &bytes_leidos,
            portMAX_DELAY
        );

        size_t muestras_leidas = bytes_leidos / sizeof(int32_t);

        // 2. Procesamiento muestra a muestra aplicando la convolucion con h[n]
        for (size_t indice = 0; indice < muestras_leidas; indice++) {
            int32_t muestra_32 = buffer_rx_32bits[indice];

            // Alineacion de 32 a 16 bits y atenuacion del microfono
            int32_t muestra_base = muestra_32 >> 15;
            float muestra_flotante = (float)muestra_base * FACTOR_GANANCIA_ENTRADA;
            int16_t x_n = (int16_t)muestra_flotante;

            // --- ETAPA DE RETARDO MEDIANTE BUFFER CIRCULAR ---
            // Calculo del indice de lectura: (indice_escritura - D + MAX) % MAX
            size_t indice_lectura = 0;
            if (indice_escritura_circular >= muestras_retardo_d) {
                indice_lectura = indice_escritura_circular - muestras_retardo_d;
            } else {
                indice_lectura = (indice_escritura_circular + MAX_MUESTRAS_RETARDO) - muestras_retardo_d;
            }

            // Lectura de la muestra pasada x[n - D]
            int16_t x_retardada = buffer_circular_retardo[indice_lectura];

            // Almacenamiento de la muestra actual x[n] en el buffer circular
            buffer_circular_retardo[indice_escritura_circular] = x_n;

            // Incremento y envoltura del puntero circular de escritura
            indice_escritura_circular = indice_escritura_circular + 1;
            if (indice_escritura_circular >= MAX_MUESTRAS_RETARDO) {
                indice_escritura_circular = 0;
            }

            // --- EVALUACION DE LA ECUACION EN DIFERENCIAS LTI ---
            // y[n] = x[n] + alpha * x[n - D]
            float componente_directa = (float)x_n;
            float componente_eco     = GANANCIA_ECO_ALPHA * (float)x_retardada;
            float suma_y             = componente_directa + componente_eco;

            // Saturacion para evitar distorsion por desbordamiento de 16 bits
            if (suma_y > 32767.0f) {
                suma_y = 32767.0f;
            } else if (suma_y < -32768.0f) {
                suma_y = -32768.0f;
            }

            buffer_tx_16bits[indice] = (int16_t)suma_y;
        }

        // 3. Envio del bloque procesado de 16 bits hacia el amplificador
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

    // Limpieza de buffers y apagado del amplificador al soltar el boton
    i2s_zero_dma_buffer(I2S_NUM_1);
    inicializar_buffer_eco(); // Limpia el buffer circular para evitar colas de eco residuales
    delay(50);
    digitalWrite(I2S_SPK_SD, LOW);

    Serial.println("Efecto de eco finalizado.");
}