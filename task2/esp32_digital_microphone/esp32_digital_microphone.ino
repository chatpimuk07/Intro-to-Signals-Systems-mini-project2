/*
 * ======================================================================================
 * Signals & Systems / Digital Signal Processing Practical Labs
 * Task 2: Real-Time Audio Sampling using ESP32 & I2S Digital Microphone (INMP441)
 *
 * Target MCU: ESP32
 * Sampling Rate (Fs): 16 kS/s (16,000 Hz)
 * Block Size (N): 1024 samples
 * Protocol: I2S DMA -> Binary Stream over USB-Serial (2 Mbaud)
 * Frame Format: [0xAA, 0x55, 0xAA, 0x55] (4B) + Seq (2B) + N (2B) + Audio (2048B)
 * ======================================================================================
 */

#include <Arduino.h>
#include <driver/i2s.h>

// ---------------------- Pin Configuration ----------------------
#define I2S_SCK_PIN       14    // Serial Clock (BCLK)
#define I2S_WS_PIN        25    // Word Select / Left-Right Clock (LRCLK) - changed from GPIO 15 to avoid strapping pin
#define I2S_SD_PIN        32    // Serial Data (DIN)

// ---------------------- DSP Parameters -------------------------
#define SAMPLE_RATE_HZ    16000 // 16 kS/s per Task 2 specification
#define BLOCK_SIZE        1024  // N = 1024 samples per block
#define SERIAL_BAUD_RATE  2000000 // 2 Mbaud for high-speed binary transfer

// 4-byte sync header to detect frame start on host
const uint8_t SYNC_HEADER[4] = { 0xAA, 0x55, 0xAA, 0x55 };

// Buffers for 32-bit raw I2S data and processed 16-bit PCM output
int32_t raw_i2s_buffer[BLOCK_SIZE];
int16_t audio_samples[BLOCK_SIZE];

void initI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE_HZ,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // INMP441 sends 24-bit data in 32-bit slot
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Left channel (L/R pin connected to GND)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = BLOCK_SIZE,
        .use_apll = false
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() < 1000);
    initI2S();
}

void loop() {
    size_t bytesRead = 0;
    // Read 1 block of audio samples (1024 samples) from I2S DMA buffer
    i2s_read(I2S_NUM_0, (void*)raw_i2s_buffer, sizeof(raw_i2s_buffer), &bytesRead, portMAX_DELAY);

    if (bytesRead == sizeof(raw_i2s_buffer)) {
        static uint16_t packetSequence = 0;
        packetSequence++;

        // Shift by 12 bits to scale 24-bit audio to optimal 16-bit gain
        for (int i = 0; i < BLOCK_SIZE; i++) {
            audio_samples[i] = (int16_t)(raw_i2s_buffer[i] >> 12);
        }

        // 1. Send 4-byte Sync Header
        Serial.write(SYNC_HEADER, sizeof(SYNC_HEADER));

        // 2. Send 2-byte Sequence Number
        Serial.write((uint8_t*)&packetSequence, sizeof(packetSequence));

        // 3. Send 2-byte Block Size (N = 1024)
        uint16_t nSamples = BLOCK_SIZE;
        Serial.write((uint8_t*)&nSamples, sizeof(nSamples));

        // 4. Send 1024 audio samples in binary format (1024 * 2 bytes = 2048 bytes)
        Serial.write((const uint8_t*)audio_samples, BLOCK_SIZE * sizeof(int16_t));
        Serial.flush();
    }
}