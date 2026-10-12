/*
 * ======================================================================================
 * Signals & Systems / Digital Signal Processing Practical Labs
 * Task 1: Real-Time Audio Sampling using ESP32 and Analog Electret Microphone Module
 * 
 * Target MCU: ESP32 / ESP32-S3
 * Sampling Rate (Fs): 10 kS/s (10,000 Hz) -> Interval: 100 microseconds
 * Block Size (N): 1024 samples
 * Buffer Strategy: Double-buffer (Ping-Pong buffer) to isolate acquisition from serial I/O
 * Timing Source: Hardware Timer Interrupt (strictly non-blocking, jitter-free)
 * Transmission: Binary data streaming over USB-Serial (1 Mbaud / 2 Mbaud)
 * Verification: GPIO Timing pin toggles on every sample (probed via oscilloscope)
 * ======================================================================================
 */

#include <Arduino.h>
#include <esp_arduino_version.h>

// --------------------------- Pin Configuration ---------------------------
#define ADC_INPUT_PIN       34    // GPIO 34 (ADC1_CH6) - Input-only, ideal for analog input
#define TIMING_DEBUG_PIN    25    // GPIO 25 - Toggled on each sample for oscilloscope timing verification

// ----------------------- DSP Sampling Parameters -------------------------
#define SAMPLE_RATE_HZ      10000 // 10 kS/s (Fs = 10 kHz)
#define SAMPLE_INTERVAL_US  (1000000 / SAMPLE_RATE_HZ) // 100 microseconds per sample
#define BLOCK_SIZE          1024  // N = 1024 samples per block
#define SERIAL_BAUD_RATE    2000000 // 2,000,000 baud (2 Mbaud) for high-throughput binary transfer

// ------------------------- Sync Header & Protocol -----------------------
// 4-byte synchronization magic sequence to guarantee packet frame alignment on host
const uint8_t SYNC_HEADER[4] = { 0xAA, 0x55, 0xAA, 0x55 };

// ------------------------ Double Buffering State ------------------------
// Two buffers of 1024 16-bit integers
uint16_t buffer0[BLOCK_SIZE];
uint16_t buffer1[BLOCK_SIZE];
uint16_t* pingPongBuffers[2] = { buffer0, buffer1 };

// Volatile state variables accessed both in ISR and main loop
volatile uint8_t  activeBufferIndex = 0;   // Buffer currently being filled by ADC ISR (0 or 1)
volatile uint16_t sampleCounter     = 0;   // Current sample index inside active buffer
volatile bool     bufferReady       = false;// Flag indicating a block is complete and ready to send
volatile uint8_t  readyBufferIndex  = 0;   // Index of the buffer ready for transmission
volatile uint32_t bufferOverrunCount= 0;   // Counter for buffer overrun events
volatile bool     timingPinState    = false;// Current logic state of the timing debug pin

// Hardware timer object and FreeRTOS task handle
hw_timer_t *samplingTimer = NULL;
TaskHandle_t samplingTaskHandle = NULL;

// -------------------------------------------------------------------------
// Hardware Timer Interrupt Service Routine (ISR)
// Called strictly every 100 microseconds (10,000 times/second)
// -------------------------------------------------------------------------
void IRAM_ATTR onSampleTimerISR() {
    // 1. Toggle debug GPIO pin to allow hardware verification via oscilloscope
    timingPinState = !timingPinState;
    digitalWrite(TIMING_DEBUG_PIN, timingPinState);

    // 2. Notify the high-priority sampling task immediately
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(samplingTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// -------------------------------------------------------------------------
// Dedicated High-Priority Sampling Task
// Executed immediately upon timer notification outside ISR context,
// allowing analogRead() to acquire 12-bit ADC data safely.
// -------------------------------------------------------------------------
void samplingTask(void *pvParameters) {
    while (true) {
        // Wait for notification from Hardware Timer ISR (zero CPU overhead while waiting)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Perform analog-to-digital conversion (12-bit: 0 - 4095)
        uint16_t rawAdcValue = (uint16_t)analogRead(ADC_INPUT_PIN);

        // Store sample in active ping-pong buffer
        pingPongBuffers[activeBufferIndex][sampleCounter++] = rawAdcValue;

        // Check if block of N=1024 samples is full
        if (sampleCounter >= BLOCK_SIZE) {
            sampleCounter = 0;

            if (bufferReady) {
                bufferOverrunCount++;
            }

            readyBufferIndex = activeBufferIndex;
            bufferReady = true;
            activeBufferIndex ^= 1;
        }
    }
}

// -------------------------------------------------------------------------
// Setup Hardware Timer (Compatible with ESP32 Arduino Core 2.x and 3.x)
// -------------------------------------------------------------------------
void initHardwareTimer() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    // ESP32 Arduino Core 3.x+ (ESP-IDF v5.x gptimer architecture)
    samplingTimer = timerBegin(1000000); // 1 MHz resolution (1 tick = 1 microsecond)
    timerAttachInterrupt(samplingTimer, &onSampleTimerISR);
    timerAlarm(samplingTimer, SAMPLE_INTERVAL_US, true, 0); // 100 us alarm, auto-reload
#else
    // ESP32 Arduino Core 2.x (ESP-IDF v4.x hw_timer architecture)
    // Prescaler 80 divides 80 MHz APB clock to 1 MHz (1 tick = 1 microsecond)
    samplingTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(samplingTimer, &onSampleTimerISR, true);
    timerAlarmWrite(samplingTimer, SAMPLE_INTERVAL_US, true); // 100 us interval, auto-reload
    timerAlarmEnable(samplingTimer);
#endif
}

void setup() {
    // Initialize Serial Port with high baud rate
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() < 1000) {
        ; // Wait up to 1 second for serial port to open (if native USB)
    }

    // Configure GPIO pins
    pinMode(TIMING_DEBUG_PIN, OUTPUT);
    digitalWrite(TIMING_DEBUG_PIN, LOW);

    pinMode(ADC_INPUT_PIN, INPUT);

    // Configure ESP32 ADC
    analogReadResolution(12); // 12-bit ADC (values: 0 to 4095)
    analogSetAttenuation(ADC_11db); // Full-scale input ~0V - 3.1V (safe for 3.3V electret mic with DC bias)

    // Clear buffer memory
    memset(buffer0, 0, sizeof(buffer0));
    memset(buffer1, 0, sizeof(buffer1));

    // Create dedicated sampling task with highest priority before timer starts
    xTaskCreatePinnedToCore(
        samplingTask,
        "SamplingTask",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,
        &samplingTaskHandle,
        1 // Run on Core 1
    );

    // Initialize and start the hardware sampling timer
    initHardwareTimer();
}

void loop() {
    // Check if an acquisition block is ready to be streamed to host
    if (bufferReady) {
        // Read buffer index and clear flag
        uint8_t bufToSend = readyBufferIndex;
        bufferReady = false;

        // Static block sequence counter
        static uint16_t packetSequence = 0;
        packetSequence++;

        // 1. Send 4-byte Sync Header
        Serial.write(SYNC_HEADER, sizeof(SYNC_HEADER));

        // 2. Send 2-byte Packet Sequence Number
        Serial.write((uint8_t*)&packetSequence, sizeof(packetSequence));

        // 3. Send 2-byte Block Size (N = 1024)
        uint16_t nSamples = BLOCK_SIZE;
        Serial.write((uint8_t*)&nSamples, sizeof(nSamples));

        // 4. Send 1024 samples in binary format (1024 * 2 bytes = 2048 bytes)
        Serial.write((const uint8_t*)pingPongBuffers[bufToSend], BLOCK_SIZE * sizeof(uint16_t));

        // 5. Ensure serial buffer is flushed
        Serial.flush();
    }
}
