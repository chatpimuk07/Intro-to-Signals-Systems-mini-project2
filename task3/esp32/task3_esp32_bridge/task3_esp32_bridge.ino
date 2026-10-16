/* =========================================================================
 * Signals & Systems / DSP Practical Labs - Task 3 (v2)
 * ESP32 bridge : DE10-Lite FPGA (SPI slave)  -->  ESP32 (SPI master)  -->  PC
 *
 *  1. Wait until FPGA_DATA_READY = HIGH (an unread 1024-sample block exists)
 *  2. Read one complete frame over SPI (mode 0, 4 MHz):
 *        A5 5A | seq(2) | N(2) | status(2) | N x uint16      (big-endian)
 *     The FPGA clears DATA_READY automatically when CS rises after a frame.
 *  3. Check the frame and forward it to the PC over USB-serial (binary):
 *        AA 55 AA 55 | <2056-byte FPGA frame> | sum16 (little-endian)
 *     sum16 = 16-bit sum of all frame bytes.
 *
 * The ESP32 only moves data.  Sampling timing is generated entirely by the
 * FPGA's 50 MHz clock, so nothing here can affect the sampling interval.
 *
 * Budget per block (64 ms @ 16 kS/s): SPI 2056 B @ 4 MHz ~ 4.2 ms,
 * UART 2062 B @ 2 Mbaud ~ 10.3 ms  -> plenty of margin.
 *
 * Board: "ESP32 Dev Module" (or ESP32-S3 - change the pins below).
 * ========================================================================= */
#include <Arduino.h>
#include <SPI.h>

// ---------------- pins (ESP32 VSPI defaults) ----------------
static const int PIN_SCK   = 18;   // -> DE10-Lite ARDUINO_IO13 (PIN_AB20)
static const int PIN_MISO  = 19;   // <- DE10-Lite ARDUINO_IO12 (PIN_Y19)
static const int PIN_MOSI  = 23;   //    not connected (FPGA ignores MOSI)
static const int PIN_CS    = 5;    // -> DE10-Lite ARDUINO_IO10 (PIN_AB19)
static const int PIN_READY = 4;    // <- DE10-Lite ARDUINO_IO9  (PIN_AA17)

// ---------------- link parameters ----------------
static const uint32_t SPI_HZ    = 4000000;   // reduce to 2 MHz if frames are corrupted
static const uint32_t BAUD      = 2000000;   // must match the Python script
static const size_t   N         = 1024;
static const size_t   HDR       = 8;
static const size_t   FRAME     = HDR + 2 * N;          // 2056 bytes
static const uint8_t  SYNC[4]   = {0xAA, 0x55, 0xAA, 0x55};

static uint8_t frame[FRAME];
static uint32_t goodFrames = 0, badFrames = 0;

static void readFrame() {
  memset(frame, 0, FRAME);
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(2);                 // FPGA latches bank/seq on CS falling edge
  SPI.transfer(frame, FRAME);           // full-duplex, received data overwrites buffer
  delayMicroseconds(1);
  digitalWrite(PIN_CS, HIGH);           // FPGA: frame complete -> DATA_READY low
  SPI.endTransaction();
}

static bool frameValid() {
  if (frame[0] != 0xA5 || frame[1] != 0x5A) return false;
  uint16_t n = (uint16_t(frame[4]) << 8) | frame[5];
  return n == N;
}

void setup() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_READY, INPUT_PULLDOWN);

  Serial.setTxBufferSize(8192);         // whole packet fits -> write() never blocks long
  Serial.begin(BAUD);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);   // CS driven manually
}

void loop() {
  if (digitalRead(PIN_READY) != HIGH) return;

  readFrame();
  if (!frameValid()) {                  // e.g. wiring problem / FPGA not configured
    badFrames++;
    delay(1);
    return;
  }
  goodFrames++;

  uint16_t sum = 0;
  for (size_t i = 0; i < FRAME; i++) sum += frame[i];
  uint8_t tail[2] = {uint8_t(sum & 0xFF), uint8_t(sum >> 8)};

  Serial.write(SYNC, sizeof(SYNC));
  Serial.write(frame, FRAME);
  Serial.write(tail, sizeof(tail));
}
