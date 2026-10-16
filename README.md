# Intro to Signals & Systems - Mini-Project II: Real-Time Audio DSP on ESP32

This repository contains the source code, hardware configuration, and MATLAB visualization scripts for **Mini-Project II (Signals & Systems / Digital Signal Processing Practical Labs)**. 

The project demonstrates real-time audio sampling, double-buffering, high-throughput binary transmission over USB-Serial, and real-time DSP visualization (Time-Domain Waveform, FFT Magnitude Spectrum with Hann Windowing, and Spectrogram Waterfall) using the ESP32 microcontroller.

---

## Project Structure

```text
.
├── task1/                                  # Task 1: Analog Electret Microphone Module
│   ├── task1_esp32_audio/
│   │   └── task1_esp32_audio.ino           # ESP32 firmware (Timer ISR + FreeRTOS + ADC)
│   └── matlab/
│       └── untitled.m                      # MATLAB script for real-time oscilloscope & FFT
├── task2/                                  # Task 2: Digital I2S Microphone (INMP441)
│   ├── esp32_digital_microphone/
│   │   └── esp32_digital_microphone.ino    # ESP32 firmware (I2S DMA RX stream)
│   └── matlab/
│       └── matlab.m                        # MATLAB real-time GUI (Waveform, FFT, Spectrogram)
└── task3/                                  # Task 3: MAX 10 FPGA ADC Audio Sampling + ESP32 Bridge
    ├── fpga/                               # DE10-Lite FPGA Design (Verilog HDL + QSF)
    │   ├── rtl/                            # Top-level, 16 kHz sample rate generator, BRAM, SPI
    │   ├── testbench/                      # ModelSim / Icarus Verilog testbench
    │   └── de10_lite_pin_assignments.qsf   # Quartus pin assignments
    ├── esp32_fpga_receiver/
    │   └── esp32_fpga_receiver.ino         # ESP32 SPI Master to USB-Serial (2 Mbaud)
    ├── matlab/
    │   └── task3_visualization.m           # MATLAB real-time GUI (Waveform, FFT, Spectrogram)
    └── python_gui/
        └── task3_advanced_dsp_app.py       # Advanced Mini-Project Workstation (Tuner, Timbre, Events)
```

---

## Task 1: Real-Time Audio Sampling using Analog Microphone & Hardware Timer

### Overview
- **Target MCU:** ESP32 (ESP-WROOM-32)
- **Sensor:** Analog Electret Microphone Module
- **Sampling Frequency ($F_s$):** $10\text{ kS/s}$ ($10,000\text{ Hz}$), corresponding to $T_s = 100\ \mu\text{s}$ per sample.
- **Block Size ($N$):** $1024$ samples.
- **Timing Architecture:**
  - Jitter-free **Hardware Timer Interrupt (`hw_timer_t` / `gptimer`)** triggering strictly every $100\ \mu\text{s}$.
  - The ISR toggles a timing debug pin (GPIO 25) and unblocks a dedicated, high-priority sampling FreeRTOS task via `vTaskNotifyGiveFromISR()`.
  - **Ping-Pong (Double Buffering)** isolates real-time ADC conversion from serial transmission.
- **Hardware Timing Verification:**
  - An oscilloscope connected to GPIO 25 proves accurate periodic interrupt timing ($f = 5\text{ kHz}$ toggle frequency, period = $200\ \mu\text{s}$ across both states $\implies 100\ \mu\text{s}$ interrupt interval).

### Hardware Wiring
| ESP32 Pin | Peripheral / Module Pin | Function |
| :--- | :--- | :--- |
| **GPIO 34** | Analog Mic OUT | ADC1 Channel 6 (12-bit ADC: 0 – 4095, Attenuation: 11 dB) |
| **GPIO 25** | Oscilloscope Probe | Timing verification debug pin (toggles each sample) |
| **3V3 / GND** | VCC / GND | Power supply |

### Communication Protocol
Data packets are streamed over USB-Serial at **2,000,000 baud (2 Mbaud)**:
1. **Sync Header:** 4 bytes (`0xAA, 0x55, 0xAA, 0x55`)
2. **Packet Sequence:** 2 bytes (`uint16`, little-endian)
3. **Block Size ($N$):** 2 bytes (`uint16`, 1024)
4. **Payload:** 2048 bytes ($1024 \times \text{uint16}$, 12-bit raw ADC values)

---

## Task 2: Digital Audio Acquisition via I2S Microphone (INMP441)

### Overview
- **Target MCU:** ESP32
- **Sensor:** INMP441 Omnidirectional MEMS Digital Microphone with I2S interface.
- **Sampling Frequency ($F_s$):** $16\text{ kS/s}$ ($16,000\text{ Hz}$).
- **Block Size ($N$):** $1024$ samples.
- **Data Acquisition:** Hardware **I2S DMA Receiver** running in Master RX mode (`ESP_INTR_FLAG_LEVEL1`, 4 DMA buffers $\times$ 1024 samples).
- **Gain Scaling:** Raw 24-bit I2S data in 32-bit slot is bit-shifted (`>> 12`) to produce optimal 16-bit PCM values (`int16_t`).
- **MATLAB Real-Time DSP:**
  - **Time-Domain Waveform:** Normalized amplitude $[-1.0, 1.0]$.
  - **FFT Magnitude Spectrum:** Single-sided FFT with a periodic **Hann Window** to eliminate spectral leakage.
  - **Spectrogram Waterfall:** Dynamic rolling matrix display ($513\text{ frequency bins} \times 70\text{ time frames}$) scaled in dBFS.

### Hardware Wiring (INMP441)
| ESP32 Pin | INMP441 Pin | Signal Description |
| :--- | :--- | :--- |
| **GPIO 14** | SCK / BCLK | Continuous Serial Clock |
| **GPIO 25** | WS / LRCLK | Word Select (Left/Right Clock) |
| **GPIO 32** | SD / DIN | Serial Data |
| **GND** | L/R | Left Channel selection |
| **3V3 / GND** | VDD / GND | 3.3V Power & Ground |

### Communication Protocol
- Baud Rate: **2,000,000 baud (2 Mbaud)**
- Frame format:
  `[0xAA, 0x55, 0xAA, 0x55] (4B) + Seq (2B) + N (2B) + Audio PCM Data (1024 * 2B = 2048B)`

---

## Task 3: Audio Sampling using MAX 10 FPGA ADC & Analog Microphone

### Overview
- **Target Hardware:** Terasic DE10-Lite FPGA (`10M50DAF484C7G`) + ESP32 Bridge.
- **Sampling Frequency ($F_s$):** $16\text{ kS/s}$ ($16,000\text{ Hz}$).
- **Block Size ($N$):** $1024$ samples ($64.0\text{ ms}$ per block).
- **FPGA Architecture:**
  - Jitter-free hardware sampling strobe generated directly from 50 MHz crystal clock ($50\text{ MHz} / 16\text{ kHz} = 3125\text{ ticks}$).
  - Hardware timing verification pin (`PIN_AB17` / Arduino IO8) toggling at $8.0\text{ kHz}$ for oscilloscope probe verification.
  - Dual-bank Ping-Pong Block RAM ($2 \times 1024 \times 16\text{-bit}$, M9K BRAMs) isolating ADC sampling from readout.
  - High-speed SPI Slave streaming blocks to ESP32 Master at $10\text{ MHz}$.
- **Host Applications:**
  - MATLAB GUI: Real-time Time-Domain, Hann-windowed FFT spectrum, and Spectrogram.
  - Python Advanced DSP Workstation: High-precision musical pitch tuner (HPS/Autocorrelation with cents deviation), acoustic feature extraction (RMS, ZCR, spectral centroid, SNR, THD), and smart event classifier (claps, whistles, speech).

---

## Getting Started

### 1. Arduino IDE Setup
1. Install the **esp32 by Espressif Systems** board package in Arduino IDE.
2. Select your board (e.g., `ESP32 Dev Module`).
3. Set **Upload Speed** to `921600` or `115200`.
4. Open and flash:
   - Task 1: `task1/task1_esp32_audio/task1_esp32_audio.ino`
   - Task 2: `task2/esp32_digital_microphone/esp32_digital_microphone.ino`
   - Task 3: `task3/esp32_fpga_receiver/esp32_fpga_receiver.ino`

### 2. Quartus Prime Setup (Task 3 FPGA)
1. Open Quartus Prime and load project with top module `de10_lite_audio_top.v`.
2. Apply `task3/fpga/de10_lite_pin_assignments.qsf`.
3. Compile and program the DE10-Lite via USB-Blaster.

### 3. MATLAB & Python Setup & Execution
1. For MATLAB:
   - Task 1: `task1/matlab/untitled.m`
   - Task 2: `task2/matlab/matlab.m`
   - Task 3: `task3/matlab/task3_visualization.m`
2. For Python Advanced DSP Workstation (Task 3 Mini-Project):
   ```bash
   .venv/bin/python task3/python_gui/task3_advanced_dsp_app.py
   ```

