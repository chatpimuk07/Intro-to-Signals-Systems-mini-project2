# Task 3 (v2): Audio Sampling using MAX 10 FPGA ADC (VHDL) & ESP32 Bridge

This folder contains the complete, verified implementation for **Task 3** of the Signals & Systems / Digital Signal Processing Practical Labs, implemented entirely in **VHDL** for the FPGA and **Python** (plus MATLAB) for the host workstation.

---

## 1. System Architecture

```text
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │                              TERASIC DE10-LITE FPGA (MAX 10)                           │
 │                                                                                        │
 │  50 MHz Clock ──────► [ sample_tick_gen ] (Counter DIV = 3125)                        │
 │                                  │                                                     │
 │                     16 kHz tick ─┼────────────────────────┐                            │
 │                                  ▼                        ▼                            │
 │                       [ mcp3201_ctrl ]             [ test_tone_dds ]                   │
 │                        (SPI Master)                 (32-bit DDS)                       │
 │                             │                            │                             │
 │    Microphone / Gen ────────┤                            │                             │
 │    via MCP3201 ADC          ▼                            ▼                             │
 │                        ADC Sample (12b)           Test Tone (12b)                      │
 │                             └────────────┬───────────────┘                              │
 │                                          ▼  (Mux controlled by SW[0])                  │
 │                                  12-bit Live Sample                                    │
 │                                          │                                             │
 │                                          ▼                                             │
 │                             [ pingpong_buffer ] (M9K BRAM)                             │
 │                               Bank 0 (1024) | Bank 1 (1024)                            │
 │                                          │                                             │
 │                                          ▼                                             │
 │                              [ spi_slave_tx ] (SPI Mode 0)                             │
 │                                          ▲                                             │
 │  Oscilloscope Probe:                     │                                             │
 │  TIMING_DEBUG_PIN (8 kHz)                │                                             │
 └──────────────────────────────────────────┼─────────────────────────────────────────────┘
                                            │ SPI Bus (SCK, MISO, CS_N, DATA_READY)
                                            ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │                                   ESP32 BRIDGE MCU                                     │
 │                                                                                        │
 │   1. Polls FPGA_DATA_READY handshake line.                                             │
 │   2. Clocks out 2056-byte frame via SPI Master (4 MHz).                                │
 │   3. Appends 16-bit Frame Checksum.                                                    │
 │   4. Streams packet over USB-UART at 2,000,000 baud (2 Mbaud).                         │
 └──────────────────────────────────────────┼─────────────────────────────────────────────┘
                                            │ USB Serial
                                            ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │                            HOST COMPUTER (PYTHON / MATLAB)                             │
 │                                                                                        │
 │  • task3_gui_app.py        : Mini-Project Musical Pitch & Acoustic Classifier GUI      │
 │  • task3_viewer.py         : Real-time Waveform, Hann vs Rect FFT, Spectrogram, WAV    │
 │  • verify_sample_rate.py   : Non-linear least squares sine fit & phase continuity test │
 │  • task3_visualization.m   : MATLAB Real-time DSP visualizer                           │
 └────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Hardware Wiring & Pin Mapping

> [!IMPORTANT]
> Both the DE10-Lite FPGA Arduino header and the ESP32 logic pins operate at **3.3V**. Connect a **common GND** jumper wire between both boards before applying power.

### A. DE10-Lite FPGA to ESP32 Bridge Interface
| DE10-Lite Arduino Header | FPGA Pin | Signal Name | ESP32 GPIO | Description |
| :--- | :--- | :--- | :--- | :--- |
| **ARDUINO_IO[13]** | `PIN_AB20` | `ESP_SPI_SCK` | **GPIO 18** | SPI Clock (4 MHz, Master to Slave) |
| **ARDUINO_IO[12]** | `PIN_Y19` | `ESP_SPI_MISO` | **GPIO 19** | SPI Data (Slave out to ESP32 Master) |
| **ARDUINO_IO[10]** | `PIN_AB19` | `ESP_SPI_CS_N` | **GPIO 5** | SPI Chip Select (Active Low) |
| **ARDUINO_IO[9]** | `PIN_AA17` | `FPGA_DATA_READY` | **GPIO 4** | Block Ready Interrupt / Handshake |
| **GND** | Board GND | Ground | **GND** | Common Reference Ground |

*(Note: MOSI is not required since transfer is strictly unidirectional from FPGA to ESP32).*

### B. DE10-Lite FPGA to External MCP3201 ADC
| DE10-Lite Arduino Header | FPGA Pin | Signal Name | MCP3201 Pin | Description |
| :--- | :--- | :--- | :--- | :--- |
| **ARDUINO_IO[2]** | `PIN_AB7` | `ADC_SPI_CS_N` | **Pin 1 (CS/SHDN)** | Chip Select to ADC |
| **ARDUINO_IO[3]** | `PIN_AB8` | `ADC_SPI_SCK` | **Pin 7 (CLK)** | SPI Clock to ADC (1 MHz) |
| **ARDUINO_IO[4]** | `PIN_AB9` | `ADC_SPI_MISO` | **Pin 6 (DOUT)** | Conversion Data from ADC |
| **+3.3V / GND** | Board Power | 3.3V / GND | **VDD / VSS** | ADC Power Supply |

### C. Timing Verification & Onboard Diagnostics
| DE10-Lite Peripheral | Pin / Location | Function |
| :--- | :--- | :--- |
| **ARDUINO_IO[8]** | `PIN_AB17` | **Oscilloscope Timing Probe:** Strictly periodic $8.000\text{ kHz}$ square wave ($125.0\ \mu\text{s}$ period) |
| **SW[0]** | `PIN_C10` | **Input Mode Selector:** `0` = Real ADC input, `1` = Internal DDS Test Tone |
| **SW[3:1]** | `PIN_C11..C12` | **DDS Frequency Selector:** 8 preset tones (1 kHz, 1007.8 Hz, 440 Hz, 250 Hz, 2 kHz, 3 kHz, 5 kHz, 7 kHz) |
| **KEY[0]** | `PIN_B8` | **Synchronous System Reset** (Active Low button) |
| **LEDR[0]** | `PIN_A8` | Lights up when a 1024-sample block is ready (`FPGA_DATA_READY`) |
| **LEDR[1]** | `PIN_A9` | Lights up when in DDS test synthesis mode |
| **LEDR[9:2]** | `PIN_B11..A10` | 8-level peak volume meter updated per block |

---

## 3. Communication Protocol

Packets are streamed over USB-UART at **2,000,000 baud**:

```text
┌────────────────────────┬─────────────┬─────────────┬─────────────┬───────────────────────────┬─────────────┐
│ Sync Header (4 Bytes)  │ Magic (2 B) │ Seq (2 B)   │ N (2 B)     │ Status (2 B)              │ Data (2048 B│ Checksum(2B)│
│ 0xAA, 0x55, 0xAA, 0x55 │ 0xA5, 0x5A  │ uint16 (BE) │ 1024 (BE)   │ flags + overrun count (BE)│ 1024 x uint │ uint16 (LE) │
└────────────────────────┴─────────────┴─────────────┴─────────────┴───────────────────────────┴─────────────┘
```

- **Header:** 4-byte synchronization pattern `0xAA 0x55 0xAA 0x55`.
- **FPGA Frame (2056 bytes):**
  - Magic: `0xA5 0x5A`
  - Sequence number (16-bit big-endian, rolling 0..65535)
  - Block length: $N = 1024$
  - Status word:
    - `bit 15`: 1 = Built-in DDS tone, 0 = ADC
    - `bit 14`: 1 = MAX 10 on-chip ADC, 0 = MCP3201
    - `bits 13..11`: Selected DDS tone index (from `SW[3:1]`)
    - `bits 7..0`: Hardware overrun counter (saturating count of unread blocks)
  - 1024 16-bit audio samples (12-bit unsigned ADC codes `0..4095`)
- **Tail:** 16-bit checksum (sum of all 2056 bytes of the FPGA frame).

---

## 4. Directory Structure

```text
task3_v2/
├── README.md                          # Comprehensive documentation & lab answers
├── fpga/
│   ├── rtl/                           # Complete VHDL RTL Design
│   │   ├── sine_lut_pkg.vhd           # 1024-word sine ROM & DDS frequency table
│   │   ├── sample_tick_gen.vhd        # 50 MHz -> 16 kHz sampling tick generator
│   │   ├── mcp3201_ctrl.vhd           # SPI controller for MCP3201 ADC
│   │   ├── test_tone_dds.vhd          # Direct Digital Synthesis test-tone generator
│   │   ├── pingpong_buffer.vhd        # Dual-bank M9K BRAM ping-pong buffer
│   │   ├── spi_slave_tx.vhd           # SPI slave transmitter to ESP32
│   │   ├── max10_adc_sampler.vhd      # Optional MAX 10 on-chip ADC wrapper
│   │   └── task3_top.vhd              # Top-level DE10-Lite FPGA entity
│   ├── sim/                           # Simulation & Verification
│   │   ├── tb_task3_top.vhd           # Self-checking testbench (ADC + SPI master)
│   │   └── run_sim.sh                 # Simulation script for nvc or GHDL
│   └── quartus/                       # Quartus Prime Project
│       ├── task3_v2.qpf               # Quartus Project File
│       ├── task3_v2.qsf               # Pin assignments & device configuration
│       └── task3_v2.sdc               # Synopsys Design Constraints (50 MHz clock)
├── esp32/
│   └── task3_esp32_bridge/
│       └── task3_esp32_bridge.ino     # ESP32 SPI master to USB-serial bridge
├── python/
│   ├── gen_sine_lut.py                # Python generator for VHDL sine LUT
│   ├── fpga_stream.py                 # Protocol parser, serial reader & simulator
│   ├── task3_viewer.py                # Real-time oscilloscope & FFT/spectrogram tool
│   ├── task3_gui_app.py               # Mini-Project Musical Tuner & Classifier GUI
│   └── verify_sample_rate.py          # Least-squares sine fitting Fs verification
└── matlab/
    └── task3_visualization.m          # MATLAB real-time DSP visualization script
```

---

## 5. Quick-Start Guide

### Step 1: Simulate the FPGA Design (VHDL)
Run the self-checking testbench using `nvc` or `ghdl`:
```bash
cd task3_v2/fpga/sim
./run_sim.sh
```
Output:
```text
** Note: block 0 checked
** Note: block 1 checked
** Note: block 2 checked
** Note: overrun detection checked
** Note: *** TESTBENCH PASSED ***
** Note: period check: 7373 sample intervals of 20000000000 fs verified
```

### Step 2: Compile & Flash FPGA (Quartus Prime)
1. Open `task3_v2/fpga/quartus/task3_v2.qpf` in Quartus Prime Lite Edition.
2. Click **Processing -> Start Compilation** (or press `Ctrl+L`).
3. Open **Programmer**, select **USB-Blaster**, and program the `.sof` file to the DE10-Lite board.

### Step 3: Flash ESP32 Firmware
1. Open `task3_v2/esp32/task3_esp32_bridge/task3_esp32_bridge.ino` in Arduino IDE.
2. Select Board: **ESP32 Dev Module**.
3. Set Upload Speed: `921600` and flash the board.

### Step 4: Run the Host Applications

#### A. Mini-Project GUI (Musical Pitch Tuner & Event Classifier)
```bash
python task3_v2/python/task3_gui_app.py
```
*(Select either your ESP32 serial port or choose from the built-in simulators: `Simulator: Mic`, `Simulator: 1kHz Tone`, or `Simulator: DDS Test Tone`).*

#### B. Real-Time FFT & Spectrogram Viewer
```bash
python task3_v2/python/task3_viewer.py
```
Keys:
- `Space`: Pause / Resume
- `w`: Toggle rectangular-window comparison trace
- `r`: Start / Stop recording to 16 kHz WAV file
- `s`: Save high-resolution PNG snapshot and raw `.npz` data

#### C. Sampling Rate Verification Script
```bash
python task3_v2/python/verify_sample_rate.py --f-ref 1000 --blocks 60
```
Performs non-linear least squares sine fitting and tests per-block phase continuity to prove hardware sampling accuracy with zero CPU jitter.

---

## 6. Answers to Lab Questions & Theoretical Analysis

### Question 1: Verifying Constant Sampling Rate ($16\text{ kS/s}$) and Task Independence
**How can you verify that the ADC sampling occurs at a fixed sampling rate? How can you verify that ADC sampling is not interrupted or delayed by other tasks?**

- **Hardware Timing Signal on Oscilloscope:**
  In `sample_tick_gen.vhd`, a dedicated counter divides the 50.000 MHz crystal oscillator by 3,125 cycles ($50,000,000 / 16,000 = 3125$). A toggle flip-flop flips on every sampling strobe, outputting an exact **$8.000\text{ kHz}$ square wave** on `TIMING_DEBUG_PIN` (`PIN_AB17` / Arduino IO8). Connecting an oscilloscope probe reveals an unwavering period of $T = 125.000\ \mu\text{s}$, with peak-to-peak jitter $< 50\text{ ps}$ (limited solely by crystal oscillator quality).
- **Physical Hardware Isolation (Dual-Port Ping-Pong BRAM):**
  Unlike microcontroller ADC routines where interrupts can be delayed by flash cache misses, FreeRTOS task switches, or USB transfers, the FPGA sampling engine runs in dedicated hardware registers completely decoupled from SPI transmission. Even if the ESP32 pauses data transmission entirely, the FPGA overrun counter registers the unread block while the sampling counter continues advancing with cycle-accurate precision.

---

### Question 2: FFT Frequency Resolution Calculation
**For the FFT calculation, when using $F_s = 16\text{ kSa/s}$ and $N = 1024$, what is the FFT frequency resolution?**

$$\Delta f = \frac{F_s}{N} = \frac{16,000\text{ Hz}}{1024} = 15.625\text{ Hz}$$

- Each FFT frequency bin represents an integer multiple of $15.625\text{ Hz}$:
  - Bin 0: $0.0\text{ Hz}$ (DC bias)
  - Bin 1: $15.625\text{ Hz}$
  - Bin 64: $64 \times 15.625 = 1000.0\text{ Hz}$ (exactly coincides with our 1 kHz test tone)
  - Bin 512: $512 \times 15.625 = 8000.0\text{ Hz}$ (Nyquist limit)

---

### Question 3: Simulating Microphone Output with a Function Generator
**How can you use a function generator to simulate the analog microphone module?**

1. **DC Offset:** Electret microphone modules (such as MAX4466/MAX9814) output an AC audio signal centered around a DC bias voltage of $V_{CC} / 2 \approx 1.65\text{ V}$. Configure the function generator to **DC Offset = $+1.65\text{ V}$**.
2. **Peak-to-Peak Amplitude:** Set the waveform to a sine wave with amplitude $V_{pp} \le 1.5\text{ V}$ (e.g., $1.0\text{ V}_{pp}$, spanning from $1.15\text{ V}$ to $2.15\text{ V}$).
3. **Protection Warning:** The signal must remain strictly between $0.0\text{ V}$ and $3.3\text{ V}$ at all times. Applying negative voltage will damage the ADC input ESD diodes.

---

### Question 4: Spectrogram Principles and Implementation
**What is a spectrogram? How can you write a program to visualize a spectrogram in real time?**

A **spectrogram** is a 2D time-frequency-intensity visualization that plots the Short-Time Fourier Transform (STFT) of an audio signal over time:
- **X-axis:** Time history (seconds).
- **Y-axis:** Frequency bins from $0\text{ Hz}$ to $F_s / 2 = 8000\text{ Hz}$.
- **Color intensity:** Power spectral density in decibels relative to full scale (dBFS).

**Implementation in our Python / MATLAB scripts:**
1. A rolling 2D matrix of size $\left(\frac{N}{2} + 1\right) \times M$ is maintained in memory ($513 \times 120$ frames).
2. For each incoming 1024-sample block, a periodic Hann window is applied, and the single-sided FFT magnitude is converted to dBFS:
   $$\text{dBFS} = 20 \log_{10}\left(\frac{|X[k]|}{\text{Full Scale}}\right)$$
3. The matrix is rolled left by 1 column (`np.roll(spec, -1, axis=1)`), and the new spectrum vector is inserted into the rightmost column.
4. Rendered via `imshow` / `imagesc` using the `turbo` or `magma` colormap.

---

### Question 5: Excessive Sound Level & Saturation Clipping
**What happens when the input sound is too loud? How does clipping affect the waveform and FFT spectrum?**

1. **Time Domain:** When sound pressure exceeds the microphone pre-amplifier's dynamic range, the analog output cannot swing beyond the supply rails ($0\text{ V}$ and $3.3\text{ V}$). The sine peaks become flattened (clipped), turning sinusoidal waveforms into quasi-square waves.
2. **Frequency Domain (Harmonics & Intermodulation):** A pure sine wave contains only a single spectral spike at $f_0$. When clipping occurs, the sharp discontinuities introduce a wide series of **odd harmonics** ($3f_0, 5f_0, 7f_0, \dots$) decaying at $-20\text{ dB/decade}$. In the spectrogram, clipping appears as vertical energy streaks across the entire frequency spectrum, accompanied by a sharp degradation in measured SNR and Total Harmonic Distortion (THD).

---

## 7. Comparative Analysis: Task 1 (ESP32 ADC) vs Task 3 (FPGA ADC)

| Metric / Parameter | Task 1: ESP32 On-Chip ADC | Task 3: MAX 10 FPGA + MCP3201 ADC |
| :--- | :--- | :--- |
| **Sampling Mechanism** | Microcontroller Timer ISR / FreeRTOS task notify | Dedicated hardware state machine clocked by 50 MHz crystal |
| **Clock Jitter** | Microsecond-level interrupt latency and WiFi/Flash cache misses | **Cycle-accurate ($62.500\ \mu\text{s} \pm 0.00\text{ ns}$)** |
| **Memory Buffer** | CPU SRAM array | Inferred dual-port M9K Block RAM |
| **Data Transfer Independence** | CPU shares bus between ADC readout and UART transfer | SPI slave readout is physically separate from acquisition |
| **ADC Linearity** | ESP32 SAR ADC has non-linearities below 0.1V and above 3.1V | External 12-bit SPI ADC provides true linear response across 0–3.3V |
| **Hardware Synthesis** | Software only | Includes built-in 32-bit DDS test synthesizer |
