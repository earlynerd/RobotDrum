# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Development Commands

This is a PlatformIO project targeting ESP32 (PICO32 board, Arduino framework).

- `pio run` — compile firmware
- `pio run -t upload -e pico32` — flash to board
- `pio device monitor -b 115200` — serial monitor (baud must match `Serial.begin(115200)`)
- `pio test -e pico32` — run unit tests (test suites live in `test/`)

## Architecture

**ESP32 firmware for a robot drum machine** — 10 electromagnetic mallets strike physical drums, controlled via BLE MIDI. An I2S MEMS microphone captures audio for FFT-based pitch detection and automated calibration.

### Core modules

- **`src/RobotDrum.cpp`** — Main entrypoint. Orchestrates audio pipeline, mallet scheduling, BLE MIDI reception, FFT analysis, and serial command dispatch. Contains `setup()` and `loop()`.
- **`lib/Mallet/`** — Mallet actuator driver. State machine: IDLE→TRIGGER→STRIKE→COAST→REBOUND→IDLE. PWM via ESP32 LEDC at 18 kHz/10-bit. Supports delayed triggers with velocity-mapped strike profiles and per-mallet calibration models.
- **`lib/I2Ssampler/`** — Audio capture. `I2SSampler` base class with double-buffered I2S DMA. `I2SMEMSSampler` handles SPH0645 MEMS mic (24-bit from 32-bit I2S slots). Runs on FreeRTOS Core 0 (reader) and Core 1 (writer).
- **`src/CalibrationRuntime.cpp/h`** — Automated calibration: strikes mallets at varying power, detects impact via mic threshold, measures strike-to-contact lag, captures pitch via FFT, and fits soft/hard calibration models.
- **`src/Calibration.cpp/h`** — EEPROM persistence for calibration data (512 bytes, magic `0x52444232`).
- **`src/AudioDiagnostics.cpp/h`** — Serial diagnostic commands for real-time audio metrics.
- **`src/SerialCommands.cpp/h`** — Command dispatcher (function pointer table mapping keywords to handlers).
- **`lib/FFT/`** — Header-only FFT library (4096-point with Hann window).
- **`lib/Goertzel/`** — Goertzel algorithm for targeted frequency detection.

### Audio pipeline

I2S capture (16,384 Hz, 128-sample blocks) → DC offset removal (EMA) → dynamic normalization → 4096-point FFT → harmonic scoring → autocorrelation validation → MIDI note output.

### Multi-core design

- **Core 0:** I2S DMA interrupt handler, buffer reads
- **Core 1:** Audio writer task (DC removal, envelope tracking, FFT frame capture), main `loop()` (mallet updates, serial commands, FFT analysis)

Shared state uses `volatile` variables; FreeRTOS task notifications synchronize buffer swaps.

### Hardware configuration

10 mallets on pins `{15, 4, 12, 32, 27, 26, 25, 2, 13, 33}` mapped to MIDI notes `{52, 55, 57, 60, 62, 64, 65, 67, 69, 72}`. I2S mic on BCK=5, WS=9, DATA=10.

### Serial commands (at runtime)

`status`, `cal [idx]`, `calvel`, `arp`, `audiostatus`, `micdiag`, `micprobe [ms]`, `micch`, `micshift <0-8>`, `micmode`, `ffttest [idx]`, `setnote <idx> <midi>`

## Code Style

- 2-space indentation, braces on own line for functions/control blocks
- `camelCase` for functions/variables, `PascalCase` for classes
- Descriptive constants near top of module, no magic numbers
- Hardware-facing code goes in `lib/<ModuleName>/`, orchestration stays in `src/`
- Commit format: `area: imperative summary` (e.g., `audio: clamp FFT magnitude`)
