# Repository Guidelines

## Project Structure & Module Organization
This repository is a PlatformIO firmware project for an ESP32-based robot drum controller.

- `src/`: application entrypoint and runtime logic (`RobotDrum.cpp`).
- `include/`: project headers shared across modules.
- `lib/`: private and vendored libraries (for example `Mallet`, `I2Ssampler`, `FFT`, `Goertzel`, `MIDI_Library`).
- `test/`: PlatformIO test area (currently scaffolded; add unit/integration tests here).
- `platformio.ini`: board/environment config (`env:pico32`) and library dependencies.

Keep new hardware-facing code in focused modules under `lib/<ModuleName>/` and keep `src/` for orchestration logic.

## Build, Test, and Development Commands
Use PlatformIO from the repository root:

- `pio run`: compile firmware for the default environment.
- `pio run -e pico32`: compile explicitly for the configured ESP32 target.
- `pio run -t upload -e pico32`: flash firmware to the board.
- `pio device monitor -b 115200`: open serial monitor (matches `Serial.begin(115200)`).
- `pio test -e pico32`: run tests under `test/` when test suites are present.

## Coding Style & Naming Conventions
Use C++ (Arduino/ESP32) conventions already present in the codebase:

- 2-space indentation, braces on their own line for functions/control blocks.
- `camelCase` for functions/variables (`identifyNote`, `malletUpdate`).
- `PascalCase` for classes/types (`Mallet`, `I2SSampler`).
- Prefer descriptive constants (`numMallets`, `midiLeadTime`) over magic numbers.
- Keep pin mappings and timing constants near the top of the module.

## Testing Guidelines
Add tests in `test/` using PlatformIO’s unit test runner.

- Name files by behavior, e.g. `test_mallet_timing.cpp`, `test_fft_note_detection.cpp`.
- Focus tests on deterministic logic (timing math, note mapping, filters).
- For hardware-dependent paths, isolate pure logic into testable functions.

## Commit & Pull Request Guidelines
Current history uses short messages (`fft`, `initial commit`), but contributors should prefer clear, imperative commits:

- Format: `area: short imperative summary` (example: `audio: clamp FFT magnitude before note detect`).
- Keep commits scoped to one concern.
- PRs should include: purpose, key changes, hardware impact, and validation steps (`pio run`, `pio test`, device monitor output when relevant).
- Link related issues and include serial log snippets/screenshots for behavior changes.
