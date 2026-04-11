# pico-spec — AI Agent Context

ZX Spectrum emulator for Raspberry Pi Pico (RP2040 / RP2350), ported from [ESPectrum](https://github.com/EremusOne/ESPectrum).
Emulates ZX Spectrum 48K, 128K, Pentagon 128/512/1024K, Byte computer, and ALF TV Game with 100% cycle-accurate Z80 CPU.

## Language & Build

- **Language**: C++17 / C11 (embedded, no OS — bare-metal Pico SDK)
- **Build system**: CMake + Pico SDK 2.2.0
- **Toolchain**: ARM GCC 14.2
- **Build**:
  ```bash
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j$(nproc)
  ```
  Output: `bin/Release/*.uf2` — flash via USB mass-storage mode.
- **Board selection**: set variables in `CMakeLists.txt` (`PICO_PC`, `MURM2`, `ZERO`, `ZERO2`, `PICO_DV`). Default = Murmulator 1.x.

## Supported Hardware Boards

| Board              | Define    | PSRAM GPIO | Notes                    |
|--------------------|-----------|------------|--------------------------|
| Murmulator 1.x     | (default) | 19         |                          |
| Murmulator 2.0     | `MURM2`   | 8          |                          |
| Olimex PICO-PC     | `PICO_PC` | 8          | PWM audio on GP27/GP28   |
| Pimoroni Pico DV   | `PICO_DV` | 47         | HDMI via DVI on GP6+     |
| Waveshare PiZero   | `ZERO`    | —          | RP2040                   |
| Waveshare PiZero 2 | `ZERO2`   | 47         | RP2350                   |

## Architecture / Key Source Files

| File                | Role                                                      |
|---------------------|-----------------------------------------------------------|
| `src/main.cpp`      | Entry point, hardware init, multicore dispatch            |
| `src/ESPectrum.cpp`  | Main emulation loop (`loop()`), `setup()`, `reset()`, audio mixing |
| `src/ESPectrum.h`   | Central header: audio buffers, timing constants, globals  |
| `src/CPU.cpp`       | Z80 CPU execution loop (per-frame cycle counting)         |
| `src/Z80_JLS/`      | Z80 core by J.L. Sánchez — instruction decode/execute     |
| `src/Z80_JLS.cpp`   | Z80 opcode implementation (large, ~138 KB)                |
| `src/Ports.cpp`     | I/O port handling: ULA, AY-3-8912, KR580VI53 (8253 PIT), FDD, Kempston |
| `src/Ports.h`       | Port structures (PIT8253Channel, etc.)                    |
| `src/Video.cpp`     | VGA/HDMI/TFT/TV rendering, border, multicolor effects    |
| `src/MemESP.cpp`    | Memory management: RAM banking, ROM paging, contention    |
| `src/AySound.cpp`   | AY-3-8912 PSG emulation (ayemu-based)                     |
| `src/Tape.cpp`      | TAP/TZX tape loading/saving                               |
| `src/Snapshot.cpp`  | SNA/Z80 snapshot load/save                                |
| `src/Config.cpp`    | Persistent configuration (SD card)                        |
| `src/OSDMain.cpp`   | On-screen display menus & dialogs                         |
| `src/FileUtils.cpp` | FatFS-based SD card file system                           |
| `src/pwm_audio.cpp` | PWM audio output driver                                   |
| `src/wd1793.cpp`    | WD1793 floppy disk controller (Beta Disk)                 |

### Driver Subsystems (`drivers/`)

- `vga-nextgen` — VGA output via PIO
- `hdmi` — DVI/HDMI output
- `tv`, `tv-software` — Composite TV output
- `st7789` — TFT display driver
- `psram` — SPI PSRAM (virtual memory for >128K models)
- `fatfs`, `sdcard` — FatFS + SD card
- `nespad` — NES gamepad via shift register
- `ps2kbd`, `ps2` — PS/2 & USB keyboard
- `audio` — Low-level audio

## Emulated Hardware

- **CPU**: Z80 @ 3.5 MHz (Spectrum) / 3.5 MHz (Pentagon), with cycle-accurate contention
- **ULA**: Border, screen, floating bus, snow effect, beeper (port 0xFE)
- **AY-3-8912**: 3-channel PSG, TurboSound (dual AY) support
- **KR580VI53 (Intel 8253 PIT)**: 3-channel square wave synthesizer for Byte computer
  - Channels configured via ports 0x8E/0xAE/0xCE (data) and 0xEE (control)
  - `pitGenSound()` generates audio; `pitChannels[]` stores per-channel state
  - On reset: channels are zeroed (silent). ROM programs them during init.
- **WD1793**: Beta Disk floppy controller (TRD/SCL images)
- **Memory**: 16K pages, virtual memory via SPI PSRAM for Pentagon 512/1024K and Murmuzavr (up to 32 MB)
- **Covox**: DAC sound via port 0xFB or 0xDD

## Audio System

Audio is mixed per-frame into `audioBuffer_L[]` / `audioBuffer_R[]`:
1. **Beeper** — oversampled into `overSamplebuf[]`, then downsampled
2. **AY** — `chip0` / `chip1` generate into `SamplebufAY_L[]` / `SamplebufAY_R[]`
3. **PIT** — `pitGenSound()` fills `audioBufferPIT[]`
4. **Covox** — fills `audioBufferCovox[]`
5. Final mix clamps to 0–255, sent via `pwm_audio_write()`

## Conventions

- `IRAM_ATTR` / `__not_in_flash_func()` — hot functions placed in RAM for speed
- Code uses 4-space indentation (some files use 2-space after recent reformatting)
- Comments mix English, Spanish (original ESPectrum), and Russian (Byte/PIT additions)
- `Z80Ops::isByte` — flag for Byte computer mode (enables PIT sound, special contention)
- `Z80Ops::isALF` — flag for ALF TV Game mode
- `Z80Ops::isPentagon` — Pentagon timing (no contention)

## Display Outputs

Configured at build time via CMake:
- `VGA_HDMI` (default) — VGA + HDMI via PIO, 8 pins from `VGA_BASE_PIN`
- `TFT` — SPI TFT (ST7789 / ILI9341)
- `TV` / `SOFTTV` — Composite video
