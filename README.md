# pico-spec

ESPectrum (1.2) port for Raspberry RP2040 or RP2350 SoC<br/>

In progress...<br/>
<br/>
[Original repo](https://github.com/EremusOne/ESPectrum)

![ESPectrum](https://zxespectrum.speccy.org/wp-content/uploads/2023/05/ESPectrum-logo-v02-2.png)

This is an emulator of the Sinclair ZX Spectrum compatible computers running on RP2040 or RP2350 SoC powered boards.

Board supported:
 - "Murmulator 1.x" + Raspberry "Pi Pico" / "Pi Pico 2" or compatible;
 - "Murmulator 2.0" + Raspberry "Pi Pico" / "Pi Pico 2" or compatible;
 - Waveshare "RP2040-PiZero" / "RP2350-PiZero" + use PCM5122 for best sound;
 - Pimoroni "Pico DV Demo Base" + Raspberry "Pi Pico" / "Pi Pico 2" or compatible;
 - Olimex "RP2040-PICO-PC" board + Raspberry "Pi Pico" / "Pi Pico 2" or compatible.

Best performance for case Pimoroni "Pico Plus 2" is used.

## Features

- ZX Spectrum 48K, 128K, Pentagon 128k/512k/1024k, Byte and ALF TV Game. 100% cycle accurate emulation.
- State of the art Z80 emulation (Authored by [José Luis Sánchez](https://github.com/jsanchezv/z80cpp))
- Selectable Sinclair 48K, Sinclair 128K and Amstrad +2 english and spanish ROMs. Byte and ALF TV Game - russian ROMs, + Pentagons with Gluck services ROMs & TR-DOS 5.05D ROM.
- Possibility of using custom ROM with easy flashing procedure from SD card.
- ZX81+ IF2 ROM by courtesy Paul Farrow with .P file loading from SD card.
- Timex SCLD video modes emulation (hi-res 512->256 OR-merge, hi-color, dual-screen).
- Pentagon 16-color video mode (Pentagon only): per-pixel 16-color attribute mode toggleable from the OSD Video menu.
- VGA/HDMI output with 5 selectable video modes: 640x480@60Hz, 640x480@50Hz, 720x480@60Hz, 720x576@60Hz, 720x576@50Hz.
- Hot video mode switching without reboot (VGA/HDMI).
- VGA/HDMI scanlines effect.
- HDMI dither effect for ULA+ (RP2350 only): optional Bayer-look palette dithering applied via ISR.
- HDMI audio output (RP2350 only).
- TV-composite video out.
- PCM5122 I2S audio DAC support (Waveshare PiZero boards - https://www.waveshare.com/wiki/PCM5122-Audio-Board-A).
- Multicolor attribute effects emulated (Bifrost*2, Nirvana and Nirvana+ engines).
- Border effects emulated (Aquaplane, The Sentinel, Overscan demo).
- Floating bus effect emulated (Arkanoid, Sidewize).
- Snow effect accurate emulation (as [described](https://spectrumcomputing.co.uk/forums/viewtopic.php?t=8240) by Weiv and MartianGirl).
- Gigascreen support (Choose between three modes: On, Off, or Auto) (RP2350 only).
- Selectable color palettes: Pulsar (default), Alone, Grayscale, Mars, Ocean (Unreal Speccy compatible format).
- Custom palettes support: load user-defined palettes from `/palette.nvs` file on SD card (up to 11 custom palettes, 3x3 RGB color transform matrix).
- Ula+ support (https://sinclair.wiki.zxnet.co.uk/wiki/ULAplus).
- Murmuzavr (up to 32 MB) support.
- Z80 DMA / zxnDMA emulation: Port #0B (MB02+) and Port #6B (DATA-GEAR) modes (RP2350 only).
- Contended memory and contended I/O emulation.
- AY-3-8912 / TurboSound emulation.
- SAA1099 sound chip emulation (https://en.wikipedia.org/wiki/Philips_SAA1099).
- General Sound (GS) emulation: dedicated Z80 @ 12 MHz on core1 with 2 MB sample RAM, ring-buffered DAC, host→GS FIFO for no-handshake loaders. Auto-enabled on RP2350 boards with butter PSRAM.
- MIDI support: external UART output (AY bit-bang, ShamaZX) and built-in software synthesizer (RP2350 only).
- Beeper & Mic emulation (Cobra’s Arc).
- Dual keyboard support: you can connect two devices: first using PS/2 protocol and second using USB at the same time.
- PS/2 Joystick emulation (Cursor, Sinclair, Kempston and Fuller).
- Two real joysticks support (Up to 8 button joysticks).
- Emulation of Betadisk interface with four drives and TRD, SCL, UDI and FDI (read and write) support. Fast and realtime modes. Per-drive Write Protect, inline drive status in the Drives menu, F5 slot-picker popup (F2 toggle WP, F8 eject) when mounting from the file browser.
- MB-02+ disk interface emulation: WD2797 FDC, Z80-DMA, 512KB SRAM paging, BS-DOS 308, MBD disk images, 4 drives, NMI menu (RP2350 only).
- esxDOS support (DivMMC, DivIDE, DivSD) — [esxdos.org](https://esxdos.org/index.html).
- Z-Controller emulation: raw SD card access via ports #57/#77, mutually exclusive with esxDOS and MB-02+ (RP2350 only).
- FDD activity LED indicator and mechanical head click/seek sound emulation (optional, toggled via Betadisk menu).
- Realtime (with OSD) TZX, TAP and PZX file loading.
- Flashload of TZX/TAP/PZX files (standard loaders only).
- Rodolfo Guerra's ROMs fast load routines support with on the fly standard speed blocks translation.
- TAP file saving to SD card.
- SNA and Z80 snapshot loading.
- Snapshot saving and loading with named slots. Quick load/save hotkeys.
- ZIP archive support: browse, extract, load and delete files inside ZIP archives.
- Configurable keyboard hotkeys with hint display in menus.
- Enhanced debugger: multi-breakpoint (up to 20), memory editor, port read/write breakpoints.
- Hardware info menu: Chip Info (model, cores, frequency, VREG voltage), Board Info (flash, PSRAM, SDK version) and Emulator Info (machine, video, sound, input and storage configuration).
- ZX Keyboard overlay (main menu → ZX Keyboard): full-screen bitmap of the Spectrum keyboard for quick reference. Thanks to @const_bill and @tecnocat.
- Overclock menu: CPU frequency (RP2350: 252/378/504 MHz; RP2040: 252/378 MHz), Flash frequency (33–166 MHz), PSRAM frequency (66–166 MHz), VReg voltage (RP2350: 1.15–1.80 V).
- Complete file navigation system with autoindexing, folder support and search functions.
- Complete OSD menu in two languages: English & Spanish.
- BMP screen capture to SD Card (thanks David Crespo 😉).

## Byte Emulation Details (https://zxbyte.org/)

- 48K ROM
- 128K ROM + TR-DOS
- 128K ROM + TR-DOS + Mr. Gluk Reset Service
- Sovmest (COBMECT) Mode (more accurate emulation of a real ZX Spectrum 48/128)
- Support for the KR580VI53 (a clone of the Intel 8253) three-channel timer

## Installing

You can flash the binaries directly to the board: [Releases](https://github.com/DnCraptor/pico-spec/releases)

## Keyboard functions

Default hotkey bindings (all hotkeys except F1 and ALT+F1 are reconfigurable via OSD menu):

- F1 Main menu
- F2 Load (SNA,Z80,P)
- F3 Load custom snapshot
- F4 Save custom snapshot
- F5 Load file (TAP, TZX, PZX, TRD, SCL, UDI, FDI, MBD, SNA, Z80, MMC, HDF, DSK, ZIP)
- F6 Play/Stop tape
- F7 Tape Browser
- F8 CPU / Tape load stats ( [CPU] microsecs per CPU cycle, [IDL] idle microsecs, [FPS] Frames per second, [FND] FPS w/no delay applied )
- F9 Volume down
- F10 Volume up
- F11 Hard reset
- F12 Reset RP2040/RP2350
- ~ (Tilde) Max speed toggle
- Pause Pause
- ALT+F1 Hardware info
- ALT+F2 Turbo mode
- ALT+F5 Debug
- ALT+F6 Disk menu
- ALT+F7 Breakpoint list
- ALT+F8 Jump to address
- ALT+F9 Input poke
- ALT+F10 NMI (Pentagon: modal menu with NMI / Magic Button options)
- ALT+F11 Reset to... (modal menu: Service/Gluk, TR-DOS, 128K, 48K — depends on machine)
- ALT+F12 USB Boot / Update Firmware
- ALT+PageUp Switch Gigascreen mode ON/OFF
- ALT+F3 Quick load snapshot
- ALT+F4 Quick save snapshot
- ALT+CTRL+Home Switch HDMI video mode (60Hz cycle)
- ALT+CTRL+End Switch HDMI video mode (50Hz cycle)
- PrntScr BMP screen capture (Folder /spec/.c at SDCard)
- WASD/KL - Kempston joystick parallel-emulation

## How to flash custom ROMs

Two custom ROMs can be installed: one for the 48K architecture and another for the 128K architecture.

The "Update firmware" option is now changed to the "Update" menu with three options: firmware, custom ROM 48K, and custom ROM 128K.

Just like updating the firmware requires a file named "firmware.bin" in the root directory of the SD card, for the emulator to install the custom ROMs, the files must be placed in the mentioned root directory and named as "48custom.rom" and "128custom.rom" respectively.

For the 48K architecture, the ROM file size must be 16384 bytes.

For the 128K architecture, it can be either 16kb or 32kb. If it's 16kb, the second bank of the custom ROM will be flashed with the second bank of the standard Sinclair 128K ROM.

It is important to note that for custom ROMs, fast loading of taps can be used, but the loading should be started manually, considering the possibility that the "traps" of the ROM loading routine might not work depending on the flashed ROM. For example, with Rodolfo Guerra's ROMs, both loading and recording traps using the SAVE command work perfectly.

Finally, keep in mind that when updating the firmware, you will need to re-flash the custom ROMs afterward, so I recommend leaving the files "48custom.rom" and "128custom.rom" on the card for the custom ROMs you wish to use.

## MIDI Support

The emulator supports MIDI output on RP2350 boards only. Enable it in the OSD menu: **Audio → MIDI**.

Three modes are available:

- **AY** — Decodes bit-bang UART transmitted through AY-3-8912 register 14 (IOPortA, bit 2). Software like [zx-midiplayer](https://github.com/UzixLS/zx-midiplayer) uses this method in "128std" / "TS1" / "TS2" output modes. MIDI bytes are sent to an external synth via UART TX pin at 31250 baud.
- **ShamaZX** — Emulates the ShamaZX parallel MIDI interface (SAM2695 synth module). Port 0xA0CF is used for TX data, port 0xA1CF for status (bit 6 = busy). This corresponds to the "ShamaZX" output mode in [zx-midiplayer](https://github.com/UzixLS/zx-midiplayer). Output via UART TX pin.
- **Software** — Built-in software MIDI synthesizer. No external hardware needed — MIDI is synthesized directly on the RP2350 and mixed into the audio output. Supports 16-voice polyphony, General MIDI program changes, velocity, channel volume, expression, pan, and pitch bend. Works with both AY bit-bang and ShamaZX protocols. When Software mode is selected, a **Synth Preset** submenu appears with 8 presets:
  - **GM** — General MIDI mapping: different waveforms per instrument family (triangle for piano/pipes, saw for strings/bass, square for organs/brass, noise for percussion).
  - **Piano** — All instruments rendered as triangle wave with natural decay.
  - **Chiptune** — All square wave with varied duty cycles, no filtering — classic 8-bit sound.
  - **Strings** — All saw wave, slow attack, long sustain, warm low-pass filter.
  - **Rock** — Bright and punchy: saw for most instruments, square for organ/brass/reed.
  - **Organ** — All square wave (75% duty), sustained tone with minimal decay.
  - **Music Box** — Triangle wave with fast decay and low sustain — delicate and percussive.
  - **Synth** — All saw wave with medium low-pass filter.

### MIDI TX Pin Configuration

The MIDI TX pin is configured per board in `CMakeLists.txt` via `MIDI_TX_PIN`. Default values by board:

| Board | MIDI_TX_PIN |
|-------|-------------|
| Murmulator | 22 |
| Waveshare PiZero | 26 |
| Pimoroni Pico DV | 21 |
| Olimex RP2040-PICO-PC | 22 |

**Note:** On Murmulator boards, MIDI TX and real tape input share the same pin (GPIO 22). When MIDI is enabled, real tape loading is disabled. Disable MIDI in the menu to use real tape input.

### Connecting to Raspberry Pi 3/4 as MIDI Host

You can use a Raspberry Pi 3 or 4 as a USB MIDI host with a hardware synth or software synthesizer (e.g. FluidSynth).

**Wiring** (directly, no optocoupler needed for short connections):

```
RP2350 Board              Raspberry Pi 3/4
─────────────             ────────────────
MIDI_TX_PIN  ──────────── GPIO 15 (RXD, pin 10)
+5V          ──────────── +5V (e.g. pin 2)
GND          ──────────── GND (e.g. pin 6)
```

## How to build
### Windows 10+
 - Install VSCode [pico-setup-windows-x64-standalone.exe](https://github.com/raspberrypi/pico-setup-windows/releases) it will tune up environment and install default SDK 1.5.1;
 - In VSCode install [Raspberry Pi Pico](https://t.me/ZX_MURMULATOR/42804/194110) plugin, to make other SDK versions available and auto-load;
 - Import this project, and agree on all requests from the plugin (it may be required to wait some times on these steps);
 - Tune up build to be [Pico/Release](https://t.me/ZX_MURMULATOR/42804/214274)
 - Set required variables in your local copy of [CMakeLists.txt](https://github.com/DnCraptor/pico-spec/blob/main/CMakeLists.txt)
 - [Clean/Reconfigure](https://t.me/ZX_MURMULATOR/42804/214276)
 - Build.
### Linux
 - Install dependencies: build-essential, gcc-arm-none-eabi
 - Clone pico-sdk from [its repository](https://github.com/raspberrypi/pico-sdk) into directory near this project.
`git clone --recursive https://github.com/raspberrypi/pico-sdk`
Your filesystem tree must be look like:
```
 Base folder
   |-- pico-sdk
   |-- pico-spec
        |-- build
        |-- drivers
        |-- src
```
 - Configure building options in `pico-spec/CMakeLists.txt` - pico board, video&audio output, etc.

#### CMake build options

| Option | Description |
|--------|-------------|
| `-DMURM=ON` | Build for Murmulator 1.x |
| `-DMURM2=ON` | Build for Murmulator 2.0 (default) |
| `-DPICO_PC=ON` | Build for Olimex RP2040-PICO-PC |
| `-DPICO_DV=ON` | Build for Pimoroni Pico DV Demo Base |
| `-DZERO=ON` | Build for Waveshare RP2040-PiZero |
| `-DZERO2=ON` | Build for Waveshare RP2350-PiZero |
| `-DVGA_HDMI=ON` | VGA/HDMI output (default) |
| `-DSOFTTV=ON` | Software composite TV output |
| `-DTV=ON` | Hardware composite TV output |
| `-DTFT=ON` | TFT display output |
| `-DILI9341=ON` | ILI9341 TFT display output |
| `-DPICO_PC_DBG_UART=ON` | PICO_PC: enable UART0 on DBG1 header (GP0=TX, GP1=RX) for Debug Probe. Auto-remaps PS/2 keyboard to GP10/GP11 to free the pins. |
| `-DTFT_ST7789=ON` | ST7789 TFT display variant |

#### Multi-target build script

To build firmware for all supported boards and display variants at once, use the `build_all.sh` / `build_all.bat` / `build_all.ps1` scripts in the project root. They build each `(target, display)` pair in its own directory (`build-<TARGET>[-<DISPLAY>]/`) and collect `.uf2` artifacts into `pico-spec/firmware/`.

```
./build_all.sh [--clean] [-j JOBS_PER_BUILD] [-p MAX_PARALLEL] [TARGETS...]
```

- Targets: `MURM_P1 MURM_P2 MURM2_P1 MURM2_P2 PICO_PC PICO_DV ZERO ZERO2` (default: all)
- `--clean` — wipe build dirs first (default: incremental rebuild)
- `-j` — threads per target build (default: `nproc / MAX_PARALLEL`)
- `-p` — max number of targets built concurrently (default: 3)
- Env vars: `BUILD_TYPE` (default `MinSizeRel`), `MAX_PARALLEL`, `JOBS_PER_BUILD`, `CMAKE_GENERATOR`
- Uses `ccache` automatically if installed (`apt install ccache` for ~2-5× faster rebuilds)
- Per-target logs are written to `build-logs/`

Single-target builds produce artifacts in `pico-spec/bin/`.

## Thanks to

- [Original repo](https://github.com/EremusOne/ESPectrum)
- [Murmulator community](https://t.me/ZX_MURMULATOR)

