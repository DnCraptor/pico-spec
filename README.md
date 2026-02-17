# pico-spec

ESPectrum (1.2) port for Raspberry RP2040 or RP2350 SoC<br/>

In progress...<br/>
<br/>
[Original repo](https://github.com/EremusOne/ESPectrum)

![ESPectrum](https://zxespectrum.speccy.org/wp-content/uploads/2023/05/ESPectrum-logo-v02-2.png)

This is an emulator of the Sinclair ZX Spectrum comparible computers running on RP2040 or RP2350 SoC powered boards.

Board supported:
 - "Murmulator 1.x" + Raspberry "Pi Pico" / "Pi Pico 2" or compartible;
 - "Murmulator 2.0" + Raspberry "Pi Pico" / "Pi Pico 2" or compartible;
 - Waveshare "RP2040-PiZero" / "RP2350-PiZero" + use PCM510x for best sound;
 - Pimoroni "Pico DV Demo Base" + Raspberry "Pi Pico" / "Pi Pico 2" or compartible;
 - Olimex "RP2040-PICO-PC" board + Raspberry "Pi Pico" / "Pi Pico 2" or compartible.

Best performance for case Pimoroni "Pico Plus 2" is used.

## Features

- ZX Spectrum 48K, 128K, Pentagon 128k/512k/1024k, Byte and ALF TV Game. 100% cycle accurate emulation.
- State of the art Z80 emulation (Authored by [José Luis Sánchez](https://github.com/jsanchezv/z80cpp))
- Selectable Sinclair 48K, Sinclair 128K and Amstrad +2 english and spanish ROMs. Byte and ALF TV Game - russian ROMs, + Pentagons with Gluck services ROMs & TR-DOS 5.05D ROM.
- Possibility of using custom ROM with easy flashing procedure from SD card.
- ZX81+ IF2 ROM by courtesy Paul Farrow with .P file loading from SD card.
- 6 bpp VGA output in three modes: Standard VGA (60 and 70hz), VGA 50hz and CRT 15khz 50hz.
- VGA/HDMI fake scanlines effect.
- TV-composite viseo out.
- Multicolor attribute effects emulated (Bifrost*2, Nirvana and Nirvana+ engines).
- Border effects emulated (Aquaplane, The Sentinel, Overscan demo).
- Floating bus effect emulated (Arkanoid, Sidewize).
- Snow effect accurate emulation (as [described](https://spectrumcomputing.co.uk/forums/viewtopic.php?t=8240) by Weiv and MartianGirl).
- Gigascreen support.
- Murmuzavr (up to 32 MB) support.
- Contended memory and contended I/O emulation.
- AY-3-8912 / TurboSound emulation.
- SAA1099 sound chip emulation (https://en.wikipedia.org/wiki/Philips_SAA1099).
- Beeper & Mic emulation (Cobra’s Arc).
- Dual keyboard support: you can connect two devices: first using PS/2 protocol and second using USB at the same time.
- PS/2 Joystick emulation (Cursor, Sinclair, Kempston and Fuller).
- Two real joysticks support (Up to 8 button joysticks).
- Emulation of Betadisk interface with four drives and TRD (read and write) and SCL (read only) support. Fast and tealtime modes.
- Realtime (with OSD) TZX and TAP file loading.
- Flashload of TAP files.
- Rodolfo Guerra's ROMs fast load routines support with on the fly standard speed blocks translation.
- TAP file saving to SD card.
- SNA and Z80 snapshot loading.
- Snapshot saving and loading.
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

- F1 Main menu
- F2 Load (SNA,Z80,P)
- F3 Load custom snapshot
- F4 Save customn snapshot
- F5 Select TAP file
- F6 Play/Stop tape
- F7 Tape Browser
- F8 CPU / Tape load stats ( [CPU] microsecs per CPU cycle, [IDL] idle microsecs, [FPS] Frames per second, [FND] FPS w/no delay applied )
- F9 Volume down
- F10 Volume up
- F11 Hard reset
- F12 Reset RP2040/RP2350
- ALT+F1 Hardware info
- ALT+F2 Turbo mode
- ALT+F3 Set port reading breakpoint
- ALT+F4 Set port writing breakpoint
- ALT+F5 Debug
- ALT+F7 Breakpoint at address
- ALT+F8 Jump to address
- ALT+F9 Input poke
- ALT+F10 NMI
- ALT+F11 Gluck service (in case available)
- ALT+F12 Update Firmware
- ALT+PageUp Switch Gigascreen mode ON/OFF
- Pause Pause
- PrntScr BMP screen capture (Folder /spec/.c at SDCard)
- WASD/KL - Kempstron joystik parallel-emulation

## How to flash custom ROMs

Two custom ROMs can be installed: one for the 48K architecture and another for the 128K architecture.

The "Update firmware" option is now changed to the "Update" menu with three options: firmware, custom ROM 48K, and custom ROM 128K.

Just like updating the firmware requires a file named "firmware.bin" in the root directory of the SD card, for the emulator to install the custom ROMs, the files must be placed in the mentioned root directory and named as "48custom.rom" and "128custom.rom" respectively.

For the 48K architecture, the ROM file size must be 16384 bytes.

For the 128K architecture, it can be either 16kb or 32kb. If it's 16kb, the second bank of the custom ROM will be flashed with the second bank of the standard Sinclair 128K ROM.

It is important to note that for custom ROMs, fast loading of taps can be used, but the loading should be started manually, considering the possibility that the "traps" of the ROM loading routine might not work depending on the flashed ROM. For example, with Rodolfo Guerra's ROMs, both loading and recording traps using the SAVE command work perfectly.

Finally, keep in mind that when updating the firmware, you will need to re-flash the custom ROMs afterward, so I recommend leaving the files "48custom.rom" and "128custom.rom" on the card for the custom ROMs you wish to use.

## Own build way (Win 10+)
 - Install VSCode [pico-setup-windows-x64-standalone.exe](https://github.com/raspberrypi/pico-setup-windows/releases) it will tune up environment and install default SDK 1.5.1;
 - In VSCode install [Rapberri Pi Pico](https://t.me/ZX_MURMULATOR/42804/194110) plugin, to make other SDK versions available and auto-load;
 - Import this project, and agree on all requests from the plugin (it may be required to wait some times on these steps);
 - Tune up build to be [Pico/Release](https://t.me/ZX_MURMULATOR/42804/214274)
 - Set required variables in your local copy of [CMakeLists.txt](https://github.com/DnCraptor/pico-spec/blob/main/CMakeLists.txt)
 - [Clean/Reconfigure](https://t.me/ZX_MURMULATOR/42804/214276)
 - Build.

## Thanks to

- [Original repo](https://github.com/EremusOne/ESPectrum)
- [Murmulator comunity](https://t.me/ZX_MURMULATOR)

