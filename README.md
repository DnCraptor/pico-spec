# pico-spec

ESPectrum (1.2) port for ZX Murmulator dev. board (Raspberry Pi Pico RP2040 or Raspberry Pi Pico 2 RP2350 SoC)<br/>

In progress...<br/>
<br/>
[Original repo](https://github.com/EremusOne/ESPectrum)

![ESPectrum](https://zxespectrum.speccy.org/wp-content/uploads/2023/05/ESPectrum-logo-v02-2.png)

This is an emulator of the Sinclair ZX Spectrum computer running on Murmulator RP2040 SoC powered boards.

Currently, it can be used with Raspberry Pico Pi (2) board, installed on ZX Murmulator board (VGA versions).

Just connect a VGA monitor or CRT TV (with special VGA-RGB cable needed), a PS/2 keyboard, prepare a SD Card as needed and power via microUSB.

This project is based on David Crespo excellent work on [ZX-ESPectrum-Wiimote](https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote) which is a fork of the [ZX-ESPectrum](https://github.com/rampa069/ZX-ESPectrum) project by Rampa and Queru which was inspired by Pete's Todd [PaseVGA](https://github.com/retrogubbins/paseVGA) project.

## Features

- ZX Spectrum 48K, 128K and Pentagon 128K 100% cycle accurate emulation (no PSRAM needed).
- State of the art Z80 emulation (Authored by [José Luis Sánchez](https://github.com/jsanchezv/z80cpp))
- Selectable Sinclair 48K, Sinclair 128K and Amstrad +2 english and spanish ROMs.
- Possibility of using one 48K and one 128K custom ROM with easy flashing procedure from SD card.
- ZX81+ IF2 ROM by courtesy Paul Farrow with .P file loading from SD card.
- 6 bpp VGA output in three modes: Standard VGA (60 and 70hz), VGA 50hz and CRT 15khz 50hz.
- VGA fake scanlines effect.
- Support for two aspect ratios: 16:9 or 4:3 monitors (using 360x200 or 320x240 modes)
- Multicolor attribute effects emulated (Bifrost*2, Nirvana and Nirvana+ engines).
- Border effects emulated (Aquaplane, The Sentinel, Overscan demo).
- Floating bus effect emulated (Arkanoid, Sidewize).
- Snow effect accurate emulation (as [described](https://spectrumcomputing.co.uk/forums/viewtopic.php?t=8240) by Weiv and MartianGirl).
- Contended memory and contended I/O emulation.
- AY-3-8912 sound emulation.
- Beeper & Mic emulation (Cobra’s Arc).
- Dual PS/2 keyboard support: you can connect two devices using PS/2 protocol at the same time.
- PS/2 Joystick emulation (Cursor, Sinclair, Kempston and Fuller).
- Two real joysticks support (Up to 8 button joysticks) using [ESPjoy adapter](https://antoniovillena.es/store/product/espjoy-for-espectrum/) or DIY DB9 to PS/2 converter.
- Emulation of Betadisk interface with four drives and TRD (read and write) and SCL (read only) support.
- Realtime (with OSD) TZX and TAP file loading.
- Flashload of TAP files.
- Rodolfo Guerra's ROMs fast load routines support with on the fly standard speed blocks translation.
- TAP file saving to SD card.
- SNA and Z80 snapshot loading.
- Snapshot saving and loading.
- Complete file navigation system with autoindexing, folder support and search functions.
- Complete OSD menu in two languages: English & Spanish.
- BMP screen capture to SD Card (thanks David Crespo 😉).

## Work in progress

- +2A/+3 models.
- DSK support.

## Installing

You can flash the binaries directly to the board if do not want to mess with code and compilers. Check the [releases section](https://github.com/EremusOne/ZX-ESPectrum-IDF/releases)

## Compiling and installing

Quick start from PlatformIO:
- Clone this repo and Open from VSCode/PlatFormIO
- Execute task: Upload
- Enjoy

Windows, GNU/Linux and MacOS/X. This version has been developed using PlatformIO.

#### Install platformIO:

- There is an extension for Atom and VSCode, please check [this website](https://platformio.org/).
- Select your board, pico32 which behaves just like the TTGo VGA32.

#### Compile and flash it

`PlatformIO > Project Tasks > Build `, then

`PlatformIO > Project Tasks > Upload`.

Run these tasks (`Upload` also does a `Build`) whenever you make any change in the code.

#### Prepare micro SD Card

The SD card should be formatted in FAT16 / FAT32.

Just that: then put your .sna, .z80, .p, .tap, .trd and .scl whenever you like and create and use folders as you need.

There's also no need to sort files using external utilities: the emulator creates and updates indexes to sort files in folders by itself.

## PS/2 Keyboard functions

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
- F12 Reset RP2040
- ALT+F1 Hardware info
- ALT+F2 Turbo mode
- ALT+F3 Set port reading breakpoint
- ALT+F4 Set port writing breakpoint
- ALT+F5 Debug
- ALT+F7 Breakpoint at address
- ALT+F8 Jump to address
- ALT+F9 Input poke
- ALT+F10 NMI
- ALT+F12 Update Firmware
- Pause Pause
- PrntScr BMP screen capture (Folder /spec/.c at SDCard)

## How to flash custom ROMs

Two custom ROMs can be installed: one for the 48K architecture and another for the 128K architecture.

The "Update firmware" option is now changed to the "Update" menu with three options: firmware, custom ROM 48K, and custom ROM 128K.

Just like updating the firmware requires a file named "firmware.bin" in the root directory of the SD card, for the emulator to install the custom ROMs, the files must be placed in the mentioned root directory and named as "48custom.rom" and "128custom.rom" respectively.

For the 48K architecture, the ROM file size must be 16384 bytes.

For the 128K architecture, it can be either 16kb or 32kb. If it's 16kb, the second bank of the custom ROM will be flashed with the second bank of the standard Sinclair 128K ROM.

It is important to note that for custom ROMs, fast loading of taps can be used, but the loading should be started manually, considering the possibility that the "traps" of the ROM loading routine might not work depending on the flashed ROM. For example, with Rodolfo Guerra's ROMs, both loading and recording traps using the SAVE command work perfectly.

Finally, keep in mind that when updating the firmware, you will need to re-flash the custom ROMs afterward, so I recommend leaving the files "48custom.rom" and "128custom.rom" on the card for the custom ROMs you wish to use.

## Hardware configuration and pinout

Pin assignment in `hardpins.h` is set to match the boards we've tested emulator in, use it as-is, or change it to your own preference.

## Project links

- [Website](https://zxespectrum.speccy.org)
- [Patreon](https://www.patreon.com/ESPectrum)
- [Youtube Channel](https://www.youtube.com/@ZXESPectrum)
- [Twitter](https://twitter.com/ZX_ESPectrum)
- [Telegram](https://t.me/ZXESPectrum)

## Supported hardware

- [Lilygo FabGL VGA32](https://www.lilygo.cc/products/fabgl-vga32?_pos=1&_sid=b28e8cac0&_ss=r)
- [Antonio Villena's ESPectrum board](https://antoniovillena.es/store/product/espectrum/) and [ESPjoy add-on](https://antoniovillena.es/store/product/espjoy-for-espectrum/)
- [ESP32-SBC-FabGL board from Olimex](https://www.olimex.com/Products/Retro-Computers/ESP32-SBC-FabGL/open-source-hardware)

## Thanks to

- [David Crespo](https://youtube.com/Davidprograma) for his friendly help and support and his excellent work at his [Youtube Channel](https://youtube.com/Davidprograma) and the [ZX-ESPectrum-Wiimote](https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote) emulator.
- Pete Todd, developer of the original project [PaseVGA](https://github.com/retrogubbins/paseVGA).
- Ramón Martínez ["Rampa"](https://github.com/rampa069) and Jorge Fuertes ["Queru"](https://github.com/jorgefuertes) who improved PaseVGA in the first [ZX-ESPectrum](https://github.com/rampa069/ZX-ESPectrum).
- Z80 Emulation derived from [z80cpp](https://github.com/jsanchezv/z80cpp), authored by José Luis Sánchez.
- VGA Driver by Murmulator comunity
- AY-3-8912 emulation from [libayemu by Alexander Sashnov](https://asashnov.github.io/libayemu.html).
- PS2 Driver from Fabrizio di Vittorio for his [FabGL library](https://github.com/fdivitto/FabGL).
- [Paul Farrow](http://www.fruitcake.plus.com/index.html) for his kind permission to include his amazing ZX81+ IF2 ROM.
- Azesmbog for testing and providing very valuable info to make the emu more precise.
- David Carrión for hardware and ZX keyboard code.
- ZjoyKiLer for his testing, code and ideas.
- [Ackerman](https://github.com/rpsubc8/ESP32TinyZXSpectrum) for his code and ideas.
- [Mark Woodmass](https://specemu.zxe.io) and [Juan Carlos González Amestoy](https://www.retrovirtualmachine.org) for his excellent emulators and his help with wd1793 emulation and many other things.
- [Rodolfo Guerra](https://sites.google.com/view/rodolfoguerra) for his wonderful enhanced ROMs and his help for adding tape load turbo mode support to the emulator.
- Weiv and [MartianGirl](https://github.com/MartianGirl) for his detailed analysis of Snow effect.
- [Antonio Villena](https://antoniovillena.es/store) for creating the ESPectrum board.
- Tsvetan Usunov from [Olimex Ltd](https://www.olimex.com).
- [Amstrad PLC](http://www.amstrad.com) for the ZX-Spectrum ROM binaries [liberated for emulation purposes](http://www.worldofspectrum.org/permits/amstrad-roms.txt).
- [Jean Thomas](https://github.com/jeanthom/ESP32-APLL-cal) for his ESP32 APLL calculator.

## Thanks also to all this writters, hobbist and documenters

- [Retrowiki](http://retrowiki.es/) especially the people at [ESP32 TTGO VGA32](http://retrowiki.es/viewforum.php?f=114) subforum.
- [RetroReal](https://www.youtube.com/@retroreal) for his kindness and hospitality and his great work.
- Rodrigo Méndez [Ron](https://www.twitch.tv/retrocrypta)
- Armand López [El Viejoven FX](https://www.youtube.com/@ElViejovenFX)
- Javi Ortiz [El Spectrumero](https://www.youtube.com/@ElSpectrumeroJaviOrtiz) 
- José Luis Rodríguez [VidaExtraRetro](https://www.twitch.tv/vidaextraretro)
- [El Mundo del Spectrum](http://www.elmundodelspectrum.com/)
- [Microhobby magazine](https://es.wikipedia.org/wiki/MicroHobby).
- [The World of Spectrum](http://www.worldofspectrum.org/)
- Dr. Ian Logan & Dr. Frank O'Hara for [The Complete Spectrum ROM Disassembly book](http://freestuff.grok.co.uk/rom-dis/).
- Chris Smith for the The [ZX-Spectrum ULA book](http://www.zxdesign.info/book/).

## And all the involved people from the golden age

- [Sir Clive Sinclair](https://en.wikipedia.org/wiki/Clive_Sinclair).
- [Christopher Curry](https://en.wikipedia.org/wiki/Christopher_Curry).
- [The Sinclair Team](https://en.wikipedia.org/wiki/Sinclair_Research).
- [Lord Alan Michael Sugar](https://en.wikipedia.org/wiki/Alan_Sugar).
- [Investrónica team](https://es.wikipedia.org/wiki/Investr%C3%B3nica).
- [Matthew Smith](https://en.wikipedia.org/wiki/Matthew_Smith_(games_programmer)) for [Manic Miner](https://en.wikipedia.org/wiki/Manic_Miner).
