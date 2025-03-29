/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramón Martinez and Jorge Fuertes
https://github.com/rampa069/ZX-ESPectrum

Original project by Pete Todd
https://github.com/retrogubbins/paseVGA

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

To Contact the dev team you can write to zxespectrum@gmail.com or 
visit https://zxespectrum.speccy.org/contacto

*/

#include "Ports.h"
#include "Config.h"
#include "ESPectrum.h"
#include "Z80_JLS/z80.h"
#include "MemESP.h"
#include "Video.h"
#include "AySound.h"
#include "Tape.h"
#include "CPU.h"
#include "wd1793.h"
#include "pwm_audio.h"
#include "roms.h"

// #pragma GCC optimize("O3")

// Values calculated for BEEPER, EAR, MIC bit mask (values 0-7)
// Taken from FPGA values suggested by Rampa
//   0: ula <= 8'h00;
//   1: ula <= 8'h24;
//   2: ula <= 8'h40;
//   3: ula <= 8'h64;
//   4: ula <= 8'hB8;
//   5: ula <= 8'hC0;
//   6: ula <= 8'hF8;
//   7: ula <= 8'hFF;
// and adjusted for BEEPER_MAX_VOLUME = 97
uint8_t Ports::speaker_values[8]={ 0, 19, 34, 53, 97, 101, 130, 134 };
uint8_t Ports::port[128];
uint8_t Ports::port254 = 0;

uint8_t (*Ports::getFloatBusData)() = &Ports::getFloatBusData48;

uint8_t Ports::getFloatBusData48() {

    unsigned int currentTstates = CPU::tstates;

	unsigned int line = (currentTstates / 224) - 64;
	if (line >= 192) return 0xFF;

	unsigned char halfpix = (currentTstates % 224) - 3;
	if ((halfpix >= 125) || (halfpix & 0x04)) return 0xFF;

    int hpoffset = (halfpix >> 2) + ((halfpix >> 1) & 0x01);;
    
    if (halfpix & 0x01) return(VIDEO::grmem[VIDEO::offAtt[line] + hpoffset]);

    return(VIDEO::grmem[VIDEO::offBmp[line] + hpoffset]);

}

uint8_t Ports::getFloatBusData128() {

    unsigned int currentTstates = CPU::tstates - 1;

	unsigned int line = (currentTstates / 228) - 63;
	if (line >= 192) return 0xFF;

	unsigned char halfpix = currentTstates % 228;
	if ((halfpix >= 128) || (halfpix & 0x04)) return 0xFF;

    int hpoffset = (halfpix >> 2) + ((halfpix >> 1) & 0x01);;
    
    if (halfpix & 0x01) return(VIDEO::grmem[VIDEO::offAtt[line] + hpoffset]);

    return(VIDEO::grmem[VIDEO::offBmp[line] + hpoffset]);

}

#include "OSDMain.h"

uint8_t nes_pad2_for_alf(void);
static uint8_t newAlfBit = 0;

IRAM_ATTR uint8_t Ports::input(uint16_t address) {
    uint8_t data;
    if (address == Config::portReadBP && Config::enablePortReadBP) CPU::portBasedBP = true;
    uint8_t rambank = address >> 14;

    VIDEO::Draw(1, MemESP::ramContended[rambank]); // I/O Contention (Early)
    
    bool ia = Z80Ops::isALF;
    uint8_t p8 = address & 0xFF;
        if (p8 == 0xFB) { // Hidden RAM on
            MemESP::newSRAM = true;
            uint8_t* r0 = MemESP::ram[64 + MemESP::sramLatch].sync();
            if (MemESP::ramCurrent[0] != r0) {
                MemESP::ramCurrent[0] = r0;
            }
            return 0xFF;
        }
        if (p8 == 0x7B) { // Hidden RAM off
            MemESP::newSRAM = false;
            uint8_t* r0 = (MemESP::page0ram ? MemESP::ram[0].sync() : MemESP::rom[MemESP::romInUse].direct());
            if (MemESP::ramCurrent[0] != r0) {
                MemESP::ramCurrent[0] = r0;
            }
            return 0xFF;
        }
    // ULA PORT
    if ((address & 0x0001) == 0) {
        VIDEO::Draw(3, !Z80Ops::isPentagon);   // I/O Contention (Late)
        if (ia && p8 == 0xFE) {
            data = nes_pad2_for_alf(); // default port value is 0xFF.
        } else {
            data = 0xbf; // default port value is 0xBF.
            uint8_t portHigh = ~(address >> 8) & 0xff;
            for (int row = 0, mask = 0x01; row < 8; row++, mask <<= 1) {
                if ((portHigh & mask) != 0)
                    data &= port[row];
            }
        }
        if (Tape::tapeStatus == TAPE_LOADING) {
            Tape::Read();
        }
        if ((Z80Ops::is48) && (Config::Issue2)) {// Issue 2 behaviour only on Spectrum 48K
            if (port254 & 0x18) data |= 0x40;
        } else {
            if (port254 & 0x10) data |= 0x40;
        }
        if (Tape::tapeEarBit) data ^= 0x40;
    } else {
        ioContentionLate(MemESP::ramContended[rambank]);
        if (ia && bitRead(p8, 7) == 0) {
            if (bitRead(p8, 1) == 0) { // 1D
                MemESP::newSRAM = true;
                uint8_t* r0 = MemESP::ram[64 + MemESP::sramLatch].sync();
                if (MemESP::ramCurrent[0] != r0) {
                    MemESP::ramCurrent[0] = r0;
                }
            }
            else { // 1F
                MemESP::newSRAM = false;
                uint8_t* r0 = MemESP::rom[MemESP::romInUse].direct();
                if (MemESP::ramCurrent[0] != r0) {
                    MemESP::ramCurrent[0] = r0;
                }
            }
        }
        // The default port value is 0xFF.
        data = 0xff;
        // Check if TRDOS Rom is mapped.
        if (ESPectrum::trdos) {
            switch (p8) {
                case 0xFF:
                    data = ESPectrum::Betadisk.ReadSystemReg();
                    // printf("WD1793 Read Control Register: %d\n",(int)data);
                    return data;
                case 0x1F:
                    data = ESPectrum::Betadisk.ReadStatusReg();
                    // printf("WD1793 Read Status Register: %d\n",(int)data);
                    return data;
                case 0x3F:
                    data = ESPectrum::Betadisk.ReadTrackReg();
                    // printf("WD1793 Read Track Register: %d\n",(int)data);
                    return data;
                case 0x5F:
                    data = ESPectrum::Betadisk.ReadSectorReg();
                    // printf("WD1793 Read Sector Register: %d\n",(int)data);
                    return data;
                case 0x7F:
                    data = ESPectrum::Betadisk.ReadDataReg();                    
                    // printf("WD1793 Read Data Register: %d\n",(int)data);                    
                    return data;                    
            }
        }

        ///if (ESPectrum::ps2mouse && Config::mouse == 1)
        {
            if((address & 0x05ff) == 0x01df) {
                return (uint8_t) ESPectrum::mouseX;
            }
            if((address & 0x05ff) == 0x05df) {
                return (uint8_t) ESPectrum::mouseY;
            }
            if((address & 0x05ff) == 0x00df) {
                return 0xff & (ESPectrum::mouseButtonL ? 0xfd : 0xff) & (ESPectrum::mouseButtonR ? 0xfe : 0xff);
            }
        }

        // Kempston Joystick
        if (
            (Config::joystick == JOY_KEMPSTON) &&
            ((address & 0x00E0) == 0 || p8 == 0xDF || p8 == Config::kempstonPort)
        ) return ia ? (port[0x1F] ^ 0xA0) : port[Config::kempstonPort];

        // Fuller Joystick
        if (Config::joystick == JOY_FULLER && p8 == 0x7F) return port[0x7f];

        // Sound (AY-3-8912)
        if (ESPectrum::AY_emu) {
            if ((address & 0xC002) == 0xC000) {
                if (ia) {
                    return chips[AySound::selected_chip]->getRegisterData() | newAlfBit;
                }
                return chips[AySound::selected_chip]->getRegisterData();
            }
        }
        if (!Z80Ops::isPentagon) {
            data = getFloatBusData();
            if ((!Z80Ops::is48) && ((address & 0x8002) == 0)) {
                // //  Solo en el modelo 128K, pero no en los +2/+2A/+3, si se lee el puerto
                // //  0x7ffd, el valor leído es reescrito en el puerto 0x7ffd.
                // //  http://www.speccy.org/foro/viewtopic.php?f=8&t=2374
                if (!MemESP::pagingLock) {
                    MemESP::pagingLock = bitRead(data, 5);
                    uint8_t page = (data & 0x7);
                    if (MemESP::bankLatch != page) {
                        MemESP::bankLatch = page;
                        MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch].sync();
                        MemESP::ramContended[3] = MemESP::bankLatch & 0x01 ? true: false;
                    }
                    if (MemESP::videoLatch != bitRead(data, 3)) {
                        MemESP::videoLatch = bitRead(data, 3);
                        VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[7].direct() : MemESP::ram[5].direct();
                    }
                    MemESP::romLatch = bitRead(data, 4);
                    MemESP::romInUse = MemESP::romLatch;
                    uint8_t* r0 = MemESP::newSRAM ? MemESP::ram[64 + MemESP::sramLatch].sync() :
                                  (MemESP::page0ram ? MemESP::ram[0].sync() : MemESP::rom[MemESP::romInUse].direct());
                    if (MemESP::ramCurrent[0] != r0) {
                        MemESP::ramCurrent[0] = r0;
                    }
                }
            }
        }
    }
    return data;
}

IRAM_ATTR void Ports::output(uint16_t address, uint8_t data) {    
    int Audiobit;
    if (address == Config::portWriteBP && Config::enablePortWriteBP) CPU::portBasedBP = true;
    uint8_t rambank = address >> 14;

    VIDEO::Draw(1, MemESP::ramContended[rambank]); // I/O Contention (Early)

    bool ia = Z80Ops::isALF;
    if (ia) {
        if ((address & 0xFF) == 0xFE) {
            newAlfBit = (data >> 3) & 1;
        }
        if (bitRead(address, 7) == 0 && (address & 1) == 1) { // ALF ROM selector A7=0, A0=1
            uint8_t* base = bitRead(data, 7) ? gb_rom_Alf_cart : gb_rom_Alf;
            if (MemESP::ramCurrent[0] != base) { /// TODO: ensure
                int border_page = base == gb_rom_Alf ? 16 : 64;
                for (int i = 0; i < 64; ++i) {
                    MemESP::rom[i].assign_rom(i >= border_page ? gb_rom_Alf_ep : base + ((16 * i) << 10));
                }
            }
            MemESP::romInUse = (data & 0b01111111);
            while (MemESP::romInUse >= 64) MemESP::romInUse -= 64; // rolling ROM
            uint8_t* r0 = MemESP::newSRAM ? MemESP::ram[64 + MemESP::sramLatch].sync() : MemESP::rom[MemESP::romInUse].direct();
            if (MemESP::ramCurrent[0] != r0) {
                MemESP::ramCurrent[0] = r0;
            }
        }
    }
    // ULA =======================================================================
    if ((address & 0x0001) == 0) {
        port254 = data;
        // Border color
        if (VIDEO::borderColor != data & 0x07) {
            VIDEO::brdChange = true;
            if (!Z80Ops::isPentagon) VIDEO::Draw(0,true); // Seems not needed in Pentagon
            VIDEO::DrawBorder();
            VIDEO::borderColor = data & 0x07;
            VIDEO::brd = VIDEO::border32[VIDEO::borderColor];
        }
///        if (ESPectrum::ESP_delay) { // Disable beeper on turbo mode
            if (Config::tape_player)
                Audiobit = Tape::tapeEarBit ? 255 : 0; // For tape player mode
            else
                // Beeper Audio
                Audiobit = speaker_values[((data >> 2) & 0x04 ) | (Tape::tapeEarBit << 1) | ((data >> 3) & 0x01)];
            if (Audiobit != ESPectrum::lastaudioBit) {
                ESPectrum::BeeperGetSample();
                ESPectrum::lastaudioBit = Audiobit;
            }
///        }

        // AY ========================================================================
        if ((ESPectrum::AY_emu) && ((address & 0x8002) == 0x8000)) {
            if ((address & 0x4000) != 0) {
                chips[AySound::selected_chip]->selectRegister(data);
            } else {
                ESPectrum::AYGetSample();
                chips[AySound::selected_chip]->setRegisterData(data);
            }
            VIDEO::Draw(3, !Z80Ops::isPentagon);   // I/O Contention (Late)
            return;
        }
        VIDEO::Draw(3, !Z80Ops::isPentagon);   // I/O Contention (Late)
    } else {
        // AY ========================================================================
        if ((ESPectrum::AY_emu) && (Config::turbosound == 1 || Config::turbosound == 3) && address == 0xFFFD) { // NedoPC way
            if (data == 0xFF) {
                AySound::selected_chip = 0;
            }
            else if (data == 0xFE) {
                AySound::selected_chip = 1;
            }
        }
        if ((ESPectrum::AY_emu) && ((address & 0x8002) == 0x8000)) {
            if ((address & 0xFF) == 0xFF) { // Old TS way
                AySound::selected_chip = 0;
            }
            else if ((address & 0xFF) == 0xFE && Config::turbosound > 1) {
                AySound::selected_chip = 1;
            }
            else if ((address & 0x4000) != 0) {
                chips[AySound::selected_chip]->selectRegister(data);
            } else {
                ESPectrum::AYGetSample();
                chips[AySound::selected_chip]->setRegisterData(data);
            }
            ioContentionLate(MemESP::ramContended[rambank]);
            return;
        }
        // Check if TRDOS Rom is mapped.
        if (ESPectrum::trdos) {
            switch (address & 0xFF) {
                case 0xFF:
                    // printf("WD1793 Write Control Register: %d\n",data);
                    ESPectrum::Betadisk.WriteSystemReg(data);
                    break;
                case 0x1F:
                    // printf("WD1793 Write Command Register: %d\n",data);
                    ESPectrum::Betadisk.WriteCommandReg(data);
                    break;
                case 0x3F:
                    // printf("WD1793 Write Track Register: %d\n",data);                    
                    ESPectrum::Betadisk.WriteTrackReg(data);
                    break;
                case 0x5F:
                    // printf("WD1793 Write Sector Register: %d\n",data);                    
                    ESPectrum::Betadisk.WriteSectorReg(data);
                    break;
                case 0x7F:
                    // printf("WD1793 Write Data Register: %d\n",data);
                    ESPectrum::Betadisk.WriteDataReg(data);
                    break;
            }

        }
        ioContentionLate(MemESP::ramContended[rambank]);
    }
    // Pentagon only
    if (Z80Ops::isPentagon && ((address & 0x1008) == 0)) { // 1008 !-> EFF7
        if (!MemESP::pagingLock) {
            uint8_t prev = MemESP::page0ram;
            MemESP::notMore128 = bitRead(data, 2);
            MemESP::page0ram = bitRead(data, 3);
            if (MemESP::page0ram != prev) {
                uint8_t* r0 = MemESP::newSRAM ?
                          MemESP::ram[64 + MemESP::sramLatch].sync() : 
                          (MemESP::page0ram ? MemESP::ram[0].sync() : MemESP::rom[MemESP::romInUse].direct());
                if (MemESP::ramCurrent[0] != r0) {
                    MemESP::ramCurrent[0] = r0;
                }
            }
        }
    }
    // 128K, Pentagon ==================================================================
    if ((!Z80Ops::is48) && ((address & 0x8002) == 0)) { // 8002 !-> 7FFD
        if (!MemESP::pagingLock) {
            uint8_t D5 = bitRead(data, 5);
            if (Z80Ops::is1024) {
                MemESP::pagingLock = MemESP::notMore128 ? D5 : 0;
            } else {
                MemESP::pagingLock = D5;
            }
            uint8_t page = (data & 0x7);
            if ((Z80Ops::is512 || Z80Ops::is1024) && !MemESP::notMore128 && !MemESP::pagingLock) {
                uint8_t D6 = bitRead(data, 6);
                uint8_t D7 = bitRead(data, 7);
                if (D6) page += 8;
                if (D7) page += 16;
                if (Z80Ops::is1024 && D5) page += 32;
            }
            if (MemESP::bankLatch != page) {
                MemESP::bankLatch = page;
                MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch].sync();
                MemESP::ramContended[3] = Z80Ops::isPentagon ? false : (MemESP::bankLatch & 0x01 ? true: false);
            }
            MemESP::romLatch = bitRead(data, 4);
            if (!ia) {
                MemESP::romInUse = MemESP::romLatch;
            }
            if (MemESP::newSRAM) {
                MemESP::sramLatch = MemESP::romLatch;
            }
            uint8_t* r0 = MemESP::newSRAM ?
                          MemESP::ram[64 + MemESP::sramLatch].sync() :
                         (MemESP::page0ram ? MemESP::ram[0].sync() : MemESP::rom[MemESP::romInUse].direct());
            if (MemESP::ramCurrent[0] != r0) {
                MemESP::ramCurrent[0] = r0;
            }
            if (MemESP::videoLatch != bitRead(data, 3)) {
                MemESP::videoLatch = bitRead(data, 3);
                VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[7].direct() : MemESP::ram[5].direct();
            }
        }
    }
}

IRAM_ATTR void Ports::ioContentionLate(bool contend) {
    if (contend) {
        VIDEO::Draw(1, true);
        VIDEO::Draw(1, true);
        VIDEO::Draw(1, true);        
    } else {
        VIDEO::Draw(3, false);
    }
}
