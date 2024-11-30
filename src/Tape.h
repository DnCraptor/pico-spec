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

#ifndef Tape_h
#define Tape_h

#include <inttypes.h>
#include <vector>
#include <string>
#include "ff.h"

using namespace std;

// Tape file types
#define TAPE_FTYPE_EMPTY 0
#define TAPE_FTYPE_TAP 1
#define TAPE_FTYPE_TZX 2
#define TAPE_FTYPE_WAV 3
#define TAPE_FTYPE_MP3 4

// Tape status definitions
#define TAPE_STOPPED 0
#define TAPE_LOADING 1

// Saving status
#define SAVE_STOPPED 0
#define TAPE_SAVING 1

// Tape phases
#define TAPE_PHASE_STOPPED 0
#define TAPE_PHASE_SYNC 1
#define TAPE_PHASE_SYNC1 2
#define TAPE_PHASE_SYNC2 3
#define TAPE_PHASE_DRB 4
#define TAPE_PHASE_PAUSE 5
#define TAPE_PHASE_TAIL 6
#define TAPE_PHASE_DATA1 7
#define TAPE_PHASE_DATA2 8
#define TAPE_PHASE_PURETONE 9
#define TAPE_PHASE_PULSESEQ 10
#define TAPE_PHASE_CSW 11
#define TAPE_PHASE_GDB_PILOTSYNC 12
#define TAPE_PHASE_GDB_DATA 13

#define TAPE_PHASE_TAIL_GDB 14
#define TAPE_PHASE_PAUSE_GDB 15
#define TAPE_PHASE_TAIL_LEN_GDB 945

#define TAPE_PHASE_END 16

// Tape sync phases lenght in microseconds
#define TAPE_SYNC_LEN 2168 // 620 microseconds for 2168 tStates (48K)
#define TAPE_SYNC1_LEN 667 // 190 microseconds for 667 tStates (48K)
#define TAPE_SYNC2_LEN 735 // 210 microseconds for 735 tStates (48K)

#define TAPE_HDR_LONG 8063   // Header sync lenght in pulses
#define TAPE_HDR_SHORT 3223  // Data sync lenght in pulses

#define TAPE_BIT0_PULSELEN 855 // tstates = 244 ms, lenght of pulse for bit 0
#define TAPE_BIT1_PULSELEN 1710 // tstates = 488 ms, lenght of pulse for bit 1

#define TAPE_PHASE_TAIL_LEN 7000 // 7000 -> 2 ms. (It seems more solid with some loader than 1 ms.)

#define TAPE_BLK_PAUSELEN 3500000UL // 1000 ms. of pause between blocks

// Tape sync phases lenght in microseconds for Rodolfo Guerra ROMs
#define TAPE_SYNC_LEN_RG 1408 // 620 microseconds for 2168 tStates (48K)
#define TAPE_SYNC1_LEN_RG 397 // 190 microseconds for 667 tStates (48K)
#define TAPE_SYNC2_LEN_RG 317 // 210 microseconds for 735 tStates (48K)

#define TAPE_HDR_LONG_RG 4835   // Header sync lenght in pulses
#define TAPE_HDR_SHORT_RG 1930  // Data sync lenght in pulses

#define TAPE_BIT0_PULSELEN_RG 325 // tstates = 244 ms, lenght of pulse for bit 0
#define TAPE_BIT1_PULSELEN_RG 649 // tstates = 488 ms, lenght of pulse for bit 1

#define TAPE_BLK_PAUSELEN_RG 1113000UL // 318 ms.

#define TAPE_LISTING_DIV 16

#define CHUNK_SIZE 1024
struct TZXBlock {
    uint8_t BlockType;   
    char FileName[11];
    uint16_t PauseLenght;
    uint32_t BlockLenght;
};

struct Symdef {
    uint8_t SymbolFlags;
    uint16_t* PulseLenghts;
};

struct Prle {
    uint8_t Symbol;
    uint16_t Repetitions;
};

class TapeBlock {
public:
    enum BlockType {
        Program_header,
        Number_array_header,
        Character_array_header,
        Code_header,
        Data_block,
        Info,
        Unassigned
    };
    uint32_t StartPosition; // Start point of this block?
};

/*
from https://eax.me/scala-wav/
Смещение   Байт  Описание
------------------------------------------------------------------
0x00 (00)  4     "RIFF", сигнатура
0x04 (04)  4     размер фала в байтах минус 8
0x08 (08)  8     "WAVEfmt "
0x10 (16)  4     16 для PCM, оставшийся размер заголовка
0x14 (20)  2     1 для PCM, иначе есть какое-то сжатие
0x16 (22)  2     число каналов - 1, 2, 3...
0x18 (24)  4     частота дискретизации
0x1c (28)  4     байт на одну секунду воспроизведения
0x20 (32)  2     байт для одного сэпла включая все каналы
0x22 (34)  2     бит в сэмпле на один канал
0x24 (36)  4     "data" (id сабчанка)
0x28 (40)  4     сколько байт данных идет далее (размер сабчанка)
0x2c (44)  -     данные
*/
typedef struct wav {
    char RIFF[4];
    uint32_t f_szie; // -8
    char WAVEfmt[8];
    uint32_t h_size; // 16
    uint16_t pcm; // 1
    uint16_t ch; // 1 or 2
    uint32_t freq;
    uint32_t byte_per_second;
    uint16_t byte_per_sample;
    uint16_t bit_per_sample; // for 1 channel
    char data[4]; // id for subchunk
    uint32_t subchunk_size;
} wav_t;

class Tape {
public:

    // Tape
    static wav_t wav;
    static uint32_t wav_offset;
    static uint32_t mp3_read;
    static FIL tape;
    static FIL cswBlock;    
    static string tapeFileName;
    static string tapeSaveName;
    static int tapeFileType;
    static uint8_t tapeEarBit;
    static uint8_t tapeStatus;
    static uint8_t SaveStatus;
    static uint8_t romLoading;
    static int tapeCurBlock;  
    static int tapeNumBlocks;  
    static uint32_t tapebufByteCount;
    static uint32_t tapePlayOffset;    
    static size_t tapeFileSize;
 
    static uint8_t tapePhase;    

    static std::vector<TapeBlock> TapeListing;

    static void Init();
    static void LoadTape(string mFile);
    static void Play();
    static void Stop();
    static void Read();
    static bool FlashLoad();
    static void Save();

    static uint32_t CalcTapBlockPos(int block);
    static uint32_t CalcTZXBlockPos(int block);    
    static string tapeBlockReadData(int Blocknum);
    static string tzxBlockReadData(int Blocknum);    

private:

    static void (*GetBlock)();

    static void MP3_Open(string name);
    static void MP3_GetBlock();
    static void WAV_Open(string name);
    static void WAV_GetBlock();
    static void TAP_Open(string name);
    static void TAP_GetBlock();
    static void TZX_Open(string name);
    static void TZX_GetBlock();
    static void TZX_BlockLen(TZXBlock &blockdata);

    static int inflateCSW(int blocknumber, long startPos, long data_length);

    // Tape timing values
    static uint16_t tapeSyncLen;
    static uint16_t tapeSync1Len;
    static uint16_t tapeSync2Len;
    static uint16_t tapeBit0PulseLen; // lenght of pulse for bit 0
    static uint16_t tapeBit1PulseLen; // lenght of pulse for bit 1
    static uint16_t tapeHdrLong;  // Header sync lenght in pulses
    static uint16_t tapeHdrShort; // Data sync lenght in pulses
    static uint32_t tapeBlkPauseLen; 
    static uint8_t tapeLastByteUsedBits;
    static uint8_t tapeEndBitMask;
    static uint32_t tapeNext;

    static uint8_t tapeCurByte;
    static uint64_t tapeStart;
    static uint16_t tapeHdrPulses;
    static uint32_t tapeBlockLen;
    static uint8_t tapeBitMask;

    static uint16_t nLoops;
    static uint16_t loopStart;
    static uint32_t loop_tapeBlockLen;
    static uint32_t loop_tapebufByteCount;
    static bool loop_first;

    static uint16_t callSeq;
    static int callBlock;

    static int CSW_SampleRate;
    static int CSW_PulseLenght;    
    static uint8_t CSW_CompressionType;
    static uint32_t CSW_StoredPulses;

    // GDB vars
    static uint32_t totp;
    static uint8_t npp;
    static uint16_t asp;
    static uint32_t totd;
    static uint8_t npd;
    static uint16_t asd;
    static uint32_t curGDBSymbol;
    static uint8_t curGDBPulse;
    static uint8_t GDBsymbol;    
    static uint8_t nb; 
    static uint8_t curBit;       
    static bool GDBEnd;     
    static Symdef* SymDefTable;

};


#endif
