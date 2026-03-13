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

#include <stdio.h>
#include <vector>
#include <string>
#include <inttypes.h>

using namespace std;

#include "Tape.h"
#include "FileUtils.h"
#include "OSDMain.h"
#include "Config.h"
#include "messages.h"
#include "Debug.h"
#include "Z80_JLS/z80.h"

int fseek (FIL* stream, long offset, int origin);
inline static void fclose(FIL& stream) {
    f_close(&stream);
}
#define ftell(x) f_tell(&x)
#define feof(x) f_eof(&x)
inline void rewind(FIL& f) {
    f_lseek(&f, 0);
}
size_t fread(uint8_t* v, size_t sz1, size_t sz2, FIL& f);

// PZX block tags (4 ASCII bytes, little-endian)
enum PZXTag : uint32_t {
    PZX_PZXT = 0x54585A50, // "PZXT" - header
    PZX_PULS = 0x534C5550, // "PULS" - pulse sequence
    PZX_DATA = 0x41544144, // "DATA" - data block
    PZX_PAUS = 0x53554150, // "PAUS" - pause
    PZX_BRWS = 0x53575242, // "BRWS" - browse point
    PZX_STOP = 0x504F5453, // "STOP" - stop tape
};

// PZX static variable definitions
uint16_t Tape::pzxPulseRep = 0;
uint32_t Tape::pzxPulseDur = 0;
uint32_t Tape::pzxPulseBlockEnd = 0;
uint16_t Tape::pzxS0[256];
uint16_t Tape::pzxS1[256];
uint8_t Tape::pzxP0 = 0;
uint8_t Tape::pzxP1 = 0;
uint8_t Tape::pzxCurSymPulse = 0;
uint32_t Tape::pzxBitCount = 0;
uint16_t Tape::pzxTailLen = 0;
uint32_t Tape::pzxDataBlockEnd = 0;
bool Tape::pzxFlashCont = false;

// Read PZX block tag and size, leaves file position after the size field
void Tape::PZX_BlockLen(uint32_t &tag, uint32_t &size) {
    FIL* tape = &Tape::tape;
    tag = readByteFile(tape) | (readByteFile(tape) << 8) |
          (readByteFile(tape) << 16) | (readByteFile(tape) << 24);
    size = readByteFile(tape) | (readByteFile(tape) << 8) |
           (readByteFile(tape) << 16) | (readByteFile(tape) << 24);
}

void Tape::PZX_Open(string name) {

    if (tapeFileType != TAPE_FTYPE_EMPTY) {
        fclose(tape);
        tapeFileType = TAPE_FTYPE_EMPTY;
    }

    string fname = FileUtils::TAP_Path + "/" + name;

    if (f_open(&tape, fname.c_str(), FA_READ) != FR_OK) {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR, LEVEL_ERROR);
        tapeFileType = TAPE_FTYPE_EMPTY;
        return;
    }

    tapeFileSize = f_size(&tape);
    if (tapeFileSize == 0) {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR, LEVEL_ERROR);
        fclose(tape);
        tapeFileType = TAPE_FTYPE_EMPTY;
        return;
    }

    // Check PZX header: first block must be "PZXT"
    uint32_t tag, size;
    PZX_BlockLen(tag, size);
    if (tag != PZX_PZXT) {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR, LEVEL_ERROR);
        fclose(tape);
        tapeFileType = TAPE_FTYPE_EMPTY;
        return;
    }

    // Check version
    FIL* tp = &tape;
    uint8_t major = readByteFile(tp);
    if (major > 1) {
        OSD::osdCenteredMsg("Unsupported PZX version", LEVEL_ERROR);
        fclose(tape);
        tapeFileType = TAPE_FTYPE_EMPTY;
        return;
    }

    // Skip rest of PZXT block
    fseek(tp, 8 + size, SEEK_SET);

    tapeFileName = name;

    Tape::TapeListing.clear();
    std::vector<TapeBlock>().swap(TapeListing);

    int tapeListIndex = 0;
    uint32_t tapeContentIndex = 8 + size; // position after PZXT block

    TapeBlock block;

    while (tapeContentIndex < tapeFileSize) {
        uint32_t blockTag, blockSize;

        fseek(tp, tapeContentIndex, SEEK_SET);
        PZX_BlockLen(blockTag, blockSize);

        if (blockSize == 0 && blockTag == 0) break; // safety

        if ((tapeListIndex & (TAPE_LISTING_DIV - 1)) == 0) {
            block.StartPosition = tapeContentIndex;
            TapeListing.push_back(block);
        }

        tapeListIndex++;
        tapeContentIndex += 8 + blockSize;
    }

    tapeCurBlock = 0;
    tapeNumBlocks = tapeListIndex;

    tapeFileType = TAPE_FTYPE_PZX;

    rewind(Tape::tape);
}

uint32_t Tape::CalcPZXBlockPos(int block) {
    int TapeBlockRest = block & (TAPE_LISTING_DIV - 1);
    int CurrentPos = TapeListing[block / TAPE_LISTING_DIV].StartPosition;

    FIL* tp = &tape;
    fseek(tp, CurrentPos, SEEK_SET);

    while (TapeBlockRest-- != 0) {
        uint32_t tag, size;
        PZX_BlockLen(tag, size);
        CurrentPos += 8 + size;
        fseek(tp, CurrentPos, SEEK_SET);
    }

    return CurrentPos;
}

void Tape::PZX_GetBlock() {
    FIL* tape = &Tape::tape;

    // Always seek to correct block position (PZX blocks are self-contained,
    // unlike TZX which chains via tapeCurByte)
    CalcPZXBlockPos(tapeCurBlock);

    for (;;) {
        if (tapeCurBlock >= tapeNumBlocks) {
            // Emit a final edge + short pause so loaders waiting for
            // the next edge can finish processing the last bit.
            tapeEarBit ^= 1;
            tapePhase = TAPE_PHASE_END;
            tapeNext = 7000;
            return;
        }

        // Read block header
        tapePlayOffset = f_tell(tape); // update progress for OSD %
        uint32_t tag, size;
        PZX_BlockLen(tag, size);
        uint32_t blockDataStart = f_tell(tape);
        uint32_t blockEnd = blockDataStart + size;

        switch (tag) {
            case PZX_PZXT:
                // Skip to next block
                fseek(tape, blockEnd, SEEK_SET);
                break;

            case PZX_PULS: {
                // Initial pulse level is low.
                tapeEarBit = 0;
                pzxPulseBlockEnd = blockEnd;

                // Decode pulse entries, skipping zero-duration ones
                while ((uint32_t)f_tell(tape) < pzxPulseBlockEnd) {
                    pzxPulseRep = 1;
                    uint16_t w = readByteFile(tape) | (readByteFile(tape) << 8);
                    if (w > 0x8000) {
                        pzxPulseRep = w & 0x7FFF;
                        w = readByteFile(tape) | (readByteFile(tape) << 8);
                    }
                    if (w >= 0x8000) {
                        pzxPulseDur = ((uint32_t)(w & 0x7FFF) << 16) | (readByteFile(tape) | (readByteFile(tape) << 8));
                    } else {
                        pzxPulseDur = w;
                    }

                    if (pzxPulseDur == 0) {
                        // Zero-duration: toggle for odd count, then continue to next entry
                        if (pzxPulseRep & 1) tapeEarBit ^= 1;
                        continue;
                    }

                    // Found a non-zero pulse entry
                    tapePhase = TAPE_PHASE_PZX_PULS;
                    tapeNext = pzxPulseDur;
                    return;
                }

                // All entries were zero-duration or empty block
                break;
            }

            case PZX_DATA: {
                uint32_t count_field = readByteFile(tape) | (readByteFile(tape) << 8) |
                                       (readByteFile(tape) << 16) | (readByteFile(tape) << 24);
                pzxBitCount = count_field & 0x7FFFFFFF;
                uint8_t initialLevel = (count_field >> 31) & 1;

                pzxTailLen = readByteFile(tape) | (readByteFile(tape) << 8);
                pzxP0 = readByteFile(tape);
                pzxP1 = readByteFile(tape);

                // Read pulse sequences
                for (int i = 0; i < pzxP0; i++)
                    pzxS0[i] = readByteFile(tape) | (readByteFile(tape) << 8);
                for (int i = 0; i < pzxP1; i++)
                    pzxS1[i] = readByteFile(tape) | (readByteFile(tape) << 8);

                pzxDataBlockEnd = blockEnd;

                // Determine pause: PZX DATA blocks don't have explicit pause,
                // so set to 0 (next block handles timing)
                tapeBlkPauseLen = 0;

                // Set initial ear bit
                tapeEarBit = initialLevel;

                // Check if standard 2-pulse encoding - can use existing DATA1/DATA2 phases
                if (pzxP0 == 2 && pzxP1 == 2 && pzxBitCount > 0) {
                    tapeBit0PulseLen = pzxS0[0];
                    tapeBit1PulseLen = pzxS1[0];
                    tapeBit0PulseLen2 = pzxS0[1];
                    tapeBit1PulseLen2 = pzxS1[1];

                    // Calculate data bytes
                    tapebufByteCount = 0;
                    tapeBlockLen = (pzxBitCount + 7) / 8;
                    uint8_t lastBits = pzxBitCount & 7;
                    tapeLastByteUsedBits = lastBits ? lastBits : 8;
                    tapeBitMask = 0x80;
                    tapeEndBitMask = 0x80;
                    if ((1 == tapeBlockLen) && (tapeLastByteUsedBits < 8))
                        tapeEndBitMask >>= tapeLastByteUsedBits;

                    tapeCurByte = readByteFile(tape);

                    tapePhase = TAPE_PHASE_DATA1;
                    tapeNext = tapeCurByte & tapeBitMask ? tapeBit1PulseLen : tapeBit0PulseLen;
                } else if (pzxBitCount > 0) {
                    // Multi-pulse symbols: use PZX_DATA phase
                    tapeBitMask = 0x80;
                    tapeCurByte = readByteFile(tape);
                    pzxCurSymPulse = 0;

                    uint16_t* seq = (tapeCurByte & tapeBitMask) ? pzxS1 : pzxS0;
                    tapePhase = TAPE_PHASE_PZX_DATA;
                    tapeNext = seq[0];
                } else {
                    // Zero bits: just output tail
                    if (pzxTailLen > 0) {
                        tapePhase = TAPE_PHASE_TAIL;
                        tapeNext = pzxTailLen;
                    } else {
                        tapeCurBlock++;
                        CalcPZXBlockPos(tapeCurBlock);
                        continue;
                    }
                }

                return;
            }

            case PZX_PAUS: {
                uint32_t dur_field = readByteFile(tape) | (readByteFile(tape) << 8) |
                                     (readByteFile(tape) << 16) | (readByteFile(tape) << 24);
                uint32_t duration = dur_field & 0x7FFFFFFF;
                uint8_t level = (dur_field >> 31) & 1;

                tapeEarBit = level;

                if (duration == 0) {
                    fseek(tape, blockEnd, SEEK_SET);
                    break;
                }

                tapePhase = TAPE_PHASE_PAUSE;
                tapeBlkPauseLen = duration;
                tapeNext = duration;
                return;
            }

            case PZX_BRWS:
                fseek(tape, blockEnd, SEEK_SET);
                break;

            case PZX_STOP: {
                uint16_t flags = 0;
                if (size >= 2)
                    flags = readByteFile(tape) | (readByteFile(tape) << 8);

                if (flags == 1 && !Z80Ops::is48) {
                    // Stop only in 48K mode, but we're not in 48K
                    fseek(tape, blockEnd, SEEK_SET);
                    break;
                }

                Tape::Stop();

                if (tapeCurBlock < (tapeNumBlocks - 1)) {
                    tapeCurBlock++;
                } else {
                    tapeCurBlock = 0;
                    rewind(Tape::tape);
                }
                return;
            }

            default:
                // Unknown block, skip it
                fseek(tape, blockEnd, SEEK_SET);
                break;
        }

        // Non-returning blocks: advance to next
        tapeCurBlock++;
        CalcPZXBlockPos(tapeCurBlock);
    }
}

string Tape::pzxBlockReadData(int Blocknum) {
    char buf[48];

    CalcPZXBlockPos(Blocknum);

    FIL* tp = &tape;
    uint32_t tag, size;
    PZX_BlockLen(tag, size);

    string blktype;
    int tapeBlkLen = -1;

    // Convert tag to string for display
    char tagStr[5];
    tagStr[0] = tag & 0xFF;
    tagStr[1] = (tag >> 8) & 0xFF;
    tagStr[2] = (tag >> 16) & 0xFF;
    tagStr[3] = (tag >> 24) & 0xFF;
    tagStr[4] = 0;

    if (tag == PZX_PZXT) {
        blktype = "Header       ";
    } else if (tag == PZX_PULS) {
        blktype = "Pulse seq    ";
    } else if (tag == PZX_DATA) {
        blktype = "Data         ";
        // Read bit count to show data length
        uint32_t count_field = readByteFile(tp) | (readByteFile(tp) << 8) |
                               (readByteFile(tp) << 16) | (readByteFile(tp) << 24);
        tapeBlkLen = ((count_field & 0x7FFFFFFF) + 7) / 8;
    } else if (tag == PZX_PAUS) {
        blktype = "Pause        ";
        uint32_t dur = readByteFile(tp) | (readByteFile(tp) << 8) |
                       (readByteFile(tp) << 16) | (readByteFile(tp) << 24);
        tapeBlkLen = (dur & 0x7FFFFFFF) / 3500; // show in ms
    } else if (tag == PZX_BRWS) {
        blktype = "Browse       ";
    } else if (tag == PZX_STOP) {
        blktype = "Stop tape    ";
    } else {
        blktype = tagStr;
        blktype += "         ";
    }

    if (tapeBlkLen >= 0)
        snprintf(buf, sizeof(buf), "%04d %s % 6d\n", Blocknum + 1, blktype.c_str(), tapeBlkLen);
    else
        snprintf(buf, sizeof(buf), "%04d %s\n", Blocknum + 1, blktype.c_str());

    return buf;
}
