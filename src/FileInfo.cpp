#include <stdio.h>
#include <string.h>
#include <string>

using namespace std;

#include "FileInfo.h"

#if PICO_RP2040
// Stub — saves ~2KB RAM (static FIL, lineStarts/lineLens arrays, format tables)
void FileInfo::viewInfo(const string& path) {}
#else

#include "FileUtils.h"
#include "Video.h"
#include "OSDMain.h"
#include "ESPectrum.h"

extern Font Font6x8;

// Static FIL to avoid stack bloat
static FIL s_infoFile;

static const char* getBaseName(const char* name) {
    const char* slash = strrchr(name, '/');
    return slash ? slash + 1 : name;
}

static void formatSize(char* buf, size_t bufSz, FSIZE_t size) {
    if (size >= 1024 * 1024)
        snprintf(buf, bufSz, "%luMB", (unsigned long)(size / (1024 * 1024)));
    else if (size >= 1024)
        snprintf(buf, bufSz, "%luKB", (unsigned long)(size / 1024));
    else
        snprintf(buf, bufSz, "%luB", (unsigned long)size);
}

// Parse info string into lines vector (skip title which is line 0)
static int parseLines(const string& info, const char** lineStarts, int* lineLens, int maxLines) {
    int count = 0;
    size_t pos = 0;
    while (pos < info.size() && count < maxLines) {
        size_t nl = info.find('\n', pos);
        lineStarts[count] = info.c_str() + pos;
        lineLens[count] = (nl != string::npos) ? (int)(nl - pos) : (int)(info.size() - pos);
        count++;
        pos = (nl != string::npos) ? nl + 1 : info.size();
    }
    return count;
}

// Draw content area for current scroll position
static void drawContent(const char** lineStarts, int* lineLens, int totalLines,
                        int scrollPos, int visRows, uint16_t bx, uint16_t by, uint16_t w,
                        uint8_t menuCols) {
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));

    for (int r = 0; r < visRows; r++) {
        int idx = scrollPos + r + 1; // +1 to skip title
        VIDEO::vga.setCursor(bx + OSD_FONT_W + 1, by + 1 + OSD_FONT_H * (r + 1));
        // Build fixed-width line padded with spaces (overwrites previous content)
        char buf[44];
        int maxChars = menuCols - 2; // leave margin for scrollbar
        if (idx < totalLines) {
            int len = lineLens[idx] > maxChars ? maxChars : lineLens[idx];
            memcpy(buf, lineStarts[idx], len);
            memset(buf + len, ' ', maxChars - len);
            buf[maxChars] = 0;
        } else {
            memset(buf, ' ', maxChars);
            buf[maxChars] = 0;
        }
        VIDEO::vga.print(buf);
    }

    // Scroll indicator on right edge if content is scrollable
    uint16_t sx = bx + w - 3;
    uint16_t sy = by + 1 + OSD_FONT_H;
    int barH = visRows * OSD_FONT_H;
    if (totalLines - 1 > visRows) {
        int contentLines = totalLines - 1;
        int thumbH = barH * visRows / contentLines;
        if (thumbH < 3) thumbH = 3;
        int thumbY = (barH - thumbH) * scrollPos / (contentLines - visRows);
        VIDEO::vga.fillRect(sx, sy, 2, barH, zxColor(7, 0));
        VIDEO::vga.fillRect(sx, sy + thumbY, 2, thumbH, zxColor(0, 0));
    } else {
        VIDEO::vga.fillRect(sx, sy, 2, barH, zxColor(7, 1));
    }
}

// Draw info box with scrolling and wait for ESC
static void showInfoBox(const string& info, int lineCount) {
    const int MAX_LINES = 128;
    static const char* lineStarts[MAX_LINES];
    static int lineLens[MAX_LINES];
    int totalLines = parseLines(info, lineStarts, lineLens, MAX_LINES);
    if (totalLines < 2) return;

    int contentLines = totalLines - 1; // excluding title
    int visRows = contentLines > 19 ? 19 : contentLines;
    int rows = visRows + 1; // +1 for title

    uint8_t menuCols = 42;
    uint16_t w = menuCols * OSD_FONT_W + 2;
    uint16_t h = rows * OSD_FONT_H + 2;
    uint16_t bx = OSD::scrAlignCenterX(w);
    uint16_t by = OSD::scrAlignCenterY(h);

    VIDEO::SaveRect.save(bx, by, w, h);

    VIDEO::vga.setFont(Font6x8);
    VIDEO::vga.rect(bx, by, w, h, zxColor(0, 0));

    // Title bar
    VIDEO::vga.fillRect(bx + 1, by + 1, w - 2, OSD_FONT_H, zxColor(0, 0));
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(bx + OSD_FONT_W + 1, by + 1);
    char titleBuf[42];
    int titleLen = lineLens[0] > 40 ? 40 : lineLens[0];
    memcpy(titleBuf, lineStarts[0], titleLen);
    titleBuf[titleLen] = 0;
    VIDEO::vga.print(titleBuf);

    // Initial content background (once)
    VIDEO::vga.fillRect(bx + 1, by + 1 + OSD_FONT_H, w - 2, visRows * OSD_FONT_H, zxColor(7, 1));

    int scrollPos = 0;
    int maxScroll = contentLines > visRows ? contentLines - visRows : 0;

    drawContent(lineStarts, lineLens, totalLines, scrollPos, visRows, bx, by, w, menuCols);

    // Scroll loop
    fabgl::VirtualKeyItem Menukey;
    while (1) {
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            if (ESPectrum::readKbd(&Menukey)) {
                if (!Menukey.down) continue;
                bool redraw = false;
                int oldScroll = scrollPos;

                if (is_up(Menukey.vk) || Menukey.vk == fabgl::VK_UP) {
                    if (scrollPos > 0) { scrollPos--; redraw = true; }
                } else if (is_down(Menukey.vk) || Menukey.vk == fabgl::VK_DOWN) {
                    if (scrollPos < maxScroll) { scrollPos++; redraw = true; }
                } else if (Menukey.vk == fabgl::VK_PAGEUP) {
                    scrollPos -= visRows;
                    if (scrollPos < 0) scrollPos = 0;
                    redraw = (scrollPos != oldScroll);
                } else if (Menukey.vk == fabgl::VK_PAGEDOWN) {
                    scrollPos += visRows;
                    if (scrollPos > maxScroll) scrollPos = maxScroll;
                    redraw = (scrollPos != oldScroll);
                } else if (is_home(Menukey.vk) || Menukey.vk == fabgl::VK_HOME) {
                    if (scrollPos != 0) { scrollPos = 0; redraw = true; }
                } else if (Menukey.vk == fabgl::VK_END) {
                    if (scrollPos != maxScroll) { scrollPos = maxScroll; redraw = true; }
                } else if (Menukey.vk == fabgl::VK_DPAD_UP || Menukey.vk == fabgl::VK_DPAD_DOWN
                        || Menukey.vk == fabgl::VK_DPAD_LEFT || Menukey.vk == fabgl::VK_DPAD_RIGHT) {
                    // Ignore joystick dpad events
                } else {
                    goto done;
                }
                if (redraw)
                    drawContent(lineStarts, lineLens, totalLines, scrollPos, visRows, bx, by, w, menuCols);
            }
        }
        sleep_ms(5);
    }
done:
    VIDEO::SaveRect.restore_last();
}

// ---- TAP ----
static void viewTAP(FIL* f, FSIZE_t fileSize, string& info, int& lines) {
    f_lseek(f, 0);
    int blockNum = 0;
    while (f_tell(f) + 2 <= fileSize && lines < 256) {
        uint8_t lo, hi;
        UINT br;
        f_read(f, &lo, 1, &br); if (br != 1) break;
        f_read(f, &hi, 1, &br); if (br != 1) break;
        uint16_t blkLen = lo | (hi << 8);
        if (blkLen == 0) break;

        FSIZE_t blkStart = f_tell(f);
        uint8_t flagByte = 255;
        if (blkStart + blkLen <= fileSize) {
            f_read(f, &flagByte, 1, &br);
        }

        char line[48];
        if (flagByte == 0 && blkLen == 19) {
            // Header block
            uint8_t blocktype;
            f_read(f, &blocktype, 1, &br);
            char fname[11];
            f_read(f, fname, 10, &br);
            fname[10] = 0;
            // Trim trailing spaces
            for (int i = 9; i >= 0 && fname[i] == ' '; i--) fname[i] = 0;

            const char* typeName;
            switch (blocktype) {
                case 0: typeName = "Program  "; break;
                case 1: typeName = "Num array"; break;
                case 2: typeName = "Chr array"; break;
                case 3: typeName = "Code     "; break;
                default: typeName = "Data     "; break;
            }
            snprintf(line, sizeof(line), "%03d %-9s %-10s%6d", blockNum + 1, typeName, fname, blkLen);
        } else {
            snprintf(line, sizeof(line), "%03d Data              %10d", blockNum + 1, blkLen);
        }
        info += line;
        info += "\n";
        lines++;

        f_lseek(f, blkStart + blkLen);
        blockNum++;
    }

}

// ---- TZX ----
static void viewTZX(FIL* f, FSIZE_t fileSize, string& info, int& lines) {
    // Skip 10-byte TZX header
    f_lseek(f, 10);
    int blockNum = 0;

    while (f_tell(f) < fileSize && lines < 256) {
        uint8_t blockType;
        UINT br;
        f_read(f, &blockType, 1, &br);
        if (br != 1) break;

        FSIZE_t blockStart = f_tell(f);
        uint32_t dataLen = 0;
        const char* typeName = "Unknown";
        int displayLen = -1;

        switch (blockType) {
            case 0x10: { // Standard speed data
                typeName = "Standard ";
                uint8_t hdr[4]; f_read(f, hdr, 4, &br);
                dataLen = 4 + (hdr[2] | (hdr[3] << 8));
                displayLen = (hdr[2] | (hdr[3] << 8));
                break;
            }
            case 0x11: { // Turbo speed data
                typeName = "Turbo    ";
                uint8_t hdr[0x12]; f_read(f, hdr, 0x12, &br);
                dataLen = 0x12 + (hdr[0x0F] | (hdr[0x10] << 8) | (hdr[0x11] << 16));
                displayLen = hdr[0x0F] | (hdr[0x10] << 8) | (hdr[0x11] << 16);
                break;
            }
            case 0x12: typeName = "PureTone "; dataLen = 4; break;
            case 0x13: { typeName = "Pulses   "; uint8_t n; f_read(f, &n, 1, &br); dataLen = 1 + n * 2; break; }
            case 0x14: { // Pure data
                typeName = "PureData ";
                uint8_t hdr[0x0A]; f_read(f, hdr, 0x0A, &br);
                dataLen = 0x0A + (hdr[7] | (hdr[8] << 8) | (hdr[9] << 16));
                displayLen = hdr[7] | (hdr[8] << 8) | (hdr[9] << 16);
                break;
            }
            case 0x15: { typeName = "DirectRec"; uint8_t hdr[8]; f_read(f, hdr, 8, &br); dataLen = 8 + (hdr[5] | (hdr[6] << 8) | (hdr[7] << 16)); break; }
            case 0x18: case 0x19: { uint8_t hdr[4]; f_read(f, hdr, 4, &br); dataLen = 4 + (hdr[0] | (hdr[1] << 8) | (hdr[2] << 16) | (hdr[3] << 24)); typeName = blockType == 0x18 ? "CSW      " : "GDB      "; break; }
            case 0x20: { typeName = "Pause    "; uint8_t hdr[2]; f_read(f, hdr, 2, &br); dataLen = 2; displayLen = hdr[0] | (hdr[1] << 8); break; }
            case 0x21: { typeName = "GrpStart "; uint8_t n; f_read(f, &n, 1, &br); dataLen = 1 + n; break; }
            case 0x22: typeName = "GrpEnd   "; dataLen = 0; break;
            case 0x23: typeName = "Jump     "; dataLen = 2; break;
            case 0x24: typeName = "LoopStart"; dataLen = 2; break;
            case 0x25: typeName = "LoopEnd  "; dataLen = 0; break;
            case 0x26: { uint8_t hdr[2]; f_read(f, hdr, 2, &br); dataLen = 2 + (hdr[0] | (hdr[1] << 8)) * 2; typeName = "Call     "; break; }
            case 0x27: typeName = "Return   "; dataLen = 0; break;
            case 0x28: { uint8_t hdr[2]; f_read(f, hdr, 2, &br); dataLen = 2 + (hdr[0] | (hdr[1] << 8)); typeName = "Select   "; break; }
            case 0x2A: typeName = "Stop48K  "; dataLen = 4; break;
            case 0x2B: typeName = "SignalLvl"; dataLen = 5; break;
            case 0x30: { typeName = "Text     "; uint8_t n; f_read(f, &n, 1, &br); dataLen = 1 + n; break; }
            case 0x31: { typeName = "Message  "; uint8_t hdr[2]; f_read(f, hdr, 2, &br); dataLen = 2 + hdr[1]; break; }
            case 0x32: { typeName = "ArchInfo "; uint8_t hdr[2]; f_read(f, hdr, 2, &br); dataLen = 2 + (hdr[0] | (hdr[1] << 8)); break; }
            case 0x33: { typeName = "HW Type  "; uint8_t n; f_read(f, &n, 1, &br); dataLen = 1 + n * 3; break; }
            case 0x35: { typeName = "Custom   "; uint8_t hdr[20]; f_read(f, hdr, 20, &br); dataLen = 20 + (hdr[16] | (hdr[17] << 8) | (hdr[18] << 16) | (hdr[19] << 24)); break; }
            case 0x5A: typeName = "Glue     "; dataLen = 9; break;
            default: {
                // Unknown block — try to read 4-byte length
                uint8_t hdr[4]; f_read(f, hdr, 4, &br);
                dataLen = 4 + (hdr[0] | (hdr[1] << 8) | (hdr[2] << 16) | (hdr[3] << 24));
                break;
            }
        }

        char line[48];
        if (displayLen >= 0)
            snprintf(line, sizeof(line), "%03d %-9s %16d", blockNum + 1, typeName, displayLen);
        else
            snprintf(line, sizeof(line), "%03d %-9s", blockNum + 1, typeName);
        info += line;
        info += "\n";
        lines++;

        f_lseek(f, blockStart + dataLen);
        blockNum++;
    }

}

// ---- PZX ----
static void viewPZX(FIL* f, FSIZE_t fileSize, string& info, int& lines) {
    f_lseek(f, 0);
    int blockNum = 0;

    while (f_tell(f) + 8 <= fileSize && lines < 256) {
        uint8_t hdr[8];
        UINT br;
        f_read(f, hdr, 8, &br);
        if (br != 8) break;

        uint32_t tag = hdr[0] | (hdr[1] << 8) | (hdr[2] << 16) | (hdr[3] << 24);
        uint32_t size = hdr[4] | (hdr[5] << 8) | (hdr[6] << 16) | (hdr[7] << 24);

        const char* typeName;
        int displayLen = -1;
        switch (tag) {
            case 0x54585A50: typeName = "Header   "; break; // PZXT
            case 0x534C5550: typeName = "Pulse    "; break; // PULS
            case 0x41544144: typeName = "Data     "; displayLen = size > 0 ? size : -1; break; // DATA
            case 0x53554150: typeName = "Pause    "; break; // PAUS
            case 0x53575242: typeName = "Browse   "; break; // BRWS
            case 0x504F5453: typeName = "Stop     "; break; // STOP
            default: typeName = "Unknown  "; break;
        }

        char line[48];
        if (displayLen >= 0)
            snprintf(line, sizeof(line), "%03d %-9s %16d", blockNum + 1, typeName, displayLen);
        else
            snprintf(line, sizeof(line), "%03d %-9s", blockNum + 1, typeName);
        info += line;
        info += "\n";
        lines++;

        f_lseek(f, f_tell(f) - 8 + 8 + size);
        blockNum++;
    }

}

// ---- TRD ----
static void viewTRD(FIL* f, FSIZE_t fileSize, string& info, int& lines) {
    if (fileSize < 0x0900) return; // Need at least track 0 + sector 9

    // Read disk info from sector 9 (offset 0x08E1)
    uint8_t diskInfo[16];
    UINT br;
    f_lseek(f, 0x08E1);
    f_read(f, diskInfo, 16, &br);

    uint8_t diskType = diskInfo[2]; // offset 0x08E3
    uint8_t numFiles = diskInfo[3]; // offset 0x08E4
    uint16_t freeSectors = diskInfo[4] | (diskInfo[5] << 8); // offset 0x08E5

    const char* diskTypeStr;
    switch (diskType) {
        case 0x16: diskTypeStr = "80T 2S"; break;
        case 0x17: diskTypeStr = "40T 2S"; break;
        case 0x18: diskTypeStr = "80T 1S"; break;
        case 0x19: diskTypeStr = "40T 1S"; break;
        default:   diskTypeStr = ""; break;
    }

    char titleExtra[32];
    snprintf(titleExtra, sizeof(titleExtra), " %s %dF", diskTypeStr, numFiles);
    // Insert before the newline in title
    size_t nlPos = info.find('\n');
    info.insert(nlPos, titleExtra);

    // Read catalog entries from sectors 0-7 (128 entries × 16 bytes)
    f_lseek(f, 0);
    for (int i = 0; i < 128 && lines < 256; i++) {
        uint8_t entry[16];
        f_read(f, entry, 16, &br);
        if (br != 16) break;

        // Skip deleted/empty entries
        if (entry[0] == 0x00) break; // End of catalog
        if (entry[0] == 0x01) continue; // Deleted

        // entry: 8-byte name, 1-byte type, 2-byte param, 2-byte length_bytes, 1-byte sector_size, 1-byte start_sector, 1-byte start_track
        char name[9];
        memcpy(name, entry, 8);
        name[8] = 0;
        char ext = entry[8];
        uint16_t lenBytes = entry[11] | (entry[12] << 8);
        uint8_t lenSectors = entry[13];

        // Use sector-based size if byte length is 0
        uint32_t size = lenBytes ? lenBytes : (uint32_t)lenSectors * 256;

        char line[48];
        snprintf(line, sizeof(line), "%-8s.%c %18lu", name, ext, (unsigned long)size);
        info += line;
        info += "\n";
        lines++;
    }

    // Free space
    char freeLine[32];
    snprintf(freeLine, sizeof(freeLine), "Free: %luKB", (unsigned long)freeSectors / 4);
    info += freeLine;
    info += "\n";
    lines++;
}

// ---- SCL ----
static void viewSCL(FIL* f, FSIZE_t fileSize, string& info, int& lines) {
    if (fileSize < 9) return;

    uint8_t sclHdr[9];
    UINT br;
    f_lseek(f, 0);
    f_read(f, sclHdr, 9, &br);

    if (memcmp(sclHdr, "SINCLAIR", 8) != 0) return;
    uint8_t numFiles = sclHdr[8];

    char titleExtra[16];
    snprintf(titleExtra, sizeof(titleExtra), " %dF", numFiles);
    size_t nlPos = info.find('\n');
    info.insert(nlPos, titleExtra);

    // Read 14-byte catalog entries
    for (int i = 0; i < numFiles && lines < 256; i++) {
        uint8_t entry[14];
        f_read(f, entry, 14, &br);
        if (br != 14) break;

        char name[9];
        memcpy(name, entry, 8);
        name[8] = 0;
        char ext = entry[8];
        uint16_t start = entry[9] | (entry[10] << 8);
        uint8_t lenParam = entry[11];
        uint16_t lenSectors = entry[12] | (entry[13] << 8);

        uint32_t size = (uint32_t)lenSectors * 256;

        char line[48];
        snprintf(line, sizeof(line), "%-8s.%c %18lu", name, ext, (unsigned long)size);
        info += line;
        info += "\n";
        lines++;
    }
}

// ---- SNA ----
static void viewSNA(FIL* f, FSIZE_t fileSize, string& info, int& lines) {
    if (fileSize < 27) return;

    uint8_t hdr[27];
    UINT br;
    f_lseek(f, 0);
    f_read(f, hdr, 27, &br);

    const char* arch;
    if (fileSize == 49179)
        arch = "48K";
    else if (fileSize == 131103 || fileSize == 147487)
        arch = "128K";
    else if (fileSize == 131103 + (8 + 16) * 16384 || fileSize == 147487 + (8 + 16) * 16384)
        arch = "P512";
    else if (fileSize == 131103 + (8 + 16 + 32) * 16384 || fileSize == 147487 + (8 + 16 + 32) * 16384)
        arch = "P1024";
    else
        arch = "?";

    // Insert arch into title
    char titleExtra[8];
    snprintf(titleExtra, sizeof(titleExtra), " %s", arch);
    size_t nlPos = info.find('\n');
    info.insert(nlPos, titleExtra);

    // Parse registers: I, HL', DE', BC', AF', HL, DE, BC, IY, IX, IFF2, R, AF, SP, IM, Border
    uint8_t regI = hdr[0];
    uint16_t HLx = hdr[1] | (hdr[2] << 8);
    uint16_t DEx = hdr[3] | (hdr[4] << 8);
    uint16_t BCx = hdr[5] | (hdr[6] << 8);
    uint16_t AFx = hdr[7] | (hdr[8] << 8);
    uint16_t HL = hdr[9] | (hdr[10] << 8);
    uint16_t DE = hdr[11] | (hdr[12] << 8);
    uint16_t BC = hdr[13] | (hdr[14] << 8);
    uint16_t IY = hdr[15] | (hdr[16] << 8);
    uint16_t IX = hdr[17] | (hdr[18] << 8);
    uint8_t IFF2 = (hdr[19] & 0x04) ? 1 : 0;
    uint8_t R = hdr[20];
    uint16_t AF = hdr[21] | (hdr[22] << 8);
    uint16_t SP = hdr[23] | (hdr[24] << 8);
    uint8_t IM = hdr[25];
    uint8_t border = hdr[26];

    // For 128K, PC is at offset 49179
    uint16_t PC = 0;
    if (fileSize == 49179) {
        // 48K: PC is on stack
        // We'd need to read RAM to get it, skip for now
        PC = 0;
    } else if (fileSize > 49179 + 2) {
        f_lseek(f, 49179);
        uint8_t pcb[2];
        f_read(f, pcb, 2, &br);
        PC = pcb[0] | (pcb[1] << 8);
    }

    char line[48];
    if (PC != 0) {
        snprintf(line, sizeof(line), "PC:%04X SP:%04X IM:%d Brd:%d", PC, SP, IM, border);
    } else {
        snprintf(line, sizeof(line), "SP:%04X IM:%d Border:%d", SP, IM, border);
    }
    info += line; info += "\n"; lines++;

    snprintf(line, sizeof(line), "AF:%04X BC:%04X DE:%04X HL:%04X", AF, BC, DE, HL);
    info += line; info += "\n"; lines++;

    snprintf(line, sizeof(line), "IX:%04X IY:%04X I:%02X R:%02X", IX, IY, regI, R);
    info += line; info += "\n"; lines++;

    snprintf(line, sizeof(line), "AF'%04X BC'%04X DE'%04X HL'%04X", AFx, BCx, DEx, HLx);
    info += line; info += "\n"; lines++;
}

// ---- Z80 ----
static void viewZ80(FIL* f, FSIZE_t fileSize, string& info, int& lines) {
    if (fileSize < 30) return;

    uint8_t hdr[32];
    UINT br;
    f_lseek(f, 0);
    f_read(f, hdr, 30, &br);

    uint16_t PC = hdr[6] | (hdr[7] << 8);
    uint8_t z80ver = 1;
    const char* arch = "48K";

    if (PC == 0) {
        // v2 or v3
        f_read(f, hdr + 30, 2, &br);
        uint16_t ahbLen = hdr[30] | (hdr[31] << 8);
        if (ahbLen == 23) z80ver = 2;
        else if (ahbLen == 54 || ahbLen == 55) z80ver = 3;

        uint8_t mch;
        f_lseek(f, 34);
        f_read(f, &mch, 1, &br);

        // Read actual PC
        f_lseek(f, 32);
        uint8_t pcb[2];
        f_read(f, pcb, 2, &br);
        PC = pcb[0] | (pcb[1] << 8);

        if (z80ver == 2) {
            if (mch == 0 || mch == 1) arch = "48K";
            else if (mch == 3 || mch == 4) arch = "128K";
        } else if (z80ver == 3) {
            if (mch == 0 || mch == 1 || mch == 3) arch = "48K";
            else if (mch >= 4 && mch <= 7) arch = "128K";
            else if (mch == 9) arch = "Pentagon";
            else if (mch == 12 || mch == 13) arch = "128K";
        }
    }

    char titleExtra[16];
    snprintf(titleExtra, sizeof(titleExtra), " %s v%d", arch, z80ver);
    size_t nlPos = info.find('\n');
    info.insert(nlPos, titleExtra);

    uint16_t AF = (hdr[0] << 8) | hdr[1]; // A, F (big-endian in Z80)
    uint16_t BC = hdr[2] | (hdr[3] << 8);
    uint16_t HL = hdr[4] | (hdr[5] << 8);
    uint16_t SP = hdr[8] | (hdr[9] << 8);
    uint8_t regI = hdr[10];
    uint8_t R = hdr[11];
    uint8_t border = (hdr[12] >> 1) & 0x07;
    uint16_t DE = hdr[13] | (hdr[14] << 8);
    uint16_t BCx = hdr[15] | (hdr[16] << 8);
    uint16_t DEx = hdr[17] | (hdr[18] << 8);
    uint16_t HLx = hdr[19] | (hdr[20] << 8);
    uint16_t AFx = (hdr[21] << 8) | hdr[22]; // A', F' (big-endian)
    uint16_t IY = hdr[23] | (hdr[24] << 8);
    uint16_t IX = hdr[25] | (hdr[26] << 8);
    uint8_t IM = hdr[29] & 0x03;

    char line[48];
    snprintf(line, sizeof(line), "PC:%04X SP:%04X IM:%d Brd:%d", PC, SP, IM, border);
    info += line; info += "\n"; lines++;

    snprintf(line, sizeof(line), "AF:%04X BC:%04X DE:%04X HL:%04X", AF, BC, DE, HL);
    info += line; info += "\n"; lines++;

    snprintf(line, sizeof(line), "IX:%04X IY:%04X I:%02X R:%02X", IX, IY, regI, R);
    info += line; info += "\n"; lines++;

    snprintf(line, sizeof(line), "AF'%04X BC'%04X DE'%04X HL'%04X", AFx, BCx, DEx, HLx);
    info += line; info += "\n"; lines++;
}

// ---- FDI ----
static void viewFDI(FIL* f, FSIZE_t fileSize, string& info, int& lines) {
    if (fileSize < 14) return;

    uint8_t hdr[14];
    UINT br;
    f_lseek(f, 0);
    f_read(f, hdr, 14, &br);

    uint16_t cyls = hdr[4] | (hdr[5] << 8);
    uint16_t sides = hdr[6] | (hdr[7] << 8);

    char line[48];
    snprintf(line, sizeof(line), "Cyls: %d  Sides: %d", cyls, sides);
    info += line; info += "\n"; lines++;

    // FDI has description at offset 14, length in header bytes 8-9
    uint16_t descOff = hdr[8] | (hdr[9] << 8);
    if (descOff > 0 && descOff < fileSize) {
        f_lseek(f, descOff);
        char desc[40];
        UINT rd;
        f_read(f, desc, sizeof(desc) - 1, &rd);
        desc[rd] = 0;
        // Truncate at first null or newline
        for (unsigned i = 0; i < rd; i++) {
            if (desc[i] == 0 || desc[i] == '\n' || desc[i] == '\r') { desc[i] = 0; break; }
        }
        if (desc[0]) {
            info += desc;
            info += "\n";
            lines++;
        }
    }
}

// ---- UDI ----
static void viewUDI(FIL* f, FSIZE_t fileSize, string& info, int& lines) {
    if (fileSize < 16) return;

    uint8_t hdr[16];
    UINT br;
    f_lseek(f, 0);
    f_read(f, hdr, 16, &br);

    // UDI: signature "UDI!", version, cyls, sides, ...
    uint8_t cyls = hdr[9] + 1;  // 0-based
    uint8_t sides = hdr[10] + 1; // 0-based

    char line[48];
    snprintf(line, sizeof(line), "Cyls: %d  Sides: %d", cyls, sides);
    info += line; info += "\n"; lines++;
}

// ---- HDF ----
static void viewHDF(FIL* f, FSIZE_t fileSize, string& info, int& lines) {
    if (fileSize < 0x76) return;

    uint8_t sig[7];
    UINT br;
    f_lseek(f, 0);
    f_read(f, sig, 7, &br);

    if (memcmp(sig, "RS-IDE\x1a", 7) != 0) {
        info += "Not a valid HDF file\n"; lines++;
        return;
    }

    // IDE IDENTIFY data at offset 0x16
    uint8_t ident[106];
    f_lseek(f, 0x16);
    f_read(f, ident, 106, &br);

    uint16_t cyls = ident[2] | (ident[3] << 8);
    uint16_t heads = ident[6] | (ident[7] << 8);
    uint16_t sectors = ident[12] | (ident[13] << 8);

    char line[48];
    snprintf(line, sizeof(line), "CHS: %d/%d/%d", cyls, heads, sectors);
    info += line; info += "\n"; lines++;

    uint32_t totalMB = (uint32_t)cyls * heads * sectors / 2048;
    snprintf(line, sizeof(line), "Capacity: %luMB", (unsigned long)totalMB);
    info += line; info += "\n"; lines++;
}

// ---- Main dispatcher ----
void FileInfo::viewInfo(const string& path) {
    FIL& f = s_infoFile;
    if (f_open(&f, path.c_str(), FA_READ) != FR_OK)
        return;

    FSIZE_t fileSize = f_size(&f);
    const char* baseName = getBaseName(path.c_str());
    string ext = FileUtils::getLCaseExt(path);

    char sizeBuf[16];
    formatSize(sizeBuf, sizeof(sizeBuf), fileSize);

    string info = string(baseName) + " (" + sizeBuf + ")\n";
    int lines = 1; // title line

    if (ext == "tap") viewTAP(&f, fileSize, info, lines);
    else if (ext == "tzx") viewTZX(&f, fileSize, info, lines);
    else if (ext == "pzx") viewPZX(&f, fileSize, info, lines);
    else if (ext == "trd") viewTRD(&f, fileSize, info, lines);
    else if (ext == "scl") viewSCL(&f, fileSize, info, lines);
    else if (ext == "sna") viewSNA(&f, fileSize, info, lines);
    else if (ext == "z80") viewZ80(&f, fileSize, info, lines);
    else if (ext == "fdi") viewFDI(&f, fileSize, info, lines);
    else if (ext == "udi") viewUDI(&f, fileSize, info, lines);
    else if (ext == "hdf") viewHDF(&f, fileSize, info, lines);
    else if (ext == "mmc") { /* just show filename + size (title) */ }

    f_close(&f);

    if (lines < 2 && ext != "mmc") return; // Nothing to show beyond title

    showInfoBox(info, lines);
}
#endif // !PICO_RP2040
