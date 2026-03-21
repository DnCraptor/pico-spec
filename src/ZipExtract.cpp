#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

#include "miniz/miniz.h"

using namespace std;

#include "ZipExtract.h"
#include "FileUtils.h"
#include "MemESP.h"
#include "Video.h"
#include "OSDMain.h"
#include "Config.h"
#include "ESPectrum.h"
#include "messages.h"

extern Font Font6x8;

#define ZIP_BUF_SIZE 512

const char* ZipExtract::TEMP_FILE = "/tmp/.zip_extract";

// Static FIL to avoid ~560 bytes on stack (FIL contains 512-byte sector buffer)
static FIL s_zipFile;
static FIL s_outFile;

static bool hasMatchingExt(const char* filename, uint8_t fileType) {
    const char* dot = strrchr(filename, '.');
    if (!dot || dot == filename) return false;
    // Skip .zip inside .zip
    if (strcasecmp(dot, ".zip") == 0) return false;

    const string& exts = FileUtils::fileTypes[fileType].fileExts;
    // Build lowercase .ext
    char dotExt[8];
    int len = strlen(dot);
    if (len > 7) return false;
    for (int i = 0; i < len; i++) dotExt[i] = tolower(dot[i]);
    dotExt[len] = 0;
    if (exts.find(dotExt) != string::npos) return true;
    // Check uppercase
    for (int i = 0; i < len; i++) dotExt[i] = toupper(dot[i]);
    if (exts.find(dotExt) != string::npos) return true;
    return false;
}

// Get lowercase extension from a C string filename
static const char* getExtFromName(const char* name) {
    const char* dot = strrchr(name, '.');
    return dot ? dot + 1 : "";
}

// Get basename (strip path) from a C string
static const char* getBaseName(const char* name) {
    const char* slash = strrchr(name, '/');
    return slash ? slash + 1 : name;
}

// Entry stored during ZIP scan — keep small for stack
struct ZipEntry {
    char name[48];         // basename only
    FSIZE_t dataOffset;
    uint32_t compressedSize;
    uint32_t uncompressedSize;
    uint16_t compression;
};

#define ZIP_MAX_ENTRIES 16

string ZipExtract::extract(const string& zipPath, uint8_t fileType) {
    FIL& zipFile = s_zipFile;
    if (f_open(&zipFile, zipPath.c_str(), FA_READ) != FR_OK)
        return "";

    FSIZE_t zipSize = f_size(&zipFile);

    // Phase 1: single-pass scan, collect matching entries
    static ZipEntry entries[ZIP_MAX_ENTRIES];
    int entryCount = 0;

    LocalFileHeader hdr;
    UINT br;

    while (entryCount < ZIP_MAX_ENTRIES) {
        FSIZE_t pos = f_tell(&zipFile);
        if (pos + sizeof(hdr) > zipSize) break;

        if (f_read(&zipFile, &hdr, sizeof(hdr), &br) != FR_OK || br != sizeof(hdr)) break;
        if (hdr.signature != ZIP_LOCAL_SIGNATURE) break;
        if (hdr.nameLen == 0 || hdr.nameLen > 250) break;

        // Read filename — use static buffer to save stack
        static char fnBuf[252];
        if (f_read(&zipFile, fnBuf, hdr.nameLen, &br) != FR_OK || br != hdr.nameLen) break;
        fnBuf[hdr.nameLen] = 0;

        // Skip extra field
        if (hdr.extraLen > 0)
            f_lseek(&zipFile, f_tell(&zipFile) + hdr.extraLen);

        FSIZE_t dataStart = f_tell(&zipFile);

        // Check: not a directory, has matching extension
        if (fnBuf[hdr.nameLen - 1] != '/' && hasMatchingExt(fnBuf, fileType)) {
            ZipEntry& e = entries[entryCount];
            const char* base = getBaseName(fnBuf);
            strncpy(e.name, base, sizeof(e.name) - 1);
            e.name[sizeof(e.name) - 1] = 0;
            e.dataOffset = dataStart;
            e.compressedSize = hdr.compressedSize;
            e.uncompressedSize = hdr.uncompressedSize;
            e.compression = hdr.compression;
            entryCount++;
        }

        // Skip compressed data to next entry
        uint32_t dataSize = hdr.compressedSize;
        if (dataSize == 0 && (hdr.flags & 0x08)) break; // data descriptor — stop
        FSIZE_t nextPos = dataStart + dataSize;
        if (nextPos > zipSize || nextPos <= pos) break; // sanity
        f_lseek(&zipFile, nextPos);
    }

    if (entryCount == 0) {
        f_close(&zipFile);
        return "";
    }


    // Phase 2: select file
    int selected = 0;
    if (entryCount > 1) {
        string menu = "Select file\n";
        for (int i = 0; i < entryCount; i++) {
            menu += entries[i].name;
            menu += "\n";
        }
        uint8_t maxRows = (entryCount < 15) ? entryCount + 1 : 16;
        uint8_t menuCols = 30;
        uint16_t w = menuCols * OSD_FONT_W + 2;
        uint16_t h = maxRows * OSD_FONT_H + 2;
        uint8_t opt = OSD::simpleMenuRun(menu,
            OSD::scrAlignCenterX(w), OSD::scrAlignCenterY(h),
            maxRows, menuCols);
        if (opt == 0) {
            f_close(&zipFile);
            return "\x1b"; // ESC = cancelled
        }
        selected = opt - 1;
    }

    // Phase 3: extract
    ZipEntry& e = entries[selected];
    f_lseek(&zipFile, e.dataOffset);

    OSD::osdCenteredMsg(OSD_ZIP_EXTRACTING[Config::lang], LEVEL_INFO, 0);

    cleanup();
    bool ok = extractFile(&zipFile, e.compression, e.compressedSize, e.uncompressedSize);
    f_close(&zipFile);

    if (!ok) return "";

    // Rename temp to include correct extension
    char extBuf[8];
    const char* rawExt = getExtFromName(e.name);
    int elen = strlen(rawExt);
    if (elen > 6) elen = 6;
    for (int i = 0; i < elen; i++) extBuf[i] = tolower(rawExt[i]);
    extBuf[elen] = 0;

    string finalPath = string(TEMP_FILE) + "." + extBuf;
    f_unlink(finalPath.c_str());
    f_rename(TEMP_FILE, finalPath.c_str());
    return finalPath;
}

bool ZipExtract::hasMatchingExtension(const string& filename, uint8_t fileType) {
    return hasMatchingExt(filename.c_str(), fileType);
}

int ZipExtract::listFiles(const string& zipPath, uint8_t fileType, vector<string>& names) {
    names.clear();
    return 0;
}

string ZipExtract::extractByIndex(const string& zipPath, int fileIndex) {
    return "";
}

bool ZipExtract::extractFile(FIL* zipFile, uint16_t compression, uint32_t compressedSize, uint32_t uncompressedSize) {
    if (compression == 0)
        return extractStored(zipFile, compressedSize);
    if (compression == 8)
        return extractDeflate(zipFile, compressedSize);
    return false;
}

bool ZipExtract::extractStored(FIL* zipFile, uint32_t size) {
    FIL& outFile = s_outFile;
    if (f_open(&outFile, TEMP_FILE, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return false;

    uint8_t buf[ZIP_BUF_SIZE];
    UINT br, bw;
    uint32_t remaining = size;

    while (remaining > 0) {
        uint32_t chunk = (remaining < ZIP_BUF_SIZE) ? remaining : ZIP_BUF_SIZE;
        if (f_read(zipFile, buf, chunk, &br) != FR_OK || br != chunk) {
            f_close(&outFile);
            return false;
        }
        if (f_write(&outFile, buf, chunk, &bw) != FR_OK || bw != chunk) {
            f_close(&outFile);
            return false;
        }
        remaining -= chunk;
    }

    f_close(&outFile);
    return true;
}

bool ZipExtract::extractDeflate(FIL* zipFile, uint32_t compressedSize) {
    FIL& outFile = s_outFile;
    if (f_open(&outFile, TEMP_FILE, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return false;

    uint8_t s_inbuf[ZIP_BUF_SIZE];
    uint8_t s_outbuf[ZIP_BUF_SIZE];
    z_stream stream;

    memset(&stream, 0, sizeof(stream));
    stream.next_in = s_inbuf;
    stream.avail_in = 0;
    stream.next_out = s_outbuf;
    stream.avail_out = ZIP_BUF_SIZE;

    uint32_t infile_remaining = compressedSize;

    // Borrow Spectrum RAM page as inflate dictionary (same as inflateCSW)
    uint32_t *speccyram = (uint32_t *)MemESP::ram[1].sync(6);
    VIDEO::SaveRect.store_ram(speccyram, 0x8000);
    MemESP::ram[1].cleanup();
    MemESP::ram[3].cleanup();

    if (inflateInit2(&stream, -MZ_DEFAULT_WINDOW_BITS, MemESP::ram[1].sync(6))) {
        VIDEO::SaveRect.restore_ram(speccyram, 0x8000);
        f_close(&outFile);
        return false;
    }

    bool success = true;
    UINT br, bw;

    for (;;) {
        if (!stream.avail_in) {
            uint32_t n = (infile_remaining < ZIP_BUF_SIZE) ? infile_remaining : ZIP_BUF_SIZE;
            if (n > 0) {
                if (f_read(zipFile, s_inbuf, n, &br) != FR_OK || br != n) {
                    success = false;
                    break;
                }
                stream.next_in = s_inbuf;
                stream.avail_in = n;
                infile_remaining -= n;
            }
        }

        int status = inflate(&stream, (infile_remaining == 0 && stream.avail_in == 0) ? Z_FINISH : Z_SYNC_FLUSH);

        if ((status == Z_STREAM_END) || (!stream.avail_out)) {
            uint32_t n = ZIP_BUF_SIZE - stream.avail_out;
            if (n > 0) {
                if (f_write(&outFile, s_outbuf, n, &bw) != FR_OK || bw != n) {
                    success = false;
                    break;
                }
            }
            stream.next_out = s_outbuf;
            stream.avail_out = ZIP_BUF_SIZE;
        }

        if (status == Z_STREAM_END) break;
        if (status != Z_OK && status != Z_BUF_ERROR) {
            success = false;
            break;
        }
    }

    inflateEnd(&stream);
    f_close(&outFile);
    VIDEO::SaveRect.restore_ram(speccyram, 0x8000);

    return success;
}

void ZipExtract::viewInfo(const string& zipPath) {
    FIL& zipFile = s_zipFile;
    if (f_open(&zipFile, zipPath.c_str(), FA_READ) != FR_OK)
        return;

    FSIZE_t zipSize = f_size(&zipFile);
    LocalFileHeader hdr;
    UINT br;
    static char fnBuf[252];

    // Build info string for simpleMenuRun
    // First line = title (archive name + size)
    const char* zipName = getBaseName(zipPath.c_str());
    char sizeBuf[16];
    if (zipSize >= 1024 * 1024)
        snprintf(sizeBuf, sizeof(sizeBuf), "%luMB", (unsigned long)(zipSize / (1024 * 1024)));
    else if (zipSize >= 1024)
        snprintf(sizeBuf, sizeof(sizeBuf), "%luKB", (unsigned long)(zipSize / 1024));
    else
        snprintf(sizeBuf, sizeof(sizeBuf), "%luB", (unsigned long)zipSize);

    string info = string(zipName) + " (" + sizeBuf + ")\n";

    int fileCount = 0;
    while (fileCount < 32) {
        FSIZE_t pos = f_tell(&zipFile);
        if (pos + sizeof(hdr) > zipSize) break;
        if (f_read(&zipFile, &hdr, sizeof(hdr), &br) != FR_OK || br != sizeof(hdr)) break;
        if (hdr.signature != ZIP_LOCAL_SIGNATURE) break;
        if (hdr.nameLen == 0 || hdr.nameLen > 250) break;

        if (f_read(&zipFile, fnBuf, hdr.nameLen, &br) != FR_OK || br != hdr.nameLen) break;
        fnBuf[hdr.nameLen] = 0;

        if (hdr.extraLen > 0)
            f_lseek(&zipFile, f_tell(&zipFile) + hdr.extraLen);

        FSIZE_t dataStart = f_tell(&zipFile);

        // Skip directories
        if (fnBuf[hdr.nameLen - 1] != '/') {
            const char* base = getBaseName(fnBuf);
            char line[48];
            if (hdr.uncompressedSize >= 1024 * 1024)
                snprintf(line, sizeof(line), "%.30s %luMB", base, (unsigned long)(hdr.uncompressedSize / (1024 * 1024)));
            else if (hdr.uncompressedSize >= 1024)
                snprintf(line, sizeof(line), "%.30s %luKB", base, (unsigned long)(hdr.uncompressedSize / 1024));
            else
                snprintf(line, sizeof(line), "%.30s %luB", base, (unsigned long)hdr.uncompressedSize);
            info += line;
            info += "\n";
            fileCount++;
        }

        uint32_t dataSize = hdr.compressedSize;
        if (dataSize == 0 && (hdr.flags & 0x08)) break;
        FSIZE_t nextPos = dataStart + dataSize;
        if (nextPos > zipSize || nextPos <= pos) break;
        f_lseek(&zipFile, nextPos);
    }

    f_close(&zipFile);

    if (fileCount == 0) return;

    // Draw info box manually and wait for any key
    uint8_t rows = fileCount + 1; // +1 for title
    if (rows > 16) rows = 16;
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
    // Print title (first line of info)
    size_t nlPos = info.find('\n');
    if (nlPos != string::npos)
        VIDEO::vga.print(info.substr(0, nlPos).c_str());

    // Content lines
    VIDEO::vga.fillRect(bx + 1, by + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7, 1));
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));

    size_t pos = (nlPos != string::npos) ? nlPos + 1 : 0;
    for (int r = 0; r < rows - 1 && pos < info.size(); r++) {
        size_t nl = info.find('\n', pos);
        string line = (nl != string::npos) ? info.substr(pos, nl - pos) : info.substr(pos);
        VIDEO::vga.setCursor(bx + OSD_FONT_W + 1, by + 1 + OSD_FONT_H * (r + 1));
        VIDEO::vga.print(line.c_str());
        pos = (nl != string::npos) ? nl + 1 : info.size();
    }

    // Wait for any key
    fabgl::VirtualKeyItem Menukey;
    while (1) {
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            if (ESPectrum::readKbd(&Menukey)) {
                if (Menukey.down) break;
            }
        }
        sleep_ms(5);
    }

    VIDEO::SaveRect.restore_last();
}

int ZipExtract::extractAll(const string& zipPath, const string& destDir) {
    FIL& zipFile = s_zipFile;
    if (f_open(&zipFile, zipPath.c_str(), FA_READ) != FR_OK)
        return 0;

    FSIZE_t zipSize = f_size(&zipFile);
    LocalFileHeader hdr;
    UINT br;
    static char fnBuf[252];
    int extracted = 0;

    while (true) {
        FSIZE_t pos = f_tell(&zipFile);
        if (pos + sizeof(hdr) > zipSize) break;
        if (f_read(&zipFile, &hdr, sizeof(hdr), &br) != FR_OK || br != sizeof(hdr)) break;
        if (hdr.signature != ZIP_LOCAL_SIGNATURE) break;
        if (hdr.nameLen == 0 || hdr.nameLen > 250) break;

        if (f_read(&zipFile, fnBuf, hdr.nameLen, &br) != FR_OK || br != hdr.nameLen) break;
        fnBuf[hdr.nameLen] = 0;

        if (hdr.extraLen > 0)
            f_lseek(&zipFile, f_tell(&zipFile) + hdr.extraLen);

        FSIZE_t dataStart = f_tell(&zipFile);

        // Skip directories
        if (fnBuf[hdr.nameLen - 1] != '/' && (hdr.compression == 0 || hdr.compression == 8)) {
            // Build destination path: destDir + basename
            const char* base = getBaseName(fnBuf);

            // Extract to TEMP_FILE first, then rename to dest
            // Use extractFile which writes to TEMP_FILE
            bool ok = extractFile(&zipFile, hdr.compression, hdr.compressedSize, hdr.uncompressedSize);
            if (ok) {
                char destPath[128];
                snprintf(destPath, sizeof(destPath), "%s%s", destDir.c_str(), base);
                f_unlink(destPath); // remove if exists
                f_rename(TEMP_FILE, destPath);
                extracted++;
            }
            // Re-seek past data (extractFile consumed it, but be safe)
        }

        // Skip to next entry
        FSIZE_t nextPos = dataStart + hdr.compressedSize;
        if (hdr.compressedSize == 0 && (hdr.flags & 0x08)) break;
        if (nextPos > zipSize || nextPos <= pos) break;
        f_lseek(&zipFile, nextPos);
    }

    f_close(&zipFile);
    f_unlink(TEMP_FILE); // clean up temp
    return extracted;
}

void ZipExtract::cleanup() {
    f_unlink(TEMP_FILE);
    const char* exts[] = {
        ".sna", ".z80", ".p", ".tap", ".tzx", ".pzx", ".wav", ".mp3",
        ".trd", ".scl", ".udi", ".fdi", ".mmc", ".hdf", ".rom", ".bin", NULL
    };
    for (int i = 0; exts[i]; i++) {
        char path[32];
        snprintf(path, sizeof(path), "%s%s", TEMP_FILE, exts[i]);
        f_unlink(path);
    }
}
