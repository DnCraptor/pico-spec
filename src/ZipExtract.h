#ifndef ZipExtract_h
#define ZipExtract_h

#include <string>
#include <vector>
#include <inttypes.h>
#include "ff.h"

using namespace std;

#if PICO_RP2040
// RP2040: ZIP disabled to save ~2.5 KB SRAM
class ZipExtract {
public:
    static string extract(const string&, uint8_t) { return ""; }
    static int listFiles(const string&, uint8_t, vector<string>&) { return 0; }
    static string extractByIndex(const string&, int) { return ""; }
    static int extractAll(const string&, const string&) { return 0; }
    static void viewInfo(const string&) {}
    static void cleanup() {}
};
#else

class ZipExtract {
public:
    // Extract first matching file from ZIP archive to /tmp/.zip_extract
    // Returns full path to extracted file, or "" on error
    static string extract(const string& zipPath, uint8_t fileType);

    // List supported filenames inside ZIP (filtered by fileType extensions)
    static int listFiles(const string& zipPath, uint8_t fileType, vector<string>& names);

    // Extract specific file (by index from listFiles) to /tmp/.zip_extract
    // Returns full path to extracted file, or "" on error
    static string extractByIndex(const string& zipPath, int fileIndex);

    // Extract all files from ZIP to destDir. Returns number of files extracted.
    static int extractAll(const string& zipPath, const string& destDir);

    // Show ZIP file info (name, size, file list) in OSD dialog
    static void viewInfo(const string& zipPath);

    static void cleanup();

private:
    // ZIP Local File Header (30 bytes fixed part)
    struct __attribute__((packed)) LocalFileHeader {
        uint32_t signature;        // 0x04034b50
        uint16_t versionNeeded;
        uint16_t flags;
        uint16_t compression;      // 0=stored, 8=deflate
        uint16_t modTime;
        uint16_t modDate;
        uint32_t crc32;
        uint32_t compressedSize;
        uint32_t uncompressedSize;
        uint16_t nameLen;
        uint16_t extraLen;
    };

    static const uint32_t ZIP_LOCAL_SIGNATURE = 0x04034b50;
    static const char* TEMP_FILE;

    static bool hasMatchingExtension(const string& filename, uint8_t fileType);
    static bool extractFile(FIL* zipFile, uint16_t compression, uint32_t compressedSize, uint32_t uncompressedSize);
    static bool extractStored(FIL* zipFile, uint32_t size);
    static bool extractDeflate(FIL* zipFile, uint32_t compressedSize);
};
#endif // PICO_RP2040

#endif
