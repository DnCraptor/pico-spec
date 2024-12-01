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
#include <stdlib.h>
#include <string>
#include <vector>
#include <algorithm>
#include "FileUtils.h"
#include "Config.h"
#include "CPU.h"
#include "MemESP.h"
#include "ESPectrum.h"
#include "hardpins.h"
#include "messages.h"
#include "OSDMain.h"
#include "roms.h"
#include "Video.h"

using namespace std;

string FileUtils::MountPoint = MOUNT_POINT_SD; // Start with SD
bool FileUtils::SDReady = false;
///sdmmc_card_t *FileUtils::card;

string FileUtils::SNA_Path = "/";
string FileUtils::TAP_Path = "/";
string FileUtils::DSK_Path = "/";
string FileUtils::ROM_Path = "/";
DISK_FTYPE FileUtils::fileTypes[4] = {
    {".sna,.SNA,.z80,.Z80,.p,.P",2,2,0,""},
    {".tap,.TAP,.tzx,.TZX,.wav,.WAV,.mp3,.MP3",2,2,0,""},
    {".trd,.TRD,.scl,.SCL",2,2,0,""},
    {".rom,.ROM,.bin,.BIN",2,2,0,""}
};

size_t fwrite(const void* v, size_t sz1, size_t sz2, FIL* f);
void fputs(const char* b, FIL& f) {
    size_t sz = strlen(b);
    UINT bw;
    f_write(&f, b, sz, &bw);
}
void fgets(char* b, size_t sz, FIL& f);
#define ftell(x) f_tell(&x)
#define feof(x) f_eof(&x)
inline void fclose(FIL& f) {
    f_close(&f);
}

void FileUtils::initFileSystem() {
    SDReady = mountSDCard();
    f_mkdir("/tmp");
    f_mkdir(MOUNT_POINT_SD);
    f_mkdir(MOUNT_POINT_SD DISK_ROM_DIR);
    f_mkdir(MOUNT_POINT_SD DISK_SNA_DIR);
    f_mkdir(MOUNT_POINT_SD DISK_TAP_DIR);
    f_mkdir(MOUNT_POINT_SD DISK_DSK_DIR);
    f_mkdir(MOUNT_POINT_SD DISK_SCR_DIR);
    f_mkdir(MOUNT_POINT_SD DISK_PSNA_DIR);
}

static FATFS fs;
bool FileUtils::fsMount = false;
bool FileUtils::mountSDCard() {
    fsMount = f_mount(&fs, "SD", 1) == FR_OK;
    return fsMount;
}

void FileUtils::unmountSDCard() {
    f_unmount("SD");
}

bool FileUtils::hasSNAextension(string filename)
{
    
    if (filename.substr(filename.size()-4,4) == ".sna") return true;
    if (filename.substr(filename.size()-4,4) == ".SNA") return true;
    return false;
}

bool FileUtils::hasZ80extension(string filename)
{
    if (filename.substr(filename.size()-4,4) == ".z80") return true;
    if (filename.substr(filename.size()-4,4) == ".Z80") return true;
    return false;
}

bool FileUtils::hasPextension(string filename)
{
    if (filename.substr(filename.size()-2,2) == ".p") return true;
    if (filename.substr(filename.size()-2,2) == ".P") return true;
    return false;
}

bool FileUtils::hasTAPextension(string filename)
{
    if (filename.substr(filename.size()-4,4) == ".tap") return true;
    if (filename.substr(filename.size()-4,4) == ".TAP") return true;
    return false;
}

bool FileUtils::hasTZXextension(string filename)
{
    if (filename.substr(filename.size()-4,4) == ".tzx") return true;
    if (filename.substr(filename.size()-4,4) == ".TZX") return true;
    return false;
}

bool FileUtils::hasWAVextension(string filename)
{
    if (filename.substr(filename.size()-4,4) == ".wav") return true;
    if (filename.substr(filename.size()-4,4) == ".WAV") return true;
    return false;
}

bool FileUtils::hasMP3extension(string filename)
{
    if (filename.substr(filename.size()-4,4) == ".mp3") return true;
    if (filename.substr(filename.size()-4,4) == ".MP3") return true;
    return false;
}

void FileUtils::deleteFilesWithExtension(const char *folder_path, const char *extension) {
    DIR dir;
    FILINFO entry;
    if (f_opendir(&dir, folder_path) != FR_OK) {
        // perror("Unable to open directory");
        return;
    }

    while (f_readdir(&dir, &entry) == FR_OK && entry.fname[0] != '\0') {
        if (strcmp(entry.fname, ".") != 0 && strcmp(entry.fname, "..") != 0) {
            if (strstr(entry.fname, extension) != NULL) {
                char file_path[512];
                snprintf(file_path, sizeof(file_path), "%s/%s", folder_path, entry.fname);
                if (f_unlink(file_path) == 0) {
                    printf("Deleted file: %s\n", entry.fname);
                } else {
                    printf("Failed to delete file: %s\n", entry.fname);
                }
            }
        }
    }

    f_closedir(&dir);
}

// uint16_t FileUtils::countFileEntriesFromDir(String path) {
//     String entries = getFileEntriesFromDir(path);
//     unsigned short count = 0;
//     for (unsigned short i = 0; i < entries.length(); i++) {
//         if (entries.charAt(i) == ASCII_NL) {
//             count++;
//         }
//     }
//     return count;
// }

// // Get all sna files sorted alphabetically
// string FileUtils::getSortedFileList(string fileDir)
// {
    
//     // get string of unsorted filenames, separated by newlines
//     string entries = getFileEntriesFromDir(fileDir);

//     // count filenames (they always end at newline)
//     int count = 0;
//     for (int i = 0; i < entries.length(); i++) {
//         if (entries.at(i) == ASCII_NL) {
//             count++;
//         }
//     }

//     std::vector<std::string> filenames;
//     filenames.reserve(count);

//     // Copy filenames from string to vector
//     string fname = "";
//     for (int i = 0; i < entries.length(); i++) {
//         if (entries.at(i) == ASCII_NL) {
//             filenames.push_back(fname.c_str());
//             fname = "";
//         } else fname += entries.at(i);
//     }

//     // Sort vector
//     sort(filenames.begin(),filenames.end());

//     // Copy back filenames from vector to string
//     string sortedEntries = "";
//     for (int i = 0; i < count; i++) {
//         sortedEntries += filenames[i] + '\n';
//     }

//     return sortedEntries;

// }
