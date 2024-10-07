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
#include "ZXKeyb.h"

using namespace std;

string FileUtils::MountPoint = MOUNT_POINT_SD; // Start with SD
bool FileUtils::SDReady = false;
///sdmmc_card_t *FileUtils::card;

string FileUtils::SNA_Path = "/";
string FileUtils::TAP_Path = "/";
string FileUtils::DSK_Path = "/";
string FileUtils::ROM_Path = "/";
DISK_FTYPE FileUtils::fileTypes[4] = {
    {".sna,.SNA,.z80,.Z80,.p,.P",".s",2,2,0,""},
    {".tap,.TAP,.tzx,.TZX",".t",2,2,0,""},
    {".trd,.TRD,.scl,.SCL",".d",2,2,0,""},
    {".rom,.ROM,.bin,.BIN",".d",2,2,0,""}
};

size_t fwrite(const void* v, size_t sz1, size_t sz2, FIL& f);
void fputs(const char* b, FIL& f) {
    size_t sz = strlen(b);
    UINT bw;
    f_write(&f, b, sz, &bw);
    f_write(&f, "\n", 1, &bw);
}
void fgets(char* b, size_t sz, FIL& f);
#define ftell(x) f_tell(&x)
#define feof(x) f_eof(&x)
inline void fclose(FIL& f) {
    f_close(&f);
}

void FileUtils::initFileSystem() {
    if ((!ZXKeyb::Exists) && (!SDReady)) SDReady = true;
    mountSDCard();
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
bool FileUtils::mountSDCard() {
    return f_mount(&fs, "SD", 1) == FR_OK;
}

void FileUtils::unmountSDCard() {
    f_unmount("SD");
}

// String FileUtils::getAllFilesFrom(const String path) {
//     KB_INT_STOP;
//     File root = THE_FS.open("/");
//     File file = root.openNextFile();
//     String listing;

//     while (file) {
//         file = root.openNextFile();
//         String filename = file.name();
//         if (filename.startsWith(path) && !filename.startsWith(path + "/.")) {
//             listing.concat(filename.substring(path.length() + 1));
//             listing.concat("\n");
//         }
//     }
//     vTaskDelay(2);
//     KB_INT_START;
//     return listing;
// }

// void FileUtils::listAllFiles() {
//     KB_INT_STOP;
//     File root = THE_FS.open("/");
//     Serial.println("fs opened");
//     File file = root.openNextFile();
//     Serial.println("fs openednextfile");

//     while (file) {
//         Serial.print("FILE: ");
//         Serial.println(file.name());
//         file = root.openNextFile();
//     }
//     vTaskDelay(2);
//     KB_INT_START;
// }

// void FileUtils::sanitizeFilename(String filename)
// {
//     filename.replace("\n", " ");
//     filename.trim();
// }

// File FileUtils::safeOpenFileRead(String filename)
// {
//     sanitizeFilename(filename);
//     File f;
//     if (Config::slog_on)
//         Serial.printf("%s '%s'\n", MSG_LOADING, filename.c_str());
//     if (!THE_FS.exists(filename.c_str())) {
//         KB_INT_START;
//         OSD::errorHalt((String)ERR_READ_FILE + "\n" + filename);
//     }
//     f = THE_FS.open(filename.c_str(), FILE_READ);
//     vTaskDelay(2);

//     return f;
// }

// string FileUtils::getFileEntriesFromDir(string path) {

//     string filelist;

//     // printf("Getting entries from: '%s'\n", path.c_str());

//     DIR* dir = opendir(path.c_str());
//     if (dir == NULL) {
//         // OSD::errorHalt(ERR_DIR_OPEN + "\n" + path).cstr());
//     }

//     struct dirent* de = readdir(dir);
    
//     if (!de) {

//         printf("No entries found!\n");

//     } else {

//         int cnt = 0;
//         while (true) {
            
//             printf("Found file: %s\n", de->d_name);
            
//             string filename = de->d_name;

//             // printf("readdir filename -> %s\n", filename.c_str());

//             if (filename.compare(0,1,".") == 0) {
//         //        printf("HIDDEN\n");
//             } else if (filename.substr(filename.size()-4) == ".txt") {
//         //        printf("IGNORING TXT\n");
//             } else if (filename.substr(filename.size()-4) == ".TXT") {
//         //        printf("IGNORING TXT\n");
//             } else {
//         //        printf("ADDING\n");
//                 filelist += filename + "\n";
//                 cnt++;
//             }
            
//             de = readdir(dir);
//             if ((!de) || (cnt == 20)) break;
        
//         }

//     }

//     // printf(filelist.c_str());

//     closedir(dir);

//     return filelist;

// }
/**
void FileUtils::DirToFile(string fpath, uint8_t ftype) {
    char fileName[8];
    std::vector<std::string> filenames;
    filenames.reserve(MAX_FNAMES_PER_CHUNK);
    // Populate filexts with valid filename extensions
    std::vector<std::string> filexts;
    size_t pos = 0;
    string ss = fileTypes[ftype].fileExts;
    while ((pos = ss.find(",")) != std::string::npos) {
        // printf("%s , ",ss.substr(0,pos).c_str());
        filexts.push_back(ss.substr(0, pos));
        ss.erase(0, pos + 1);
    }
    // printf("%s , ",ss.substr(0).c_str());
    filexts.push_back(ss.substr(0));
    // printf("\n");

    string fdir = fpath.substr(0,fpath.length() - 1);
    DIR f_dir;
    DIR* dir = &f_dir;;
    FRESULT res = f_opendir(dir, fdir.c_str());
    if (res != FR_OK) {
        string msg = "Unable to open dir: "; msg += fdir;
        /// TODO: error dialog
        OSD::progressDialog("ERROR", msg, 0, 0);
        while(1);
        return;
    }
    string dirfile = fpath + fileTypes[ftype].indexFilename;
    // Remove previous dir file
    f_unlink(dirfile.c_str());
    OSD::progressDialog(OSD_FILE_INDEXING[Config::lang], OSD_FILE_INDEXING_1[Config::lang], 0, 0);

    // Read filenames from medium into vector, sort it, and dump into MAX_FNAMES_PER_CHUNK filenames long files
    int cnt = 0;
    int chunk_cnt = 0;
    int item_count = 0;
    int items_processed = 0;

    // Count items to process
    f_rewinddir(dir);
    FILINFO fileInfo;
    while (f_readdir(&f_dir, &fileInfo) == FR_OK && fileInfo.fname[0] != '\0') {
        item_count++;
    }
    f_rewinddir(dir);
    unsigned long h = 0, high; // Directory Hash

    OSD::elements = 0;
    OSD::ndirs = 0;

    while (f_readdir(&f_dir, &fileInfo) == FR_OK && fileInfo.fname[0] != '\0') {
        string fname = fileInfo.fname;
        // if (fname[0] == 'A') printf("Fname: %s\n",fname.c_str());
        {
            if (fname.compare(0,1,".") != 0) {
                // if (fname[0] == 'A') printf("Fname2: %s\n",fname.c_str());
                // if ((de->d_type == DT_DIR) || (std::find(filexts.begin(),filexts.end(),fname.substr(fname.size()-4)) != filexts.end())) {


                // if ((de->d_type == DT_DIR) || ((fname.size() > 3) && (std::find(filexts.begin(),filexts.end(),fname.substr(fname.size()-4)) != filexts.end()))) {
                
                size_t fpos = fname.find_last_of(".");
                if ((fileInfo.fattrib & AM_DIR) || ((fpos != string::npos) && (std::find(filexts.begin(),filexts.end(),fname.substr(fpos)) != filexts.end()))) {

                    // if (fname[0] == 'A') printf("Fname3: %s\n",fname.c_str());

                    if (fileInfo.fattrib & AM_DIR) {
                        filenames.push_back((char(32) + fname).c_str());
                        OSD::ndirs++;
                    } else {
                        filenames.push_back(fname.c_str());
                        OSD::elements++;
                    }

                    // Calc hash
                    for (int i = 0; i < fname.length(); i++) {
                        h = (h << 4) + fname[i];
                        if (high = h & 0xF0000000)
                            h ^= high >> 24;
                        h &= ~high;
                    }

                    cnt++;
                    if (cnt == MAX_FNAMES_PER_CHUNK) {
                        // Dump current chunk
                        sort(filenames.begin(),filenames.end()); // Sort vector
                        sprintf(fileName, "%d", chunk_cnt);
                        FIL f;
                        if (f_open(&f, (fpath + fileTypes[ftype].indexFilename + fileName).c_str(), FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
                            printf("Error opening filelist chunk\n");
                            // Close progress dialog
                            OSD::progressDialog("", "" , 0, 2);
                            return;
                        }
                        for (int n=0; n < MAX_FNAMES_PER_CHUNK; n++)
                            fputs((filenames[n] + std::string(63 - filenames[n].size(), ' ') + "\n").c_str(), f);
                        f_close(&f);
                        filenames.clear();
                        cnt = 0;
                        chunk_cnt++;
                        items_processed--;
                    }

                }
            }

            items_processed++;
            OSD::progressDialog("","",(float) 100 / ((float) item_count / (float) items_processed),1);

        }

    }

    // Add previous directory entry if not on root dir
    // printf("%s - %s\n",fpath.c_str(),(MountPoint + "/").c_str());
    if (fpath != (MountPoint + "/")) {
        filenames.push_back("  ..");
        cnt++;
    }

    f_closedir(dir);

    filexts.clear(); // Clear vector
    std::vector<std::string>().swap(filexts); // free memory    

    if (cnt > 0) { 
        // Dump last chunk
        sort(filenames.begin(),filenames.end()); // Sort vector
        sprintf(fileName, "%d", chunk_cnt);
        FIL f;
        if (f_open(&f, (fpath + fileTypes[ftype].indexFilename + fileName).c_str(), FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
            printf("Error opening last filelist chunk\n");
            // Close progress dialog
            OSD::progressDialog("","",0,2);
            return;
        }
        for (int n=0; n < cnt;n++) fputs((filenames[n] + std::string(63 - filenames[n].size(), ' ') + "\n").c_str(),f);
        f_close(&f);
    }

    filenames.clear(); // Clear vector
    std::vector<std::string>().swap(filenames); // free memory

    if (chunk_cnt == 0) {
        if (cnt == 0) {
            // Close progress dialog
            OSD::progressDialog("","",0,2);
            return;
        }
        rename((fpath + fileTypes[ftype].indexFilename + "0").c_str(),(fpath + fileTypes[ftype].indexFilename).c_str());   // Rename unique chunk
    } else {
        OSD::progressDialog(OSD_FILE_INDEXING[Config::lang],OSD_FILE_INDEXING_2[Config::lang],0,1);
        Mergefiles(fpath, ftype, chunk_cnt);
    }

    // Add directory hash to last line of file
    // printf("Hashcode: %lu\n",h);
    FIL fout;
    f_open(&fout, (fpath + fileTypes[ftype].indexFilename).c_str(), FA_WRITE | FA_OPEN_APPEND);
    fputs(to_string(h).c_str(),fout);
    f_close(&fout);

    // Close progress dialog
    OSD::progressDialog("", "", 0, 2);

}

void FileUtils::Mergefiles(string fpath, uint8_t ftype, int chunk_cnt) {

    char fileName[8];

    // multi_heap_info_t info;    
    // heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
    // string textout = " Total free bytes           : " + to_string(info.total_free_bytes) + "\n";
    // string textout2 = " Minimum free ever          : " + to_string(info.minimum_free_bytes) + "\n";
    // printf("%s",textout.c_str());
    // printf("%s\n",textout2.c_str());                                

    // Merge sort
    FIL file1, file2, fout;
    char fname1[64];
    char fname2[64];

    f_open(&file1, (fpath + fileTypes[ftype].indexFilename + "0").c_str(), FA_READ);
    f_open(&file2, (fpath + fileTypes[ftype].indexFilename + "1").c_str(), FA_READ);
    string bufout="";
    int bufcnt = 0;

    int  n = 1;
    while (file2.obj.fs != NULL) {

        sprintf(fileName, ".tmp%d", n);
        // printf("Creating %s\n",fileName);
        f_open(&fout, (fpath + fileName).c_str(), FA_WRITE | FA_CREATE_ALWAYS);

        fgets(fname1, sizeof(fname1), file1);
        fgets(fname2, sizeof(fname2), file2);

        while(1) {

            if (feof(file1)) {
                if (feof(file2)) break;
                bufout += fname2;
                fgets(fname2, sizeof(fname2), file2);
            }
            else if (feof(file2)) {
                if (feof(file1)) break;
                bufout += fname1;
                fgets(fname1, sizeof(fname1), file1);
            } else if (strcmp(fname1,fname2)< 0) {
                bufout += fname1;
                fgets(fname1, sizeof(fname1), file1);
            } else {
                bufout += fname2;
                fgets(fname2, sizeof(fname2), file2);
            }

            bufcnt++;

            if (bufcnt == 64) {
                // heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
                // textout = " Total free bytes           : " + to_string(info.total_free_bytes) + "\n";
                // textout2 = " Minimum free ever          : " + to_string(info.minimum_free_bytes) + "\n";
                // printf(" Buffer size: %d\n",bufout.length());
                // printf("%s",textout.c_str());
                // printf("%s\n",textout2.c_str());                                
                fwrite(bufout.c_str(),sizeof(char),bufout.length(),fout);
                bufout = "";
                bufcnt = 0;
            }

        }

        if (bufcnt) {
            fwrite(bufout.c_str(),sizeof(char),bufout.length(),fout);
        }

        fclose(file1);
        fclose (file2);
        fclose (fout);

        // Next cycle: open t<n> for read
        sprintf(fileName, ".tmp%d", n);
        f_open(&file1, (fpath + fileName).c_str(), FA_READ);

        OSD::progressDialog("","",(float) 100 / ((float) chunk_cnt / (float) n),1);

        // printf("chunkcnt: %d n: %d\n",(int) chunk_cnt, (int)n);

        n++;

        sprintf(fileName, "%d", n);
        f_open(&file2, (fpath + fileTypes[ftype].indexFilename + fileName).c_str(), FA_READ);

        // printf("n Opened: %d\n",(int)n);

    }

    // printf("Closing file1\n");

    fclose(file1);

    // printf("File1 closed\n");

    // Rename final chunk
    sprintf(fileName, ".tmp%d", n - 1);
    rename((fpath + fileName).c_str(),(fpath + fileTypes[ftype].indexFilename).c_str());

    OSD::progressDialog(OSD_FILE_INDEXING[Config::lang],OSD_FILE_INDEXING_3[Config::lang],0,1);

    // Remove temp files
    for (int n = 0; n <= chunk_cnt; n++) {
        sprintf(fileName, "%d", n);
        f_unlink((fpath + fileTypes[ftype].indexFilename + fileName).c_str());
        sprintf(fileName, ".tmp%d", n);
        f_unlink((fpath + fileName).c_str());
        OSD::progressDialog("","",(float) 100 / ((float) chunk_cnt / (float) n),1);
    }
}
*/

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
