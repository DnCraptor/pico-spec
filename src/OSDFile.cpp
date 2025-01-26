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

#include <string>
#include <algorithm>
#include <sys/stat.h>
#include "errno.h"

using namespace std;

#include "OSDMain.h"
#include "FileUtils.h"
#include "Config.h"
#include "ESPectrum.h"
#include "CPU.h"
#include "Video.h"
#include "messages.h"
#include <math.h>
#include "Z80_JLS/z80.h"
#include "Tape.h"

#include "ff.h"

inline static size_t crc(const std::string& s) {
    size_t res = 0;
    for (size_t j = 0; j < s.size(); ++j) {
        res += s[j];
    }
    return res;
}

fabgl::VirtualKey get_last_key_pressed(void);

class sorted_files {
    static const size_t rec_size = FF_LFN_BUF + 1;
    std::string folder;
    std::string idx_file;
    size_t sz = 0;
    FIL* storage_file = 0;
    bool open = false;
    inline void calc_sz() {
        sz = 0;
        storage_file = fopen2(idx_file.c_str(), FA_READ);
        if (storage_file) {
            UINT br;
            char buf[rec_size];
            while ( f_read(storage_file, buf, rec_size, &br) == FR_OK && br == rec_size ) {
                ++sz;
            }
            fclose2(storage_file);
        }
        storage_file = fopen2(idx_file.c_str(), FA_READ | FA_WRITE);
        if (storage_file) open = true;
    }
public:
    inline sorted_files() { }
    inline void close(void) { if (open && storage_file) fclose2(storage_file); open = false; }
    inline ~sorted_files() { close(); }
    inline size_t size(void) { return sz; }
    inline void unlink(void) {
        close();
        f_unlink(idx_file.c_str());
        storage_file = fopen2(idx_file.c_str(), FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
        if (storage_file) open = true;
        sz = 0;
    }
    inline void init(const std::string& folder) {
        close();
        this->folder = folder;
        const char* prefix;
        std::string s = folder;
        std::replace( s.begin(), s.end(), '/', '_');
        idx_file = "/tmp/." + s + ".idx";
        calc_sz();
    }
    inline void put(size_t i, const std::string& s) {
        f_lseek(storage_file, rec_size * i);
        UINT bw;
        char buf[rec_size] = { 0 };
        strncpy(buf, s.c_str(), rec_size - 1);
        f_write(storage_file, buf, rec_size, &bw);
    }
    inline void push(const std::string& s) {
        put(sz++, s);
    }
    inline size_t crc(void) {
        size_t res = 0;
        for (size_t i = 0; i < sz; ++i) {
            res += ::crc(get(i));
        }
        return res;
    }
    inline std::string get(size_t i) {
        f_lseek(storage_file, rec_size * i);
        UINT br;
        char buf[rec_size];
        f_read(storage_file, buf, rec_size, &br);
        return (buf);
    }
    inline std::string operator[](size_t i) {
        return get(i);
    }
    inline int cmp(const std::string s1, const std::string s2) {
        return s1.compare(s2);
    }
    inline int cmp(size_t i1, size_t i2) {
        std::string s1 = get(i1);
        std::string s2 = get(i2);
        return s1.compare(s2);
    }
    inline void swap(size_t i1, size_t i2) {
        std::string s1 = get(i1);
        std::string s2 = get(i2);
        put(i1, s2);
        put(i2, s1);
    }
    inline void vecswap(size_t i1, size_t i2, size_t num) {
        for (size_t i = 0; i < num; ++i) {
            swap(i1 + i, i2 + i);
        }
    }
    inline size_t med3(size_t a, size_t b, size_t c) {
	    return cmp(a, b) < 0 ? (cmp(b, c) < 0 ? b : (cmp(a, c) < 0 ? c : a )) : (cmp(b, c) > 0 ? b : (cmp(a, c) < 0 ? a : c ));
    }
    inline void sort(void) {
        qsort(0, sz);
    }
    void qsort(size_t ai, size_t n) {
        if (!n) return;
        size_t pn, pm, pl, d, pa, pb, pc, pd = 0;
        int r;
    loop:
        fabgl::VirtualKey lkp = get_last_key_pressed();
        if (lkp == fabgl::VirtualKey::VK_F1) return;
        size_t swap_cnt = 0;
        if (n < 7) {
            for (pm = ai + 1; pm < ai + n; ++pm) {
                for (pl = pm; pl > ai && cmp(pl - 1, pl) > 0; --pl) {
                    swap(pl, pl - 1);
                }
            }
        }
        pm = ai + (n / 2);
	    if (n > 7) {
		    pl = ai;
		    pn = ai + (n - 1);
		    if (n > 40) {
			    d = (n / 8);
			    pl = med3(pl, pl + d, pl + 2 * d);
			    pm = med3(pm - d, pm, pm + d);
    			pn = med3(pn - 2 * d, pn - d, pn);
	    	}
		    pm = med3(pl, pm, pn);
	    }
	    swap(ai, pm);
	    pa = pb = ai + 1;
        pc = pd = ai + (n - 1);
	    for (;;) {
		    while (pb <= pc && (r = cmp(pb, ai)) <= 0) {
                fabgl::VirtualKey lkp = get_last_key_pressed();
                if (lkp == fabgl::VirtualKey::VK_F1) return;
			    if (r == 0) {
				    swap_cnt = 1;
				    swap(pa, pb);
				    ++pa;
                }
			    ++pb;
		    }
		    while (pb <= pc && (r = cmp(pc, ai)) >= 0) {
                fabgl::VirtualKey lkp = get_last_key_pressed();
                if (lkp == fabgl::VirtualKey::VK_F1) return;
			    if (r == 0) {
				    swap_cnt = 1;
				    swap(pc, pd);
				    --pd;
			    }
			    --pc;
		    }
		    if (pb > pc)
			    break;
		    swap(pb, pc);
		    swap_cnt = 1;
		    ++pb;
		    --pc;
	    }
	    if (swap_cnt == 0) {  // Switch to insertion sort
		    for (pm = ai + 1; pm < ai + n; ++pm)
			    for (pl = pm; pl > ai && cmp(pl - 1, pl) > 0; --pl) {
                    fabgl::VirtualKey lkp = get_last_key_pressed();
                    if (lkp == fabgl::VirtualKey::VK_F1) return;
				    swap(pl, pl - 1);
                }
		    return;
        }
	    pn = ai + n;
	    r = min(pa - ai, pb - pa);
	    vecswap(ai, pb - r, r);
	    r = min(pd - pc, pn - pd - 1);
	    vecswap(pb, pn - r, r);
	    if ((r = pb - pa) > 1)
		qsort(ai, r);
	    if ((r = pd - pc) > 1) { 
		    // Iterate rather than recurse to save stack space
		    ai = pn - r;
		    n = r;
		    goto loop;
	    }
    }
};

static sorted_files filenames;

unsigned int OSD::elements;
unsigned int OSD::fdSearchElements;
unsigned int OSD::ndirs;
int8_t OSD::fdScrollPos;
int OSD::timeStartScroll;
int OSD::timeScroll;
uint8_t OSD::fdCursorFlash;
bool OSD::fdSearchRefresh;    

size_t fread(uint8_t* v, size_t sz1, size_t sz2, FIL& f);
int fseek (FIL* stream, long offset, int origin);
inline void fclose(FIL& f) {
    f_close(&f);
}
inline void rewind(FIL& f) {
    f_lseek(&f, 0);
}
void fgets(char* b, size_t sz, FIL& f) {
    UINT br;
    char c;
    do {
        f_read(&f, b, 1, &br);
        c = *b++;
    } while (br == 1 && c != '\n' && !f_eof(&f) && sz--);
    *b = 0;
}
#define ftell(x) f_tell(&x)
#define feof(x) f_eof(&x)

// Run a new file menu
string OSD::fileDialog(string &fdir, string title, uint8_t ftype, uint8_t mfcols, uint8_t mfrows) {
    // Position
    if (menu_level == 0) {
        x = (Config::aspect_16_9 ? 24 : 4);
        y = (Config::aspect_16_9 ? 4 : 4);
    } else {
        x = (Config::aspect_16_9 ? 24 : 8) + (60 * menu_level);
        y = 8 + (16 * menu_level);
    }

    // Columns and Rows
    cols = mfcols;
    mf_rows = mfrows + (Config::aspect_16_9 ? 0 : 1);

    if (FileUtils::fileTypes[ftype].focus > mf_rows - 1) {
        FileUtils::fileTypes[ftype].begin_row += FileUtils::fileTypes[ftype].focus - (mf_rows - 1);
        FileUtils::fileTypes[ftype].focus = mf_rows - 1;
    }

    size_t pos = 0;
    std::vector<std::string> filexts;
    string ss = FileUtils::fileTypes[ftype].fileExts;
    while ((pos = ss.find(",")) != std::string::npos) {
        filexts.push_back(ss.substr(0, pos));
        ss.erase(0, pos + 1);
    }
    filexts.push_back(ss.substr(0));

    // Size
    w = (cols * OSD_FONT_W) + 2;
    h = ((mf_rows + 1) * OSD_FONT_H) + 2;

    DIR f_dir;
    bool res = f_opendir(&f_dir, fdir.c_str()) == FR_OK;
    if (!res) {
        fdir = "/";
    } else {
        f_closedir(&f_dir);
    }

    menu = title + "\n" + fdir + "\n";
    WindowDraw(); // Draw menu outline
    fd_PrintRow(1, IS_INFO, filexts);    // Path

    // Draw blank rows
    uint8_t row = 2;
    for (; row < mf_rows; row++) {
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        menuAt(row, 0);
        VIDEO::vga.print(std::string(cols, ' ').c_str());
    }

    // Print status bar
    menuAt(row, 0);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
    VIDEO::vga.print(std::string(cols, ' ').c_str());    

    while(1) {
        filenames.init(fdir);
        fdCursorFlash = 0;
        // Count dir items and calc hash
        elements = 0;
        ndirs = 0;
        OSD::progressDialog(OSD_FILE_INDEXING[Config::lang], OSD_FILE_INDEXING_1[Config::lang], 0, 0);
        res = f_opendir(&f_dir, fdir.c_str()) == FR_OK;
        if (res) {
        
            FILINFO fileInfo;
            size_t crc = 0;
            if (fdir.size() > 1) {
                ++ndirs;
                crc += ::crc("  ..");
            }
            while (f_readdir(&f_dir, &fileInfo) == FR_OK && fileInfo.fname[0] != '\0') {
                if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
                   fabgl::VirtualKey lkp = get_last_key_pressed();
                   if (lkp == fabgl::VirtualKey::VK_F1) break;
                }
                string fname = fileInfo.fname;
                if (fname.compare(0,1,".") != 0) {
                        if (fileInfo.fattrib & AM_DIR) {
                            ++ndirs;
                            crc += ::crc(char(32) + fname);
                        }
                        else {
                            ++elements; // Count elements in dir
                            crc += ::crc(fname);
                        }
                }
            }

            f_closedir(&f_dir);
            uint32_t rcrc = filenames.crc();
            if (rcrc != crc) { // reindex
                filenames.unlink();
                if (fdir.size() > 1) {
                    filenames.push("  ..");
                }
                f_opendir(&f_dir, fdir.c_str());
                OSD::progressDialog(OSD_FILE_INDEXING[Config::lang], OSD_FILE_INDEXING_1[Config::lang], 5, 1);
                size_t f_idx = 0;
                while (f_readdir(&f_dir, &fileInfo) == FR_OK && fileInfo.fname[0] != '\0') {
                    if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
                        fabgl::VirtualKey lkp = get_last_key_pressed();
                        if (lkp == fabgl::VirtualKey::VK_F1) break;
                    }
                    string fname = fileInfo.fname;
                    if (fname.compare(0,1,".") != 0) {
                            if (fileInfo.fattrib & AM_DIR) {
                                filenames.push(char(32) + fname);
                            }
                            else {
                                filenames.push(fname);
                            }
                            ++f_idx;
                            OSD::progressDialog(
                                OSD_FILE_INDEXING[Config::lang],
                                OSD_FILE_INDEXING_1[Config::lang],
                                f_idx * 95 / (ndirs + elements) + 5,
                                1
                            );
                    }
                }
                f_closedir(&f_dir);
                filenames.sort();
            }
        }
        OSD::progressDialog(OSD_FILE_INDEXING[Config::lang], OSD_FILE_INDEXING_1[Config::lang], 100, 2);
        real_rows = ndirs + elements + 2; // Add 2 for title and status bar        
        virtual_rows = (real_rows > mf_rows ? mf_rows : real_rows);
        // printf("Real rows: %d; st_size: %d; Virtual rows: %d\n",real_rows,stat_buf.st_size,virtual_rows);
        last_begin_row = last_focus = 0;
        fdSearchElements = elements;

        if ((real_rows > mf_rows) && ((FileUtils::fileTypes[ftype].begin_row + mf_rows - 2) > real_rows)) {
            FileUtils::fileTypes[ftype].focus += (FileUtils::fileTypes[ftype].begin_row + mf_rows - 2) - real_rows;
            FileUtils::fileTypes[ftype].begin_row = real_rows - (mf_rows - 2);
        }

        fd_Redraw(title, fdir, ftype, filexts); // Draw content

        // Focus line scroll position
        fdScrollPos = 0;
        timeStartScroll = 0;
        timeScroll = 0;

        fabgl::VirtualKeyItem Menukey;
        while (1) {
            // Process external keyboard
            if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
                timeStartScroll = 0;
                timeScroll = 0;
                fdScrollPos = 0;
                // Print elements
                VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                unsigned int elem = FileUtils::fileTypes[ftype].fdMode ? fdSearchElements : elements;
                if (elem) {
                    menuAt(mfrows + (Config::aspect_16_9 ? 0 : 1), cols - (real_rows > virtual_rows ? 13 : 12));
                    char elements_txt[13];
                    int nitem = (FileUtils::fileTypes[ftype].begin_row + FileUtils::fileTypes[ftype].focus ) - (4 + ndirs) + (fdir.length() == 1);
                    snprintf(elements_txt, sizeof(elements_txt), "%d/%d ", nitem > 0 ? nitem : 0 , elem);
                    VIDEO::vga.print(std::string(12 - strlen(elements_txt), ' ').c_str());
                    VIDEO::vga.print(elements_txt);
                } else {
                    menuAt(mfrows + (Config::aspect_16_9 ? 0 : 1), cols - 13);
                    VIDEO::vga.print("             ");
                }

                if (ESPectrum::readKbd(&Menukey)) {
                    if (!Menukey.down) continue;
                    // Search first ocurrence of letter if we're not on that letter yet
                    if (((Menukey.vk >= fabgl::VK_a) && (Menukey.vk <= fabgl::VK_Z)) || ((Menukey.vk >= fabgl::VK_0) && (Menukey.vk <= fabgl::VK_9))) {
                        int fsearch;
                        if (Menukey.vk<=fabgl::VK_9)
                            fsearch = Menukey.vk + 46;
                        else if (Menukey.vk<=fabgl::VK_z)
                            fsearch = Menukey.vk + 75;
                        else if (Menukey.vk<=fabgl::VK_Z)
                            fsearch = Menukey.vk + 17;
                        {
                            uint8_t letra = rowGet(menu,FileUtils::fileTypes[ftype].focus).at(0);
                            if (letra != fsearch) { 
                                // Seek first ocurrence of letter/number
                                int cnt = 0;
                                while (cnt < filenames.size()) {
                                    std::string s = filenames[cnt++];
                                    if (s.size() && s.at(0) == char(fsearch)) break;
                                    cnt++;
                                }
                                if (cnt < filenames.size()) {
                                    last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                                    last_focus = FileUtils::fileTypes[ftype].focus;                                    
                                    if (real_rows > virtual_rows) {
                                        int m = cnt + virtual_rows - real_rows;
                                        if (m > 0) {
                                            FileUtils::fileTypes[ftype].focus = m + 2;
                                            FileUtils::fileTypes[ftype].begin_row = cnt - m + 2;
                                        } else {
                                            FileUtils::fileTypes[ftype].focus = 2;
                                            FileUtils::fileTypes[ftype].begin_row = cnt + 2;
                                        }
                                    } else {
                                        FileUtils::fileTypes[ftype].focus = cnt + 2;
                                        FileUtils::fileTypes[ftype].begin_row = 2;
                                    }
                                    fd_Redraw(title,fdir,ftype, filexts);
                                    click();
                                }
                            }
                       }
                    } else if (Menukey.vk == fabgl::VK_F3) {
                        FileUtils::fileTypes[ftype].fdMode ^= 1;
                        if (FileUtils::fileTypes[ftype].fdMode) {
                            fdCursorFlash = 63;
                            fdSearchRefresh = FileUtils::fileTypes[ftype].fileSearch != "";
                        } else {
                            menuAt(mfrows + (Config::aspect_16_9 ? 0 : 1), 1);
                            VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                            VIDEO::vga.print("      " "          ");
                            if (FileUtils::fileTypes[ftype].fileSearch != "") {
                                real_rows = ndirs + elements + 2; // Add 2 for title and status bar
                                virtual_rows = (real_rows > mf_rows ? mf_rows : real_rows);
                                last_begin_row = last_focus = 0;
                                FileUtils::fileTypes[ftype].focus = 2;
                                FileUtils::fileTypes[ftype].begin_row = 2;
                                fd_Redraw(title, fdir, ftype, filexts);
                            }
                        }
                        click();
                    } else if (is_up(Menukey.vk)) {
                        if (FileUtils::fileTypes[ftype].focus == 2 && FileUtils::fileTypes[ftype].begin_row > 2) {
                            last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                            FileUtils::fileTypes[ftype].begin_row--;
                            fd_Redraw(title, fdir, ftype, filexts);
                        } else if (FileUtils::fileTypes[ftype].focus > 2) {
                            last_focus = FileUtils::fileTypes[ftype].focus;
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus--, IS_NORMAL, filexts);
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus, IS_FOCUSED, filexts);
                        }
                        click();
                    } else if (is_down(Menukey.vk)) {
                        if (FileUtils::fileTypes[ftype].focus == virtual_rows - 1 && FileUtils::fileTypes[ftype].begin_row + virtual_rows - 2 < real_rows) {
                            last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                            FileUtils::fileTypes[ftype].begin_row++;
                            fd_Redraw(title, fdir, ftype, filexts);
                        } else if (FileUtils::fileTypes[ftype].focus < virtual_rows - 1) {
                            last_focus = FileUtils::fileTypes[ftype].focus;
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus++, IS_NORMAL, filexts);
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus, IS_FOCUSED, filexts);
                        }
                        click();
                    } else if (Menukey.vk == fabgl::VK_PAGEUP) {
                        if (FileUtils::fileTypes[ftype].begin_row > virtual_rows) {
                            FileUtils::fileTypes[ftype].focus = 2;
                            FileUtils::fileTypes[ftype].begin_row -= virtual_rows - 2;
                        } else {
                            FileUtils::fileTypes[ftype].focus = 2;
                            FileUtils::fileTypes[ftype].begin_row = 2;
                        }
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                    } else if (Menukey.vk == fabgl::VK_PAGEDOWN) {
                        if (real_rows - FileUtils::fileTypes[ftype].begin_row  - virtual_rows > virtual_rows) {
                            FileUtils::fileTypes[ftype].focus = 2;
                            FileUtils::fileTypes[ftype].begin_row += virtual_rows - 2;
                        } else {
                            FileUtils::fileTypes[ftype].focus = virtual_rows - 1;
                            FileUtils::fileTypes[ftype].begin_row = real_rows - virtual_rows + 2;
                        }
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                    } else if (is_home(Menukey.vk)) {
                        last_focus = FileUtils::fileTypes[ftype].focus;
                        last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                        FileUtils::fileTypes[ftype].focus = 2;
                        FileUtils::fileTypes[ftype].begin_row = 2;
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                    } else if (Menukey.vk == fabgl::VK_END) {
                        last_focus = FileUtils::fileTypes[ftype].focus;
                        last_begin_row = FileUtils::fileTypes[ftype].begin_row;                        
                        FileUtils::fileTypes[ftype].focus = virtual_rows - 1;
                        FileUtils::fileTypes[ftype].begin_row = real_rows - virtual_rows + 2;
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                    } else if (is_backspace(Menukey.vk)) {
                        if (FileUtils::fileTypes[ftype].fdMode) {
                            if (FileUtils::fileTypes[ftype].fileSearch.length()) {
                                FileUtils::fileTypes[ftype].fileSearch.pop_back();
                                fdSearchRefresh = true;
                                click();
                            }
                        } else {
                            if (fdir != "/") {
                                fdir.pop_back();
                                fdir = fdir.substr(0,fdir.find_last_of("/") + 1);
                                FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;
                                click();
                                break;
                            }       
                        }                  
                    } else if (is_enter_fd(Menukey.vk)) {
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        if (filedir[0] == ASCII_SPC) {
                            if (filedir[1] == ASCII_SPC) {
                                fdir.pop_back();
                                fdir = fdir.substr(0,fdir.find_last_of("/") + 1);
                            } else {
                                filedir.erase(0,1);
                                trim(filedir);
                                fdir = fdir + filedir + "/";
                            }
                            FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;
                            break;
                        } else {
                            if (menu_saverect) {
                                // Restore backbuffer data
                                VIDEO::SaveRect.restore_last();
                                menu_saverect = false;                                
                            }
                            rtrim(filedir);
                            click();
                            filenames.close();
                            return (is_return(Menukey.vk) ? "R" : "S") + filedir;
                        }
                    } else if (is_back(Menukey.vk)) {
                        // Restore backbuffer data
                        if (menu_saverect) {
                            VIDEO::SaveRect.restore_last();
                            menu_saverect = false;
                        }
                        if (FileUtils::fileTypes[ftype].fdMode) {
                            if (FileUtils::fileTypes[ftype].fileSearch.length()) {
                                FileUtils::fileTypes[ftype].fileSearch.pop_back();
                                fdSearchRefresh = true;
                            }
                        } else {
                            if (fdir != "/") {
                                fdir.pop_back();
                                fdir = fdir.substr(0,fdir.find_last_of("/") + 1);
                                FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;
                            }       
                        }                  
                        click();
                        filenames.close();
                        return "";
                    }
                }
            } else {
                if (timeStartScroll < 200) timeStartScroll++;
            }

            // TO DO: SCROLL FOCUSED LINE IF SIGNALED
            if (timeStartScroll == 200) {
                timeScroll++;
                if (timeScroll == 50) {  
                    fdScrollPos++;
                    fd_PrintRow(FileUtils::fileTypes[ftype].focus, IS_FOCUSED, filexts);
                    timeScroll = 0;
                }
            }

            if (FileUtils::fileTypes[ftype].fdMode) {
                if ((++fdCursorFlash & 0xf) == 0) {
                    menuAt(mfrows + (Config::aspect_16_9 ? 0 : 1), 1);
                    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                    VIDEO::vga.print(Config::lang ? "Busq: " : "Find: ");
                    VIDEO::vga.print(FileUtils::fileTypes[ftype].fileSearch.c_str());
                    if (fdCursorFlash > 63) {
                        VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(7, 1));
                        if (fdCursorFlash == 128) fdCursorFlash = 0;
                    }
                    VIDEO::vga.print("K");
                    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                    VIDEO::vga.print(std::string(10 - FileUtils::fileTypes[ftype].fileSearch.size(), ' ').c_str());
                }
                if (fdSearchRefresh) {
                    // Recalc items number
                    unsigned int foundcount = 0;
                    fdSearchElements = 0;
                    size_t pos = 0;
                    char buf[128];
                    char upperbuf[128];
                    string search = FileUtils::fileTypes[ftype].fileSearch;
                    std::transform(search.begin(), search.end(), search.begin(), ::toupper);
                    while(pos < filenames.size()) {
                        std::string s = filenames[pos++];
                        strncpy(buf, s.c_str(), sizeof(buf));
                        if (buf[0] == ASCII_SPC) {
                            foundcount++;
                        } else {
                            for(int i = 0; i < strlen(buf); ++i) {
                                upperbuf[i] = toupper(buf[i]);
                            }
                            char *pch = strstr(upperbuf, search.c_str());
                            if (pch != NULL) {
                                foundcount++;
                                fdSearchElements++;
                            }
                        }
                    }
                    if (foundcount) {
                        // Redraw rows
                        real_rows = foundcount + 2; // Add 2 for title and status bar
                        virtual_rows = (real_rows > mf_rows ? mf_rows : real_rows);
                        last_begin_row = last_focus = 0;
                        FileUtils::fileTypes[ftype].focus = 2;
                        FileUtils::fileTypes[ftype].begin_row = 2;
                        fd_Redraw(title, fdir, ftype, filexts);
                    }
                    fdSearchRefresh = false;
                }
            }
            sleep_ms(5);
        }
        filenames.close();
    }
    filenames.close();
}

// Redraw inside rows
void OSD::fd_Redraw(string title, string fdir, uint8_t ftype, const vector<string>& filexts) {
    if ((FileUtils::fileTypes[ftype].focus != last_focus) || (FileUtils::fileTypes[ftype].begin_row != last_begin_row)) {
        // printf("fd_Redraw\n");
        // Read bunch of rows
        menu = title + "\n" + ( fdir.length() == 1 ? fdir : fdir.substr(0,fdir.length()-1)) + "\n";
        char buf[128];
        if (FileUtils::fileTypes[ftype].fdMode == 0 || FileUtils::fileTypes[ftype].fileSearch == "") {
            int pos = FileUtils::fileTypes[ftype].begin_row - 2;
            for (int i = 2; i < virtual_rows; i++) {
                if (pos >= filenames.size()) break;
                strncpy(buf, filenames[pos++].c_str(), 128);
                menu += buf;
                menu += '\n';
            }
        } else {
            int pos = 0;
            int i = 2;
            int count = 2;
            string search = FileUtils::fileTypes[ftype].fileSearch;
            std::transform(search.begin(), search.end(), search.begin(), ::toupper);
            char upperbuf[128];
            while (1) {
                if (pos >= filenames.size()) break;
                strncpy(buf, filenames[pos++].c_str(), 128);
                if (buf[0] == ASCII_SPC) {
                    if (i >= FileUtils::fileTypes[ftype].begin_row) {
                        menu += buf;
                        menu += '\n';
                        if (++count == virtual_rows) break;                        
                    }
                    i++;
                } else {
                    for(int i=0;i<strlen(buf);i++) upperbuf[i] = toupper(buf[i]);
                    char *pch = strstr(upperbuf, search.c_str());
                    if (pch != NULL) {
                        if (i >= FileUtils::fileTypes[ftype].begin_row) {
                            menu += buf;
                            menu += '\n';
                            if (++count == virtual_rows) break;                        
                        }
                        i++;
                    }
                }
            }
        }
        fd_PrintRow(1, IS_INFO, filexts); // Print status bar
        uint8_t row = 2;
        for (; row < virtual_rows; row++) {
            if (row == FileUtils::fileTypes[ftype].focus) {
                fd_PrintRow(row, IS_FOCUSED, filexts);
            } else {
                fd_PrintRow(row, IS_NORMAL, filexts);
            }
        }
        if (real_rows > virtual_rows) {        
            menuScrollBar(FileUtils::fileTypes[ftype].begin_row);
        } else {
            for (; row < mf_rows; row++) {
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                menuAt(row, 0);
                VIDEO::vga.print(std::string(cols, ' ').c_str());
            }
        }
        last_focus = FileUtils::fileTypes[ftype].focus;
        last_begin_row = FileUtils::fileTypes[ftype].begin_row;
    }

}

// Print a virtual row
void OSD::fd_PrintRow(uint8_t virtual_row_num, uint8_t line_type, const vector<string>& filexts) {
    
    uint8_t margin;

    string line = rowGet(menu, virtual_row_num);
    
    bool isDir = (line[0] == ASCII_SPC);
    bool isExc = false;
    if (!isDir) {
        size_t fpos = line.find_last_of(".");
        if (fpos != string::npos) {
            string sbstr = line.substr(fpos);
            for (auto it = filexts.begin(); it != filexts.end(); ++it) {
                if (sbstr == *it) {
                    isExc = true;
                    break;
                }
            }
        }
    }

    trim(line);

    switch (line_type) {
    case IS_TITLE:
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
        margin = 2;
        break;
    case IS_INFO:
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
        margin = (real_rows > virtual_rows ? 3 : 2);
        break;
    case IS_FOCUSED:
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
        margin = (real_rows > virtual_rows ? 3 : 2);
        break;
    default:
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        margin = (real_rows > virtual_rows ? 3 : 2);
    }

    menuAt(virtual_row_num, 0);

    VIDEO::vga.print(" ");

    if (isDir || isExc) {

        // Directory
        if (line.length() <= cols - margin - 6)
            line = line + std::string(cols - margin - line.length(), ' ');
        else
            if (line_type == IS_FOCUSED) {
                line = line.substr(fdScrollPos);
                if (line.length() <= cols - margin - 6) {
                    fdScrollPos = -1;
                    timeStartScroll = 0; 
                }
            }

        line = line.substr(0, cols - margin - 6) + (isExc ? "   * " : " <DIR>");

    } else {

        if (line.length() <= cols - margin) {
            line = line + std::string(cols - margin - line.length(), ' ');
            line = line.substr(0, cols - margin);
        } else {
            if (line_type == IS_INFO) {
                // printf("%s %d\n",line.c_str(),line.length() - (cols - margin));
                line = ".." + line.substr(line.length() - (cols - margin) + 2);
                // printf("%s\n",line.c_str());                
            } else {
                if (line_type == IS_FOCUSED) {
                    line = line.substr(fdScrollPos);
                    if (line.length() <= cols - margin) {
                        fdScrollPos = -1;
                        timeStartScroll = 0;                    
                    }
                }                   
                line = line.substr(0, cols - margin);
            }
        }

    }
    
    VIDEO::vga.print(line.c_str());

    VIDEO::vga.print(" ");
}
