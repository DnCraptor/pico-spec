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
#include "wd1793.h"
#include "ZipExtract.h"
#include "FileInfo.h"

#include "ff.h"

#include "Debug.h"
#include "PinSerialData_595.h"

extern Font Font6x8;

// Sort version: bump to invalidate cached .idx files when sort order changes
#define SORT_VERSION 1

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
        size_t res = SORT_VERSION;
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
    inline int cmp(const std::string& s1, const std::string& s2) {
        // Case-insensitive compare; DIR_MARKER (0x01) stays lowest so dirs sort first
        size_t len = s1.size() < s2.size() ? s1.size() : s2.size();
        for (size_t i = 0; i < len; i++) {
            int c1 = (uint8_t)s1[i] == DIR_MARKER ? s1[i] : toupper((uint8_t)s1[i]);
            int c2 = (uint8_t)s2[i] == DIR_MARKER ? s2[i] : toupper((uint8_t)s2[i]);
            if (c1 != c2) return c1 - c2;
        }
        return (int)s1.size() - (int)s2.size();
    }
    inline int cmp(size_t i1, size_t i2) {
        return cmp(get(i1), get(i2));
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

// Name to navigate to after directory rescan (e.g. after create/delete)
static string fd_goto_name;

// Stack for saving file dialog position when entering subdirectories
static constexpr int FD_POS_STACK_MAX = 16;
static struct { int begin_row; int focus; } fd_pos_stack[FD_POS_STACK_MAX];
static int fd_pos_stack_top = 0;

static void fd_pos_push(int begin_row, int focus) {
    if (fd_pos_stack_top < FD_POS_STACK_MAX) {
        fd_pos_stack[fd_pos_stack_top++] = {begin_row, focus};
    }
}

static bool fd_pos_pop(int &begin_row, int &focus) {
    if (fd_pos_stack_top > 0) {
        auto &e = fd_pos_stack[--fd_pos_stack_top];
        begin_row = e.begin_row;
        focus = e.focus;
        return true;
    }
    return false;
}

unsigned int OSD::elements;
unsigned int OSD::fdSearchElements;
unsigned int OSD::ndirs;
int8_t OSD::fdScrollPos;
int OSD::timeStartScroll;
int OSD::timeScroll;
uint8_t OSD::fdCursorFlash;
bool OSD::fdSearchRefresh;

// File dialog layout constants
// Total dialog: FDLG_LIST_COLS + 1 (sep) + FDLG_SIDE_COLS = FDLG_TOTAL_COLS
static const uint8_t FDLG_LIST_COLS = 36;
static const uint8_t FDLG_SIDE_COLS = 11;
static const uint8_t FDLG_TOTAL_COLS = FDLG_LIST_COLS + FDLG_SIDE_COLS;
// Active list column width — set per-dialog (FDLG_LIST_COLS for DISK_ALLFILE, cols otherwise)
static uint8_t fd_list_cols = 0;

// Sidebar key labels — 9 chars each (padded), displayed in the right panel
// activeKey: VK of currently active action (0=none), shown highlighted
static const struct { fabgl::VirtualKey vk; const char *label; } fd_sidebar_items[] = {
    { fabgl::VK_F1, "F1 Info   " },
    { fabgl::VK_F3, "F3 Find   " },
    { fabgl::VK_F4, "F4 Unzip  " },
    { fabgl::VK_F6, "F6 Rename " },
    { fabgl::VK_F7, "F7 MkDir  " },
    { fabgl::VK_F8, "F8 Del    " },
    { fabgl::VK_F9, "F9 TRD    " },
};
static const int FDLG_SIDE_ITEMS = 7;

// Draw the sidebar panel (right of vertical separator).
// activeKey: if nonzero, that item is highlighted (action in progress).
static void fd_DrawSidebar(int ox, int oy, int mf_rows, fabgl::VirtualKey activeKey = fabgl::VK_NONE) {
    VIDEO::vga.setFont(Font6x8);
    int sx = ox + 1 + (FDLG_LIST_COLS + 1) * OSD_FONT_W; // pixel x of sidebar
    // Vertical separator — footer colour
    for (int row = 1; row <= mf_rows; row++) {
        VIDEO::vga.setCursor(ox + 1 + FDLG_LIST_COLS * OSD_FONT_W, oy + 1 + row * OSD_FONT_H);
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
        VIDEO::vga.print("|");
    }
    // Sidebar items: same colour as footer (white on blue); active item inverted
    for (int i = 0; i < FDLG_SIDE_ITEMS; i++) {
        int row = 1 + i;
        if (row > mf_rows) break;
        bool active = (fd_sidebar_items[i].vk == activeKey);
        VIDEO::vga.setCursor(sx, oy + 1 + row * OSD_FONT_H);
        if (active)
            VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(7, 1)); // inverted when active
        else
            VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0)); // white on blue (footer)
        VIDEO::vga.print(fd_sidebar_items[i].label);
    }
    // Fill remaining sidebar rows with footer background
    for (int row = 1 + FDLG_SIDE_ITEMS; row <= mf_rows; row++) {
        VIDEO::vga.setCursor(sx, oy + 1 + row * OSD_FONT_H);
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
        VIDEO::vga.print("          ");
    }
}

// Show a label in the footer and run inlineTextEdit for DISK_ALLFILE dialogs.
// Returns the entered string (or "\x1B" for Escape, "" for empty+Enter).
// Restores the footer (status bar background) after editing.
static string fd_FooterTextEdit(int ox, int oy, int mfrows_val, int cols_val, const char *label, const string &initial) {
    int footerRow = mfrows_val;
    VIDEO::vga.setFont(Font6x8);
    VIDEO::vga.setCursor(ox + 1, oy + 1 + footerRow * OSD_FONT_H);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0)); // white on blue (footer)
    int labelLen = strlen(label);
    VIDEO::vga.print(label);
    int inputCols = cols_val - labelLen - 1;
    if (inputCols < 4) inputCols = 4;
    // Pad rest of footer
    string padded(inputCols, ' ');
    VIDEO::vga.print(padded.c_str());
    // Input field starts right after label
    int ex = ox + 1 + (1 + labelLen) * OSD_FONT_W;
    int ey = oy + 1 + footerRow * OSD_FONT_H;
    string result = OSD::inlineTextEdit(ex, ey, inputCols, initial);
    // Restore footer (status bar background)
    VIDEO::vga.setCursor(ox + 1, oy + 1 + footerRow * OSD_FONT_H);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
    VIDEO::vga.print(string(cols_val, ' ').c_str());
    return result;
}

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
string OSD::fileDialog(string &fdir, const string& title, uint8_t ftype, uint8_t mfcols, uint8_t mfrows) {
    if (Config::audio_driver == 3) send_to_595(LOW(AY_Enable));
    fd_pos_stack_top = 0;
    fd_goto_name.clear();
    // Position
    if (menu_level == 0) {
        x = (Config::aspect_16_9 ? 24 : 4);
        y = (Config::aspect_16_9 ? 4 : 4);
    } else {
        x = (Config::aspect_16_9 ? 24 : 8) + (60 * menu_level);
        y = 8 + (16 * menu_level);
    }

    // Columns and Rows
    // DISK_ALLFILE uses a sidebar layout: total cols = list + sep + sidebar
    cols = (ftype == DISK_ALLFILE) ? FDLG_TOTAL_COLS : mfcols;
    fd_list_cols = (ftype == DISK_ALLFILE) ? FDLG_LIST_COLS : cols;
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

    // Clamp position to screen boundaries
    if (x + w > scrW) x = scrW - w;
    if (y + h > scrH) y = scrH - h;

    DIR f_dir;
    bool res = f_opendir(&f_dir, fdir.c_str()) == FR_OK;
    if (!res) {
        fdir = "/";
    } else {
        f_closedir(&f_dir);
    }

    menu = title + "\n" + fdir + "\n";
    WindowDraw(); // Draw menu outline
    if (ftype == DISK_ALLFILE)
        fd_DrawSidebar(x, y, mf_rows);
    fd_PrintRow(1, IS_INFO, filexts);    // Path

    // Draw blank rows (list area only for DISK_ALLFILE)
    uint8_t listCols = (ftype == DISK_ALLFILE) ? FDLG_LIST_COLS : cols;
    uint8_t row = 2;
    for (; row < mf_rows; row++) {
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        menuAt(row, 0);
        VIDEO::vga.print(std::string(listCols, ' ').c_str());
    }

    // Print status bar (full width)
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
            size_t crc = SORT_VERSION;
            if (fdir.size() > 1) {
                ++ndirs;
                crc += ::crc(string(2, DIR_MARKER) + "..");
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
                            crc += ::crc(char(DIR_MARKER) + fname);
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
                    filenames.push(string(2, DIR_MARKER) + "..");
                }
                if (f_opendir(&f_dir, fdir.c_str()) != FR_OK) break;
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
                                filenames.push(char(DIR_MARKER) + fname);
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

        // Navigate to a specific file/dir after rescan
        if (!fd_goto_name.empty()) {
            int cnt = filenames.size();
            int found = -1;
            for (int i = 0; i < cnt; i++) {
                string s = filenames[i];
                if (s == fd_goto_name) { found = i; break; }
                // For delete: fd_goto_name may not exist, find next item at same index
            }
            if (found < 0) {
                // Deleted file: position at same index (or last item)
                found = 0;
                for (int i = 0; i < cnt; i++) {
                    string s = filenames[i];
                    if (s >= fd_goto_name) { found = i; break; }
                    found = i;
                }
            }
            // Convert filenames index to row position (rows start at 2, dirs first)
            int row = found + 2;
            if (real_rows > mf_rows) {
                int max_begin = real_rows - (mf_rows - 2);
                if (row < mf_rows) {
                    FileUtils::fileTypes[ftype].begin_row = 2;
                    FileUtils::fileTypes[ftype].focus = row;
                } else {
                    FileUtils::fileTypes[ftype].begin_row = row - 2;
                    if (FileUtils::fileTypes[ftype].begin_row > max_begin)
                        FileUtils::fileTypes[ftype].begin_row = max_begin;
                    FileUtils::fileTypes[ftype].focus = row - FileUtils::fileTypes[ftype].begin_row + 2;
                }
            } else {
                FileUtils::fileTypes[ftype].begin_row = 2;
                FileUtils::fileTypes[ftype].focus = row < real_rows ? row : real_rows - 1;
            }
            fd_goto_name.clear();
        }

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
                // Redraw search field when a key is pressed (fdCursorFlash reset separately)
                if (FileUtils::fileTypes[ftype].fdMode)
                    fdCursorFlash = 7; // force immediate redraw on next idle tick
                if (ESPectrum::readKbd(&Menukey)) {
                    if (!Menukey.down) continue;
                    // F4 on a ZIP file = extract ZIP to current folder
                    if (Menukey.vk == fabgl::VK_F4 && ftype == DISK_ALLFILE) {
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        if (filedir[0] != DIR_MARKER && FileUtils::hasZIPextension(filedir)) {
                            if (menu_saverect) {
                                VIDEO::SaveRect.restore_last();
                                menu_saverect = false;
                            }
                            rtrim(filedir);
                            click();
                            filenames.close();
                            string(). swap(menu); // release menu heap buffer
                            if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable));
                            return "X" + filedir; // X prefix = extract ZIP
                        }
                        click();
                        continue;
                    }
                    // F7 = create new directory
                    if (Menukey.vk == fabgl::VK_F7 && ftype == DISK_ALLFILE) {
                        fd_DrawSidebar(x, y, mf_rows, fabgl::VK_F7);
                        string newname = fd_FooterTextEdit(x, y, mfrows + (Config::aspect_16_9 ? 0 : 1), cols, "MkDir: ", "");
                        if (newname != "\x1B" && !newname.empty()) {
                            string fullpath = fdir + newname;
                            f_mkdir(fullpath.c_str());
                            fd_goto_name = char(DIR_MARKER) + newname;
                            click();
                            break;
                        }
                        fd_DrawSidebar(x, y, mf_rows);
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                        continue;
                    }
                    // F6 = rename file or folder
                    if (Menukey.vk == fabgl::VK_F6 && ftype == DISK_ALLFILE) {
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        rtrim(filedir);
                        bool isDir = filedir[0] == DIR_MARKER;
                        if (isDir) filedir = filedir.substr(1);
                        if (!filedir.empty() && !(isDir && filedir == "..")) {
                            fd_DrawSidebar(x, y, mf_rows, fabgl::VK_F6);
                            string newname = fd_FooterTextEdit(x, y, mfrows + (Config::aspect_16_9 ? 0 : 1), cols, "Rename: ", filedir);
                            if (newname != "\x1B" && !newname.empty() && newname != filedir) {
                                string oldpath = fdir + filedir;
                                string newpath = fdir + newname;
                                f_rename(oldpath.c_str(), newpath.c_str());
                                fd_goto_name = isDir ? char(DIR_MARKER) + newname : newname;
                                click();
                                break;
                            }
                            fd_DrawSidebar(x, y, mf_rows);
                            fd_Redraw(title, fdir, ftype, filexts);
                        }
                        click();
                        continue;
                    }
                    // F9 = create new empty TRD disk image
                    if (Menukey.vk == fabgl::VK_F9 && ftype == DISK_ALLFILE) {
                        fd_DrawSidebar(x, y, mf_rows, fabgl::VK_F9);
                        string newname = fd_FooterTextEdit(x, y, mfrows + (Config::aspect_16_9 ? 0 : 1), cols, "TRD: ", "");
                        if (newname != "\x1B" && !newname.empty()) {
                            string fname = newname;
                            if (fname.size() < 4 || fname.substr(fname.size() - 4) != ".trd")
                                fname += ".trd";
                            string fullpath = fdir + fname;
                            OSD::progressDialog(OSD_FILE_CREATING_TRD[Config::lang], fname, 0, 0);
                            rvmWD1793CreateEmptyTRD(fullpath.c_str());
                            OSD::progressDialog("", "", 100, 1);
                            OSD::progressDialog("", "", 0, 2);
                            fd_goto_name = fname;
                            click();
                            break;
                        }
                        fd_DrawSidebar(x, y, mf_rows);
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                        continue;
                    }
                    // F1 = view file info
                    if (Menukey.vk == fabgl::VK_F1 && ftype == DISK_ALLFILE) {
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        if (filedir[0] != DIR_MARKER) {
                            rtrim(filedir);
                            string fullpath = fdir + filedir;
                            if (FileUtils::hasZIPextension(filedir))
                                ZipExtract::viewInfo(fullpath);
                            else
                                FileInfo::viewInfo(fullpath);
                            fd_Redraw(title, fdir, ftype, filexts);
                        }
                        click();
                        continue;
                    }
                    // F8 = delete file or folder with confirmation
                    if ((Menukey.vk == fabgl::VK_F8 || Menukey.vk == fabgl::VK_DELETE)
                        && ftype == DISK_ALLFILE) {
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        rtrim(filedir);
                        bool isDir = filedir[0] == DIR_MARKER;
                        if (isDir) filedir = filedir.substr(1); // strip DIR_MARKER
                        // Don't allow deleting ".." entry
                        if (!filedir.empty() && !(isDir && filedir == "..")) {
                            string fullpath = fdir + filedir;
                            const char *dlgTitle = isDir
                                ? OSD_FILE_DELETE_DIR_TITLE[Config::lang]
                                : OSD_FILE_DELETE_TITLE[Config::lang];
                            if (OSD::msgDialog(dlgTitle, filedir) == DLG_YES) {
                                if (isDir) {
                                    OSD::progressDialog(OSD_FILE_DELETING[Config::lang], filedir, 0, 0);
                                    FileUtils::deleteDirRecursive(fullpath.c_str());
                                    OSD::progressDialog("", "", 100, 1);
                                    OSD::progressDialog("", "", 0, 2);
                                    fd_goto_name = char(DIR_MARKER) + filedir;
                                } else {
                                    f_unlink(fullpath.c_str());
                                    fd_goto_name = filedir;
                                }
                                click();
                                break; // re-scan directory
                            }
                            // Redraw after dialog
                            if (ftype == DISK_ALLFILE) fd_DrawSidebar(x, y, mf_rows);
                            fd_Redraw(title, fdir, ftype, filexts);
                        }
                        continue;
                    }
                    // Search first ocurrence of letter if we're not on that letter yet
                    if (((Menukey.vk >= fabgl::VK_a) && (Menukey.vk <= fabgl::VK_Z)) || ((Menukey.vk >= fabgl::VK_0) && (Menukey.vk <= fabgl::VK_9))) {
                        if (FileUtils::fileTypes[ftype].fdMode && Menukey.ASCII >= 32 && Menukey.ASCII < 127) {
                            if (FileUtils::fileTypes[ftype].fileSearch.size() < 10) {
                                FileUtils::fileTypes[ftype].fileSearch += (char)toupper(Menukey.ASCII);
                                fdSearchRefresh = true;
                                click();
                            }
                            continue;
                        }
                        int fsearch;
                        if (Menukey.vk<=fabgl::VK_9)
                            fsearch = Menukey.vk + 46;
                        else if (Menukey.vk<=fabgl::VK_z)
                            fsearch = Menukey.vk + 75;
                        else if (Menukey.vk<=fabgl::VK_Z)
                            fsearch = Menukey.vk + 17;
                        fsearch = toupper(fsearch);
                        {
                            // Current file index in filenames
                            int cur_idx = (FileUtils::fileTypes[ftype].begin_row - 2) + (FileUtils::fileTypes[ftype].focus - 2);
                            // Get first real char of current entry (skip DIR_MARKER)
                            std::string cur_s = (cur_idx >= 0 && cur_idx < (int)filenames.size()) ? filenames[cur_idx] : "";
                            uint8_t letra = 0;
                            for (size_t ci = 0; ci < cur_s.size(); ci++) {
                                if ((uint8_t)cur_s[ci] != DIR_MARKER) { letra = toupper(cur_s[ci]); break; }
                            }
                            // Start search from next entry if already on matching letter (cycle through)
                            int start = (letra == fsearch) ? cur_idx + 1 : 0;
                            int cnt = -1;
                            int total = (int)filenames.size();
                            // Search from start to end, then wrap around from 0 to start
                            for (int i = 0; i < total; i++) {
                                int idx = (start + i) % total;
                                std::string s = filenames[idx];
                                // Skip DIR_MARKER prefix to get real first char
                                for (size_t ci = 0; ci < s.size(); ci++) {
                                    if ((uint8_t)s[ci] != DIR_MARKER) {
                                        if (toupper(s[ci]) == fsearch) cnt = idx;
                                        break;
                                    }
                                }
                                if (cnt >= 0) break;
                            }
                            if (cnt >= 0 && cnt != cur_idx) {
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
                    } else if (Menukey.vk == fabgl::VK_F3) {
                        FileUtils::fileTypes[ftype].fdMode ^= 1;
                        if (FileUtils::fileTypes[ftype].fdMode) {
                            fdCursorFlash = 7; // next tick (++&0x7==0) draws immediately
                            // Entering search mode — highlight F3 in sidebar, clear footer
                            if (ftype == DISK_ALLFILE) fd_DrawSidebar(x, y, mf_rows, fabgl::VK_F3);
                            menuAt(mfrows + (Config::aspect_16_9 ? 0 : 1), 1);
                            VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                            VIDEO::vga.print(std::string(cols - 2, ' ').c_str());
                        } else {
                            // Leaving search mode — restore sidebar and full list
                            if (ftype == DISK_ALLFILE) fd_DrawSidebar(x, y, mf_rows);
                            menuAt(mfrows + (Config::aspect_16_9 ? 0 : 1), 1);
                            VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                            VIDEO::vga.print(std::string(cols - 2, ' ').c_str());
                            if (FileUtils::fileTypes[ftype].fileSearch != "") {
                                FileUtils::fileTypes[ftype].fileSearch = "";
                                real_rows = ndirs + elements + 2;
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
                                if (!fd_pos_pop(FileUtils::fileTypes[ftype].begin_row,
                                                FileUtils::fileTypes[ftype].focus))
                                    FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;
                                click();
                                break;
                            }
                        }
                    } else if (is_enter_fd(Menukey.vk)) {
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        if (filedir[0] == DIR_MARKER) {
                            if (filedir[1] == DIR_MARKER) {
                                // Going up to parent dir — restore saved position
                                fdir.pop_back();
                                fdir = fdir.substr(0,fdir.find_last_of("/") + 1);
                                if (!fd_pos_pop(FileUtils::fileTypes[ftype].begin_row,
                                                FileUtils::fileTypes[ftype].focus))
                                    FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;
                            } else {
                                // Entering subdirectory — save current position
                                fd_pos_push(FileUtils::fileTypes[ftype].begin_row,
                                            FileUtils::fileTypes[ftype].focus);
                                filedir.erase(0,1);
                                fdir = fdir + filedir + "/";
                                FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;
                            }
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
                            string().swap(menu); // release menu heap buffer
                            if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable));
                            return (is_return(Menukey.vk) ? "R" : "S") + filedir;
                        }
                    } else if (is_back(Menukey.vk)) {
                        // Restore backbuffer data
                        if (menu_saverect) {
                            VIDEO::SaveRect.restore_last();
                            menu_saverect = false;
                        }
                        // Keep current dir and position so next F5 reopens here
                        click();
                        filenames.close();
                        string().swap(menu); // release menu heap buffer
                        if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable));
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
                        if (buf[0] == DIR_MARKER) {
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
                // Blink cursor in search field — redraw every tick, cursor blinks via fdCursorFlash
                if ((++fdCursorFlash & 0x7) == 0) {
                    const char *label = Config::lang ? "Busq: " : "Find: ";
                    int labelLen = strlen(label);
                    int footerRow = mfrows + (Config::aspect_16_9 ? 0 : 1);
                    menuAt(footerRow, 1);
                    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                    VIDEO::vga.print(label);
                    const string &srch = FileUtils::fileTypes[ftype].fileSearch;
                    // Field width: fill footer up to the counter area (cols-2 - labelLen chars)
                    int fieldLen = cols - 1 - labelLen;
                    int cur = (int)srch.size();
                    bool cursorOn = (fdCursorFlash & 0x20) == 0; // ~160ms on/off at 5ms*8 ticks
                    for (int p = 0; p < fieldLen; p++) {
                        char ch = (p < cur) ? srch[p] : ' ';
                        bool isCursor = (p == cur);
                        if (isCursor && cursorOn)
                            VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 1));
                        else
                            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                        char s[2] = { ch, 0 };
                        VIDEO::vga.print(s);
                    }
                    if (fdCursorFlash >= 128) fdCursorFlash = 0;
                }
            }
            sleep_ms(5);
        }
        filenames.close();
    }
    filenames.close();
    string().swap(menu); // release menu heap buffer
    if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable));
    return "";
}

// Redraw inside rows
void OSD::fd_Redraw(const string& title, const string& fdir, uint8_t ftype, const vector<string>& filexts) {
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
                if (buf[0] == DIR_MARKER) {
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
            // For sidebar layout, scrollbar goes inside the list area, not full cols
            uint8_t saved_cols = cols;
            if (fd_list_cols != cols) cols = fd_list_cols;
            menuScrollBar(FileUtils::fileTypes[ftype].begin_row);
            cols = saved_cols;
            if (fd_list_cols != saved_cols) fd_DrawSidebar(x, y, mf_rows);
        } else {
            for (; row < mf_rows; row++) {
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                menuAt(row, 0);
                VIDEO::vga.print(std::string(fd_list_cols, ' ').c_str());
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

    if (line.empty() || line == "<Unknown menu row>") {
        // Row beyond end of file list — print blank line
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        menuAt(virtual_row_num, 0);
        VIDEO::vga.print(std::string(fd_list_cols, ' ').c_str());
        return;
    }

    bool isDir = (line[0] == DIR_MARKER);
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

    // Remove DIR_MARKER prefix before display, preserve spaces in filenames
    while (!line.empty() && line[0] == (char)DIR_MARKER) line.erase(0, 1);
    rtrim(line);

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

    uint8_t lc = fd_list_cols; // effective list width for this dialog
    if (isDir || isExc) {

        // Directory
        if (line.length() <= (size_t)(lc - margin - 6))
            line = line + std::string(lc - margin - line.length(), ' ');
        else
            if (line_type == IS_FOCUSED) {
                line = line.substr(fdScrollPos);
                if (line.length() <= (size_t)(lc - margin - 6)) {
                    fdScrollPos = -1;
                    timeStartScroll = 0;
                }
            }

        line = line.substr(0, lc - margin - 6) + (isExc ? "   * " : " <DIR>");

    } else {

        if (line.length() <= (size_t)(lc - margin)) {
            line = line + std::string(lc - margin - line.length(), ' ');
            line = line.substr(0, lc - margin);
        } else {
            if (line_type == IS_INFO) {
                line = ".." + line.substr(line.length() - (lc - margin) + 2);
            } else {
                if (line_type == IS_FOCUSED) {
                    line = line.substr(fdScrollPos);
                    if (line.length() <= (size_t)(lc - margin)) {
                        fdScrollPos = -1;
                        timeStartScroll = 0;
                    }
                }
                line = line.substr(0, lc - margin);
            }
        }

    }
    
    VIDEO::vga.print(line.c_str());

    VIDEO::vga.print(" ");
}
