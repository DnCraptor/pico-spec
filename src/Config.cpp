#include "Config.h"
#include "MemESP.h"
#include "roms.h"
#include "FileUtils.h"
#include "ESPectrum.h"
#include "fabutils.h"
#include "messages.h"
#include "OSDMain.h"

string   Config::arch = "48K";
string   Config::romSet = "48K";
string   Config::romSet48 = "48K";
string   Config::romSet128 = "128K";
string   Config::romSetPent = "128Kp";
string   Config::pref_arch = "48K";
string   Config::pref_romSet_48 = "48K";
string   Config::pref_romSet_128 = "128K";
string   Config::pref_romSetPent = "128Kp";
string   Config::ram_file = NO_RAM_FILE;
string   Config::last_ram_file = NO_RAM_FILE;

bool     Config::slog_on = false;
const bool     Config::aspect_16_9 = false;
uint8_t  Config::videomode = 0; // 0 -> SAFE VGA, 1 -> 50HZ VGA, 2 -> 50HZ CRT
///uint8_t  Config::esp32rev = 0;
uint8_t  Config::lang = 0;
bool     Config::AY48 = true;
bool     Config::Issue2 = true;
bool     Config::flashload = true;
bool     Config::tape_player = false; // Tape player mode
bool     Config::tape_timing_rg = false; // Rodolfo Guerra ROMs tape timings

uint8_t  Config::joystick1 = JOY_SINCLAIR1;
uint8_t  Config::joystick2 = JOY_SINCLAIR2;
uint16_t Config::joydef[24] = { 
    fabgl::VK_6,
    fabgl::VK_7,
    fabgl::VK_9,
    fabgl::VK_8,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_0,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_1,
    fabgl::VK_2,
    fabgl::VK_4,
    fabgl::VK_3,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_5,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE
};

uint8_t  Config::joyPS2 = JOY_KEMPSTON;
uint8_t  Config::AluTiming = 0;
uint8_t  Config::ps2_dev2 = 0; // Second port PS/2 device: 0 -> None, 1 -> PS/2 keyboard, 2 -> PS/2 Mouse (TO DO)
bool     Config::CursorAsJoy = false;
int8_t   Config::CenterH = 0;
int8_t   Config::CenterV = 0;

string   Config::SNA_Path = "/";
string   Config::TAP_Path = "/";
string   Config::DSK_Path = "/";

uint16_t Config::SNA_begin_row = 1;
uint16_t Config::SNA_focus = 1;
uint8_t  Config::SNA_fdMode = 0;
string   Config::SNA_fileSearch = "";

uint16_t Config::TAP_begin_row = 1;
uint16_t Config::TAP_focus = 1;
uint8_t  Config::TAP_fdMode = 0;
string   Config::TAP_fileSearch = "";

uint16_t Config::DSK_begin_row = 1;
uint16_t Config::DSK_focus = 1;
uint8_t  Config::DSK_fdMode = 0;
string   Config::DSK_fileSearch = "";

uint8_t Config::scanlines = 0;
uint8_t Config::render = 0;

bool     Config::TABasfire1 = false;
bool     Config::StartMsg = true;

void Config::requestMachine(string newArch, string newRomSet)
{
    arch = newArch;
    if (arch == "48K") {
        if (newRomSet=="") romSet = "48K"; else romSet = newRomSet;
        if (newRomSet=="") romSet48 = "48K"; else romSet48 = newRomSet;        
        if (romSet48 == "48K")
            MemESP::rom[0] = (uint8_t *) gb_rom_0_sinclair_48k;
        else if (romSet48 == "48Kes")
            MemESP::rom[0] = (uint8_t *) gb_rom_0_48k_es;
        else if (romSet48 == "48Kcs") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_48k_custom;
        }
    } else if (arch == "128K") {
        if (newRomSet=="") romSet = "128K"; else romSet = newRomSet;
        if (newRomSet=="") romSet128 = "128K"; else romSet128 = newRomSet;                
        if (romSet128 == "128K") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_sinclair_128k;
            MemESP::rom[1] = (uint8_t *) gb_rom_1_sinclair_128k;
        } else if (romSet128 == "128Kes") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_128k_es;
            MemESP::rom[1] = (uint8_t *) gb_rom_1_128k_es;
        } else if (romSet128 == "128Kcs") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_128k_custom;
            MemESP::rom[1] = (uint8_t *) gb_rom_0_128k_custom;
            MemESP::rom[1] += (16 << 10); /// 16392;
        } else if (romSet128 == "+2") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_plus2;
            MemESP::rom[1] = (uint8_t *) gb_rom_1_plus2;
        } else if (romSet128 == "+2es") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_plus2_es;
            MemESP::rom[1] = (uint8_t *) gb_rom_1_plus2_es;
        } else if (romSet128 == "ZX81+") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_s128_zx81;
            MemESP::rom[1] = (uint8_t *) gb_rom_1_sinclair_128k;
        }
    } else if (arch == "Pentagon") {
        if (newRomSet=="") romSet = "128Kp"; else romSet = newRomSet;
        if (romSetPent=="") romSetPent = "128Kp"; else romSetPent = newRomSet;                
        if (romSetPent == "128Kcs") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_128k_custom;
            MemESP::rom[1] = (uint8_t *) gb_rom_0_128k_custom;
            MemESP::rom[1] += (16 << 10); /// 16392;
        } else {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_pentagon_128k;
            MemESP::rom[1] = (uint8_t *) gb_rom_1_pentagon_128k;
        }
    }
    MemESP::rom[4] = (uint8_t *) gb_rom_4_trdos_503;
}

static bool nvs_get_str(FIL& handle, const char* key, string& v, const vector<string>& sts) {
    string k = key; k += '=';
    for(const string& s: sts) {
        if ( strncmp(k.c_str(), s.c_str(), k.size()) == 0 ) {
            if ( s.size() <= k.size() ) {
                return false;
            }
            v = s.c_str() + k.size();
            return true;
        }
    }
    return false;
}
static void nvs_get_b(FIL& handle, const char* key, bool& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(handle, key, t, sts)) {
        v = (t == "true");
    }
}
static void nvs_get_i8(FIL& handle, const char* key, int8_t& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(handle, key, t, sts)) {
        v = atoi(t.c_str());
    }
}
static void nvs_get_u8(FIL& handle, const char* key, uint8_t& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(handle, key, t, sts)) {
        v = atoi(t.c_str());
    }
}
static void nvs_get_u16(FIL& handle, const char* key, uint16_t& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(handle, key, t, sts)) {
        v = atoi(t.c_str());
    }
}

// Read config from FS
void Config::load() {
    // Open
    // printf("\n");
    // printf("Opening Non-Volatile Storage (NVS) handle... ");
    FIL handle;
    string nvs = MOUNT_POINT_SD STORAGE_NVS;
    FRESULT err = f_open(&handle, nvs.c_str(), FA_READ);
    if (err != FR_OK) {
///        OSD::osdCenteredMsg("Error opening file:\n" + nvs + "\n", LEVEL_ERROR, 5000);
        return;
    } else {
        vector<string> sts;
        UINT br;
        char c;
        string s;
        while(!f_eof(&handle)) {
            if (f_read(&handle, &c, 1, &br) != FR_OK) {
                f_close(&handle);
                return;
            }
            if (c == '\n') {
                sts.push_back(s);
                s.clear();
            } else {
                s += c;
            }
        }
        f_close(&handle);

        nvs_get_str(handle, "arch", arch, sts);
        nvs_get_str(handle, "romSet", romSet, sts);
        nvs_get_str(handle, "romSet48", romSet48, sts);
        nvs_get_str(handle, "romSet128", romSet128, sts);
        nvs_get_str(handle, "romSetPent", romSetPent, sts);
        nvs_get_str(handle, "pref_arch", pref_arch, sts);
        nvs_get_str(handle, "pref_romSet_48", pref_romSet_48, sts);
        nvs_get_str(handle, "pref_romSet_128", pref_romSet_128, sts);
        nvs_get_str(handle, "pref_romSetPent", pref_romSetPent, sts);
        nvs_get_str(handle, "ram", ram_file, sts);
///        nvs_get_str(handle, "slog", slog, sts);
///        nvs_get_str(handle, "sdstorage", FileUtils::MountPoint, sts);
///        nvs_get_b(handle, "asp169", aspect_16_9, sts);
        nvs_get_u8(handle, "videomode", Config::videomode, sts);
        nvs_get_b(handle, "AY48", AY48, sts);
        nvs_get_b(handle, "Issue2", Issue2, sts);
        nvs_get_b(handle, "flashload", flashload, sts);
        nvs_get_b(handle, "tape_player", tape_player, sts);
        nvs_get_b(handle, "tape_timing_rg", tape_timing_rg, sts);
        nvs_get_u8(handle, "joystick1", Config::joystick1, sts);
        nvs_get_u8(handle, "joystick2", Config::joystick2, sts);

        // Read joystick definition
        for (int n=0; n < 24; n++) {
            char joykey[9];
            sprintf(joykey,"joydef%02u",n);
            // printf("%s\n",joykey);
            nvs_get_u16(handle, "joystick2", Config::joydef[n], sts);
        }

        nvs_get_u8(handle, "joyPS2", Config::joyPS2, sts);
        nvs_get_u8(handle, "AluTiming", Config::AluTiming, sts);
        nvs_get_u8(handle, "PS2Dev2", Config::ps2_dev2, sts);
        nvs_get_b(handle, "CursorAsJoy", CursorAsJoy, sts);
        nvs_get_i8(handle, "CenterH", Config::CenterH, sts);
        nvs_get_i8(handle, "CenterV", Config::CenterV, sts);
        nvs_get_str(handle, "SNA_Path", SNA_Path, sts);
        nvs_get_str(handle, "TAP_Path", TAP_Path, sts);
        nvs_get_str(handle, "DSK_Path", DSK_Path, sts);
        nvs_get_u16(handle, "SNA_begin_row", Config::SNA_begin_row, sts);
        nvs_get_u16(handle, "TAP_begin_row", Config::TAP_begin_row, sts);
        nvs_get_u16(handle, "DSK_begin_row", Config::DSK_begin_row, sts);
        nvs_get_u16(handle, "SNA_focus", Config::SNA_focus, sts);
        nvs_get_u16(handle, "TAP_focus", Config::TAP_focus, sts);
        nvs_get_u16(handle, "DSK_focus", Config::DSK_focus, sts);
        nvs_get_u8(handle, "SNA_fdMode", Config::SNA_fdMode, sts);
        nvs_get_u8(handle, "TAP_fdMode", Config::TAP_fdMode, sts);
        nvs_get_u8(handle, "DSK_fdMode", Config::DSK_fdMode, sts);
        nvs_get_str(handle, "SNA_fileSearch", SNA_fileSearch, sts);
        nvs_get_str(handle, "TAP_fileSearch", TAP_fileSearch, sts);
        nvs_get_str(handle, "DSK_fileSearch", DSK_fileSearch, sts);
        nvs_get_u8(handle, "scanlines", Config::scanlines, sts);
        nvs_get_u8(handle, "render", Config::render, sts);
        nvs_get_b(handle, "TABasfire1", Config::TABasfire1, sts);
        nvs_get_b(handle, "StartMsg", Config::StartMsg, sts);
    }
}

void Config::save() {
    Config::save("all");
}

static void nvs_set_str(FIL& handle, const char* name, const char* val) {
    UINT btw;
    f_write(&handle, name, strlen(name), &btw);
    f_write(&handle, "=", 1, &btw);
    f_write(&handle, val, strlen(val), &btw);
    f_write(&handle, "\n", 1, &btw);
}
static void nvs_set_i8(FIL& handle, const char* name, int8_t val) {
    string v = to_string(val);
    nvs_set_str(handle, name, v.c_str());
}
static void nvs_set_u8(FIL& handle, const char* name, uint8_t val) {
    string v = to_string(val);
    nvs_set_str(handle, name, v.c_str());
}
static void nvs_set_u16(FIL& handle, const char* name, uint16_t val) {
    string v = to_string(val);
    nvs_set_str(handle, name, v.c_str());
}

// Dump actual config to FS
void Config::save(string value) {
    FIL handle;
    string nvs = MOUNT_POINT_SD STORAGE_NVS;
    FRESULT err = f_open(&handle, nvs.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (err != FR_OK) {
        OSD::osdCenteredMsg("Error opening file:\n" + nvs + "\n", LEVEL_ERROR, 5000);
    } else {
        nvs_set_str(handle,"arch",arch.c_str());
        nvs_set_str(handle,"romSet",romSet.c_str());
        nvs_set_str(handle,"romSet48",romSet48.c_str());
        nvs_set_str(handle,"romSet128",romSet128.c_str());
        nvs_set_str(handle,"romSetPent",romSetPent.c_str());
        nvs_set_str(handle,"pref_arch",pref_arch.c_str());
        nvs_set_str(handle,"pref_romSet_48",pref_romSet_48.c_str());
        nvs_set_str(handle,"pref_romSet_128",pref_romSet_128.c_str());
        nvs_set_str(handle,"pref_romSetPent",pref_romSetPent.c_str());
        nvs_set_str(handle,"ram",ram_file.c_str());   
        nvs_set_str(handle,"slog",slog_on ? "true" : "false");
///        nvs_set_str(handle,"sdstorage", FileUtils::MountPoint);
        nvs_set_str(handle,"asp169",aspect_16_9 ? "true" : "false");
        nvs_set_u8(handle,"videomode",Config::videomode);
        nvs_set_u8(handle,"language",Config::lang);
        nvs_set_str(handle,"AY48",AY48 ? "true" : "false");
        nvs_set_str(handle,"Issue2",Issue2 ? "true" : "false");
        nvs_set_str(handle,"flashload",flashload ? "true" : "false");
        nvs_set_str(handle,"tape_player",tape_player ? "true" : "false");
        nvs_set_str(handle,"tape_timing_rg",tape_timing_rg ? "true" : "false");
        nvs_set_u8(handle,"joystick1",Config::joystick1);
        nvs_set_u8(handle,"joystick2",Config::joystick2);
        // Write joystick definition
        for (int n=0; n < 24; n++) {
            char joykey[9];
            sprintf(joykey,"joydef%02u",n);
            nvs_set_u16(handle,joykey,Config::joydef[n]);
        }
        nvs_set_u8(handle,"joyPS2",Config::joyPS2);
        nvs_set_u8(handle,"AluTiming",Config::AluTiming);
        nvs_set_u8(handle,"PS2Dev2",Config::ps2_dev2);
        nvs_set_str(handle,"CursorAsJoy", CursorAsJoy ? "true" : "false");
        nvs_set_i8(handle,"CenterH",Config::CenterH);
        nvs_set_i8(handle,"CenterV",Config::CenterV);
        nvs_set_str(handle,"SNA_Path",FileUtils::SNA_Path.c_str());
        nvs_set_str(handle,"TAP_Path",FileUtils::TAP_Path.c_str());
        nvs_set_str(handle,"DSK_Path",FileUtils::DSK_Path.c_str());
        nvs_set_u16(handle,"SNA_begin_row",Config::SNA_begin_row);
        nvs_set_u16(handle,"TAP_begin_row",Config::TAP_begin_row);
        nvs_set_u16(handle,"DSK_begin_row",Config::DSK_begin_row);
        nvs_set_u16(handle,"SNA_focus",Config::SNA_focus);
        nvs_set_u16(handle,"TAP_focus",Config::TAP_focus);
        nvs_set_u16(handle,"DSK_focus",Config::DSK_focus);
        nvs_set_u8(handle,"SNA_fdMode",Config::SNA_fdMode);
        nvs_set_u8(handle,"TAP_fdMode",Config::TAP_fdMode);
        nvs_set_u8(handle,"DSK_fdMode",Config::DSK_fdMode);
        nvs_set_str(handle,"SNA_fileSearch",Config::SNA_fileSearch.c_str());
        nvs_set_str(handle,"TAP_fileSearch",Config::TAP_fileSearch.c_str());
        nvs_set_str(handle,"DSK_fileSearch",Config::DSK_fileSearch.c_str());
        nvs_set_u8(handle,"scanlines",Config::scanlines);
        nvs_set_u8(handle,"render",Config::render);
        nvs_set_str(handle,"TABasfire1", TABasfire1 ? "true" : "false");
        nvs_set_str(handle,"StartMsg", StartMsg ? "true" : "false");
        f_close(&handle);
    }
    // printf("Config saved OK\n");
}

void Config::setJoyMap(uint8_t joynum, uint8_t joytype) {
    fabgl::VirtualKey newJoy[12];
    for (int n=0; n < 12; n++) newJoy[n] = fabgl::VK_NONE;
    // Ask to overwrite map with default joytype values
    string title = (joynum == 1 ? "Joystick 1" : "Joystick 2");
    string msg = OSD_DLG_SETJOYMAPDEFAULTS[Config::lang];
    uint8_t res = OSD::msgDialog(title,msg);
    if (res == DLG_YES) {
        switch (joytype) {
        case JOY_CURSOR:
            newJoy[0] = fabgl::VK_5;
            newJoy[1] = fabgl::VK_8;
            newJoy[2] = fabgl::VK_7;
            newJoy[3] = fabgl::VK_6;
            newJoy[6] = fabgl::VK_0;
            break;
        case JOY_KEMPSTON:
            newJoy[0] = fabgl::VK_KEMPSTON_LEFT;
            newJoy[1] = fabgl::VK_KEMPSTON_RIGHT;
            newJoy[2] = fabgl::VK_KEMPSTON_UP;
            newJoy[3] = fabgl::VK_KEMPSTON_DOWN;
            newJoy[6] = fabgl::VK_KEMPSTON_FIRE;
            newJoy[7] = fabgl::VK_KEMPSTON_ALTFIRE;
            break;
        case JOY_SINCLAIR1:
            newJoy[0] = fabgl::VK_6;
            newJoy[1] = fabgl::VK_7;
            newJoy[2] = fabgl::VK_9;
            newJoy[3] = fabgl::VK_8;
            newJoy[6] = fabgl::VK_0;
            break;
        case JOY_SINCLAIR2:
            newJoy[0] = fabgl::VK_1;
            newJoy[1] = fabgl::VK_2;
            newJoy[2] = fabgl::VK_4;
            newJoy[3] = fabgl::VK_3;
            newJoy[6] = fabgl::VK_5;
            break;
        case JOY_FULLER:
            newJoy[0] = fabgl::VK_FULLER_LEFT;
            newJoy[1] = fabgl::VK_FULLER_RIGHT;
            newJoy[2] = fabgl::VK_FULLER_UP;
            newJoy[3] = fabgl::VK_FULLER_DOWN;
            newJoy[6] = fabgl::VK_FULLER_FIRE;
            break;
        }
    }
    // Fill joystick values in Config and clean Kempston or Fuller values if needed
    int m = (joynum == 1) ? 0 : 12;
    for (int n = m; n < m + 12; n++) {
        bool save = false;
        if (newJoy[n - m] != fabgl::VK_NONE) {
            ESPectrum::JoyVKTranslation[n] = newJoy[n - m];
            save = true;
        } else {
            if (joytype != JOY_KEMPSTON) {
                if (ESPectrum::JoyVKTranslation[n] >= fabgl::VK_KEMPSTON_RIGHT && ESPectrum::JoyVKTranslation[n] <= fabgl::VK_KEMPSTON_ALTFIRE) {
                    ESPectrum::JoyVKTranslation[n] = fabgl::VK_NONE;
                    save = true;
                }
            }
            if (joytype != JOY_FULLER) {
                if (ESPectrum::JoyVKTranslation[n] >= fabgl::VK_FULLER_RIGHT && ESPectrum::JoyVKTranslation[n] <= fabgl::VK_FULLER_FIRE) {
                    ESPectrum::JoyVKTranslation[n] = fabgl::VK_NONE;
                    save = true;                
                }
            }
        }
        if (save) {
            // Save to config (only changes)
            if (Config::joydef[n] != (uint16_t) ESPectrum::JoyVKTranslation[n]) {
                Config::joydef[n] = (uint16_t) ESPectrum::JoyVKTranslation[n];
                char joykey[9];
                sprintf(joykey,"joydef%02u",n);
                Config::save(joykey);
                // printf("%s %u\n",joykey, joydef[n]);
            }
        }
    }
}
