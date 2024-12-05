#include "Config.h"
#include "MemESP.h"
#include "roms.h"
#include "FileUtils.h"
#include "ESPectrum.h"
#include "fabutils.h"
#include "messages.h"
#include "OSDMain.h"
#include "psram_spi.h"
#include "pwm_audio.h"

string   Config::arch = "48K";
string   Config::romSet = "48K";
string   Config::romSet48 = "48K";
string   Config::romSet128 = "128K";
string   Config::romSetPent = "128Kp";
string   Config::romSetP512 = "128Kp";
string   Config::romSetP1M = "128Kp";
string   Config::pref_arch = "48K";
string   Config::pref_romSet_48 = "48K";
string   Config::pref_romSet_128 = "128K";
string   Config::pref_romSetPent = "128Kp";
string   Config::pref_romSetP512 = "128Kp";
string   Config::pref_romSetP1M = "128Kp";
string   Config::ram_file = NO_RAM_FILE;
string   Config::last_ram_file = NO_RAM_FILE;

bool     Config::slog_on = false;
const bool     Config::aspect_16_9 = false;
///uint8_t  Config::esp32rev = 0;
uint8_t  Config::lang = 0;
bool     Config::AY48 = true;
bool     Config::Issue2 = true;
bool     Config::flashload = true;
bool     Config::tape_player = false; // Tape player mode
volatile bool Config::real_player = false;
bool     Config::tape_timing_rg = false; // Rodolfo Guerra ROMs tape timings
bool     Config::rightSpace = true;

uint8_t  Config::joystick = JOY_KEMPSTON;
uint16_t Config::joydef[12] = {
    fabgl::VK_KEMPSTON_LEFT,  // 0
    fabgl::VK_KEMPSTON_RIGHT, // 1
    fabgl::VK_KEMPSTON_UP,    // 2
    fabgl::VK_KEMPSTON_DOWN,  // 3
    fabgl::VK_KEMPSTON_START, // 4
    fabgl::VK_KEMPSTON_SELECT,// 5
    fabgl::VK_KEMPSTON_FIRE,  // 6 A
    fabgl::VK_KEMPSTON_ALTFIRE,//7 B
    fabgl::VK_NONE,           // 8 C
    fabgl::VK_NONE,           // 9  X
    fabgl::VK_NONE,           // 10 Y
    fabgl::VK_NONE            // 11 Z
};

uint8_t  Config::AluTiming = 0;
uint8_t  Config::joy2cursor = 0;
uint8_t  Config::secondJoy = 2;
uint8_t  Config::kempstonPort = 0x1F;
uint8_t  Config::throtling = DEFAULT_THROTTLING;
bool     Config::CursorAsJoy = false;
int8_t   Config::CenterH = 0;
int8_t   Config::CenterV = 0;

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
        if (romSet48 == "48Kcs") {
#if !CARTRIDGE_AS_CUSTOM
#if NO_SEPARATE_48K_CUSTOM
            MemESP::rom[0].assign_rom(gb_rom_0_128k_custom);
#else
            MemESP::rom[0].assign_rom(gb_rom_0_48k_custom);
#endif
#else
            MemESP::rom[0].assign_rom(gb_rom_Alf_cart);
#endif
        } else
#if !NO_SPAIN_ROM_48k
        if (romSet48 == "48Kes")
            MemESP::rom[0].assign_rom(gb_rom_0_48k_es);
        else
#endif
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_48k);
    } else if (arch == "ALF") {
        const uint8_t* base = gb_rom_Alf;
        for (int i = 0; i < 64; ++i) {
            MemESP::rom[i].assign_rom(i >= 16 ? gb_rom_Alf_ep : base + ((16 * i) << 10));
        }
    } else if (arch == "128K") {
        if (newRomSet=="") romSet = "128K"; else romSet = newRomSet;
        if (newRomSet=="") romSet128 = "128K"; else romSet128 = newRomSet;
        if (romSet128 == "128Kcs") {
#if !CARTRIDGE_AS_CUSTOM
            MemESP::rom[0].assign_rom(gb_rom_0_128k_custom);
            MemESP::rom[1].assign_rom(gb_rom_0_128k_custom + (16 << 10)); /// 16392;
#else
            MemESP::rom[0].assign_rom(gb_rom_Alf_cart);
            MemESP::rom[1].assign_rom(gb_rom_Alf_cart + (16 << 10)); /// 16392;
#endif
#if !NO_SPAIN_ROM_128k
        } else if (romSet128 == "128Kes") {
            MemESP::rom[0].assign_rom(gb_rom_0_128k_es);
            MemESP::rom[1].assign_rom(gb_rom_1_128k_es);
        } else if (romSet128 == "+2es") {
            MemESP::rom[0].assign_rom(gb_rom_0_plus2_es);
            MemESP::rom[1].assign_rom(gb_rom_1_plus2_es);
        } else if (romSet128 == "+2") {
            MemESP::rom[0].assign_rom(gb_rom_0_plus2);
            MemESP::rom[1].assign_rom(gb_rom_1_plus2);
        } else if (romSet128 == "ZX81+") {
            MemESP::rom[0].assign_rom(gb_rom_0_s128_zx81);
            MemESP::rom[1].assign_rom(gb_rom_1_sinclair_128k);
#endif
        } else {
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_128k);
            MemESP::rom[1].assign_rom(gb_rom_1_sinclair_128k);
        }
    } else { // Pentagon by default
        if (newRomSet=="") romSet = "128Kp"; else romSet = newRomSet;
        if (romSetPent=="") romSetPent = "128Kp"; else romSetPent = newRomSet;
        if (romSetPent == "128Kcs") {
#if !CARTRIDGE_AS_CUSTOM
            MemESP::rom[0].assign_rom(gb_rom_0_128k_custom);
            MemESP::rom[1].assign_rom(gb_rom_0_128k_custom + (16 << 10)); /// 16392;
#else
            MemESP::rom[0].assign_rom(gb_rom_Alf_cart);
            MemESP::rom[1].assign_rom(gb_rom_Alf_cart + (16 << 10)); /// 16392;
#endif
        } else {
            MemESP::rom[0].assign_rom(gb_rom_pentagon_128k);
            MemESP::rom[1].assign_rom(gb_rom_pentagon_128k + (16 << 10));
        }
    }
    MemESP::rom[4].assign_rom(gb_rom_4_trdos_503);
}

static bool nvs_get_str(const char* key, string& v, const vector<string>& sts) {
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
static void nvs_get_b(const char* key, bool& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = (t == "true");
    }
}
static void nvs_get_i(const char* key, int& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
    }
}
static void nvs_get_i8(const char* key, int8_t& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
    }
}
static void nvs_get_u8(const char* key, uint8_t& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
    }
}
static void nvs_get_u16(const char* key, uint16_t& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
    }
}

// Read config from FS
void Config::load() {
    string nvs = MOUNT_POINT_SD STORAGE_NVS;
    FIL* handle = fopen2(nvs.c_str(), FA_READ);
    if (!handle) {
        return;
    } else {
        vector<string> sts;
        UINT br;
        char c;
        string s;
        while(!f_eof(handle)) {
            if (f_read(handle, &c, 1, &br) != FR_OK) {
                fclose2(handle);
                return;
            }
            if (c == '\n') {
                sts.push_back(s);
                s.clear();
            } else {
                s += c;
            }
        }
        fclose2(handle);

        nvs_get_str("arch", arch, sts);
        nvs_get_str("romSet", romSet, sts);
        nvs_get_str("romSet48", romSet48, sts);
        nvs_get_str("romSet128", romSet128, sts);
        nvs_get_str("romSetPent", romSetPent, sts);
        nvs_get_str("romSetP512", romSetP512, sts);
        nvs_get_str("romSetP1M", romSetP1M, sts);
        nvs_get_str("pref_arch", pref_arch, sts);
        nvs_get_str("pref_romSet_48", pref_romSet_48, sts);
        nvs_get_str("pref_romSet_128", pref_romSet_128, sts);
        nvs_get_str("pref_romSetPent", pref_romSetPent, sts);
        nvs_get_str("pref_romSetP512", pref_romSetP512, sts);
        nvs_get_str("pref_romSetP1M", pref_romSetP1M, sts);
        nvs_get_str("ram", ram_file, sts);
        nvs_get_b("AY48", AY48, sts);
        nvs_get_b("Issue2", Issue2, sts);
        nvs_get_b("flashload", flashload, sts);
        nvs_get_b("rightSpace", rightSpace, sts);
        nvs_get_b("tape_player", tape_player, sts);
        bool b; nvs_get_b("real_player", b, sts);
        if (real_player && !b) {
            pcm_audio_in_stop();
        }
        real_player = b;
        nvs_get_b("tape_timing_rg", tape_timing_rg, sts);
        nvs_get_u8("joystick", Config::joystick, sts);

        // Read joystick definition
        for (int n = 0; n < 12; ++n) {
            char joykey[16];
            snprintf(joykey, 16, "joydef%02u", n);
            // printf("%s\n",joykey);
            nvs_get_u16(joykey, Config::joydef[n], sts);
        }

        nvs_get_u8("AluTiming", Config::AluTiming, sts);
        nvs_get_u8("joy2cursor", Config::joy2cursor, sts);
        nvs_get_u8("secondJoy", Config::secondJoy, sts);
        nvs_get_u8("kempstonPort", Config::kempstonPort, sts);
#if !PICO_RP2040
        nvs_get_u8("throtling2", Config::throtling, sts);
#else
        nvs_get_u8("throtling", Config::throtling, sts);
#endif
        nvs_get_b("CursorAsJoy", CursorAsJoy, sts);
        nvs_get_i8("CenterH", Config::CenterH, sts);
        nvs_get_i8("CenterV", Config::CenterV, sts);
        nvs_get_str("SNA_Path", FileUtils::SNA_Path, sts);
        nvs_get_str("TAP_Path", FileUtils::TAP_Path, sts);
        nvs_get_str("DSK_Path", FileUtils::DSK_Path, sts);
        nvs_get_str("ROM_Path", FileUtils::ROM_Path, sts);
        for (size_t i = 0; i < 4; ++i) {
            DISK_FTYPE& ft = FileUtils::fileTypes[i];
            const string s = "fileTypes" + to_string(i);
            nvs_get_i((s + ".begin_row").c_str(), ft.begin_row, sts);
            nvs_get_i((s + ".focus").c_str(), ft.focus, sts);
            nvs_get_u8((s + ".fdMode").c_str(), ft.fdMode, sts);
            nvs_get_str((s + ".fileSearch").c_str(), ft.fileSearch, sts);
        }
        nvs_get_u8("scanlines", Config::scanlines, sts);
        nvs_get_u8("render", Config::render, sts);
        nvs_get_b("TABasfire1", Config::TABasfire1, sts);
        nvs_get_b("StartMsg", Config::StartMsg, sts);
    }
}

static void nvs_set_str(FIL* handle, const char* name, const char* val) {
    UINT btw;
    f_write(handle, name, strlen(name), &btw);
    f_write(handle, "=", 1, &btw);
    f_write(handle, val, strlen(val), &btw);
    f_write(handle, "\n", 1, &btw);
}
static void nvs_set_i(FIL* handle, const char* name, int val) {
    string v = to_string(val);
    nvs_set_str(handle, name, v.c_str());
}
static void nvs_set_i8(FIL* handle, const char* name, int8_t val) {
    string v = to_string(val);
    nvs_set_str(handle, name, v.c_str());
}
static void nvs_set_u8(FIL* handle, const char* name, uint8_t val) {
    string v = to_string(val);
    nvs_set_str(handle, name, v.c_str());
}
static void nvs_set_u16(FIL* handle, const char* name, uint16_t val) {
    string v = to_string(val);
    nvs_set_str(handle, name, v.c_str());
}

// Dump actual config to FS
void Config::save() {
    if (!FileUtils::fsMount) return;
    string nvs = MOUNT_POINT_SD STORAGE_NVS;
    FIL* handle = fopen2(nvs.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (handle) {
        nvs_set_str(handle,"arch",arch.c_str());
        nvs_set_str(handle,"romSet",romSet.c_str());
        nvs_set_str(handle,"romSet48",romSet48.c_str());
        nvs_set_str(handle,"romSet128",romSet128.c_str());
        nvs_set_str(handle,"romSetPent",romSetPent.c_str());
        nvs_set_str(handle,"romSetP512",romSetP512.c_str());
        nvs_set_str(handle,"romSetP1M",romSetP1M.c_str());
        nvs_set_str(handle,"pref_arch",pref_arch.c_str());
        nvs_set_str(handle,"pref_romSet_48",pref_romSet_48.c_str());
        nvs_set_str(handle,"pref_romSet_128",pref_romSet_128.c_str());
        nvs_set_str(handle,"pref_romSetPent",pref_romSetPent.c_str());
        nvs_set_str(handle,"pref_romSetP512",pref_romSetP512.c_str());
        nvs_set_str(handle,"pref_romSetP1M",pref_romSetP1M.c_str());
        nvs_set_str(handle,"ram",ram_file.c_str());   
        nvs_set_str(handle,"slog",slog_on ? "true" : "false");
///        nvs_set_str(handle,"sdstorage", FileUtils::MountPoint);
///        nvs_set_str(handle,"asp169",aspect_16_9 ? "true" : "false");
        nvs_set_u8(handle,"language", Config::lang);
        nvs_set_str(handle,"AY48", AY48 ? "true" : "false");
        nvs_set_str(handle,"Issue2", Issue2 ? "true" : "false");
        nvs_set_str(handle,"flashload", flashload ? "true" : "false");
        nvs_set_str(handle,"tape_player", tape_player ? "true" : "false");
        nvs_set_str(handle,"real_player", real_player ? "true" : "false");
        nvs_set_str(handle,"rightSpace", rightSpace ? "true" : "false");
        nvs_set_str(handle,"tape_timing_rg",tape_timing_rg ? "true" : "false");
        nvs_set_u8(handle,"joystick", Config::joystick);
        // Write joystick definition
        for (int n = 0; n < 12; ++n) {
            char joykey[16];
            snprintf(joykey, 16, "joydef%02u", n);
            nvs_set_u16(handle, joykey, Config::joydef[n]);
        }
        nvs_set_u8(handle,"AluTiming",Config::AluTiming);
        nvs_set_u8(handle,"joy2cursor",Config::joy2cursor);
        nvs_set_u8(handle,"secondJoy",Config::secondJoy);
        nvs_set_u8(handle,"kempstonPort",Config::kempstonPort);
#if !PICO_RP2040
        nvs_set_u8(handle,"throtling2",Config::throtling);
#else
        nvs_set_u8(handle,"throtling",Config::throtling);
#endif
        nvs_set_str(handle,"CursorAsJoy", CursorAsJoy ? "true" : "false");
        nvs_set_i8(handle,"CenterH",Config::CenterH);
        nvs_set_i8(handle,"CenterV",Config::CenterV);
        nvs_set_str(handle,"SNA_Path",FileUtils::SNA_Path.c_str());
        nvs_set_str(handle,"TAP_Path",FileUtils::TAP_Path.c_str());
        nvs_set_str(handle,"DSK_Path",FileUtils::DSK_Path.c_str());
        nvs_set_str(handle,"ROM_Path",FileUtils::ROM_Path.c_str());
        for (size_t i = 0; i < 4; ++i) {
            const DISK_FTYPE& ft = FileUtils::fileTypes[i];
            const string s = "fileTypes" + to_string(i);
            nvs_set_i(handle, (s + ".begin_row").c_str(), ft.begin_row);
            nvs_set_i(handle, (s + ".focus").c_str(), ft.focus);
            nvs_set_u8(handle, (s + ".fdMode").c_str(), ft.fdMode);
            nvs_set_str(handle, (s + ".fileSearch").c_str(), ft.fileSearch.c_str());
        }
        nvs_set_u8(handle,"scanlines",Config::scanlines);
        nvs_set_u8(handle,"render",Config::render);
        nvs_set_str(handle,"TABasfire1", TABasfire1 ? "true" : "false");
        nvs_set_str(handle,"StartMsg", StartMsg ? "true" : "false");
        fclose2(handle);
    }
    // printf("Config saved OK\n");
}

void Config::setJoyMap(uint8_t joytype) {
    for (int n = 0; n < 12; n++) joydef[n] = fabgl::VK_NONE;
    // Ask to overwrite map with default joytype values
    string title = "Joystick";
    string msg = OSD_DLG_SETJOYMAPDEFAULTS[Config::lang];
    uint8_t res = OSD::msgDialog(title, msg);
    if (res == DLG_YES) {
        joydef[0] = fabgl::VK_KEMPSTON_LEFT;
        joydef[1] = fabgl::VK_KEMPSTON_RIGHT;
        joydef[2] = fabgl::VK_KEMPSTON_UP;
        joydef[3] = fabgl::VK_KEMPSTON_DOWN;
        joydef[6] = fabgl::VK_KEMPSTON_FIRE;
        if (joytype == JOY_KEMPSTON) {
            joydef[4] = fabgl::VK_KEMPSTON_START;
            joydef[5] = fabgl::VK_KEMPSTON_SELECT;
            joydef[7] = fabgl::VK_KEMPSTON_ALTFIRE;
        }
        Config::save();
    }
}
