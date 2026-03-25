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

bool     Config::loaded = false;
bool     Config::slog_on = false;
const bool     Config::aspect_16_9 = false;
///uint8_t  Config::esp32rev = 0;
uint8_t  Config::lang = 0;
bool     Config::AY48 = true;
#if !PICO_RP2040
bool     Config::SAA1099 = false;
uint8_t  Config::midi = 0;
uint8_t  Config::midi_synth_preset = 0;
#endif
uint16_t Config::cpu_mhz = CPU_MHZ;
uint16_t Config::max_flash_freq = 66;
uint16_t Config::max_psram_freq = 166;
bool     Config::Issue2 = true;
bool     Config::flashload = true;
bool     Config::tape_player = false; // Tape player mode
volatile bool Config::real_player = false;
bool     Config::tape_timing_rg = false; // Rodolfo Guerra ROMs tape timings
bool     Config::rightSpace = true;
bool     Config::wasd = true;
Config::BreakPoint Config::breakPoints[Config::MAX_BREAKPOINTS];
int Config::numBreakPoints = 0;
int Config::numPcBP = 0;
int Config::numPortReadBP = 0;
int Config::numPortWriteBP = 0;
int Config::numMemWriteBP = 0;
int Config::numMemReadBP = 0;

uint8_t  Config::joystick = JOY_KEMPSTON;
uint16_t Config::joydef[12] = {
    fabgl::VK_DPAD_LEFT,  // 0
    fabgl::VK_DPAD_RIGHT, // 1
    fabgl::VK_DPAD_UP,    // 2
    fabgl::VK_DPAD_DOWN,  // 3
    fabgl::VK_DPAD_START, // 4
    fabgl::VK_DPAD_SELECT,// 5
    fabgl::VK_DPAD_FIRE,  // 6 A
    fabgl::VK_DPAD_ALTFIRE,//7 B
    fabgl::VK_NONE,       // 8 C
    fabgl::VK_NONE,       // 9  X
    fabgl::VK_NONE,       // 10 Y
    fabgl::VK_NONE        // 11 Z
};

uint8_t  Config::AluTiming = 0;
uint8_t  Config::ayConfig = 0;
#if !defined(PICO_RP2040)
uint8_t  Config::turbosound = 3; // BOTH
#else
uint8_t  Config::turbosound = 0; // OFF
#endif
uint8_t  Config::covox = 0; // NONE
uint8_t  Config::joy2cursor = true;
uint8_t  Config::secondJoy = 2; // NPAD#2
uint8_t  Config::kempstonPort = 0x37;
uint8_t  Config::throtling = DEFAULT_THROTTLING;
bool     Config::CursorAsJoy = true;
bool     Config::trdosFastMode = false;
bool     Config::trdosWriteProtect = true;
bool     Config::trdosSoundLed = false;
uint8_t  Config::trdosBios = 2; // Default: 5.05D
#if !PICO_RP2040
uint8_t  Config::esxdos = 0;
string   Config::esxdos_mmc_image = "";
string   Config::esxdos_hdf_image[2] = {"", ""};
#endif

uint8_t Config::scanlines = 0;
uint8_t Config::render = 0;

bool     Config::TABasfire1 = false;
bool     Config::StartMsg = true;
signed char Config::aud_volume = 0;
uint8_t  Config::hdmi_video_mode = Config::VM_640x480_60;
uint8_t  Config::vga_video_mode = Config::VM_640x480_60;
bool     Config::v_sync_enabled = false;
bool     Config::gigascreen_enabled = false;
uint8_t  Config::gigascreen_onoff = 0;
#if !PICO_RP2040
bool     Config::ulaplus = false;
#endif
uint8_t  Config::audio_driver = 0;
extern "C" uint8_t  video_driver = 0;
bool     Config::byte_cobmect_mode = false;

Config::HotkeyBinding Config::hotkeys[Config::HK_COUNT];

void Config::initHotkeys() {
    // Default bindings — must match HK_* enum order
    static const HotkeyBinding defaults[HK_COUNT] = {
        { fabgl::VK_F1,     false, false, true  }, // HK_MAIN_MENU  — readonly
        { fabgl::VK_F2,     false, false, false }, // HK_LOAD_SNA
        { fabgl::VK_F3,     false, false, false }, // HK_PERSIST_LOAD
        { fabgl::VK_F4,     false, false, false }, // HK_PERSIST_SAVE
        { fabgl::VK_F5,     false, false, false }, // HK_LOAD_ANY
        { fabgl::VK_F6,     false, false, false }, // HK_TAPE_PLAY
        { fabgl::VK_F7,     false, false, false }, // HK_TAPE_BROWSER
        { fabgl::VK_F8,     false, false, false }, // HK_STATS
        { fabgl::VK_F9,     false, false, false }, // HK_VOL_DOWN
        { fabgl::VK_F10,    false, false, false }, // HK_VOL_UP
        { fabgl::VK_F11,    false, false, false }, // HK_HARD_RESET
        { fabgl::VK_F12,    false, false, false }, // HK_REBOOT
        { fabgl::VK_TILDE,  false, false, false }, // HK_MAX_SPEED
        { fabgl::VK_PAUSE,  false, false, false }, // HK_PAUSE
        { fabgl::VK_F1,     true,  false, true  }, // HK_HW_INFO    — readonly
        { fabgl::VK_F2,     true,  false, false }, // HK_TURBO
        { fabgl::VK_F5,     true,  false, false }, // HK_DEBUG
        { fabgl::VK_F6,     true,  false, false }, // HK_DISK
        { fabgl::VK_F10,    true,  false, false }, // HK_NMI
        { fabgl::VK_F11,    true,  false, false }, // HK_RESET_TO
        { fabgl::VK_F12,    true,  false, false }, // HK_USB_BOOT
        { fabgl::VK_PAGEUP, true,  false, false }, // HK_GIGASCREEN
        { fabgl::VK_F7,     true,  false, false }, // HK_BP_LIST
        { fabgl::VK_F8,     true,  false, false }, // HK_JUMP_TO
        { fabgl::VK_F9,     true,  false, false }, // HK_POKE
        { fabgl::VK_HOME,   true,  true,  false }, // HK_VIDMODE_60
        { fabgl::VK_END,    true,  true,  false }, // HK_VIDMODE_50
    };
    for (int i = 0; i < HK_COUNT; i++)
        hotkeys[i] = defaults[i];
}

void Config::requestMachine(string newArch, string newRomSet)
{
    arch = newArch;
    if (arch == "48K") {
        if (newRomSet=="") romSet = "48K"; else romSet = newRomSet;
        if (newRomSet=="") romSet48 = "48K"; else romSet48 = newRomSet;
        if (romSet48 == "48Kcs") {
#if !CARTRIDGE_AS_CUSTOM || NO_ALF
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
        if (romSet48 == "48Kby")
            MemESP::rom[0].assign_rom(Config::byte_cobmect_mode ? gb_rom_0_byte_sovmest_48k : gb_rom_0_byte_48k);
        else
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_48k);
    }
#if !NO_ALF
    else if (arch == "ALF") {
        const uint8_t* base = gb_rom_Alf;
        for (int i = 0; i < 64; ++i) {
            MemESP::rom[i].assign_rom(i >= 16 ? gb_rom_Alf_ep : base + ((16 * i) << 10));
        }
        Config::kempstonPort = 0x1F; // TODO: ensure, save?
    }
#endif
    else if (arch == "128K") {
        if (newRomSet=="") romSet = "128K"; else romSet = newRomSet;
        if (newRomSet=="") romSet128 = "128K"; else romSet128 = newRomSet;
        if (romSet128 == "128Kcs") {
#if !CARTRIDGE_AS_CUSTOM || NO_ALF
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
        } else if (romSet128 == "128Kby" || romSet128 == "128Kbg") {
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_128k);
            MemESP::rom[1].assign_rom(gb_rom_0_byte_48k);
            if (romSet128 == "128Kbg") {
                MemESP::rom[3].assign_rom(gb_rom_gluk);
            }
        }
        else {
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_128k);
            MemESP::rom[1].assign_rom(gb_rom_1_sinclair_128k);
        }
    } else { // Pentagon by default
        if (newRomSet=="") romSet = "128Kp"; else romSet = newRomSet;
        if (romSetPent=="") romSetPent = "128Kp"; else romSetPent = newRomSet;
        if (romSetPent == "128Kcs") {
#if !CARTRIDGE_AS_CUSTOM || NO_ALF
            MemESP::rom[0].assign_rom(gb_rom_0_128k_custom);
            MemESP::rom[1].assign_rom(gb_rom_0_128k_custom + (16 << 10)); /// 16392;
#else
            MemESP::rom[0].assign_rom(gb_rom_Alf_cart);
            MemESP::rom[1].assign_rom(gb_rom_Alf_cart + (16 << 10)); /// 16392;
#endif
        } else {
            MemESP::rom[0].assign_rom(gb_rom_pentagon_128k);
            MemESP::rom[1].assign_rom(gb_rom_pentagon_128k + (16 << 10));
            if (romSetPent == "128Kpg") {
                MemESP::rom[3].assign_rom(gb_rom_gluk);
            }
        }
    }
    switch (Config::trdosBios) {
        case 0: MemESP::rom[4].assign_rom(gb_rom_4_trdos_503); break;
        case 1: MemESP::rom[4].assign_rom(gb_rom_4_trdos_504tm); break;
        default: MemESP::rom[4].assign_rom(gb_rom_4_trdos_505d); break;
    }
}

// RAM fallback for Config when no SD card
static string nvs_ram_buf;

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
static bool nvs_get_u8(const char* key, uint8_t& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
        return true;
    }
    return false;
}
static void nvs_get_u16(const char* key, uint16_t& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
    }
}
static void nvs_get_sc(const char* key, signed char& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
    }
}

void Config::load2() {
    string nvs = MOUNT_POINT_SD STORAGE_NVS;
    FIL* handle = fopen2(nvs.c_str(), FA_READ);
    if (!handle) {
        return;
    }
    // Parse line-by-line without loading entire file into vector
    // Only need drive0..drive3 .file entries
    UINT br;
    char c;
    string s;
    while(!f_eof(handle)) {
        if (f_read(handle, &c, 1, &br) != FR_OK) {
            fclose2(handle);
            return;
        }
        if (c == '\n') {
            // Check if this line is a driveN.file= entry
            for (size_t i = 0; i < 4; ++i) {
                char prefix[16];
                snprintf(prefix, sizeof(prefix), "drive%u.file=", (unsigned)i);
                size_t plen = strlen(prefix);
                if (s.length() >= plen && s.compare(0, plen, prefix) == 0) {
                    std::string fn = s.substr(plen);
                    if (!fn.empty()) {
                        rvmWD1793InsertDisk(&ESPectrum::fdd, i, fn);
                    }
                }
            }
            s.clear();
        } else {
            s += c;
        }
    }
    fclose2(handle);
}

#if TFT
extern "C" uint8_t TFT_FLAGS;
extern "C" uint8_t TFT_INVERSION;
#endif

// Parse NVS data from a raw string into vector of lines
static void nvs_parse_lines(const string& data, vector<string>& sts) {
    string s;
    for (char c : data) {
        if (c == '\n') {
            sts.push_back(s);
            s.clear();
        } else {
            s += c;
        }
    }
    if (!s.empty()) sts.push_back(s);
}

// Read config from FS
void Config::load() {
    initHotkeys(); // fill defaults before overriding from NVS
    vector<string> sts;
    if (FileUtils::fsMount) {
        string nvs = MOUNT_POINT_SD STORAGE_NVS;
        FIL* handle = fopen2(nvs.c_str(), FA_READ);
        if (!handle) {
            return;
        }
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
    } else if (!nvs_ram_buf.empty()) {
        nvs_parse_lines(nvs_ram_buf, sts);
    } else {
        return;
    }
    {

        #if TFT
        nvs_get_u8("TFT_FLAGS", TFT_FLAGS, sts);
        nvs_get_u8("TFT_INVERSION", TFT_INVERSION, sts);
        #endif
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
#if !PICO_RP2040
        nvs_get_b("SAA1099", SAA1099, sts);
        nvs_get_u8("midi", midi, sts);
        nvs_get_u8("midipreset", midi_synth_preset, sts);
#endif
        nvs_get_u16("cpu_mhz", cpu_mhz, sts);
#if PICO_RP2040
        if (cpu_mhz != 252 && cpu_mhz != 378)
            cpu_mhz = CPU_MHZ;
#else
        if (cpu_mhz != 252 && cpu_mhz != 378 && cpu_mhz != 504)
            cpu_mhz = CPU_MHZ;
#endif
        nvs_get_u16("max_flash_freq", max_flash_freq, sts);
        if (max_flash_freq == 0) max_flash_freq = 66;
        nvs_get_u16("max_psram_freq", max_psram_freq, sts);
        if (max_psram_freq == 0) max_psram_freq = 166;
        nvs_get_b("Issue2", Issue2, sts);
        nvs_get_b("flashload", flashload, sts);
        nvs_get_b("rightSpace", rightSpace, sts);
        nvs_get_b("wasd", wasd, sts);
        // Load typed breakpoints array
        for (int i = 0; i < MAX_BREAKPOINTS; i++) {
            breakPoints[i] = {0xFFFF, BP_NONE};
            char key[16];
            snprintf(key, sizeof(key), "bp%d", i);
            nvs_get_u16(key, breakPoints[i].addr, sts);
            uint8_t t = BP_NONE;
            snprintf(key, sizeof(key), "bpt%d", i);
            nvs_get_u8(key, t, sts);
            breakPoints[i].type = (BPType)t;
            if (breakPoints[i].type == BP_NONE) breakPoints[i].addr = 0xFFFF;
        }
        // Migrate old single breakPoint
        {
            bool anyLoaded = false;
            for (int i = 0; i < MAX_BREAKPOINTS; i++)
                if (breakPoints[i].type != BP_NONE) { anyLoaded = true; break; }
            if (!anyLoaded) {
                uint16_t oldBP = 0xFFFF; bool oldEnable = false;
                nvs_get_u16("breakPoint", oldBP, sts);
                nvs_get_b("enableBreakPoint", oldEnable, sts);
                if (oldEnable && oldBP != 0xFFFF)
                    breakPoints[0] = {oldBP, BP_PC};
                // Migrate old port BPs
                uint16_t oldPR = 0xFFFF, oldPW = 0xFFFF;
                bool oldPRe = false, oldPWe = false;
                nvs_get_u16("portReadBP", oldPR, sts);
                nvs_get_b("enablePortReadBP", oldPRe, sts);
                if (oldPRe && oldPR != 0xFFFF)
                    breakPoints[1] = {oldPR, BP_PORT_READ};
                nvs_get_u16("portWriteBP", oldPW, sts);
                nvs_get_b("enablePortWriteBP", oldPWe, sts);
                if (oldPWe && oldPW != 0xFFFF)
                    breakPoints[2] = {oldPW, BP_PORT_WRITE};
            }
        }
        recountBP();
        nvs_get_b("tape_player", tape_player, sts);
        bool b; nvs_get_b("real_player", b, sts);
#if LOAD_WAV_PIO
        if (real_player && !b) {
            pcm_audio_in_stop();
        }
#endif
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
        nvs_get_u8("ayConfig", Config::ayConfig, sts);
        nvs_get_u8("turbosound", Config::turbosound, sts);
        nvs_get_u8("covox", Config::covox, sts);
#if !defined(PICO_RP2040)
        nvs_get_u8("throtling2", Config::throtling, sts);
#else
        nvs_get_u8("throtling1", Config::throtling, sts);
#endif
        nvs_get_b("CursorAsJoy", CursorAsJoy, sts);
        nvs_get_b("trdosFastMode", trdosFastMode, sts);
        nvs_get_b("trdosWriteProtect", trdosWriteProtect, sts);
        nvs_get_b("trdosSoundLed", trdosSoundLed, sts);
        nvs_get_u8("trdosBios", trdosBios, sts);
#if !PICO_RP2040
        nvs_get_u8("esxdos", esxdos, sts);
        // Migrate old bool key
        { bool old_divmmc = false; nvs_get_b("divmmc", old_divmmc, sts); if (old_divmmc && esxdos == 0) esxdos = 1; }
        nvs_get_str("esxdos_mmc", esxdos_mmc_image, sts);
        nvs_get_str("esxdos_hdf", esxdos_hdf_image[0], sts);
        nvs_get_str("esxdos_hd1", esxdos_hdf_image[1], sts);
#endif
        nvs_get_str("SNA_Path", FileUtils::SNA_Path, sts);
        nvs_get_str("TAP_Path", FileUtils::TAP_Path, sts);
        nvs_get_str("DSK_Path", FileUtils::DSK_Path, sts);
        nvs_get_str("ROM_Path", FileUtils::ROM_Path, sts);
        nvs_get_str("IMG_Path", FileUtils::IMG_Path, sts);
        nvs_get_str("ALL_Path", FileUtils::ALL_Path, sts);
        for (size_t i = 0; i < 6; ++i) {
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
        nvs_get_sc("AudVolume", Config::aud_volume, sts);
        // Try new format first, fallback to old bool-based format for migration
        if (!nvs_get_u8("hdmi_vmode", Config::hdmi_video_mode, sts)) {
            // Migration from old format
            bool fb = false, hb = false, fb60 = false;
            int old_mode = 0;
            nvs_get_b("full_border", fb, sts);
            nvs_get_b("half_border", hb, sts);
            nvs_get_b("full_border_60", fb60, sts);
            nvs_get_i("hdmi_video_mode", old_mode, sts);
            Config::hdmi_video_mode = hb ? VM_720x480_60 : fb60 ? VM_720x576_60 : fb ? VM_720x576_50 : (old_mode > 0 ? VM_640x480_50 : VM_640x480_60);
        }
        if (!nvs_get_u8("vga_vmode", Config::vga_video_mode, sts)) {
            int old_mode = 0;
            nvs_get_i("vga_video_mode", old_mode, sts);
            Config::vga_video_mode = old_mode > 0 ? VM_640x480_50 : VM_640x480_60;
        }
        nvs_get_b("v_sync_enabled", v_sync_enabled, sts);
        #if PICO_RP2350
        nvs_get_b("gigascreen_enabled", gigascreen_enabled, sts);
        nvs_get_u8("gigascreen_onoff", gigascreen_onoff, sts);
        #endif
        #if PICO_RP2040
        // RP2040 can't handle 720x modes — not enough RAM for framebuffer
        if (hdmi_video_mode >= VM_720x480_60) hdmi_video_mode = VM_640x480_60;
        if (vga_video_mode >= VM_720x480_60) vga_video_mode = VM_640x480_60;
        #endif
        #if !PICO_RP2040
        nvs_get_b("ulaplus", ulaplus, sts);
        #endif
        std::string v;
        nvs_get_str("audio_driver", v, sts);
        if (v == "pwm") Config::audio_driver = 1;
        else if (v == "i2s") Config::audio_driver = 2;
        else if (v == "ay") Config::audio_driver = 3;
        else if (v == "hdmi") Config::audio_driver = 4;
        nvs_get_str("video_driver", v, sts);
        if (v == "VGA" || v == "vga") video_driver = 1;
        else if (v == "HDMI" || v == "hdmi" || v == "DVI" || v == "dvi") video_driver = 2;
        nvs_get_b("byte_cobmect_mode", byte_cobmect_mode, sts);
        // Load hotkey bindings (defaults already set by initHotkeys() before load)
        for (int i = 0; i < HK_COUNT; i++) {
            char key[12];
            snprintf(key, sizeof(key), "hkVK%02d", i);
            nvs_get_u16(key, hotkeys[i].vk, sts);
            uint8_t mod = (hotkeys[i].alt ? 2 : 0) | (hotkeys[i].ctrl ? 1 : 0);
            snprintf(key, sizeof(key), "hkMod%02d", i);
            nvs_get_u8(key, mod, sts);
            hotkeys[i].alt  = (mod >> 1) & 1;
            hotkeys[i].ctrl = (mod     ) & 1;
        }
        int mem_pg_cnt = 0;
        nvs_get_i("MEM_PG_CNT", mem_pg_cnt, sts);
        if (mem_pg_cnt < 8 || mem_pg_cnt > 2048) MEM_PG_CNT = 64;
        #if PICO_RP2040
        else if (mem_pg_cnt > 512) MEM_PG_CNT = 512;
        #endif
        else MEM_PG_CNT = mem_pg_cnt;
    }
    loaded = true;
}

static void nvs_set_str(string& buf, const char* name, const char* val) {
    buf += name;
    buf += '=';
    buf += val;
    buf += '\n';
}
static void nvs_set_i(string& buf, const char* name, int val) {
    nvs_set_str(buf, name, to_string(val).c_str());
}
static void nvs_set_i8(string& buf, const char* name, int8_t val) {
    nvs_set_str(buf, name, to_string(val).c_str());
}
static void nvs_set_u8(string& buf, const char* name, uint8_t val) {
    nvs_set_str(buf, name, to_string(val).c_str());
}
static void nvs_set_u16(string& buf, const char* name, uint16_t val) {
    nvs_set_str(buf, name, to_string(val).c_str());
}
static void nvs_set_sc(string& buf, const char* name, signed char val) {
    nvs_set_str(buf, name, to_string(val).c_str());
}

// Dump actual config to FS
void Config::save() {
    static string buf;
    buf.clear();
    if (buf.capacity() < 2048) buf.reserve(2048);
    nvs_set_u16(buf,"cpu_mhz", cpu_mhz);
    nvs_set_u16(buf,"max_flash_freq", max_flash_freq);
    nvs_set_u16(buf,"max_psram_freq", max_psram_freq);

    #if TFT
    nvs_set_u8(buf,"TFT_FLAGS", TFT_FLAGS);
    nvs_set_u8(buf,"TFT_INVERSION", TFT_INVERSION);
    #endif
    nvs_set_str(buf,"arch",arch.c_str());
    nvs_set_str(buf,"romSet",romSet.c_str());
    nvs_set_str(buf,"romSet48",romSet48.c_str());
    nvs_set_str(buf,"romSet128",romSet128.c_str());
    nvs_set_str(buf,"romSetPent",romSetPent.c_str());
    nvs_set_str(buf,"romSetP512",romSetP512.c_str());
    nvs_set_str(buf,"romSetP1M",romSetP1M.c_str());
    nvs_set_str(buf,"pref_arch",pref_arch.c_str());
    nvs_set_str(buf,"pref_romSet_48",pref_romSet_48.c_str());
    nvs_set_str(buf,"pref_romSet_128",pref_romSet_128.c_str());
    nvs_set_str(buf,"pref_romSetPent",pref_romSetPent.c_str());
    nvs_set_str(buf,"pref_romSetP512",pref_romSetP512.c_str());
    nvs_set_str(buf,"pref_romSetP1M",pref_romSetP1M.c_str());
    nvs_set_str(buf,"ram",ram_file.c_str());
    nvs_set_str(buf,"slog",slog_on ? "true" : "false");
///        nvs_set_str(buf,"sdstorage", FileUtils::MountPoint);
///        nvs_set_str(buf,"asp169",aspect_16_9 ? "true" : "false");
    nvs_set_u8(buf,"language", Config::lang);
    nvs_set_str(buf,"AY48", AY48 ? "true" : "false");
#if !PICO_RP2040
    nvs_set_str(buf,"SAA1099", SAA1099 ? "true" : "false");
    nvs_set_u8(buf,"midi", midi);
    nvs_set_u8(buf,"midipreset", midi_synth_preset);
#endif
    nvs_set_u8(buf,"ayConfig", Config::ayConfig);
    nvs_set_u8(buf,"turbosound", Config::turbosound);
    nvs_set_u8(buf,"covox", Config::covox);
    nvs_set_str(buf,"Issue2", Issue2 ? "true" : "false");
    nvs_set_str(buf,"flashload", flashload ? "true" : "false");
    nvs_set_str(buf,"tape_player", tape_player ? "true" : "false");
    nvs_set_str(buf,"real_player", real_player ? "true" : "false");
    nvs_set_str(buf,"rightSpace", rightSpace ? "true" : "false");
    nvs_set_str(buf,"wasd", wasd ? "true" : "false");
    nvs_set_str(buf,"tape_timing_rg",tape_timing_rg ? "true" : "false");
    // Save typed breakpoints array
    for (int i = 0; i < MAX_BREAKPOINTS; i++) {
        char key[16];
        snprintf(key, sizeof(key), "bp%d", i);
        nvs_set_u16(buf, key, breakPoints[i].addr);
        snprintf(key, sizeof(key), "bpt%d", i);
        nvs_set_u8(buf, key, (uint8_t)breakPoints[i].type);
    }
    nvs_set_u8(buf,"joystick", Config::joystick);
    // Write joystick definition
    for (int n = 0; n < 12; ++n) {
        char joykey[16];
        snprintf(joykey, 16, "joydef%02u", n);
        nvs_set_u16(buf, joykey, Config::joydef[n]);
    }
    nvs_set_u8(buf,"AluTiming",Config::AluTiming);
    nvs_set_u8(buf,"joy2cursor",Config::joy2cursor);
    nvs_set_u8(buf,"secondJoy",Config::secondJoy);
    nvs_set_u8(buf,"kempstonPort",Config::kempstonPort);
#if !defined(PICO_RP2040)
    nvs_set_u8(buf,"throtling2",Config::throtling);
#else
    nvs_set_u8(buf,"throtling1",Config::throtling);
#endif
    nvs_set_str(buf,"CursorAsJoy", CursorAsJoy ? "true" : "false");
    nvs_set_str(buf,"trdosFastMode", trdosFastMode ? "true" : "false");
    nvs_set_str(buf,"trdosWriteProtect", trdosWriteProtect ? "true" : "false");
    nvs_set_str(buf,"trdosSoundLed", trdosSoundLed ? "true" : "false");
    nvs_set_u8(buf,"trdosBios", trdosBios);
#if !PICO_RP2040
    nvs_set_u8(buf,"esxdos", esxdos);
    nvs_set_str(buf,"esxdos_mmc", esxdos_mmc_image.c_str());
    nvs_set_str(buf,"esxdos_hdf", esxdos_hdf_image[0].c_str());
    nvs_set_str(buf,"esxdos_hd1", esxdos_hdf_image[1].c_str());
#endif
    nvs_set_str(buf,"SNA_Path",FileUtils::SNA_Path.c_str());
    nvs_set_str(buf,"TAP_Path",FileUtils::TAP_Path.c_str());
    nvs_set_str(buf,"DSK_Path",FileUtils::DSK_Path.c_str());
    nvs_set_str(buf,"ROM_Path",FileUtils::ROM_Path.c_str());
    nvs_set_str(buf,"IMG_Path",FileUtils::IMG_Path.c_str());
    nvs_set_str(buf,"ALL_Path",FileUtils::ALL_Path.c_str());
    for (size_t i = 0; i < 6; ++i) {
        const DISK_FTYPE& ft = FileUtils::fileTypes[i];
        string s = "fileTypes" + to_string(i);
        nvs_set_i(buf, (s + ".begin_row").c_str(), ft.begin_row);
        nvs_set_i(buf, (s + ".focus").c_str(), ft.focus);
        nvs_set_u8(buf, (s + ".fdMode").c_str(), ft.fdMode);
        nvs_set_str(buf, (s + ".fileSearch").c_str(), ft.fileSearch.c_str());
        if (i < 4) {
            s = "drive" + to_string(i);
            nvs_set_str(buf, (s + ".file").c_str(), ESPectrum::fdd.disk[i] ? ESPectrum::fdd.disk[i]->fname.c_str() : "");
        }
    }
    nvs_set_u8(buf,"scanlines",Config::scanlines);
    nvs_set_u8(buf,"render",Config::render);
    nvs_set_str(buf,"TABasfire1", TABasfire1 ? "true" : "false");
    nvs_set_str(buf,"StartMsg", StartMsg ? "true" : "false");
    nvs_set_sc(buf,"AudVolume", ESPectrum::aud_volume);
    nvs_set_u8(buf,"hdmi_vmode",Config::hdmi_video_mode);
    nvs_set_u8(buf,"vga_vmode",Config::vga_video_mode);
    nvs_set_str(buf,"v_sync_enabled", Config::v_sync_enabled ? "true" : "false");
    nvs_set_str(buf,"gigascreen_enabled", Config::gigascreen_enabled ? "true" : "false");
    nvs_set_u8(buf,"gigascreen_onoff", Config::gigascreen_onoff);
    #if !PICO_RP2040
    nvs_set_str(buf,"ulaplus", Config::ulaplus ? "true" : "false");
    #endif
    nvs_set_str(buf,"audio_driver", Config::audio_driver == 0 ? "auto" :
        (Config::audio_driver == 1) ? "pwm" : (Config::audio_driver == 2) ? "i2s" :
        (Config::audio_driver == 3) ? "ay" : "hdmi"
    );
    nvs_set_str(buf,"video_driver", video_driver == 0 ? "auto" : (video_driver == 1) ? "vga" : "hdmi");
    nvs_set_str(buf,"byte_cobmect_mode", Config::byte_cobmect_mode ? "true" : "false");
    // Save hotkey bindings
    for (int i = 0; i < HK_COUNT; i++) {
        char key[12];
        snprintf(key, sizeof(key), "hkVK%02d", i);
        nvs_set_u16(buf, key, hotkeys[i].vk);
        snprintf(key, sizeof(key), "hkMod%02d", i);
        uint8_t mod = (hotkeys[i].alt ? 2 : 0) | (hotkeys[i].ctrl ? 1 : 0);
        nvs_set_u8(buf, key, mod);
    }
    nvs_set_i(buf,"MEM_PG_CNT", MEM_PG_CNT);

    if (FileUtils::fsMount) {
        if (!loaded) {
            // Config was never loaded from file — refuse to overwrite
            // existing storage.nvs with defaults
            FILINFO fi;
            if (f_stat(MOUNT_POINT_SD STORAGE_NVS, &fi) == FR_OK) {
                Debug::log("Config::save BLOCKED — not loaded, file exists (%u bytes)", fi.fsize);
                return;
            }
        }
        // Atomic write: write to .tmp, then rename over the original
        static const char* nvs_tmp = MOUNT_POINT_SD STORAGE_NVS ".tmp";
        static const char* nvs_path = MOUNT_POINT_SD STORAGE_NVS;
        FIL* handle = fopen2(nvs_tmp, FA_WRITE | FA_CREATE_ALWAYS);
        if (handle) {
            UINT bw;
            FRESULT wr = f_write(handle, buf.c_str(), buf.size(), &bw);
            fclose2(handle);
            if (wr == FR_OK && bw == buf.size()) {
                f_unlink(nvs_path);
                f_rename(nvs_tmp, nvs_path);
            } else {
                // Write failed — remove incomplete temp, keep original intact
                f_unlink(nvs_tmp);
                Debug::log("Config::save FAILED — write error (wr=%d, bw=%u/%u)", wr, bw, buf.size());
            }
        }
    }
    // Always keep in RAM (for session persistence without SD)
    nvs_ram_buf = std::move(buf);
}

#define VMODE_PENDING_FILE MOUNT_POINT_SD "/vmode_pending.nvs"

void Config::savePendingVideoMode() {
    if (!FileUtils::fsMount) return;
    string buf;
    nvs_set_u8(buf, "hdmi_vmode", Config::hdmi_video_mode);
    nvs_set_u8(buf, "vga_vmode", Config::vga_video_mode);
    FIL* handle = fopen2(VMODE_PENDING_FILE, FA_WRITE | FA_CREATE_ALWAYS);
    if (handle) {
        UINT bw;
        f_write(handle, buf.c_str(), buf.size(), &bw);
        fclose2(handle);
    }
}

bool Config::loadPendingVideoMode(uint8_t &hdmi_vm, uint8_t &vga_vm) {
    if (!FileUtils::fsMount) return false;
    FIL* handle = fopen2(VMODE_PENDING_FILE, FA_READ);
    if (!handle) return false;

    vector<string> sts;
    UINT br;
    char c;
    string s;
    while (!f_eof(handle)) {
        if (f_read(handle, &c, 1, &br) != FR_OK) {
            fclose2(handle);
            return false;
        }
        if (c == '\n') {
            sts.push_back(s);
            s.clear();
        } else {
            s += c;
        }
    }
    fclose2(handle);

    nvs_get_u8("hdmi_vmode", hdmi_vm, sts);
    nvs_get_u8("vga_vmode", vga_vm, sts);
    return true;
}

void Config::clearPendingVideoMode() {
    f_unlink(VMODE_PENDING_FILE);
}

void Config::setJoyMap(uint8_t joytype) {
    for (int n = 0; n < 12; n++) joydef[n] = fabgl::VK_NONE;
    // Ask to overwrite map with default joytype values
    string title = "Joystick";
    string msg = OSD_DLG_SETJOYMAPDEFAULTS[Config::lang];
    uint8_t res = OSD::msgDialog(title, msg);
    if (res == DLG_YES) {
        joydef[0] = fabgl::VK_DPAD_LEFT;
        joydef[1] = fabgl::VK_DPAD_RIGHT;
        joydef[2] = fabgl::VK_DPAD_UP;
        joydef[3] = fabgl::VK_DPAD_DOWN;
        joydef[6] = fabgl::VK_DPAD_FIRE;
        if (joytype == JOY_KEMPSTON) {
            joydef[4] = fabgl::VK_DPAD_START;
            joydef[5] = fabgl::VK_DPAD_SELECT;
            joydef[7] = fabgl::VK_DPAD_ALTFIRE;
        }
        Config::save();
    }
}
