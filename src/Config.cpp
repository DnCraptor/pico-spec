#include "Config.h"
#include "MemESP.h"
#include "roms.h"
#include "FileUtils.h"
#include "fabutils.h"

string   Config::arch = "48K";
string   Config::romSet = "48K";
string   Config::romSet48 = "48K";
string   Config::romSet128 = "128K";
string   Config::pref_arch = "48K";
string   Config::pref_romSet_48 = "48K";
string   Config::pref_romSet_128 = "128K";
string   Config::ram_file = NO_RAM_FILE;
string   Config::last_ram_file = NO_RAM_FILE;

bool     Config::slog_on = false;
bool     Config::aspect_16_9 = false;
uint8_t  Config::videomode = 0; // 0 -> SAFE VGA, 1 -> 50HZ VGA, 2 -> 50HZ CRT
uint8_t  Config::esp32rev = 0;
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
            MemESP::rom[0] += 8;
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
            MemESP::rom[0] += 8;

            MemESP::rom[1] = (uint8_t *) gb_rom_0_128k_custom;
            MemESP::rom[1] += 16392;

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

        if (newRomSet=="") romSet = "Pentagon"; else romSet = newRomSet;

        MemESP::rom[0] = (uint8_t *) gb_rom_0_pentagon_128k;
        MemESP::rom[1] = (uint8_t *) gb_rom_1_pentagon_128k;

    }

    MemESP::rom[4] = (uint8_t *) gb_rom_4_trdos_503;

    // MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse];
    // MemESP::ramCurrent[1] = MemESP::ram[5];
    // MemESP::ramCurrent[2] = MemESP::ram[2];
    // MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch];

    // MemESP::ramContended[0] = false;
    // MemESP::ramContended[1] = arch == "Pentagon" ? false : true;
    // MemESP::ramContended[2] = false;
    // MemESP::ramContended[3] = false;
  
}

// Read config from FS
void Config::load() {
/**
    // Open
    // printf("\n");
    // printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t handle;
    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        // printf("Done\n");

        size_t required_size;
        char* str_data;
        
        err = nvs_get_str(handle, "arch", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "arch", str_data, &required_size);
            // printf("arch:%s\n",str_data);
            arch = str_data;
            
            // FORCE MODEL FOR TESTING
            // arch = "48K";
            
            free(str_data);
        } else {
            // No nvs data found. Save it
            nvs_close(handle);
            Config::save();
            return;
        }

        err = nvs_get_str(handle, "romSet", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "romSet", str_data, &required_size);
            // printf("romSet:%s\n",str_data);
            romSet = str_data;
            free(str_data);
        }

        err = nvs_get_str(handle, "romSet48", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "romSet48", str_data, &required_size);
            // printf("romSet48:%s\n",str_data);
            romSet48 = str_data;
            free(str_data);
        }

        err = nvs_get_str(handle, "romSet128", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "romSet128", str_data, &required_size);
            // printf("romSet128:%s\n",str_data);
            romSet128 = str_data;
            free(str_data);
        }

        err = nvs_get_str(handle, "pref_arch", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "pref_arch", str_data, &required_size);
            // printf("pref_arch:%s\n",str_data);
            pref_arch = str_data;
            free(str_data);
        }

        err = nvs_get_str(handle, "pref_romSet_48", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "pref_romSet_48", str_data, &required_size);
            // printf("pref_romSet_48:%s\n",str_data);
            pref_romSet_48 = str_data;
            free(str_data);
        }

        err = nvs_get_str(handle, "pref_romSet_128", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "pref_romSet_128", str_data, &required_size);
            // printf("pref_romSet_128:%s\n",str_data);
            pref_romSet_128 = str_data;
            free(str_data);
        }

        err = nvs_get_str(handle, "ram", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "ram", str_data, &required_size);
            // printf("ram:%s\n",str_data);
            ram_file = str_data;
            free(str_data);
        }

        err = nvs_get_str(handle, "slog", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "slog", str_data, &required_size);
            // printf("slog:%s\n",str_data);
            slog_on = strcmp(str_data, "false");            
            free(str_data);

            // slog_on = true; // Force for testing

        }

        err = nvs_get_str(handle, "sdstorage", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "sdstorage", str_data, &required_size);
            // printf("sdstorage:%s\n",str_data);

            // Force SD from now on
            FileUtils::MountPoint = MOUNT_POINT_SD;

            free(str_data);
        }

        err = nvs_get_str(handle, "asp169", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "asp169", str_data, &required_size);
            // printf("asp169:%s\n",str_data);
            aspect_16_9 = strcmp(str_data, "false");
            free(str_data);
        }

        err = nvs_get_u8(handle, "videomode", &Config::videomode);
        if (err == ESP_OK) {
            // printf("videomode:%u\n",Config::videomode);
        }


        err = nvs_get_u8(handle, "language", &Config::lang);
        if (err == ESP_OK) {
            // printf("language:%u\n",Config::lang);
        }

        err = nvs_get_str(handle, "AY48", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "AY48", str_data, &required_size);
            // printf("AY48:%s\n",str_data);
            AY48 = strcmp(str_data, "false");
            free(str_data);
        }

        err = nvs_get_str(handle, "Issue2", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "Issue2", str_data, &required_size);
            // printf("Issue2:%s\n",str_data);
            Issue2 = strcmp(str_data, "false");
            free(str_data);
        }

        err = nvs_get_str(handle, "flashload", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "flashload", str_data, &required_size);
            // printf("Flashload:%s\n",str_data);
            flashload = strcmp(str_data, "false");
            free(str_data);
        }

        err = nvs_get_str(handle, "tape_player", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "tape_player", str_data, &required_size);
            // printf("Tape player:%s\n",str_data);
            tape_player = strcmp(str_data, "false");
            free(str_data);
        }

        err = nvs_get_str(handle, "tape_timing_rg", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "tape_timing_rg", str_data, &required_size);
            // printf("Tape timing RG:%s\n",str_data);
            tape_timing_rg = strcmp(str_data, "false");
            free(str_data);
        }

        err = nvs_get_u8(handle, "joystick1", &Config::joystick1);
        if (err == ESP_OK) {
            // printf("joystick1:%u\n",Config::joystick1);
        }

        err = nvs_get_u8(handle, "joystick2", &Config::joystick2);
        if (err == ESP_OK) {
            // printf("joystick2:%u\n",Config::joystick2);
        }

        // Read joystick definition
        for (int n=0; n < 24; n++) {
            char joykey[9];
            sprintf(joykey,"joydef%02u",n);
            // printf("%s\n",joykey);
            err = nvs_get_u16(handle, joykey, &Config::joydef[n]);
            if (err == ESP_OK) {
                // printf("joydef00:%u\n",Config::joydef[n]);
            }
        }

        err = nvs_get_u8(handle, "joyPS2", &Config::joyPS2);
        if (err == ESP_OK) {
            // printf("joyPS2:%u\n",Config::joyPS2);
        }

        err = nvs_get_u8(handle, "AluTiming", &Config::AluTiming);
        if (err == ESP_OK) {
            // printf("AluTiming:%u\n",Config::AluTiming);
        }

        err = nvs_get_u8(handle, "PS2Dev2", &Config::ps2_dev2);
        if (err == ESP_OK) {
            // printf("PS2Dev2:%u\n",Config::ps2_dev2);
        }

        err = nvs_get_str(handle, "CursorAsJoy", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "CursorAsJoy", str_data, &required_size);
            // printf("CursorAsJoy:%s\n",str_data);
            CursorAsJoy = strcmp(str_data, "false");
            free(str_data);
        }

        err = nvs_get_i8(handle, "CenterH", &Config::CenterH);
        if (err == ESP_OK) {
            // printf("PS2Dev2:%u\n",Config::ps2_dev2);
        }

        err = nvs_get_i8(handle, "CenterV", &Config::CenterV);
        if (err == ESP_OK) {
            // printf("PS2Dev2:%u\n",Config::ps2_dev2);
        }

        err = nvs_get_str(handle, "SNA_Path", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "SNA_Path", str_data, &required_size);
            // printf("SNA_Path:%s\n",str_data);
            SNA_Path = str_data;
            free(str_data);
        }

        err = nvs_get_str(handle, "TAP_Path", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "TAP_Path", str_data, &required_size);
            // printf("TAP_Path:%s\n",str_data);
            TAP_Path = str_data;
            free(str_data);
        }

        err = nvs_get_str(handle, "DSK_Path", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "DSK_Path", str_data, &required_size);
            // printf("DSK_Path:%s\n",str_data);
            DSK_Path = str_data;
            free(str_data);
        }

        err = nvs_get_u16(handle, "SNA_begin_row", &Config::SNA_begin_row);
        if (err == ESP_OK) {
            // printf("SNA_begin_row:%u\n",Config::SNA_begin_row);
        }

        err = nvs_get_u16(handle, "TAP_begin_row", &Config::TAP_begin_row);
        if (err == ESP_OK) {
            // printf("TAP_begin_row:%u\n",Config::TAP_begin_row);
        }

        err = nvs_get_u16(handle, "DSK_begin_row", &Config::DSK_begin_row);
        if (err == ESP_OK) {
            // printf("begin_row:%u\n",Config::DSK_begin_row);
        }

        err = nvs_get_u16(handle, "SNA_focus", &Config::SNA_focus);
        if (err == ESP_OK) {
            // printf("SNA_focus:%u\n",Config::SNA_focus);
        }

        err = nvs_get_u16(handle, "TAP_focus", &Config::TAP_focus);
        if (err == ESP_OK) {
            // printf("TAP_focus:%u\n",Config::TAP_focus);
        }

        err = nvs_get_u16(handle, "DSK_focus", &Config::DSK_focus);
        if (err == ESP_OK) {
            // printf("DSK_focus:%u\n",Config::DSK_focus);
        }

        err = nvs_get_u8(handle, "SNA_fdMode", &Config::SNA_fdMode);
        if (err == ESP_OK) {
            // printf("SNA_fdMode:%u\n",Config::SNA_fdMode);
        }

        err = nvs_get_u8(handle, "TAP_fdMode", &Config::TAP_fdMode);
        if (err == ESP_OK) {
            // printf("TAP_fdMode:%u\n",Config::TAP_fdMode);
        }

        err = nvs_get_u8(handle, "DSK_fdMode", &Config::DSK_fdMode);
        if (err == ESP_OK) {
            // printf("DSK_fdMode:%u\n",Config::DSK_fdMode);
        }

        err = nvs_get_str(handle, "SNA_fileSearch", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "SNA_fileSearch", str_data, &required_size);
            // printf("SNA_fileSearch:%s\n",str_data);
            SNA_fileSearch = str_data;
            free(str_data);
        }

        err = nvs_get_str(handle, "TAP_fileSearch", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "TAP_fileSearch", str_data, &required_size);
            // printf("TAP_fileSearch:%s\n",str_data);
            TAP_fileSearch = str_data;
            free(str_data);
        }

        err = nvs_get_str(handle, "DSK_fileSearch", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "DSK_fileSearch", str_data, &required_size);
            // printf("DSK_fileSearch:%s\n",str_data);
            DSK_fileSearch = str_data;
            free(str_data);
        }

        err = nvs_get_u8(handle, "scanlines", &Config::scanlines);
        if (err == ESP_OK) {
            // printf("scanlines:%u\n",Config::scanlines);
        }

        err = nvs_get_u8(handle, "render", &Config::render);
        if (err == ESP_OK) {
            // printf("render:%u\n",Config::render);
        }

        err = nvs_get_str(handle, "TABasfire1", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "TABasfire1", str_data, &required_size);
            // printf("TABasfire1:%s\n",str_data);
            TABasfire1 = strcmp(str_data, "false");
            free(str_data);
        }

        err = nvs_get_str(handle, "StartMsg", NULL, &required_size);
        if (err == ESP_OK) {
            str_data = (char *)malloc(required_size);
            nvs_get_str(handle, "StartMsg", str_data, &required_size);
            // printf("StartMsg:%s\n",str_data);
            StartMsg = strcmp(str_data, "false");
            free(str_data);
        }

        // Close
        nvs_close(handle);
    }
*/
}

void Config::save() {
    Config::save("all");
}

// Dump actual config to FS
void Config::save(string value) {
/***
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    // printf("\n");
    // printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t handle;
    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        // printf("Done\n");


        if((value=="arch") || (value=="all"))
            nvs_set_str(handle,"arch",arch.c_str());

        if((value=="romSet") || (value=="all"))
            nvs_set_str(handle,"romSet",romSet.c_str());

        if((value=="romSet48") || (value=="all"))
            nvs_set_str(handle,"romSet48",romSet48.c_str());

        if((value=="romSet128") || (value=="all"))
            nvs_set_str(handle,"romSet128",romSet128.c_str());

        if((value=="pref_arch") || (value=="all"))
            nvs_set_str(handle,"pref_arch",pref_arch.c_str());

        if((value=="pref_romSet_48") || (value=="all"))
            nvs_set_str(handle,"pref_romSet_48",pref_romSet_48.c_str());

        if((value=="pref_romSet_128") || (value=="all"))
            nvs_set_str(handle,"pref_romSet_128",pref_romSet_128.c_str());

        if((value=="ram") || (value=="all"))
            nvs_set_str(handle,"ram",ram_file.c_str());   

        if((value=="slog") || (value=="all"))
            nvs_set_str(handle,"slog",slog_on ? "true" : "false");

        if((value=="sdstorage") || (value=="all"))
            nvs_set_str(handle,"sdstorage",FileUtils::MountPoint == MOUNT_POINT_SD ? "true" : "false");

        if((value=="asp169") || (value=="all"))
            nvs_set_str(handle,"asp169",aspect_16_9 ? "true" : "false");

        if((value=="videomode") || (value=="all"))
            nvs_set_u8(handle,"videomode",Config::videomode);

        if((value=="language") || (value=="all"))
            nvs_set_u8(handle,"language",Config::lang);

        if((value=="AY48") || (value=="all"))
            nvs_set_str(handle,"AY48",AY48 ? "true" : "false");

        if((value=="Issue2") || (value=="all"))
            nvs_set_str(handle,"Issue2",Issue2 ? "true" : "false");

        if((value=="flashload") || (value=="all"))
            nvs_set_str(handle,"flashload",flashload ? "true" : "false");

        if((value=="tape_player") || (value=="all"))
            nvs_set_str(handle,"tape_player",tape_player ? "true" : "false");

        if((value=="tape_timing_rg") || (value=="all"))
            nvs_set_str(handle,"tape_timing_rg",tape_timing_rg ? "true" : "false");

        if((value=="joystick1") || (value=="all"))
            nvs_set_u8(handle,"joystick1",Config::joystick1);

        if((value=="joystick2") || (value=="all"))
            nvs_set_u8(handle,"joystick2",Config::joystick2);

        // Write joystick definition
        for (int n=0; n < 24; n++) {
            char joykey[9];
            sprintf(joykey,"joydef%02u",n);
            if((value == joykey) || (value=="all")) {
                nvs_set_u16(handle,joykey,Config::joydef[n]);
                // printf("%s %u\n",joykey, joydef[n]);
            }
        }

        if((value=="joyPS2") || (value=="all"))
            nvs_set_u8(handle,"joyPS2",Config::joyPS2);

        if((value=="AluTiming") || (value=="all"))
            nvs_set_u8(handle,"AluTiming",Config::AluTiming);

        if((value=="PS2Dev2") || (value=="all"))
            nvs_set_u8(handle,"PS2Dev2",Config::ps2_dev2);

        if((value=="CursorAsJoy") || (value=="all"))
            nvs_set_str(handle,"CursorAsJoy", CursorAsJoy ? "true" : "false");

        if((value=="CenterH") || (value=="all"))
            nvs_set_i8(handle,"CenterH",Config::CenterH);

        if((value=="CenterV") || (value=="all"))
            nvs_set_i8(handle,"CenterV",Config::CenterV);

        if((value=="SNA_Path") || (value=="all"))
            nvs_set_str(handle,"SNA_Path",Config::SNA_Path.c_str());

        if((value=="TAP_Path") || (value=="all"))
            nvs_set_str(handle,"TAP_Path",Config::TAP_Path.c_str());

        if((value=="DSK_Path") || (value=="all"))
            nvs_set_str(handle,"DSK_Path",Config::DSK_Path.c_str());

        if((value=="SNA_begin_row") || (value=="all"))
            nvs_set_u16(handle,"SNA_begin_row",Config::SNA_begin_row);

        if((value=="TAP_begin_row") || (value=="all"))
            nvs_set_u16(handle,"TAP_begin_row",Config::TAP_begin_row);

        if((value=="DSK_begin_row") || (value=="all"))
            nvs_set_u16(handle,"DSK_begin_row",Config::DSK_begin_row);

        if((value=="SNA_focus") || (value=="all"))
            nvs_set_u16(handle,"SNA_focus",Config::SNA_focus);

        if((value=="TAP_focus") || (value=="all"))
            nvs_set_u16(handle,"TAP_focus",Config::TAP_focus);

        if((value=="DSK_focus") || (value=="all"))
            nvs_set_u16(handle,"DSK_focus",Config::DSK_focus);

        if((value=="SNA_fdMode") || (value=="all"))
            nvs_set_u8(handle,"SNA_fdMode",Config::SNA_fdMode);

        if((value=="TAP_fdMode") || (value=="all"))
            nvs_set_u8(handle,"TAP_fdMode",Config::TAP_fdMode);

        if((value=="DSK_fdMode") || (value=="all"))
            nvs_set_u8(handle,"DSK_fdMode",Config::DSK_fdMode);

        if((value=="SNA_fileSearch") || (value=="all"))
            nvs_set_str(handle,"SNA_fileSearch",Config::SNA_fileSearch.c_str());

        if((value=="TAP_fileSearch") || (value=="all"))
            nvs_set_str(handle,"TAP_fileSearch",Config::TAP_fileSearch.c_str());

        if((value=="DSK_fileSearch") || (value=="all"))
            nvs_set_str(handle,"DSK_fileSearch",Config::DSK_fileSearch.c_str());

        if((value=="scanlines") || (value=="all"))
            nvs_set_u8(handle,"scanlines",Config::scanlines);

        if((value=="render") || (value=="all"))
            nvs_set_u8(handle,"render",Config::render);

        if((value=="TABasfire1") || (value=="all"))
            nvs_set_str(handle,"TABasfire1", TABasfire1 ? "true" : "false");

        if((value=="StartMsg") || (value=="all"))
            nvs_set_str(handle,"StartMsg", StartMsg ? "true" : "false");

        // printf("Committing updates in NVS ... ");

        err = nvs_commit(handle);
        if (err != ESP_OK) {
            printf("Error (%s) commiting updates to NVS!\n", esp_err_to_name(err));
        }
        
        // printf("Done\n");

        // Close
        nvs_close(handle);
    }
    // printf("Config saved OK\n");
*/
}
