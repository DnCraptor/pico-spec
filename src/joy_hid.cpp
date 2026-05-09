#include <host/usbh.h>
#include <string.h>
#include <stdio.h>
#include "xinput_host.h"
#include "tusb.h"

struct input_bits_t {
    bool a: true;
    bool b: true;
    bool select: true;
    bool start: true;
    bool right: true;
    bool left: true;
    bool up: true;
    bool down: true;
};

extern input_bits_t gamepad1_bits;

//Since https://github.com/hathach/tinyusb/pull/2222, we can add in custom vendor drivers easily
usbh_class_driver_t const* usbh_app_driver_get_cb(uint8_t* driver_count) {
    *driver_count = 1;
    return &usbh_xinput_driver;
}

// Snapshot for OSD "HID devices" dialog (XInput side).
#ifndef CFG_TUH_XINPUT
#define CFG_TUH_XINPUT 1
#endif
struct xinput_snap_t {
    bool     mounted;
    uint8_t  dev_addr;
    uint8_t  type;
    uint8_t  connected;
    uint16_t vid;
    uint16_t pid;
    uint32_t report_total;
    uint16_t last_pad_size;
    uint8_t  last_pad[16]; // raw xinput_gamepad_t bytes
};
static xinput_snap_t xinput_snap[CFG_TUH_XINPUT];

#include "Config.h"
#include "ESPectrum.h"
#include "Video.h"

// Mapping mirrors hid_app.cpp process_ds4_gamepad/process_hid_gamepad:
// D-pad+stick → Kempston (VK_DPAD_*) + OSD nav (VK_MENU_*), no cursor
// overlay. A=Fire, B=AltFire, Start=F1/Start, Back=Select, X/Y/LB/RB →
// VK_JOY_X/Y/Z/C for joydef rebinding. Left analog stick falls back to
// D-pad direction so games requiring Kempston work without the hat.
void tuh_xinput_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    auto xid_itf = (xinputh_interface_t *)report;
    const xinput_gamepad_t* p = &xid_itf->pad;

    if (instance < CFG_TUH_XINPUT) {
        xinput_snap_t& s = xinput_snap[instance];
        s.report_total++;
        s.connected = xid_itf->connected;
        uint16_t n = sizeof(s.last_pad) < sizeof(xinput_gamepad_t) ? sizeof(s.last_pad) : sizeof(xinput_gamepad_t);
        memcpy(s.last_pad, p, n);
        s.last_pad_size = n;
    }

    const uint16_t btns = p->wButtons;

    bool dpad_up    = (btns & XINPUT_GAMEPAD_DPAD_UP)    != 0;
    bool dpad_down  = (btns & XINPUT_GAMEPAD_DPAD_DOWN)  != 0;
    bool dpad_left  = (btns & XINPUT_GAMEPAD_DPAD_LEFT)  != 0;
    bool dpad_right = (btns & XINPUT_GAMEPAD_DPAD_RIGHT) != 0;

    // Stick fallback: ~25% deflection threshold (sThumb is int16, ±32767).
    bool stick_up    = p->sThumbLY >  8000;
    bool stick_down  = p->sThumbLY < -8000;
    bool stick_left  = p->sThumbLX < -8000;
    bool stick_right = p->sThumbLX >  8000;

    bool up    = dpad_up    || stick_up;
    bool down  = dpad_down  || stick_down;
    bool left  = dpad_left  || stick_left;
    bool right = dpad_right || stick_right;

    bool btn_a     = (btns & XINPUT_GAMEPAD_A) != 0;
    bool btn_b     = (btns & XINPUT_GAMEPAD_B) != 0;
    bool btn_x     = (btns & XINPUT_GAMEPAD_X) != 0;
    bool btn_y     = (btns & XINPUT_GAMEPAD_Y) != 0;
    bool btn_lb    = (btns & XINPUT_GAMEPAD_LEFT_SHOULDER)  != 0;
    bool btn_rb    = (btns & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0;
    bool btn_start = (btns & XINPUT_GAMEPAD_START) != 0;
    bool btn_back  = (btns & XINPUT_GAMEPAD_BACK)  != 0;

    if (up != gamepad1_bits.up) {
        joyPushData(fabgl::VirtualKey::VK_MENU_UP, up);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_UP, up);
    }
    if (down != gamepad1_bits.down) {
        joyPushData(fabgl::VirtualKey::VK_MENU_DOWN, down);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_DOWN, down);
    }
    if (left != gamepad1_bits.left) {
        joyPushData(fabgl::VirtualKey::VK_MENU_LEFT, left);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_LEFT, left);
    }
    if (right != gamepad1_bits.right) {
        joyPushData(fabgl::VirtualKey::VK_MENU_RIGHT, right);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_RIGHT, right);
    }

    if (btn_a != gamepad1_bits.b) {
        joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, btn_a);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_FIRE, btn_a);
    }
    if (btn_b != gamepad1_bits.a) {
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, btn_b);
    }
    if (btn_start != gamepad1_bits.start) {
        joyPushData(fabgl::VirtualKey::VK_MENU_HOME, btn_start);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_START, btn_start);
    }
    if (btn_back != gamepad1_bits.select) {
        joyPushData(fabgl::VirtualKey::VK_MENU_BS, btn_back);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_SELECT, btn_back);
    }

    static bool prev_x = false, prev_y = false, prev_lb = false, prev_rb = false;
    if (btn_x  != prev_x)  { kbdPushData(fabgl::VirtualKey::VK_JOY_X, btn_x);  prev_x  = btn_x; }
    if (btn_y  != prev_y)  { kbdPushData(fabgl::VirtualKey::VK_JOY_Y, btn_y);  prev_y  = btn_y; }
    if (btn_lb != prev_lb) { kbdPushData(fabgl::VirtualKey::VK_JOY_Z, btn_lb); prev_lb = btn_lb; }
    if (btn_rb != prev_rb) { kbdPushData(fabgl::VirtualKey::VK_JOY_C, btn_rb); prev_rb = btn_rb; }

    gamepad1_bits.up     = up;
    gamepad1_bits.down   = down;
    gamepad1_bits.left   = left;
    gamepad1_bits.right  = right;
    gamepad1_bits.a      = btn_b;
    gamepad1_bits.b      = btn_a;
    gamepad1_bits.start  = btn_start;
    gamepad1_bits.select = btn_back;

    tuh_xinput_receive_report(dev_addr, instance);
}

void tuh_xinput_mount_cb(uint8_t dev_addr, uint8_t instance, const xinputh_interface_t* xinput_itf) {
    TU_LOG1("XINPUT MOUNTED %02x %d\n", dev_addr, instance);

    if (instance < CFG_TUH_XINPUT) {
        xinput_snap_t& s = xinput_snap[instance];
        s.mounted = true;
        s.dev_addr = dev_addr;
        s.type = xinput_itf->type;
        s.connected = xinput_itf->connected;
        s.vid = 0;
        s.pid = 0;
        tuh_vid_pid_get(dev_addr, &s.vid, &s.pid);
        s.report_total = 0;
        s.last_pad_size = 0;
    }

    // If this is a Xbox 360 Wireless controller we need to wait for a connection packet
    // on the in pipe before setting LEDs etc. So just start getting data until a controller is connected.
    if (xinput_itf->type == XBOX360_WIRELESS && xinput_itf->connected == false) {
        tuh_xinput_receive_report(dev_addr, instance);
        return;
    }

    tuh_xinput_set_led(dev_addr, instance, 0, true);
    tuh_xinput_set_led(dev_addr, instance, 1, true);
    tuh_xinput_set_rumble(dev_addr, instance, 0, 0, true);
    tuh_xinput_receive_report(dev_addr, instance);
}

void tuh_xinput_umount_cb(uint8_t dev_addr, uint8_t instance) {
    TU_LOG1("XINPUT UNMOUNTED %02x %d\n", dev_addr, instance);
    if (instance < CFG_TUH_XINPUT) {
        xinput_snap[instance].mounted = false;
    }
}

extern "C" int xinput_app_format_devices_info(char* buf, int bufsz)
{
    static const char* type_name[] = { "Unknown", "XboxOne", "X360Wireless", "X360Wired", "XboxOG" };
    int pos = 0;
    if (bufsz > 0) buf[0] = '\0';

    for (uint8_t inst = 0; inst < CFG_TUH_XINPUT; inst++) {
        const xinput_snap_t& s = xinput_snap[inst];
        if (!s.mounted) continue;
        const char* tname = (s.type < 5) ? type_name[s.type] : "?";
        pos += snprintf(buf + pos, bufsz - pos,
            "XInput %u  addr=%u  %s\n"
            " VID:PID = %04X:%04X\n"
            " connected = %u\n"
            " reports rx = %lu\n",
            inst, s.dev_addr, tname,
            s.vid, s.pid,
            s.connected,
            (unsigned long)s.report_total);
        if (s.last_pad_size) {
            pos += snprintf(buf + pos, bufsz - pos, " last pad:\n ");
            for (uint16_t i = 0; i < s.last_pad_size && pos < bufsz - 8; i++) {
                pos += snprintf(buf + pos, bufsz - pos, "%02X ", s.last_pad[i]);
                if ((i & 0x07) == 0x07 && i + 1 < s.last_pad_size)
                    pos += snprintf(buf + pos, bufsz - pos, "\n ");
            }
            pos += snprintf(buf + pos, bufsz - pos, "\n");
        }
        pos += snprintf(buf + pos, bufsz - pos, "\n");
    }
    return pos;
}
