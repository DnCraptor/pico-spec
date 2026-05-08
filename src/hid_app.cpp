/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021, Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "bsp/board_api.h"
#include "tusb.h"
#include "Debug.h"
#include "Config.h"
#include "ESPectrum.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

// If your host terminal support ansi escape code such as TeraTerm
// it can be use to simulate mouse cursor movement within terminal
#define USE_ANSI_ESCAPE   0

#define MAX_REPORT  4

static uint8_t const keycode2ascii[128][2] =  { HID_KEYCODE_TO_ASCII };

// Each HID instance can has multiple reports
static struct
{
  uint8_t report_count;
  tuh_hid_report_info_t report_info[MAX_REPORT];
  uint16_t vid;
  uint16_t pid;
}hid_info[CFG_TUH_HID];

// Sony DualShock 4 (PS4 wired): VID=054C, PID=05C4 (v1) / 09CC (v2).
// 507-byte report descriptor with vendor-page reports overflows TinyUSB's
// parser, so we bypass the rpt_info table for these and decode the 64-byte
// input report (id=0x01) by hand.
static inline bool is_dualshock4(uint16_t vid, uint16_t pid)
{
  return vid == 0x054C && (pid == 0x05C4 || pid == 0x09CC);
}

// Sony DualSense (PS5): VID=054C, PID=0CE6. Same descriptor-overflow
// problem as DS4; layout differs (L2/R2 analog moved to bytes 4-5,
// face/dpad to byte 7, shoulders to byte 8).
static inline bool is_dualsense(uint16_t vid, uint16_t pid)
{
  return vid == 0x054C && pid == 0x0CE6;
}

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
///extern input_bits_t keyboard_bits;
extern input_bits_t gamepad1_bits;

void process_kbd_report(
  hid_keyboard_report_t const *report,
  hid_keyboard_report_t const *prev_report
);

static void process_mouse_report(hid_mouse_report_t const * report);
static void process_generic_report(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len);
static void process_hid_gamepad(uint8_t const* report, uint16_t len);
static void process_ds4_gamepad(uint8_t const* report, uint16_t len);
static void process_ds5_gamepad(uint8_t const* report, uint16_t len);

//--------------------------------------------------------------------+
// TinyUSB Callbacks
//--------------------------------------------------------------------+

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
  printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);

  // Interface protocol (hid_interface_protocol_enum_t)
  const char* protocol_str[] = { "None", "Keyboard", "Mouse" };
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  printf("HID Interface Protocol = %s\r\n", protocol_str[itf_protocol]);

  uint16_t vid = 0, pid = 0;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  hid_info[instance].vid = vid;
  hid_info[instance].pid = pid;
  Debug::log("HID mount: addr=%u inst=%u VID=%04X PID=%04X proto=%u desc_len=%u",
             dev_addr, instance, vid, pid, itf_protocol, desc_len);

  // By default host stack will use activate boot protocol on supported interface.
  // Therefore for this simple example, we only need to parse generic report descriptor (with built-in parser)
  if ( itf_protocol == HID_ITF_PROTOCOL_NONE )
  {
    hid_info[instance].report_count = tuh_hid_parse_report_descriptor(hid_info[instance].report_info, MAX_REPORT, desc_report, desc_len);
    printf("HID has %u reports \r\n", hid_info[instance].report_count);
    Debug::log("HID generic: %u reports", hid_info[instance].report_count);
    for (uint8_t i = 0; i < hid_info[instance].report_count; i++) {
      Debug::log("  rpt[%u]: id=%u usage_page=%04X usage=%04X",
                 i,
                 hid_info[instance].report_info[i].report_id,
                 hid_info[instance].report_info[i].usage_page,
                 hid_info[instance].report_info[i].usage);
    }
  }

  // request to receive report
  // tuh_hid_report_received_cb() will be invoked when report is available
  if ( !tuh_hid_receive_report(dev_addr, instance) )
  {
    printf("Error: cannot request to receive report\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
}

static hid_keyboard_report_t prev_report = { 0 , 0 , {0}};

#include "ff.h"

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
/*
  FIL f;
  f_open(&f, "1.log", FA_OPEN_APPEND | FA_WRITE);
  char tmp[64];
  snprintf(tmp, 64, "USB report itf_protocol %d; len: %d mod: %02Xh kc0: %02Xh\n",
      itf_protocol,
      len,
      ((hid_keyboard_report_t const*)report)->modifier,
      ((hid_keyboard_report_t const*)report)->keycode[0]
  );
  UINT bw;
  f_write(&f, tmp, strlen(tmp), &bw);
  f_close(&f);
*/
  switch (itf_protocol)
  {
    case HID_ITF_PROTOCOL_KEYBOARD:
      TU_LOG2("HID receive boot keyboard report\r\n");
      process_kbd_report( (hid_keyboard_report_t const*) report, &prev_report );
      prev_report = *(hid_keyboard_report_t const*)report;
    break;

    case HID_ITF_PROTOCOL_MOUSE:
      TU_LOG2("HID receive boot mouse report\r\n");
      process_mouse_report( (hid_mouse_report_t const*) report );
    break;

    default:
      // Generic report requires matching ReportID and contents with previous parsed report info
      process_generic_report(dev_addr, instance, report, len);
    break;
  }

  // continue to request to receive report
  if ( !tuh_hid_receive_report(dev_addr, instance) )
  {
    printf("Error: cannot request to receive report\r\n");
  }
}

//--------------------------------------------------------------------+
// Keyboard
//--------------------------------------------------------------------+

// look up new key in previous keys
static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}

//--------------------------------------------------------------------+
// Mouse
//--------------------------------------------------------------------+

void cursor_movement(int8_t x, int8_t y, int8_t wheel)
{
#if USE_ANSI_ESCAPE
  // Move X using ansi escape
  if ( x < 0)
  {
    printf(ANSI_CURSOR_BACKWARD(%d), (-x)); // move left
  }else if ( x > 0)
  {
    printf(ANSI_CURSOR_FORWARD(%d), x); // move right
  }

  // Move Y using ansi escape
  if ( y < 0)
  {
    printf(ANSI_CURSOR_UP(%d), (-y)); // move up
  }else if ( y > 0)
  {
    printf(ANSI_CURSOR_DOWN(%d), y); // move down
  }

  // Scroll using ansi escape
  if (wheel < 0)
  {
    printf(ANSI_SCROLL_UP(%d), (-wheel)); // scroll up
  }else if (wheel > 0)
  {
    printf(ANSI_SCROLL_DOWN(%d), wheel); // scroll down
  }

  printf("\r\n");
#else
  printf("(%d %d %d)\r\n", x, y, wheel);
#endif
}

#include "ESPectrum.h"

static void process_mouse_report(hid_mouse_report_t const * report)
{
    ESPectrum::mouseButtonL = report->buttons & MOUSE_BUTTON_LEFT;
    ESPectrum::mouseButtonR = report->buttons & MOUSE_BUTTON_RIGHT;
    ESPectrum::mouseX += report->x >> 2;
    ESPectrum::mouseY -= report->y >> 2; // TODO: DPI
  /**

  //------------- button state  -------------//
  uint8_t button_changed_mask = report->buttons ^ prev_report.buttons;
  if ( button_changed_mask & report->buttons)
  {
    printf(" %c%c%c ",
       report->buttons & MOUSE_BUTTON_LEFT   ? 'L' : '-',
       report->buttons & MOUSE_BUTTON_MIDDLE ? 'M' : '-',
       report->buttons & MOUSE_BUTTON_RIGHT  ? 'R' : '-');
  }

  //------------- cursor movement -------------//
  cursor_movement(report->x, report->y, report->wheel);
  */
}

//--------------------------------------------------------------------+
// HID Gamepad / Joystick
//--------------------------------------------------------------------+
// Decoded layout for cheap SNES-style USB pads (e.g. VID=081F PID=E401).
// 8-byte report:
//   [0] X axis  : 0x00=Left, 0x7F=center, 0xFF=Right
//   [1] Y axis  : 0x00=Up,   0x7F=center, 0xFF=Down
//   [5] high nibble = face buttons : 0x10=X, 0x20=A, 0x40=B, 0x80=Y
//   [6] shoulders/system           : 0x01=L, 0x02=R, 0x10=Select, 0x20=Start
// Mapping to emulator: D-pad and B (fire) feed Kempston via VK_DPAD_*; A is
// alt-fire; Start opens OSD (F1); Select is Backspace in menu.
static void process_hid_gamepad(uint8_t const* report, uint16_t len)
{
    if (len < 7) return;

    bool left  = report[0] < 0x40;
    bool right = report[0] > 0xC0;
    bool up    = report[1] < 0x40;
    bool down  = report[1] > 0xC0;

    uint8_t b5 = report[5];
    uint8_t b6 = report[6];
    bool btn_a     = (b5 & 0x20) != 0;
    bool btn_b     = (b5 & 0x40) != 0;
    bool btn_x     = (b5 & 0x10) != 0;
    bool btn_y     = (b5 & 0x80) != 0;
    bool btn_l     = (b6 & 0x01) != 0;
    bool btn_r     = (b6 & 0x02) != 0;
    bool btn_sel   = (b6 & 0x10) != 0;
    bool btn_start = (b6 & 0x20) != 0;

    // D-pad (edge-detected, drives Kempston via VK_DPAD_*)
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

    // B = fire (yellow bottom button — most ergonomic on SNES-style pad)
    if (btn_b != gamepad1_bits.b) {
        joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, btn_b);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_FIRE, btn_b);
    }
    // A = alt-fire
    if (btn_a != gamepad1_bits.a) {
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, btn_a);
    }
    // Start → OSD menu (F1), also DPAD_START for joydef
    if (btn_start != gamepad1_bits.start) {
        joyPushData(fabgl::VirtualKey::VK_MENU_HOME, btn_start);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_START, btn_start);
    }
    // Select → backspace in menu / DPAD_SELECT
    if (btn_sel != gamepad1_bits.select) {
        joyPushData(fabgl::VirtualKey::VK_MENU_BS, btn_sel);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_SELECT, btn_sel);
    }
    // X / Y / L / R — fed to joydef via VK_JOY_X/Y/Z/C-style dropdowns. Use the
    // remaining DPAD slots so users can rebind in the joystick definition menu.
    static bool prev_x = false, prev_y = false, prev_l = false, prev_r = false;
    if (btn_x != prev_x) { kbdPushData(fabgl::VirtualKey::VK_JOY_X, btn_x); prev_x = btn_x; }
    if (btn_y != prev_y) { kbdPushData(fabgl::VirtualKey::VK_JOY_Y, btn_y); prev_y = btn_y; }
    if (btn_l != prev_l) { kbdPushData(fabgl::VirtualKey::VK_JOY_Z, btn_l); prev_l = btn_l; }
    if (btn_r != prev_r) { kbdPushData(fabgl::VirtualKey::VK_JOY_C, btn_r); prev_r = btn_r; }

    gamepad1_bits.up = up;
    gamepad1_bits.down = down;
    gamepad1_bits.left = left;
    gamepad1_bits.right = right;
    gamepad1_bits.a = btn_a;
    gamepad1_bits.b = btn_b;
    gamepad1_bits.start = btn_start;
    gamepad1_bits.select = btn_sel;
}

//--------------------------------------------------------------------+
// DualShock 4 (Sony PS4) — VID 054C, PID 05C4/09CC
//--------------------------------------------------------------------+
// Report layout after stripping the leading id=0x01 byte (USB mode):
//   [0..3] LX, LY, RX, RY (0x80 = center)
//   [4]    low nibble = D-pad (0=N..7=NW, 8=neutral)
//          high nibble = Square 0x10, Cross 0x20, Circle 0x40, Triangle 0x80
//   [5]    L1 0x01, R1 0x02, L2 0x04, R2 0x08, Share 0x10,
//          Options 0x20, L3 0x40, R3 0x80
//   [6]    PS 0x01, TouchClick 0x02, plus 6-bit counter
// Mapping mirrors process_hid_gamepad(): D-pad+Cross drive Kempston,
// Circle = alt-fire, Square/Triangle/L1/R1 → VK_JOY_X/Y/Z/C for joydef.
static void process_ds4_gamepad(uint8_t const* report, uint16_t len)
{
    if (len < 7) return;

    uint8_t lx = report[0];
    uint8_t ly = report[1];
    uint8_t dpad_hat = report[4] & 0x0F;
    uint8_t face = report[4] & 0xF0;
    uint8_t b5 = report[5];

    bool stick_left  = lx < 0x40;
    bool stick_right = lx > 0xC0;
    bool stick_up    = ly < 0x40;
    bool stick_down  = ly > 0xC0;

    bool hat_up    = (dpad_hat == 7 || dpad_hat == 0 || dpad_hat == 1);
    bool hat_right = (dpad_hat == 1 || dpad_hat == 2 || dpad_hat == 3);
    bool hat_down  = (dpad_hat == 3 || dpad_hat == 4 || dpad_hat == 5);
    bool hat_left  = (dpad_hat == 5 || dpad_hat == 6 || dpad_hat == 7);
    if (dpad_hat >= 8) { hat_up = hat_down = hat_left = hat_right = false; }

    bool up    = stick_up    || hat_up;
    bool down  = stick_down  || hat_down;
    bool left  = stick_left  || hat_left;
    bool right = stick_right || hat_right;

    bool btn_square   = (face & 0x10) != 0;
    bool btn_cross    = (face & 0x20) != 0;
    bool btn_circle   = (face & 0x40) != 0;
    bool btn_triangle = (face & 0x80) != 0;
    bool btn_l1       = (b5 & 0x01) != 0;
    bool btn_r1       = (b5 & 0x02) != 0;
    bool btn_share    = (b5 & 0x10) != 0;
    bool btn_options  = (b5 & 0x20) != 0;

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

    if (btn_cross != gamepad1_bits.b) {
        joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, btn_cross);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_FIRE, btn_cross);
    }
    if (btn_circle != gamepad1_bits.a) {
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, btn_circle);
    }
    if (btn_options != gamepad1_bits.start) {
        joyPushData(fabgl::VirtualKey::VK_MENU_HOME, btn_options);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_START, btn_options);
    }
    if (btn_share != gamepad1_bits.select) {
        joyPushData(fabgl::VirtualKey::VK_MENU_BS, btn_share);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_SELECT, btn_share);
    }

    static bool prev_sq = false, prev_tr = false, prev_l1 = false, prev_r1 = false;
    if (btn_square   != prev_sq) { kbdPushData(fabgl::VirtualKey::VK_JOY_X, btn_square);   prev_sq = btn_square; }
    if (btn_triangle != prev_tr) { kbdPushData(fabgl::VirtualKey::VK_JOY_Y, btn_triangle); prev_tr = btn_triangle; }
    if (btn_l1       != prev_l1) { kbdPushData(fabgl::VirtualKey::VK_JOY_Z, btn_l1);       prev_l1 = btn_l1; }
    if (btn_r1       != prev_r1) { kbdPushData(fabgl::VirtualKey::VK_JOY_C, btn_r1);       prev_r1 = btn_r1; }

    gamepad1_bits.up = up;
    gamepad1_bits.down = down;
    gamepad1_bits.left = left;
    gamepad1_bits.right = right;
    gamepad1_bits.a = btn_circle;
    gamepad1_bits.b = btn_cross;
    gamepad1_bits.start = btn_options;
    gamepad1_bits.select = btn_share;
}

//--------------------------------------------------------------------+
// DualSense (Sony PS5) — VID 054C, PID 0CE6
//--------------------------------------------------------------------+
// Report layout after stripping id=0x01 (USB mode, ~63 bytes total):
//   [0..3] LX, LY, RX, RY (0x80 center)
//   [4]    L2 analog
//   [5]    R2 analog
//   [6]    counter
//   [7]    low nibble = D-pad hat (0..7, 8=neutral)
//          high nibble = Square 0x10, Cross 0x20, Circle 0x40, Triangle 0x80
//   [8]    L1 0x01, R1 0x02, L2btn 0x04, R2btn 0x08,
//          Create 0x10, Options 0x20, L3 0x40, R3 0x80
//   [9]    PS 0x01, TouchClick 0x02, Mute 0x04
// Same emulator mapping as DS4: Cross=Fire, Circle=AltFire,
// Options=Start/F1, Create=Select/Backspace, Square/Triangle/L1/R1
// → VK_JOY_X/Y/Z/C.
static void process_ds5_gamepad(uint8_t const* report, uint16_t len)
{
    if (len < 9) return;

    uint8_t lx = report[0];
    uint8_t ly = report[1];
    uint8_t b7 = report[7];
    uint8_t b8 = report[8];
    uint8_t dpad_hat = b7 & 0x0F;
    uint8_t face = b7 & 0xF0;

    bool stick_left  = lx < 0x40;
    bool stick_right = lx > 0xC0;
    bool stick_up    = ly < 0x40;
    bool stick_down  = ly > 0xC0;

    bool hat_up    = (dpad_hat == 7 || dpad_hat == 0 || dpad_hat == 1);
    bool hat_right = (dpad_hat == 1 || dpad_hat == 2 || dpad_hat == 3);
    bool hat_down  = (dpad_hat == 3 || dpad_hat == 4 || dpad_hat == 5);
    bool hat_left  = (dpad_hat == 5 || dpad_hat == 6 || dpad_hat == 7);
    if (dpad_hat >= 8) { hat_up = hat_down = hat_left = hat_right = false; }

    bool up    = stick_up    || hat_up;
    bool down  = stick_down  || hat_down;
    bool left  = stick_left  || hat_left;
    bool right = stick_right || hat_right;

    bool btn_square   = (face & 0x10) != 0;
    bool btn_cross    = (face & 0x20) != 0;
    bool btn_circle   = (face & 0x40) != 0;
    bool btn_triangle = (face & 0x80) != 0;
    bool btn_l1       = (b8 & 0x01) != 0;
    bool btn_r1       = (b8 & 0x02) != 0;
    bool btn_create   = (b8 & 0x10) != 0;
    bool btn_options  = (b8 & 0x20) != 0;

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

    if (btn_cross != gamepad1_bits.b) {
        joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, btn_cross);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_FIRE, btn_cross);
    }
    if (btn_circle != gamepad1_bits.a) {
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, btn_circle);
    }
    if (btn_options != gamepad1_bits.start) {
        joyPushData(fabgl::VirtualKey::VK_MENU_HOME, btn_options);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_START, btn_options);
    }
    if (btn_create != gamepad1_bits.select) {
        joyPushData(fabgl::VirtualKey::VK_MENU_BS, btn_create);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_SELECT, btn_create);
    }

    static bool prev_sq = false, prev_tr = false, prev_l1 = false, prev_r1 = false;
    if (btn_square   != prev_sq) { kbdPushData(fabgl::VirtualKey::VK_JOY_X, btn_square);   prev_sq = btn_square; }
    if (btn_triangle != prev_tr) { kbdPushData(fabgl::VirtualKey::VK_JOY_Y, btn_triangle); prev_tr = btn_triangle; }
    if (btn_l1       != prev_l1) { kbdPushData(fabgl::VirtualKey::VK_JOY_Z, btn_l1);       prev_l1 = btn_l1; }
    if (btn_r1       != prev_r1) { kbdPushData(fabgl::VirtualKey::VK_JOY_C, btn_r1);       prev_r1 = btn_r1; }

    gamepad1_bits.up = up;
    gamepad1_bits.down = down;
    gamepad1_bits.left = left;
    gamepad1_bits.right = right;
    gamepad1_bits.a = btn_circle;
    gamepad1_bits.b = btn_cross;
    gamepad1_bits.start = btn_options;
    gamepad1_bits.select = btn_create;
}

//--------------------------------------------------------------------+
// Consumer Control (multimedia keys)
//--------------------------------------------------------------------+

static uint16_t prev_consumer_usage = 0;

static void process_consumer_report(uint8_t const* report, uint16_t len)
{
  uint16_t usage = 0;
  if (len >= 2) {
    usage = (uint16_t)(report[0] | (report[1] << 8));
  } else if (len == 1) {
    usage = report[0];
  }

  if (prev_consumer_usage && prev_consumer_usage != usage) {
    switch (prev_consumer_usage) {
      case HID_USAGE_CONSUMER_VOLUME_INCREMENT: kbdPushData(fabgl::VK_VOLUMEUP, false); break;
      case HID_USAGE_CONSUMER_VOLUME_DECREMENT: kbdPushData(fabgl::VK_VOLUMEDOWN, false); break;
      case HID_USAGE_CONSUMER_MUTE:             kbdPushData(fabgl::VK_VOLUMEMUTE, false); break;
      default: break;
    }
  }

  if (usage && usage != prev_consumer_usage) {
    switch (usage) {
      case HID_USAGE_CONSUMER_VOLUME_INCREMENT: kbdPushData(fabgl::VK_VOLUMEUP, true); break;
      case HID_USAGE_CONSUMER_VOLUME_DECREMENT: kbdPushData(fabgl::VK_VOLUMEDOWN, true); break;
      case HID_USAGE_CONSUMER_MUTE:             kbdPushData(fabgl::VK_VOLUMEMUTE, true); break;
      default: break;
    }
  }

  prev_consumer_usage = usage;
}

//--------------------------------------------------------------------+
// Generic Report
//--------------------------------------------------------------------+
static void process_generic_report(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) dev_addr;

  if (is_dualshock4(hid_info[instance].vid, hid_info[instance].pid)
      && len >= 10 && report[0] == 0x01) {
    process_ds4_gamepad(report + 1, len - 1);
    return;
  }
  if (is_dualsense(hid_info[instance].vid, hid_info[instance].pid)
      && len >= 11 && report[0] == 0x01) {
    process_ds5_gamepad(report + 1, len - 1);
    return;
  }

  uint8_t const rpt_count = hid_info[instance].report_count;
  tuh_hid_report_info_t* rpt_info_arr = hid_info[instance].report_info;
  tuh_hid_report_info_t* rpt_info = NULL;

  if ( rpt_count == 1 && rpt_info_arr[0].report_id == 0)
  {
    // Simple report without report ID as 1st byte
    rpt_info = &rpt_info_arr[0];
  }else
  {
    // Composite report, 1st byte is report ID, data starts from 2nd byte
    uint8_t const rpt_id = report[0];

    // Find report id in the array
    for(uint8_t i=0; i<rpt_count; i++)
    {
      if (rpt_id == rpt_info_arr[i].report_id )
      {
        rpt_info = &rpt_info_arr[i];
        break;
      }
    }

    report++;
    len--;
  }

  if (!rpt_info)
  {
    printf("Couldn't find the report info for this report !\r\n");
    return;
  }

  // For complete list of Usage Page & Usage checkout src/class/hid/hid.h. For examples:
  // - Keyboard                     : Desktop, Keyboard
  // - Mouse                        : Desktop, Mouse
  // - Gamepad                      : Desktop, Gamepad
  // - Consumer Control (Media Key) : Consumer, Consumer Control
  // - System Control (Power key)   : Desktop, System Control
  // - Generic (vendor)             : 0xFFxx, xx
  if ( rpt_info->usage_page == HID_USAGE_PAGE_DESKTOP )
  {
    switch (rpt_info->usage)
    {
      case HID_USAGE_DESKTOP_KEYBOARD:
        TU_LOG1("HID receive keyboard report\r\n");
        // Assume keyboard follow boot report layout
        process_kbd_report( (hid_keyboard_report_t const*) report, &prev_report );
        prev_report = *(hid_keyboard_report_t const*)report;
      break;

      case HID_USAGE_DESKTOP_MOUSE:
        TU_LOG1("HID receive mouse report\r\n");
        // Assume mouse follow boot report layout
        process_mouse_report( (hid_mouse_report_t const*) report );
      break;

      case HID_USAGE_DESKTOP_GAMEPAD:
      case HID_USAGE_DESKTOP_JOYSTICK:
        process_hid_gamepad(report, len);
      break;

      default: break;
    }
  }
  else if ( rpt_info->usage_page == HID_USAGE_PAGE_CONSUMER && rpt_info->usage == HID_USAGE_CONSUMER_CONTROL )
  {
    process_consumer_report(report, len);
  }
}
