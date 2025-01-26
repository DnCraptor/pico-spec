#include <host/usbh.h>
#include "xinput_host.h"

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

#include "Config.h"
#include "ESPectrum.h"

void tuh_xinput_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    auto xid_itf = (xinputh_interface_t *)report;
    const xinput_gamepad_t* p = &xid_itf->pad;

        if ((p->wButtons & XINPUT_GAMEPAD_A) && !gamepad1_bits.a) {
            joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_FIRE, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_0 : fabgl::VirtualKey::VK_RETURN, true);
        }
        else if (!(p->wButtons & XINPUT_GAMEPAD_A) && gamepad1_bits.a) {
            joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_FIRE, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_0 : fabgl::VirtualKey::VK_RETURN, false);
        }

        if ((p->wButtons & XINPUT_GAMEPAD_B) && !gamepad1_bits.b) {
            joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_1 : fabgl::VirtualKey::VK_SPACE, true);
        }
        else if (!(p->wButtons & XINPUT_GAMEPAD_B) && gamepad1_bits.b) {
            joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_1 : fabgl::VirtualKey::VK_SPACE, false);
        }


    gamepad1_bits.a = p->wButtons & XINPUT_GAMEPAD_A;
    gamepad1_bits.b = p->wButtons & XINPUT_GAMEPAD_B;


        if ((p->wButtons & XINPUT_GAMEPAD_GUIDE) && !gamepad1_bits.start && !gamepad1_bits.select) {
            gamepad1_bits.start = true;
            gamepad1_bits.select = true;
            joyPushData(fabgl::VirtualKey::VK_F1, true);
        }
        else if (!(p->wButtons & XINPUT_GAMEPAD_GUIDE) && gamepad1_bits.start && gamepad1_bits.select) {
            gamepad1_bits.start = false;
            gamepad1_bits.select = false;
            joyPushData(fabgl::VirtualKey::VK_F1, false);
        }


        if ((p->wButtons & XINPUT_GAMEPAD_BACK) && !gamepad1_bits.select) {
            joyPushData(fabgl::VirtualKey::VK_MENU_BS, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_SELECT, true);
        }
        else if (!(p->wButtons & XINPUT_GAMEPAD_BACK) && gamepad1_bits.select) {
            joyPushData(fabgl::VirtualKey::VK_MENU_BS, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_SELECT, false);
        }
    gamepad1_bits.select = p->wButtons & XINPUT_GAMEPAD_BACK;
        if ((p->wButtons & XINPUT_GAMEPAD_START) && !gamepad1_bits.start) {
            joyPushData(fabgl::VirtualKey::VK_MENU_HOME, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_START, true);
            if (gamepad1_bits.select || Config::joy2cursor)
                joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_F1 : fabgl::VirtualKey::VK_R, true);
        }
        else if (!(p->wButtons & XINPUT_GAMEPAD_START) && gamepad1_bits.start) {
            joyPushData(fabgl::VirtualKey::VK_MENU_HOME, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_START, false);
            if (gamepad1_bits.select || Config::joy2cursor)
                joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_F1 : fabgl::VirtualKey::VK_R, false);
        }

    gamepad1_bits.start = p->wButtons & XINPUT_GAMEPAD_START;

    const uint8_t dpad = p->wButtons & 0xf;
    bool up, down, right, left;
    ///if (!dpad) {
    ///    up = p->sThumbLY > 3 || p->sThumbRY > 3;
    ///    down = p->sThumbLY < -3 || p->sThumbRY < -3;
    ///    right = p->sThumbLX > 3 || p->sThumbRX > 3;
    ///    left = p->sThumbLX < -3 || p->sThumbRX < -3;
    ///}
    ///else
    {
        down = dpad & XINPUT_GAMEPAD_DPAD_DOWN;
        up = dpad & XINPUT_GAMEPAD_DPAD_UP;
        left = dpad & XINPUT_GAMEPAD_DPAD_LEFT;
        right = dpad & XINPUT_GAMEPAD_DPAD_RIGHT;
    }

        if (up && !gamepad1_bits.up) {
            joyPushData(fabgl::VirtualKey::VK_MENU_UP, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_UP, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_PAGEUP : fabgl::VirtualKey::VK_UP, true);
        }
        else if (!up && gamepad1_bits.up) {
            joyPushData(fabgl::VirtualKey::VK_MENU_UP, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_UP, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_PAGEUP : fabgl::VirtualKey::VK_UP, false);
        }

        if (down && !gamepad1_bits.down) {
            joyPushData(fabgl::VirtualKey::VK_MENU_DOWN, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_DOWN, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_PAGEDOWN : fabgl::VirtualKey::VK_DOWN, true);
        }
        else if (!down && gamepad1_bits.down) {
            joyPushData(fabgl::VirtualKey::VK_MENU_DOWN, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_DOWN, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_PAGEDOWN : fabgl::VirtualKey::VK_DOWN, false);
        }

        if (left && !gamepad1_bits.left) {
            joyPushData(fabgl::VirtualKey::VK_MENU_LEFT, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_LEFT, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_BACKSPACE : fabgl::VirtualKey::VK_LEFT, true);
        }
        else if (!left && gamepad1_bits.left) {
            joyPushData(fabgl::VirtualKey::VK_MENU_LEFT, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_LEFT, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_BACKSPACE : fabgl::VirtualKey::VK_LEFT, false);
        }

        if (right && !gamepad1_bits.right) {
            joyPushData(fabgl::VirtualKey::VK_MENU_RIGHT, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_RIGHT, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_K : fabgl::VirtualKey::VK_RIGHT, true);
        }
        else if (!right && gamepad1_bits.right) {
            joyPushData(fabgl::VirtualKey::VK_MENU_RIGHT, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_RIGHT, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_K : fabgl::VirtualKey::VK_RIGHT, false);
        }

    gamepad1_bits.down = down;
    gamepad1_bits.up = up;
    gamepad1_bits.left = left;
    gamepad1_bits.right = right;
    /*char tmp[128];
    sprintf(tmp, "[%02x, %02x], Type: %s, Buttons %04x, LT: %02x RT: %02x, LX: %d, LY: %d, RX: %d, RY: %d\n",
                 dev_addr, instance, type_str, p->wButtons, p->bLeftTrigger, p->bRightTrigger, p->sThumbLX, p->sThumbLY, p->sThumbRX, p->sThumbRY);
    draw_text(tmp, 0,0, 15,0);*/

    tuh_xinput_receive_report(dev_addr, instance);
}

void tuh_xinput_mount_cb(uint8_t dev_addr, uint8_t instance, const xinputh_interface_t* xinput_itf) {
    TU_LOG1("XINPUT MOUNTED %02x %d\n", dev_addr, instance);
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
}
