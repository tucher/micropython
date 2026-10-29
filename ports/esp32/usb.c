/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Damien P. George
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
 */

#include "py/runtime.h"
#include "py/mphal.h"
#include "usb.h"

#if MICROPY_HW_ENABLE_USBDEV

#include "esp_mac.h"
#include "esp_rom_gpio.h"
#include "esp_private/usb_phy.h"

#include "shared/tinyusb/mp_usbd.h"

static usb_phy_handle_t phy_hdl;

void usb_phy_init(void) {
    // ref: https://github.com/espressif/esp-usb/blob/4b6a798d0bed444fff48147c8dcdbbd038e92892/device/esp_tinyusb/tinyusb.c

    // Configure USB PHY
    static const usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT,
    };

    // Init ESP USB Phy
    usb_new_phy(&phy_conf, &phy_hdl);
}

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
void usb_usj_mode(void) {
    // Switch the USB PHY back to Serial/Jtag mode, disabling OTG support
    // This should be run before jumping to bootloader.
    usb_del_phy(phy_hdl);
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_SERIAL_JTAG,
    };
    usb_new_phy(&phy_conf, &phy_hdl);
}
#endif

#if MICROPY_HW_USBDEV_NATIVE_TASK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task.h"
#include "mphalport.h"

#ifndef MICROPY_HW_USBDEV_TASK_PRIORITY
// Above the VM, which it preempts when USB needs it; below the radios.
#define MICROPY_HW_USBDEV_TASK_PRIORITY (ESP_TASK_PRIO_MIN + 5)
#endif

#ifndef MICROPY_HW_USBDEV_TASK_CORE
// The VM's core, where TinyUSB's interrupt is too. A board that streams over
// USB may move it to the other core, so its work does not take the VM's time.
#define MICROPY_HW_USBDEV_TASK_CORE MP_TASK_COREID
#endif

static void usb_task(void *arg) {
    for (;;) {
        tud_task_ext(UINT32_MAX, false);    // blocks on TinyUSB's event queue
    }
}

// TinyUSB's own task, started once TinyUSB is initialised and kept across soft
// resets.
void usb_task_start(void) {
    static TaskHandle_t task;
    if (task == NULL) {
        xTaskCreatePinnedToCore(usb_task, "tinyusb", 4096, NULL, MICROPY_HW_USBDEV_TASK_PRIORITY, &task, MICROPY_HW_USBDEV_TASK_CORE);
    }
}

#endif // MICROPY_HW_USBDEV_NATIVE_TASK

void mp_usbd_port_get_serial_number(char *serial_buf) {
    // use factory default MAC as serial ID
    uint8_t mac[8] = {0};               // the MAC is 6 bytes: the rest must not be stack garbage
    esp_efuse_mac_get_default(mac);
    MP_STATIC_ASSERT(sizeof(mac) * 2 <= MICROPY_HW_USB_DESC_STR_MAX);
    mp_usbd_hex_str(serial_buf, mac, sizeof(mac));
}

#endif // MICROPY_HW_ENABLE_USBDEV
