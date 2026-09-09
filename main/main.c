/*
 * Phase 1: USB HID gamepad stub.
 *
 * Purpose: find out whether the target host (Samsung TV / Gaming Hub) will
 * enumerate and accept a generic HID gamepad at all. Sends synthetic input
 * so you can see movement on screen without any BLE code existing yet.
 *
 * Flash over the UART port, then move the cable to the NATIVE USB port
 * (GPIO19/20) before plugging into the TV.
 */

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

#include "ble_central.h"

static const char *TAG = "s2bridge";

/* ---------------------------------------------------------------------------
 * Identity. Start here with the pid.codes test VID. If the TV refuses a
 * generic gamepad, this is the line you change to impersonate a controller
 * that is on Samsung's supported list. Read the real IDs off an actual device
 * with `lsusb` rather than trusting any number you found in a chat window.
 * ------------------------------------------------------------------------ */
#define USB_VID 0x1209
#define USB_PID 0x0001

static const tusb_desc_device_t device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

static const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_GAMEPAD()
};

static const char *string_descriptor[5] = {
    (char[]){0x09, 0x04},   /* 0: en-US */
    "DIY",                  /* 1: manufacturer */
    "S2 Bridge Test",       /* 2: product */
    "000000000001",         /* 3: serial */
    "HID Gamepad",          /* 4: HID interface */
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

static const uint8_t configuration_descriptor[] = {
    /* config number, interface count, string index, total length,
       attributes, power in mA */
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    /* interface, string index, protocol, report descriptor len,
       EP in address, size, polling interval (ms) */
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 1),
};

/* --- TinyUSB HID callbacks ------------------------------------------------ */

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer; (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    /* Rumble would arrive here later. Nothing to do in phase 1. */
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer; (void)bufsize;
}

void tud_mount_cb(void)   { ESP_LOGI(TAG, "USB mounted - host accepted the device"); }
void tud_umount_cb(void)  { ESP_LOGW(TAG, "USB unmounted"); }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; ESP_LOGW(TAG, "USB suspended"); }
void tud_resume_cb(void)  { ESP_LOGI(TAG, "USB resumed"); }

/* --- Synthetic input ------------------------------------------------------ */

static void send_fake_input(void)
{
    static float theta = 0.0f;
    static uint32_t tick = 0;

    hid_gamepad_report_t report = {0};

    /* Left stick walks a slow circle so you can see it move on screen.
     * NOTE: TinyUSB's stock gamepad report uses int8_t axes - 256 steps.
     * Fine for a liveness test, too coarse for the real bridge. You will
     * want a hand-written descriptor with 16-bit axes in phase 3. */
    report.x = (int8_t)(100.0f * cosf(theta));
    report.y = (int8_t)(100.0f * sinf(theta));
    theta += 0.05f;

    /* Cycle one button at a time, ~1s each, so you can confirm mapping. */
    report.buttons = 1u << ((tick / 100) % 12);

    /* Walk the hat through its positions. */
    report.hat = (tick / 400) % 9;

    tud_hid_report(0, &report, sizeof(report));
    tick++;
}

void app_main(void)
{
    ESP_LOGI(TAG, "starting USB HID gamepad stub");

    /* NimBLE stores its own state in NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor        = &device_descriptor,
        .string_descriptor        = string_descriptor,
        .string_descriptor_count  = sizeof(string_descriptor) / sizeof(string_descriptor[0]),
        .external_phy             = false,
        .configuration_descriptor = configuration_descriptor,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "USB initialised, waiting for host");

    /* Phase 2: BLE central runs alongside. The USB side keeps sending its
     * synthetic circle - phase 3 replaces that with decoded controller data. */
    ble_central_start();

    while (1) {
        if (tud_mounted() && tud_hid_ready()) {
            send_fake_input();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
