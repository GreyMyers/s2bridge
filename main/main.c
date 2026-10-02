/*
 * Phase 3: Switch 2 Pro Controller -> USB HID bridge.
 *
 * Enumerates as a USB HID gamepad with 16-bit axes and forwards decoded
 * controller state from the BLE side. Flash over the UART port, then plug
 * the NATIVE USB port into the host.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

#include "ble_central.h"

static const char *TAG = "s2bridge";

/*
 * Button layout. Default maps by PHYSICAL POSITION, which is what cloud
 * gaming services expect: Switch B (south) becomes Xbox A, and so on.
 * Set to 0 to map by printed label instead.
 */
#define MAP_BY_POSITION 1

#define USB_VID 0x1209
#define USB_PID 0x0001

/* --- Report ------------------------------------------------------------ */



static const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_GAMEPAD()
};

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

static const char *string_descriptor[5] = {
    (char[]){0x09, 0x04},
    "DIY",
    "S2 Bridge",
    "000000000001",
    "HID Gamepad",
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

static const uint8_t configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 1),
};

/* --- TinyUSB callbacks ------------------------------------------------- */

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
    /* Rumble would arrive here. Phase 4. */
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer; (void)bufsize;
}

void tud_mount_cb(void)   { ESP_LOGI(TAG, "USB mounted"); }
void tud_umount_cb(void)  { ESP_LOGW(TAG, "USB unmounted"); }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; }
void tud_resume_cb(void)  { ESP_LOGI(TAG, "USB resumed"); }

/* --- Mapping ----------------------------------------------------------- */

static uint8_t dpad_to_hat(uint32_t b)
{
    bool up    = b & S2_BTN_UP;
    bool down  = b & S2_BTN_DOWN;
    bool left  = b & S2_BTN_LEFT;
    bool right = b & S2_BTN_RIGHT;

    if (up    && right) return GAMEPAD_HAT_UP_RIGHT;
    if (right && down)  return GAMEPAD_HAT_DOWN_RIGHT;
    if (down  && left)  return GAMEPAD_HAT_DOWN_LEFT;
    if (left  && up)    return GAMEPAD_HAT_UP_LEFT;
    if (up)             return GAMEPAD_HAT_UP;
    if (right)          return GAMEPAD_HAT_RIGHT;
    if (down)           return GAMEPAD_HAT_DOWN;
    if (left)           return GAMEPAD_HAT_LEFT;
    return GAMEPAD_HAT_CENTERED;
}

static uint32_t map_buttons(uint32_t b)
{
    uint32_t out = 0;

#if MAP_BY_POSITION
    /* South/east/west/north -> buttons 1-4, matching Xbox physical layout */
    if (b & S2_BTN_B) out |= 1u << 0;   /* south */
    if (b & S2_BTN_A) out |= 1u << 1;   /* east  */
    if (b & S2_BTN_Y) out |= 1u << 2;   /* west  */
    if (b & S2_BTN_X) out |= 1u << 3;   /* north */
#else
    if (b & S2_BTN_A) out |= 1u << 0;
    if (b & S2_BTN_B) out |= 1u << 1;
    if (b & S2_BTN_X) out |= 1u << 2;
    if (b & S2_BTN_Y) out |= 1u << 3;
#endif

    if (b & S2_BTN_L)       out |= 1u << 4;
    if (b & S2_BTN_R)       out |= 1u << 5;
    if (b & S2_BTN_ZL)      out |= 1u << 6;
    if (b & S2_BTN_ZR)      out |= 1u << 7;
    if (b & S2_BTN_MINUS)   out |= 1u << 8;
    if (b & S2_BTN_PLUS)    out |= 1u << 9;
    if (b & S2_BTN_L_STK)   out |= 1u << 10;
    if (b & S2_BTN_R_STK)   out |= 1u << 11;
    if (b & S2_BTN_HOME)    out |= 1u << 12;
    if (b & S2_BTN_CAPTURE) out |= 1u << 13;
    if (b & S2_BTN_GL)      out |= 1u << 14;
    if (b & S2_BTN_GR)      out |= 1u << 15;

    return out;
}

/* int16 -> int8, clamped so -32768 does not wrap to +128. */
static int8_t to_i8(int16_t v)
{
    int32_t t = v >> 8;
    if (t >  127) t =  127;
    if (t < -127) t = -127;
    return (int8_t)t;
}

/* --- Main -------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "starting S2 bridge");

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
    ESP_LOGI(TAG, "USB initialised");

    ble_central_start();

    hid_gamepad_report_t report;
    hid_gamepad_report_t last = {0};
    bool have_last = false;
    int  idle_ticks = 0;

    while (1) {
        s2_state_t st;
        ble_central_get_state(&st);

        memset(&report, 0, sizeof(report));
        report.hat = GAMEPAD_HAT_CENTERED;

        if (st.connected) {
            report.x       = to_i8(st.lx);
            report.y       = to_i8(st.ly);
            report.z       = to_i8(st.rx);
            report.rz      = to_i8(st.ry);
            report.hat     = dpad_to_hat(st.buttons_raw);
            report.buttons = map_buttons(st.buttons_raw);
        }

        bool changed = !have_last || memcmp(&report, &last, sizeof(report)) != 0;

        if (changed) { idle_ticks = 0; } else { idle_ticks++; }

        /* Send on change, plus a keepalive about every 100ms. */
        if (tud_mounted() && tud_hid_ready() && (changed || idle_ticks >= 25)) {
            tud_hid_report(0, &report, sizeof(report));
            last = report;
            have_last = true;
            idle_ticks = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(4) > 0 ? pdMS_TO_TICKS(4) : 1);
    }
}
