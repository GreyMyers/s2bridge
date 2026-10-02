/*
 * Phase 3: NimBLE central for the Switch 2 Pro Controller.
 *
 * Scans, connects, reads factory stick calibration, runs the init handshake,
 * bonds, subscribes to input notifications, and decodes each report into a
 * shared state struct that the USB HID side reads.
 *
 * Protocol constants transcribed from trevlars/switch2-controllers-linux.
 */

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"

#include "ble_central.h"

static const char *TAG = "s2ble";

/* Set to 0 to connect without writing the LTK. */
#define ENABLE_BONDING 1

/* Log one decoded line per second so you can watch values without flooding. */
#define LOG_DECODED 1

/* ------------------------------------------------------------------ *
 * Protocol constants                                                   *
 * ------------------------------------------------------------------ */

#define NINTENDO_COMPANY_ID 0x0553
#define NINTENDO_VID        0x057E
#define PRO_CONTROLLER2_PID 0x2069

#define CMD_MEMORY  0x02
#define CMD_LEDS    0x09
#define CMD_FEATURE 0x0C
#define CMD_PAIR    0x15

#define SUB_MEMORY_READ     0x04
#define SUB_LEDS_SET_PLAYER 0x07
#define SUB_FEATURE_INIT    0x02
#define SUB_FEATURE_ENABLE  0x04
#define SUB_PAIR_SET_MAC    0x01
#define SUB_PAIR_LTK1       0x04
#define SUB_PAIR_LTK2       0x02
#define SUB_PAIR_FINISH     0x03

#define FEATURE_MOTION 0x04
#define FEATURE_FLAGS  (0x03 | FEATURE_MOTION)

#define LED_PLAYER_1 0x01

/* Factory stick calibration. User slots (0x001FC042 / 0x001FC062) hold a
 * Switch-side recalibration when present; factory is always valid, so we
 * read that and keep the code simple. */
#define CAL_ADDR_LEFT  0x000130A8
#define CAL_ADDR_RIGHT 0x000130E8
#define CAL_READ_LEN   0x0B

static const uint8_t PAIR_LTK1[17] = {
    0x00, 0xEA, 0xBD, 0x47, 0x13, 0x89, 0x35, 0x42, 0xC6,
    0x79, 0xEE, 0x07, 0xF2, 0x53, 0x2C, 0x6C, 0x31,
};
static const uint8_t PAIR_LTK2[17] = {
    0x00, 0x40, 0xB0, 0x8A, 0x5F, 0xCD, 0x1F, 0x9B, 0x41,
    0x12, 0x5C, 0xAC, 0xC6, 0x3F, 0x38, 0xA0, 0x73,
};

/* ab7de9be-89fe-49ad-828f-118f09df7fd2 - input reports (notify) */
static const ble_uuid128_t UUID_INPUT = BLE_UUID128_INIT(
    0xd2, 0x7f, 0xdf, 0x09, 0x8f, 0x11, 0x8f, 0x82,
    0xad, 0x49, 0xfe, 0x89, 0xbe, 0xe9, 0x7d, 0xab);

/* 649d4ac9-8eb7-4e6c-af44-1ea54fe5f005 - command write */
static const ble_uuid128_t UUID_CMD_WRITE = BLE_UUID128_INIT(
    0x05, 0xf0, 0xe5, 0x4f, 0xa5, 0x1e, 0x44, 0xaf,
    0x6c, 0x4e, 0xb7, 0x8e, 0xc9, 0x4a, 0x9d, 0x64);

/* c765a961-d9d8-4d36-a20a-5315b111836a - command response (notify) */
static const ble_uuid128_t UUID_CMD_RESP = BLE_UUID128_INIT(
    0x6a, 0x83, 0x11, 0xb1, 0x15, 0x53, 0x0a, 0xa2,
    0x36, 0x4d, 0xd8, 0xd9, 0x61, 0xa9, 0x65, 0xc7);

/* ------------------------------------------------------------------ *
 * Shared state                                                         *
 * ------------------------------------------------------------------ */

static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static s2_state_t   s_state;

bool ble_central_get_state(s2_state_t *out)
{
    portENTER_CRITICAL(&s_state_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_state_lock);
    return out->connected;
}

/* ------------------------------------------------------------------ *
 * Stick calibration                                                    *
 * ------------------------------------------------------------------ */

typedef struct {
    uint16_t cx, cy;      /* center */
    uint16_t maxx, maxy;  /* positive offsets from center */
    uint16_t minx, miny;  /* negative offsets from center */
    bool     valid;
} stick_cal_t;

static stick_cal_t s_cal_left, s_cal_right;

/* Three bytes hold two 12-bit axes. */
static inline void unpack_stick(const uint8_t *b, uint16_t *x, uint16_t *y)
{
    *x = (uint16_t)(b[0] | ((b[1] & 0x0F) << 8));
    *y = (uint16_t)((b[1] >> 4) | (b[2] << 4));
}

static void load_cal(stick_cal_t *c, const uint8_t *d)
{
    unpack_stick(&d[0], &c->cx,   &c->cy);
    unpack_stick(&d[3], &c->maxx, &c->maxy);
    unpack_stick(&d[6], &c->minx, &c->miny);

    /* Reject blank or nonsensical calibration and fall back to a sane
     * 12-bit default rather than producing dead sticks. */
    if (c->cx == 0 || c->cx == 0xFFF || c->maxx == 0 || c->minx == 0 ||
        c->maxy == 0 || c->miny == 0) {
        ESP_LOGW(TAG, "calibration looks invalid; using defaults");
        c->cx = c->cy = 0x800;
        c->maxx = c->maxy = c->minx = c->miny = 0x600;
    }
    c->valid = true;

    ESP_LOGI(TAG, "cal center=(%u,%u) max=(%u,%u) min=(%u,%u)",
             c->cx, c->cy, c->maxx, c->maxy, c->minx, c->miny);
}

static void default_cal(stick_cal_t *c)
{
    c->cx = c->cy = 0x800;
    c->maxx = c->maxy = c->minx = c->miny = 0x600;
    c->valid = false;
}

/* Scale a raw 12-bit axis into full int16 range using asymmetric
 * per-direction calibration, then apply a deadzone. */
static int16_t scale_axis(uint16_t raw, uint16_t center,
                          uint16_t max_off, uint16_t min_off, bool invert)
{
    int32_t d = (int32_t)raw - (int32_t)center;
    int32_t out;

    if (d >= 0) {
        out = max_off ? (d * 32767) / max_off : 0;
    } else {
        out = min_off ? (d * 32767) / min_off : 0;
    }

    if (out >  32767) out =  32767;
    if (out < -32767) out = -32767;

    if (out > -S2_DEADZONE && out < S2_DEADZONE) {
        out = 0;
    }
    return (int16_t)(invert ? -out : out);
}

/* ------------------------------------------------------------------ *
 * Connection state                                                     *
 * ------------------------------------------------------------------ */

typedef enum {
    STEP_IDLE = 0,
    STEP_SUB_CMD_RESP,
    STEP_READ_CAL_L,
    STEP_READ_CAL_R,
    STEP_LEDS,
    STEP_FEAT_INIT,
    STEP_FEAT_ENABLE,
    STEP_SUB_INPUT,
    STEP_BOND_MAC,
    STEP_BOND_LTK1,
    STEP_BOND_LTK2,
    STEP_BOND_FINISH,
    STEP_RUNNING,
} init_step_t;

static const char *step_name[] = {
    "idle", "sub-cmd-resp", "read-cal-L", "read-cal-R", "leds",
    "feature-init", "feature-enable", "sub-input", "bond-mac",
    "bond-ltk1", "bond-ltk2", "bond-finish", "running",
};

static struct {
    uint16_t conn_handle;
    uint8_t  own_addr_type;
    uint8_t  own_addr[6];

    uint16_t h_input;
    uint16_t h_input_cccd;
    uint16_t h_cmd_write;
    uint16_t h_cmd_resp;
    uint16_t h_cmd_resp_cccd;

    init_step_t step;
    uint8_t     pending_cmd;

    uint32_t report_count;
} s_ctx;

#define INVALID_HANDLE 0

#define MAX_CCCDS 16
static uint16_t s_cccds[MAX_CCCDS];
static int      s_cccd_count;

static void advance(void);

/* ------------------------------------------------------------------ *
 * Command framing                                                      *
 * ------------------------------------------------------------------ */

static int send_command(uint8_t cmd, uint8_t sub, const uint8_t *data, uint8_t len)
{
    uint8_t buf[8 + 64];

    if (len > sizeof(buf) - 8) {
        ESP_LOGE(TAG, "payload too long: %u", len);
        return -1;
    }

    buf[0] = cmd;
    buf[1] = 0x91;
    buf[2] = 0x01;
    buf[3] = sub;
    buf[4] = 0x00;
    buf[5] = len;
    buf[6] = 0x00;
    buf[7] = 0x00;
    if (len && data) {
        memcpy(&buf[8], data, len);
    }

    s_ctx.pending_cmd = cmd;

    int rc = ble_gattc_write_no_rsp_flat(s_ctx.conn_handle, s_ctx.h_cmd_write,
                                         buf, 8 + len);
    if (rc != 0) {
        ESP_LOGE(TAG, "write cmd %02x/%02x failed: %d", cmd, sub, rc);
    }
    return rc;
}

/* read_memory: payload is length, 7e 00 00, then a 4-byte LE address. */
static int send_read_memory(uint8_t len, uint32_t addr)
{
    uint8_t p[8];

    p[0] = len;
    p[1] = 0x7e;
    p[2] = 0x00;
    p[3] = 0x00;
    p[4] = (uint8_t)(addr      );
    p[5] = (uint8_t)(addr >>  8);
    p[6] = (uint8_t)(addr >> 16);
    p[7] = (uint8_t)(addr >> 24);

    return send_command(CMD_MEMORY, SUB_MEMORY_READ, p, sizeof(p));
}

static int on_cccd_written(uint16_t conn_handle, const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr, void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "CCCD write failed at step %s: %d",
                 step_name[s_ctx.step], error->status);
        return 0;
    }
    ESP_LOGI(TAG, "subscribed (%s)", step_name[s_ctx.step]);
    s_ctx.step++;
    advance();
    return 0;
}

static int subscribe(uint16_t cccd_handle)
{
    static const uint8_t on[2] = { 0x01, 0x00 };

    if (cccd_handle == INVALID_HANDLE) {
        ESP_LOGE(TAG, "no CCCD handle for step %s", step_name[s_ctx.step]);
        return -1;
    }
    return ble_gattc_write_flat(s_ctx.conn_handle, cccd_handle,
                                on, sizeof(on), on_cccd_written, NULL);
}

/* ------------------------------------------------------------------ *
 * Init state machine                                                   *
 * ------------------------------------------------------------------ */

static void advance(void)
{
    uint8_t payload[14];

    switch (s_ctx.step) {
    case STEP_SUB_CMD_RESP:
        subscribe(s_ctx.h_cmd_resp_cccd);
        break;

    case STEP_READ_CAL_L:
        send_read_memory(CAL_READ_LEN, CAL_ADDR_LEFT);
        break;

    case STEP_READ_CAL_R:
        send_read_memory(CAL_READ_LEN, CAL_ADDR_RIGHT);
        break;

    case STEP_LEDS:
        payload[0] = LED_PLAYER_1;
        memset(&payload[1], 0, 3);
        send_command(CMD_LEDS, SUB_LEDS_SET_PLAYER, payload, 4);
        break;

    case STEP_FEAT_INIT:
        payload[0] = FEATURE_FLAGS;
        memset(&payload[1], 0, 3);
        send_command(CMD_FEATURE, SUB_FEATURE_INIT, payload, 4);
        break;

    case STEP_FEAT_ENABLE:
        payload[0] = FEATURE_FLAGS;
        memset(&payload[1], 0, 3);
        send_command(CMD_FEATURE, SUB_FEATURE_ENABLE, payload, 4);
        break;

    case STEP_SUB_INPUT:
        subscribe(s_ctx.h_input_cccd);
        break;

#if ENABLE_BONDING
    case STEP_BOND_MAC:
        payload[0] = 0x00;
        payload[1] = 0x02;
        memcpy(&payload[2], s_ctx.own_addr, 6);
        memcpy(&payload[8], s_ctx.own_addr, 6);
        send_command(CMD_PAIR, SUB_PAIR_SET_MAC, payload, 14);
        break;

    case STEP_BOND_LTK1:
        send_command(CMD_PAIR, SUB_PAIR_LTK1, PAIR_LTK1, sizeof(PAIR_LTK1));
        break;

    case STEP_BOND_LTK2:
        send_command(CMD_PAIR, SUB_PAIR_LTK2, PAIR_LTK2, sizeof(PAIR_LTK2));
        break;

    case STEP_BOND_FINISH: {
        uint8_t zero = 0x00;
        send_command(CMD_PAIR, SUB_PAIR_FINISH, &zero, 1);
        break;
    }
#else
    case STEP_BOND_MAC:
    case STEP_BOND_LTK1:
    case STEP_BOND_LTK2:
    case STEP_BOND_FINISH:
        s_ctx.step = STEP_RUNNING;
        advance();
        break;
#endif

    case STEP_RUNNING:
        ESP_LOGI(TAG, "handshake complete - decoding input reports");
        portENTER_CRITICAL(&s_state_lock);
        s_state.connected = true;
        portEXIT_CRITICAL(&s_state_lock);
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ *
 * Report decoding                                                      *
 * ------------------------------------------------------------------ */

/*
 * 63-byte input report layout:
 *   [0..3]   timestamp, 32-bit LE
 *   [4..7]   button bitfield, 32-bit LE
 *   [10..12] left stick,  two 12-bit axes
 *   [13..15] right stick, two 12-bit axes
 *   [31]     battery
 *   [48..59] motion, 6 x int16
 */
static void decode_report(const uint8_t *r, uint16_t len)
{
    if (len < 32) {
        return;
    }

    uint32_t btn = (uint32_t)r[4]
                 | ((uint32_t)r[5] << 8)
                 | ((uint32_t)r[6] << 16)
                 | ((uint32_t)r[7] << 24);

    uint16_t lx_raw, ly_raw, rx_raw, ry_raw;
    unpack_stick(&r[10], &lx_raw, &ly_raw);
    unpack_stick(&r[13], &rx_raw, &ry_raw);

    s2_state_t st;
    st.connected = true;
    st.buttons_raw = btn;
    st.battery = r[31];

    st.lx = scale_axis(lx_raw, s_cal_left.cx,  s_cal_left.maxx,  s_cal_left.minx,  S2_INVERT_LX);
    st.ly = scale_axis(ly_raw, s_cal_left.cy,  s_cal_left.maxy,  s_cal_left.miny,  S2_INVERT_LY);
    st.rx = scale_axis(rx_raw, s_cal_right.cx, s_cal_right.maxx, s_cal_right.minx, S2_INVERT_RX);
    st.ry = scale_axis(ry_raw, s_cal_right.cy, s_cal_right.maxy, s_cal_right.miny, S2_INVERT_RY);

    portENTER_CRITICAL(&s_state_lock);
    s_state = st;
    portEXIT_CRITICAL(&s_state_lock);

#if LOG_DECODED
    if (s_ctx.report_count % 120 == 1) {
        ESP_LOGI(TAG, "btn=%08lx L=(%6d,%6d) R=(%6d,%6d) batt=%u",
                 (unsigned long)btn, st.lx, st.ly, st.rx, st.ry, st.battery);
    }
#endif
}

/* ------------------------------------------------------------------ *
 * GATT discovery                                                       *
 * ------------------------------------------------------------------ */

static int on_dsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                  uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        for (int i = 0; i < s_cccd_count; i++) {
            uint16_t h = s_cccds[i];
            if (h > s_ctx.h_input &&
                (s_ctx.h_input_cccd == INVALID_HANDLE || h < s_ctx.h_input_cccd)) {
                s_ctx.h_input_cccd = h;
            }
            if (h > s_ctx.h_cmd_resp &&
                (s_ctx.h_cmd_resp_cccd == INVALID_HANDLE || h < s_ctx.h_cmd_resp_cccd)) {
                s_ctx.h_cmd_resp_cccd = h;
            }
        }

        ESP_LOGI(TAG, "handles: input=0x%04x cccd=0x%04x cmd_write=0x%04x "
                      "cmd_resp=0x%04x cccd=0x%04x",
                 s_ctx.h_input, s_ctx.h_input_cccd, s_ctx.h_cmd_write,
                 s_ctx.h_cmd_resp, s_ctx.h_cmd_resp_cccd);

        if (s_ctx.h_input == INVALID_HANDLE || s_ctx.h_cmd_write == INVALID_HANDLE ||
            s_ctx.h_cmd_resp == INVALID_HANDLE) {
            ESP_LOGE(TAG, "missing characteristic");
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }

        s_ctx.step = STEP_SUB_CMD_RESP;
        advance();
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "descriptor discovery failed: %d", error->status);
        return 0;
    }

    if (ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16 &&
        s_cccd_count < MAX_CCCDS) {
        s_cccds[s_cccd_count++] = dsc->handle;
    }
    return 0;
}

static int on_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                  const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        s_cccd_count = 0;
        int rc = ble_gattc_disc_all_dscs(conn_handle, 1, 0xffff, on_dsc, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "disc_all_dscs failed: %d", rc);
        }
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "characteristic discovery failed: %d", error->status);
        return 0;
    }

    if (ble_uuid_cmp(&chr->uuid.u, &UUID_INPUT.u) == 0) {
        s_ctx.h_input = chr->val_handle;
    } else if (ble_uuid_cmp(&chr->uuid.u, &UUID_CMD_WRITE.u) == 0) {
        s_ctx.h_cmd_write = chr->val_handle;
    } else if (ble_uuid_cmp(&chr->uuid.u, &UUID_CMD_RESP.u) == 0) {
        s_ctx.h_cmd_resp = chr->val_handle;
    }
    return 0;
}

static int on_mtu(uint16_t conn_handle, const struct ble_gatt_error *error,
                  uint16_t mtu, void *arg)
{
    if (error->status == 0) {
        ESP_LOGI(TAG, "MTU negotiated: %u", mtu);
    } else {
        ESP_LOGW(TAG, "MTU exchange failed: %d (continuing)", error->status);
    }

    int rc = ble_gattc_disc_all_chrs(conn_handle, 1, 0xffff, on_chr, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "disc_all_chrs failed: %d", rc);
    }
    return 0;
}

/* ------------------------------------------------------------------ *
 * Scanning and connection                                              *
 * ------------------------------------------------------------------ */

static bool is_target(const struct ble_hs_adv_fields *fields)
{
    const uint8_t *m = fields->mfg_data;

    if (m == NULL || fields->mfg_data_len < 9) {
        return false;
    }
    if ((m[0] | (m[1] << 8)) != NINTENDO_COMPANY_ID) {
        return false;
    }
    if ((m[5] | (m[6] << 8)) != NINTENDO_VID) {
        return false;
    }
    return (m[7] | (m[8] << 8)) == PRO_CONTROLLER2_PID;
}

static void start_scan(void)
{
    struct ble_gap_disc_params p = {
        .itvl = 0, .window = 0, .filter_policy = 0,
        .limited = 0, .passive = 0, .filter_duplicates = 1,
    };

    int rc = ble_gap_disc(s_ctx.own_addr_type, BLE_HS_FOREVER, &p,
                          ble_central_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "scanning for Pro Controller 2");
    }
}

int ble_central_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                    event->disc.length_data) != 0) {
            return 0;
        }
        if (!is_target(&fields)) {
            return 0;
        }

        ESP_LOGI(TAG, "found controller, rssi=%d", event->disc.rssi);
        ble_gap_disc_cancel();

        struct ble_gap_conn_params cp = {
            .scan_itvl = 0x0010, .scan_window = 0x0010,
            .itvl_min = 6, .itvl_max = 12,
            .latency = 0, .supervision_timeout = 400,
            .min_ce_len = 0, .max_ce_len = 0,
        };

        rc = ble_gap_connect(s_ctx.own_addr_type, &event->disc.addr, 10000,
                             &cp, ble_central_gap_event, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "connect failed: %d", rc);
            start_scan();
        }
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "connection failed: %d", event->connect.status);
            start_scan();
            return 0;
        }
        s_ctx.conn_handle = event->connect.conn_handle;
        s_ctx.h_input = s_ctx.h_input_cccd = INVALID_HANDLE;
        s_ctx.h_cmd_write = s_ctx.h_cmd_resp = s_ctx.h_cmd_resp_cccd = INVALID_HANDLE;
        s_ctx.step = STEP_IDLE;
        s_ctx.report_count = 0;

        default_cal(&s_cal_left);
        default_cal(&s_cal_right);

        ESP_LOGI(TAG, "connected, handle=%d", s_ctx.conn_handle);

        rc = ble_gattc_exchange_mtu(s_ctx.conn_handle, on_mtu, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "MTU exchange failed to start: %d", rc);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "disconnected (reason 0x%02x) after %lu reports",
                 event->disconnect.reason, (unsigned long)s_ctx.report_count);
        s_ctx.conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ctx.step = STEP_IDLE;

        /* Release every input so nothing sticks on the host. */
        portENTER_CRITICAL(&s_state_lock);
        memset(&s_state, 0, sizeof(s_state));
        portEXIT_CRITICAL(&s_state_lock);

        start_scan();
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        uint8_t  buf[128];

        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }
        ble_hs_mbuf_to_flat(event->notify_rx.om, buf, len, NULL);

        if (event->notify_rx.attr_handle == s_ctx.h_cmd_resp) {
            if (len < 8 || buf[0] != s_ctx.pending_cmd || buf[1] != 0x01) {
                ESP_LOGE(TAG, "bad response at step %s:", step_name[s_ctx.step]);
                ESP_LOG_BUFFER_HEX(TAG, buf, len < 16 ? len : 16);
                return 0;
            }

            /* Memory reads carry their payload after the 8-byte command
             * header plus the 8-byte read header: length, 7e 00 00, addr. */
            if (s_ctx.step == STEP_READ_CAL_L && len >= 16 + 9) {
                load_cal(&s_cal_left, &buf[16]);
            } else if (s_ctx.step == STEP_READ_CAL_R && len >= 16 + 9) {
                load_cal(&s_cal_right, &buf[16]);
            } else {
                ESP_LOGI(TAG, "step %s ok", step_name[s_ctx.step]);
            }

            s_ctx.step++;
            advance();
        } else if (event->notify_rx.attr_handle == s_ctx.h_input) {
            s_ctx.report_count++;
            decode_report(buf, len);
        }
        return 0;
    }

    case BLE_GAP_EVENT_MTU:
        return 0;

    default:
        return 0;
    }
}

/* ------------------------------------------------------------------ *
 * Host startup                                                         *
 * ------------------------------------------------------------------ */

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE reset, reason=%d", reason);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr failed: %d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_ctx.own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "infer_auto failed: %d", rc);
        return;
    }
    ble_hs_id_copy_addr(s_ctx.own_addr_type, s_ctx.own_addr, NULL);
    start_scan();
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_central_start(void)
{
    s_ctx.conn_handle = BLE_HS_CONN_HANDLE_NONE;
    memset(&s_state, 0, sizeof(s_state));
    default_cal(&s_cal_left);
    default_cal(&s_cal_right);

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm    = 0;
    ble_hs_cfg.sm_sc      = 0;

    nimble_port_freertos_init(host_task);
}
