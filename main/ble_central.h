#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "host/ble_gap.h"

/* --- Tuning ------------------------------------------------------------ *
 * Flip these if an axis moves the wrong way. HID convention is +Y = down,
 * and the controller reports +Y = up, so both Y axes default to inverted.
 * Verify with `sudo evtest` before taking the board to the TV. */
#define S2_INVERT_LX 0
#define S2_INVERT_LY 1
#define S2_INVERT_RX 0
#define S2_INVERT_RY 1

/* Deadzone in scaled units (full range is +/-32767). Raise if the sticks
 * drift at rest, lower if small movements feel ignored. */
#define S2_DEADZONE 1200

/* --- Button masks ------------------------------------------------------ *
 * The report's 32-bit little-endian field at bytes 4..7. */
#define S2_BTN_Y       0x00000001u
#define S2_BTN_X       0x00000002u
#define S2_BTN_B       0x00000004u
#define S2_BTN_A       0x00000008u
#define S2_BTN_SR_R    0x00000010u
#define S2_BTN_SL_R    0x00000020u
#define S2_BTN_R       0x00000040u
#define S2_BTN_ZR      0x00000080u
#define S2_BTN_MINUS   0x00000100u
#define S2_BTN_PLUS    0x00000200u
#define S2_BTN_R_STK   0x00000400u
#define S2_BTN_L_STK   0x00000800u
#define S2_BTN_HOME    0x00001000u
#define S2_BTN_CAPTURE 0x00002000u
#define S2_BTN_C       0x00004000u
#define S2_BTN_DOWN    0x00010000u
#define S2_BTN_UP      0x00020000u
#define S2_BTN_RIGHT   0x00040000u
#define S2_BTN_LEFT    0x00080000u
#define S2_BTN_SR_L    0x00100000u
#define S2_BTN_SL_L    0x00200000u
#define S2_BTN_L       0x00400000u
#define S2_BTN_ZL      0x00800000u
#define S2_BTN_GR      0x01000000u
#define S2_BTN_GL      0x02000000u

/* Decoded controller state, shared from the BLE task to the USB task. */
typedef struct {
    bool     connected;
    uint32_t buttons_raw;   /* S2_BTN_* masks */
    int16_t  lx, ly;        /* calibrated, full int16 range */
    int16_t  rx, ry;
    uint8_t  battery;
} s2_state_t;

/* Start the NimBLE host and begin scanning for the Pro Controller 2. */
void ble_central_start(void);

/* Copy the latest decoded state. Returns false when no controller is
 * connected, in which case the struct is all zeros (neutral). */
bool ble_central_get_state(s2_state_t *out);

/* GAP event handler, shared between scanning and the connection. */
int ble_central_gap_event(struct ble_gap_event *event, void *arg);
