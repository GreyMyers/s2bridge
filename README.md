# Switch 2 Pro Controller -> USB HID bridge (ESP32-S3)

## Phase 1 (this code)
Enumerates as a generic USB HID gamepad and sends synthetic input.
Purpose: determine whether the target host accepts a homemade HID gamepad
before writing any BLE protocol code.

    idf.py set-target esp32s3
    idf.py build
    idf.py -p /dev/ttyUSB0 flash monitor

Then move the cable to the board's NATIVE USB port (GPIO19/20) and plug
into the host under test.

### Test order (cheapest feedback first)
1. Linux    - `lsusb` then `evtest`, confirm axes and buttons move
2. Android  - any gamepad tester app
3. The TV   - Gaming Hub > controller settings

### Decision point
- TV sees it            -> proceed to phase 2, keep this descriptor
- TV enumerates, ignores -> VID/PID filtering; clone a supported controller
- TV does nothing        -> check you are on the native USB port, then
                            check the TV port supplies power and is not
                            a service-only port

## Phase 2 (next)
NimBLE central: scan for manufacturer data with Nintendo company ID 0x0553,
VID 0x057E, PID 0x2069. Bond, store LTK in NVS, dump the full GATT table.
Goal is only to print the table and confirm you can subscribe.

## Phase 3
Decode the 63-byte input report, apply factory calibration, replace
send_fake_input() with real data. Move to a hand-written HID descriptor
with 16-bit axes.

## Protocol references
- novakpetya/linux-switch2 (PROTOCOL.md) - most complete writeup
- ndeadly/switch2_controller_research    - GATT UUIDs, pairing handshake
- bitaxislabs/Switch2BLE                 - report parsing reference
- LeonChrome/XinHeLianSheng-Pro2-Bridge  - existing ESP32-S3 BLE half
