#pragma once

#include "host/ble_gap.h"

/* Start the NimBLE host and begin scanning for the Pro Controller 2. */
void ble_central_start(void);

/* GAP event handler, shared between scanning and the connection. */
int ble_central_gap_event(struct ble_gap_event *event, void *arg);
