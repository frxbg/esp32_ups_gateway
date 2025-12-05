#pragma once

#include "ups_data.h"

// Initialize HID worker
void hid_worker_init(void);

// Poll UPS for data (wrapper around ups_poll_all)
bool hid_worker_poll_ups(UpsData *data);

// Check if UPS is connected
bool hid_worker_is_connected(void);
