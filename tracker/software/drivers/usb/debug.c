#include "ch.h"
#include "hal.h"

mutex_t trace_mtx; // Used internal to synchronize multiple chprintf in debug.h

uint8_t usb_trace_level = 2; // Level: Errors + Warnings

