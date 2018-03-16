#include "ch.h"
#include "hal.h"

mutex_t trace_mtx; // Used internal to synchronize multiple chprintf in debug.h

bool debug_on_usb = true;

