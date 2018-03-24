#include "ch.h"
#include "hal.h"

mutex_t trace_mtx; // Used internal to synchronize multiple chprintf in debug.h

#ifdef USB_TRACE_LEVEL
uint8_t usb_trace_level = USB_TRACE_LEVEL; // Set in makefile UDEFS
#else
uint8_t usb_trace_level = 2; // Level: Errors + Warnings
#endif
