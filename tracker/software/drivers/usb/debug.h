#ifndef __TRACE_H__
#define __TRACE_H__

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "ptime.h"
#include "config.h"
#include <string.h>
#include "usbcfg.h"
#include "usb.h"

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

extern mutex_t trace_mtx;
extern const SerialConfig uart_config;
extern bool debug_on_usb;

// Initializer for serial debug and LEDs
#define DEBUG_INIT() { \
	chMtxObjectInit(&trace_mtx); \
}

#define TRACE_BASE(format, type, args...) { \
	chMtxLock(&trace_mtx); \
	if(debug_on_usb) { \
		TRACE_BASE_USB(format, type, ##args); \
	} \
	chMtxUnlock(&trace_mtx); \
}

#define TRACE_BASE_USB(format, type, args...) { \
	if(isUSBInitialized()) { \
		if(TRACE_TIME) { \
			chprintf((BaseSequentialStream*)&SDU1, "[%8d.%03d]", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000); \
		} \
		chprintf((BaseSequentialStream*)&SDU1, "[%s]", type); \
		if(TRACE_FILE) { \
			chprintf((BaseSequentialStream*)&SDU1, "[%10s %04d]", __FILENAME__, __LINE__); \
		} \
		chprintf((BaseSequentialStream*)&SDU1, " "); \
		chprintf((BaseSequentialStream*)&SDU1, (format), ##args); \
		chprintf((BaseSequentialStream*)&SDU1, "\r\n"); \
		chThdSleep(TIME_MS2I(10)); \
	} \
}

#define TRACE_DEBUG(format, args...) TRACE_BASE(format, "DEBUG", ##args)
#define TRACE_INFO(format, args...)  TRACE_BASE(format, "     ", ##args)
#define TRACE_WARN(format, args...)  TRACE_BASE(format, "WARN ", ##args)
#define TRACE_ERROR(format, args...) TRACE_BASE(format, "ERROR", ##args)
#define TRACE_USB(format, args...)   TRACE_BASE_USB(format, "USB  ", ##args) /* only traced on USB */

#if TRACE_TIME && TRACE_FILE
#define TRACE_TAB "                                             "
#elif TRACE_TIME && !TRACE_FILE
#define TRACE_TAB "                            "
#elif !TRACE_TIME && TRACE_FILE
#define TRACE_TAB "                               "
#else
#define TRACE_TAB "              "
#endif

#define TRACE_BIN(data, len) { \
	chMtxLock(&trace_mtx); \
	chprintf((BaseSequentialStream*)&SD3, "[%8d.%03d][DEBUG] ", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000); \
	chprintf((BaseSequentialStream*)&SD3, "     > Binary data (%d bits)\r\n", (len)); \
	for(uint32_t i=0; i<((len)+7)/8; i+=8) \
		chprintf((BaseSequentialStream*)&SD3, "%s 0x%03x ... 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n", \
		TRACE_TAB, i, (data)[i], (data)[i+1], (data)[i+2], (data)[i+3], (data)[i+4], (data)[i+5], (data)[i+6], (data)[i+7]); \
	chMtxUnlock(&trace_mtx); \
}

#define TRACE_BIN_CHAR(data, len) { \
	chMtxLock(&trace_mtx); \
	chprintf((BaseSequentialStream*)&SD3, "[%8d.%03d][DEBUG] ", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000); \
	chprintf((BaseSequentialStream*)&SD3, "     > Binary data (%d bits)\r\n", (len)); \
	for(uint32_t i=0; i<((len)+7)/8; i+=8) \
		chprintf((BaseSequentialStream*)&SD3, "%s %c%c%c%c%c%c%c%c\r\n", \
		TRACE_TAB, i, (data)[i], (data)[i+1], (data)[i+2], (data)[i+3], (data)[i+4], (data)[i+5], (data)[i+6], (data)[i+7]); \
	chMtxUnlock(&trace_mtx); \
}

#endif

