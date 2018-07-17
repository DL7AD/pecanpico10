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

#define ERROR_LIST_LENGTH	64
#define ERROR_LIST_SIZE		32

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

extern char error_list[ERROR_LIST_SIZE][ERROR_LIST_LENGTH];
extern uint8_t error_counter;
extern mutex_t trace_mtx;
extern const SerialConfig uart_config;
extern uint8_t usb_trace_level;

// Initializer for serial debug and LEDs
/*
#define DEBUG_INIT() { \
	chMtxObjectInit(&trace_mtx); \
}
*/

#define TRACE_BASE(format, type, args...) { \
	if(isConsoleOutputAvailable()) { \
		if(TRACE_TIME) { \
			chprintf((BaseSequentialStream*)&SDU1, "[%8d.%03d]", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000); \
		} \
		chprintf((BaseSequentialStream*)&SDU1, "[%s]", type); \
		if(TRACE_FILE) { \
			chprintf((BaseSequentialStream*)&SDU1, "[%12s %04d]", __FILENAME__, __LINE__); \
		} \
		chprintf((BaseSequentialStream*)&SDU1, " "); \
		chprintf((BaseSequentialStream*)&SDU1, (format), ##args); \
		chprintf((BaseSequentialStream*)&SDU1, "\r\n"); \
		chThdSleep(TIME_MS2I(10)); \
	} \
}

#define TRACE_DEBUG(format, args...) if(usb_trace_level > 4) { TRACE_BASE(format, "DEBUG", ##args) }
#define TRACE_INFO(format, args...)  if(usb_trace_level > 3) { TRACE_BASE(format, "     ", ##args) }
#define TRACE_MON(format, args...)  if(usb_trace_level > 2) { TRACE_BASE(format, "     ", ##args) }
#define TRACE_WARN(format, args...)  if(usb_trace_level > 1) { TRACE_BASE(format, "WARN ", ##args) }
#define TRACE_ERROR(format, args...) { \
	if(usb_trace_level > 0) { \
		TRACE_BASE(format, "ERROR", ##args); \
	} \
	\
	uint8_t strcnt = chsnprintf(error_list[error_counter], ERROR_LIST_LENGTH, "[%8d.%03d] ", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000); \
	chsnprintf(&error_list[error_counter][strcnt], ERROR_LIST_LENGTH-strcnt, (format), ##args); \
	error_counter = (error_counter+1)%ERROR_LIST_SIZE; \
}

#if TRACE_TIME && TRACE_FILE
#define TRACE_TAB "                                               "
#elif TRACE_TIME && !TRACE_FILE
#define TRACE_TAB "                            "
#elif !TRACE_TIME && TRACE_FILE
#define TRACE_TAB "                               "
#else
#define TRACE_TAB "              "
#endif

/*
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
*/


/*
#if USE_CCM_FOR_PKT_POOL == TRUE

static inline struct pool_header *pktSystemCheck(void) {
  extern guarded_memory_pool_t *ccm_pool;
  return ((struct pool_header *)(ccm_pool->pool.next))->next;
}
#elif USE_CCM_HEAP_FOR_PKT ==  TRUE
*/
#if USE_CCM_HEAP_FOR_PKT ==  TRUE
/*
 * Memory heap integrity checking...
 */
static inline heap_header_t *pktSystemCheck(void) {
  extern memory_heap_t *ccm_heap;
  return (heap_header_t *)(ccm_heap->header.free).next;
}
#endif

#endif /* __TRACE_H__ */

