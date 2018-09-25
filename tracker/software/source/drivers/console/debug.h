#ifndef __TRACE_H__
#define __TRACE_H__

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "ptime.h"
#include "config.h"
#include <string.h>
//#include "usbcfg.h"
//#include "console.h"

#define ERROR_LIST_LENGTH	64
#define ERROR_LIST_SIZE		32

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

extern char error_list[ERROR_LIST_SIZE][ERROR_LIST_LENGTH];
extern uint8_t error_counter;
extern uint8_t current_trace_level;

#define TRACE_DEBUG(format, args...) if(current_trace_level > 4) { debug_print("DEBUG", __FILENAME__, __LINE__, format, ##args); }
#define TRACE_INFO(format, args...)  if(current_trace_level > 3) { debug_print("     ", __FILENAME__, __LINE__, format, ##args); }
#define TRACE_MON(format, args...)  if(current_trace_level > 2) { debug_print("     ", __FILENAME__, __LINE__, format, ##args); }
#define TRACE_WARN(format, args...)  if(current_trace_level > 1) { debug_print("WARN ", __FILENAME__, __LINE__, format, ##args); }
#define TRACE_ERROR(format, args...) { \
	if(current_trace_level > 0) { \
		debug_print("ERROR", __FILENAME__, __LINE__, format, ##args); \
	} \
	\
	uint8_t strcnt = chsnprintf(error_list[error_counter], ERROR_LIST_LENGTH, "[%8d.%03d] ", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000); \
	chsnprintf(&error_list[error_counter][strcnt], ERROR_LIST_LENGTH-strcnt, (format), ##args); \
	error_counter = (error_counter+1)%ERROR_LIST_SIZE; \
}
#if TRACE_TIME && TRACE_FILE && TRACE_THREAD
#define TRACE_TAB "                                                           "
#elif TRACE_TIME && TRACE_FILE
#define TRACE_TAB "                                               "
#elif TRACE_TIME && !TRACE_FILE
#define TRACE_TAB "                            "
#elif !TRACE_TIME && TRACE_FILE
#define TRACE_TAB "                               "
#else
#define TRACE_TAB "              "
#endif


/*
 * Guarded memory pool integrity checking...
 */

static inline struct pool_header *pktSystemPoolIntegrityCheck(guarded_memory_pool_t *pool) {
  return ((struct pool_header *)(pool->pool.next))->next;
}

/*
 * Memory heap integrity checking...
 */
static inline heap_header_t *pktSystemHeapIntegrityCheck(memory_heap_t *heap) {
  return (heap_header_t *)(heap->header.free).next;
}


/*===========================================================================*/
/* Macro definitions.                                                        */
/*===========================================================================*/

#define pktAssertCCMdynamicCheck(ptr)                                        \
  extern uint8_t __ram4_free__[];                                            \
  extern uint8_t __ram4_end__[];                                             \
  chDbgAssert(!((void *)ptr > (void *)__ram4_free__                          \
    && (void *)__ram4_end__ < (void *)ptr), "Object reference not in CCM")

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern mutex_t debug_mtx;

void pktConfigureSerialIO(void);
void debug_print(char *type, char* filename, uint32_t line, char* format, ...);

#endif /* __TRACE_H__ */

