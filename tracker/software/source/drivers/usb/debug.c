#include "ch.h"
#include "hal.h"
#include "debug.h"
#include "portab.h"

mutex_t debug_mtx; // Used internal to synchronize multiple chprintf in debug.h

char error_list[ERROR_LIST_SIZE][ERROR_LIST_LENGTH];
uint8_t error_counter;

/*static const SerialConfig debug_config = {
	115200,
	0,
	0,
	0
};*/

#ifdef SET_TRACE_LEVEL
uint8_t current_trace_level = SET_TRACE_LEVEL; // Set in makefile UDEFS
#else
uint8_t current_trace_level = 5; // Level: All
#endif


void debug_init(void) {
	chMtxObjectInit(&debug_mtx);
    pktSerialStart();
/*	sdStart(&SD3, &debug_config);
	palSetLineMode(LINE_IO_TXD, PAL_MODE_ALTERNATE(7));
	palSetLineMode(LINE_IO_RXD, PAL_MODE_ALTERNATE(7));*/
}

void debug_print(char *type, char* filename, uint32_t line, char* format, ...)
{
	chMtxLock(&debug_mtx);

/*	uint8_t str[256];

	va_list args;
	va_start(args, format);
	chsnprintf((char*)str, sizeof(str), format, args);
	va_end(args);*/


	if(isConsoleOutputAvailable()) {
      if(TRACE_TIME) {
          chprintf((BaseSequentialStream*)&SDU1, "[%8d.%03d]", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000);
      }
      chprintf((BaseSequentialStream*)&SDU1, "[%s]", type);
      if(TRACE_FILE) {
          chprintf((BaseSequentialStream*)&SDU1, "[%12s %04d]", filename, line);
      }
      chprintf((BaseSequentialStream*)&SDU1, " ");
      va_list args;
      va_start(args, format);
      chvprintf((BaseSequentialStream*)&SDU1, format, args);
      va_end(args);
      chprintf((BaseSequentialStream*)&SDU1, "\r\n");
	}

	if(TRACE_TIME) {
		chprintf((BaseSequentialStream*)SERIAL_DEBUG_DRIVER, "[%8d.%03d]", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000);
	}
	chprintf((BaseSequentialStream*)SERIAL_DEBUG_DRIVER, "[%s]", type);
	if(TRACE_FILE) {
		chprintf((BaseSequentialStream*)SERIAL_DEBUG_DRIVER, "[%12s %04d]", filename, line);
	}
    chprintf((BaseSequentialStream*)SERIAL_DEBUG_DRIVER, " ");
    va_list args;
    va_start(args, format);
	chvprintf((BaseSequentialStream*)SERIAL_DEBUG_DRIVER, format, args);
    va_end(args);
    chprintf((BaseSequentialStream*)SERIAL_DEBUG_DRIVER, "\r\n");

	chMtxUnlock(&debug_mtx);
}

