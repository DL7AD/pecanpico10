#include "ch.h"
#include "hal.h"
#include "debug.h"
#include "portab.h"

mutex_t mtx; // Used internal to synchronize multiple chprintf in debug.h

char error_list[ERROR_LIST_SIZE][ERROR_LIST_LENGTH];
uint8_t error_counter;

static const SerialConfig debug_config = {
	115200,
	0,
	0,
	0
};

#ifdef USB_TRACE_LEVEL
uint8_t usb_trace_level = USB_TRACE_LEVEL; // Set in makefile UDEFS
#else
uint8_t usb_trace_level = 2; // Level: Errors + Warnings
#endif


void debug_init(void) {
	chMtxObjectInit(&mtx);

	sdStart(&SD3, &debug_config);
	palSetLineMode(LINE_IO_TXD, PAL_MODE_ALTERNATE(7));
	palSetLineMode(LINE_IO_RXD, PAL_MODE_ALTERNATE(7));
}

void debug_print(char *type, char* filename, uint32_t line, char* format, ...)
{
	chMtxLock(&mtx);

	uint8_t str[256];

	va_list args;
	va_start(args, format);
	chsnprintf((char*)str, sizeof(str), format, args);
	va_end(args);


	if(isConsoleOutputAvailable()) {
		if(TRACE_TIME) {
			chprintf((BaseSequentialStream*)&SDU1, "[%8d.%03d]", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000);
		}
		chprintf((BaseSequentialStream*)&SDU1, "[%s]", type);
		if(TRACE_FILE) {
			chprintf((BaseSequentialStream*)&SDU1, "[%12s %04d]", filename, line);
		}
		chprintf((BaseSequentialStream*)&SDU1, " %s\r\n", str);
	}

	if(TRACE_TIME) {
		chprintf((BaseSequentialStream*)&SD3, "[%8d.%03d]", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000);
	}
	chprintf((BaseSequentialStream*)&SD3, "[%s]", type);
	if(TRACE_FILE) {
		chprintf((BaseSequentialStream*)&SD3, "[%12s %04d]", filename, line);
	}
	chprintf((BaseSequentialStream*)&SD3, " %s\r\n", str);

	chMtxUnlock(&mtx);
}

