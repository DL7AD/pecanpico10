#include "ch.h"
#include "hal.h"
#include "debug.h"
#include "portab.h"
#include "console.h"

mutex_t debug_mtx; // Used internal to synchronize multiple chprintf in debug.h

char error_list[ERROR_LIST_SIZE][ERROR_LIST_LENGTH];
uint8_t error_counter;

/* Setup serial channel identities. */

/*
 * The console is interactive and displays trace messages when not in CLI mode.
 */
BaseSequentialStream *console = (BaseSequentialStream *)&SERIAL_CONSOLE_DRIVER;

/*
 * The trace is an always on trace message output.
 */
BaseSequentialStream *trace = (BaseSequentialStream *)&SERIAL_TRACE_DRIVER;

/*
 * The serial out is a generally available serial output.
 */
BaseSequentialStream *serial = (BaseSequentialStream *)&SERIAL_DEBUG_DRIVER;

/*
 * The stream output is intended for dumping data (such as DSP) for analysis
 */
BaseSequentialStream *stream = (BaseSequentialStream *)&SERIAL_STREAM_DRIVER;

#ifdef SET_TRACE_LEVEL
uint8_t current_trace_level = SET_TRACE_LEVEL; // Set in makefile UDEFS
#else
uint8_t current_trace_level = 5; // Level: All
#endif

/**
 *
 */
void pktConfigureSerialIO(void) {
	chMtxObjectInit(&debug_mtx);
    pktSerialStart();

    /* Init and start USB. */
     usbObjectInit(&USBD1);

     usbStart(&USBD1, &usbcfg);

     /* Init serial over USB. */
     sduObjectInit(&SDU1);
     sduObjectInit(&SDU2);

     /* Start serial over USB. */
     sduStart(&SDU1, &serusbcfg1);
     sduStart(&SDU2, &serusbcfg2);

     chThdSleep(TIME_MS2I(100));

     /* Signal soft disconnect to the host (DP pull-up disconnected). */
     usbDisconnectBus(&USBD1);

     chThdSleep(TIME_MS2I(1500));

     /*
      * Notify host we are here.
      * If the cable is connected the host should enumerate USB.
      */
     usbConnectBus(&USBD1);
}

void debug_print(char *type, char* filename, uint32_t line, char* format, ...)
{
	chMtxLock(&debug_mtx);

/*	uint8_t str[256];

	va_list args;
	va_start(args, format);
	chsnprintf((char*)str, sizeof(str), format, args);
	va_end(args);*/

	/* TODO: Implement dynamic assignment of console driver output. */

	if(isConsoleOutputAvailable()) {
      if(TRACE_SHOW_TIME) {
          chprintf((BaseSequentialStream*)console, "[%8d.%03d]", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000);
      }
      chprintf((BaseSequentialStream*)console, "[%s]", type);
      if(TRACE_SHOW_FILE) {
          chprintf((BaseSequentialStream*)console, "[%12s %04d]", filename, line);
      }
      if(TRACE_SHOW_THREAD && TRACE_SHOW_TIME && TRACE_SHOW_FILE) {
          chprintf((BaseSequentialStream*)console, "[0x%08x]", chThdGetSelfX());
      }
      chprintf((BaseSequentialStream*)console, " ");
      va_list args;
      va_start(args, format);
      chvprintf((BaseSequentialStream*)console, format, args);
      va_end(args);
      chprintf((BaseSequentialStream*)console, "\r\n");
	}

#if ENABLE_SERIAL_TRACE == TRUE
	if(TRACE_SHOW_TIME) {
		chprintf((BaseSequentialStream*)trace, "[%8d.%03d]", chVTGetSystemTime()/CH_CFG_ST_FREQUENCY, (chVTGetSystemTime()*1000/CH_CFG_ST_FREQUENCY)%1000);
	}
	chprintf((BaseSequentialStream*)trace, "[%s]", type);
	if(TRACE_SHOW_FILE) {
		chprintf((BaseSequentialStream*)trace, "[%12s %04d]", filename, line);
	}
    if(TRACE_SHOW_THREAD && TRACE_SHOW_TIME && TRACE_SHOW_FILE) {
        chprintf((BaseSequentialStream*)trace, "[0x%08x]", chThdGetSelfX());
    }
    chprintf((BaseSequentialStream*)trace, " ");
    va_list args;
    va_start(args, format);
	chvprintf((BaseSequentialStream*)trace, format, args);
    va_end(args);
    chprintf((BaseSequentialStream*)trace, "\r\n");
#endif /* ENABLE_SERIAL_DEBUG == TRUE */

	chMtxUnlock(&debug_mtx);
}

