//#include "ch.h"
//#include "hal.h"
#include "pktconf.h"
#include "debug.h"
#include "threads.h"

/**
 * Main routine is starting up system, runs the software watchdog (module monitoring), controls LEDs
 */
int main(void) {
	halInit();					// Startup HAL
	chSysInit();				// Startup RTOS
    /*
     * Setup packet system...
     * TODO: Check power state and start systems only if enough power available
     * Create memory (CCM based heap) if enabled.
     * - Most DSP related data is held in CCM SRAM2 alias.
     * - When using fixed point DSP the use of CCM may improve performance.
     * - No DMA is involved in DSP data so CCM is viable to use.
     * - The exceptions stack can also be located in CCM.
     * - See the .ld file in the cfg folder for memory layout.
     * Configure core IO/bus devices
     * Setup the debug output (UART, USB)
     * Configure radios.
     * Start system services.
     * Start packet event monitoring.
     * Start applications.
     */
    pktSystemInit();

	TRACE_INFO("MAIN > Loop active");
#if DISABLE_HW_WATCHDOG == TRUE
	TRACE_WARN("MAIN > *** Hardware watchdog is disabled ***");
#endif
#if USE_UART_FOR_CONSOLE == TRUE
    TRACE_WARN("MAIN > *** Console is set to UART channel ***");
#endif
	while(true) {
	  /* Trace events from packet decoder system. */
      pktTraceServiceEvents();
      chThdSleep(TIME_MS2I(10));
	}
}

