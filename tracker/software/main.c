#include "ch.h"
#include "hal.h"
#include "pktconf.h"

#include "debug.h"
#include "threads.h"

/**
  * Main routine is starting up system, runs the software watchdog (module monitoring), controls LEDs
  */
int main(void) {
	halInit();					// Startup HAL
	chSysInit();				// Startup RTOS

    /* Setup core IO peripherals. */
    pktConfigureCoreIO();

    /* Setup the mutex for trace output. */
    //DEBUG_INIT();

#if ACTIVATE_CONSOLE
    /* Start console. */
    pktStartConsole();
    TRACE_INFO("MAIN > Console startup");
#endif

	/*
	 * Setup buffers in CCM if available.
	 * Setup packet primary data.
	 */
	bool pkt = pktSystemInit();

    chDbgAssert(pkt == true, "failed to init packet system");

    /* Start serial debug channel(s) if selected. */
    pktSerialStart();

    /*
     * Create a packet radio service.
     * For now there is just one radio.
     */
    while(!pktServiceCreate(PKT_RADIO_1)) {
      TRACE_ERROR("MAIN > Unable to create packet radio services");
      chThdSleep(TIME_S2I(10));
    }

    pktEnableEventTrace(PKT_RADIO_1);
    TRACE_INFO("MAIN > Started packet radio service for radio %d", PKT_RADIO_1);

    TRACE_INFO("MAIN > Starting application and ancillary threads");

	// Startup threads
	start_essential_threads();	// Startup required modules (tracking manager, watchdog)
	start_user_threads();		// Startup optional modules (eg. POSITION, LOG, ...)

	TRACE_INFO("MAIN > Active");
	while(true) {
	  /* Trace events from packet decoder system. */
      pktTraceEvents();
      chThdSleep(TIME_MS2I(200));
	}
}

