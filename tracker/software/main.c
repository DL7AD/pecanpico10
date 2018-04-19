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
    sysConfigureCoreIO();

	// Init debugging (Serial debug port, LEDs)
	DEBUG_INIT();

	/*
	 * Setup buffers in CCM if available.
	 * Setup packet primary data.
	 */
	bool pkt = pktSystemInit();

    chDbgAssert(pkt == true, "failed to init packet system");

    /* Start Serial Over USB. */
    startSDU();

    /* Start serial channels if selected. */
    pktSerialStart();

    /* Create packet radio service. */
    if(!pktServiceCreate(PKT_RADIO_1)) {
      TRACE_ERROR("PKT  > Unable to create packet services");
    } else {
      pktEnableEventTrace();
    }

   TRACE_INFO("MAIN > Startup");

	// Startup threads
	start_essential_threads();	// Startup required modules (tracking manager, watchdog)
	start_user_threads();		// Startup optional modules (eg. POSITION, LOG, ...)

	while(true) {
      #if ACTIVATE_USB
          manageTraceAndShell();
          pktTraceEvents();
      #endif /* ACTIVATE_USB */
      /* Wait in a loop if nothing to do. */
      chThdSleep(TIME_MS2I(200));
	}
}

