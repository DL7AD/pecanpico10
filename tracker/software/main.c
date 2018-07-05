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

	// Init debugging (Serial debug port, LEDs)
	DEBUG_INIT();

#if ACTIVATE_USB
	/*
	 * TODO: Defer configure of USB mode.
	 * Set D+ (LINE_USB_DP) as pushpull out and low in board.h.
	 * Then delay here before ALT 10 for USB.
	 */
    /* Start Serial Over USB. */
    startSDU();
    TRACE_INFO("MAIN > USB startup");
#endif

	/*
	 * Setup buffers in CCM if available.
	 * Setup packet primary data.
	 */
	bool pkt = pktSystemInit();

    chDbgAssert(pkt == true, "failed to init packet system");

    /* Start serial diagnostic channels if selected. */
    pktSerialStart();

    /* Create packet radio service. */
    if(!pktServiceCreate(PKT_RADIO_1)) {
      TRACE_ERROR("PKT  > Unable to create packet services");
    } else {
      pktEnableEventTrace();
    }

   TRACE_INFO("MAIN > Starting threads");

	// Startup threads
	start_essential_threads();	// Startup required modules (tracking manager, watchdog)
	start_user_threads();		// Startup optional modules (eg. POSITION, LOG, ...)

	   TRACE_INFO("MAIN > Active");
	while(true) {
      #if ACTIVATE_USB
          manageTraceAndShell();
          pktTraceEvents();
      #endif /* ACTIVATE_USB */
      /* Wait in a loop if nothing to do. */
      chThdSleep(TIME_MS2I(200));
	}
}

