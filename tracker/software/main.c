#include "ch.h"
#include "hal.h"
#include "pktconf.h"

#include "debug.h"
#include "threads.h"
#include "padc.h"

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
	 // This won't actually display since USB isn't initialized yet.
	TRACE_INFO("MAIN > Startup");

	/*
	 * Setup buffers in CCM if available.
	 * Setup IO device arbitration.
	 */
	bool pkt = pktSystemInit();

    chDbgAssert(pkt == true, "failed to init packet system");

    /* Start serial channels if selected. */
    pktSerialStart();

    /* Create packet radio service. */
    if(!pktServiceCreate(PKT_RADIO_1)) {
      TRACE_ERROR("PKT  > Unable to create packet services");
    } else {
      pktEnableEventTrace();
    }

	#if ACTIVATE_USB
	startUSB();
	#endif

	// Startup threads
	start_essential_threads();	// Startup required modules (tracking manager, watchdog)
	start_user_threads();		// Startup optional modules (eg. POSITION, LOG, ...)

	while(true) {
        #if ACTIVATE_USB
		if(isUSBactive()) {
			manageShell();
			pktTraceEvents();
			continue;
		}
        #endif /* ACTIVATE_USB */
		/* Wait in a loop if nothing to do. */
        chThdSleep(TIME_S2I(1));
	}
}

