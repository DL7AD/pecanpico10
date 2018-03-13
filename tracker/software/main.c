#include "ch.h"
#include "hal.h"
#include "pktconf.h"

#include "debug.h"
#include "threads.h"
#include "padc.h"
#include "usbcfg.h"
#include "shell.h"

static const ShellCommand commands[] = {
	{"debug", debugOnUSB},
	{"picture", printPicture},
	{"log", readLog},
	{"config", printConfig},
	{"command", command2Camera},
	{"aprs_message", send_aprs_message},
	{NULL, NULL}
};

static const ShellConfig shell_cfg = {
	(BaseSequentialStream*)&SDU1,
	commands
};

/**
  * Main routine is starting up system, runs the software watchdog (module monitoring), controls LEDs
  */
int main(void) {
	halInit();					// Startup HAL
	chSysInit();				// Startup RTOS

	// Init debugging (Serial debug port, LEDs)
	DEBUG_INIT();
	TRACE_INFO("MAIN > Startup");


    /* Start serial channels. */
    pktSerialStart();

    event_listener_t pkt_el;

    /* Create packet radio service. */
    if(!pktServiceCreate(PKT_RADIO_1)) {
      TRACE_ERROR("PKT  > Unable to create packet services");
    } else {
      chEvtRegister(pktGetEventSource(&RPKTD1), &pkt_el, 1);
    }

	#if ACTIVATE_USB
	// Start USB
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);

	usbDisconnectBus(serusbcfg.usbp);
	chThdSleep(TIME_MS2I(100));
	usbStart(serusbcfg.usbp, &usbcfg);
	usbConnectBus(serusbcfg.usbp);
	usb_initialized = true;

	// Initialize shell
	thread_t *shelltp = NULL;
	event_listener_t shell_el;
	shellInit();
	chEvtRegister(&shell_terminated, &shell_el, 0);
	#endif

	// Startup threads
	start_essential_threads();	// Startup required modules (tracking manager, watchdog)
	start_user_threads();		// Startup optional modules (eg. POSITION, LOG, ...)

	while(true) {
		#if ACTIVATE_USB
		if(SDU1.config->usbp->state == USB_ACTIVE) {
			if(shelltp == NULL) {
				shelltp = chThdCreateFromHeap(NULL,
				                              THD_WORKING_AREA_SIZE(1024),
				                              "shell", NORMALPRIO + 1,
				                              shellThread,
				                              (void*)&shell_cfg);
			}
			eventmask_t evt = chEvtWaitAnyTimeout(EVENT_MASK(0) | EVENT_MASK(1), TIME_S2I(1));
			if(chThdTerminatedX(shelltp)) {
				chThdRelease(shelltp);
				shelltp = NULL;
				continue;
			}
			if(evt & EVENT_MASK(1)) {
			  eventflags_t flags = chEvtGetAndClearFlags(&pkt_el);
			  if(flags & EVT_PWM_QUEUE_FULL) {
			    TRACE_WARN("PKT  > PWM queue full");
			  }
              if(flags & EVT_PWM_FIFO_EMPTY) {
                TRACE_WARN("PKT  > PWM FIFO exhausted");
              }
			}
		}
		#endif
        if(shelltp == NULL)
          chThdSleep(TIME_S2I(1));
	}
}

