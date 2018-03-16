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
	startUSB();
	#endif

	// Startup threads
	start_essential_threads();	// Startup required modules (tracking manager, watchdog)
	start_user_threads();		// Startup optional modules (eg. POSITION, LOG, ...)

	while(true) {
		#if ACTIVATE_USB
		if(isUSBactive()) {
			startShell();

			eventmask_t evt = chEvtWaitAnyTimeout(EVENT_MASK(0) | EVENT_MASK(1), TIME_S2I(1));
			if(evt & EVENT_MASK(1)) {
			  eventflags_t flags = chEvtGetAndClearFlags(&pkt_el);
			  if(flags & EVT_PWM_QUEUE_FULL) {
			    TRACE_WARN("PKT  > PWM queue full");
			  }
              if(flags & EVT_PWM_FIFO_EMPTY) {
                TRACE_WARN("PKT  > PWM FIFO exhausted");
              }
              if(flags & EVT_AX25_NO_BUFFER) {
                TRACE_WARN("PKT  > AX25 FIFO exhausted");
              }
              if(flags & EVT_ICU_SLEEP_TIMEOUT) {
                TRACE_INFO("PKT  > PWM ICU has entered sleep");
              }
              if(flags & EVT_AX25_BUFFER_FULL) {
                TRACE_WARN("PKT  > AX25 receive buffer full");
              }
              if(flags & EVT_DECODER_ERROR) {
                TRACE_ERROR("PKT  > Decoder error");
              }
              if(flags & EVT_PWM_UNKNOWN_INBAND) {
                TRACE_ERROR("PKT  > Unknown PWM inband flag");
              }
              if(flags & EVT_ICU_OVERFLOW) {
                TRACE_WARN("PKT  > PWM ICU overflow");
              }
              if(flags & EVT_PWM_STREAM_TIMEOUT) {
                TRACE_WARN("PKT  > PWM steam timeout");
              }
              if(flags & EVT_PWM_NO_DATA) {
                TRACE_WARN("PKT  > PWM data not started from radio");
              }

			}
		}
		#endif
        chThdSleep(TIME_S2I(1));
	}
}

