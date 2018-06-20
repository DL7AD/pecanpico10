#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "shell.h"
#include "commands.h"
#include "pktconf.h"

static thread_t *shelltp;
sdu_term_t sdu_chn_state;

event_listener_t sdu1_el;

static const ShellConfig shell_cfg = {
	(BaseSequentialStream*)&SDU1,
	commands
};

/*
 *
 */
void startUSB(void) {
	usbObjectInit(&USBD1);

    usbStart(&USBD1, &usbcfg);

    /* Currently does nothing. */
	usbDisconnectBus(&USBD1);

	chThdSleep(TIME_MS2I(100));

    /* Currently does nothing. */
    usbConnectBus(&USBD1);

	sdu_chn_state = TERM_SDU_INIT;
}

/**
 * @brief   Manage trace output and shell on Serial Over USB.
 *
 */
void startSDU(void) {
  if(sdu_chn_state != TERM_SDU_INIT)
    return;
  sduObjectInit(&SDU1);
  chEvtRegister(chnGetEventSource(&SDU1), &sdu1_el, USB_SDU1_EVT);
  sduStart(&SDU1, &serusbcfg);
  sdu_chn_state = TERM_SDU_START;
}

/**
 * @brief   Manage trace output and shell on Serial Over USB.
 * @notes   TRACE output is sent to USB serial.
 * @notes   TRACE output is suspended when any key is pressed on terminal.
 * @notes   A new shell is invoked and remains active until logout.
 * @notes   TRACE output is then resumed.
 *
 * @api
 */
void manageTraceAndShell(void) {

    if(chEvtGetAndClearEvents(EVENT_MASK(USB_SDU1_EVT)) == 0)
      return;

    BaseSequentialStream *chp = (BaseSequentialStream *)&SDU1;

    eventflags_t evtf = chEvtGetAndClearFlags(&sdu1_el);

    switch(sdu_chn_state) {
    case TERM_SDU_INIT:
      break;

    case TERM_SDU_START:
      if(evtf & CHN_CONNECTED) {
        sdu_chn_state = TERM_SDU_OUT;
        chprintf(chp, "\r\n*** Terminal connected ***\r\n");
        break;
      }
      if(evtf & CHN_DISCONNECTED) {
        sdu_chn_state = TERM_SDU_IDLE;
        break;
      }
      break;

    case TERM_SDU_IDLE: {
/*      if(evtf == 0)
        break;*/
      if(evtf & CHN_CONNECTED) {
        sdu_chn_state = TERM_SDU_OUT;
        chprintf(chp, "\r\n*** Trace output enabled ***\r\n");
        //break;
      }
      break;
    } /* End case TERM_SDU_IDLE */

    case TERM_SDU_OUT: {
      if(evtf & CHN_DISCONNECTED) {
        sdu_chn_state = TERM_SDU_START;
        break;
      }
      if(evtf & CHN_INPUT_AVAILABLE) {
        /* Flush the input queue. */
        while(chnGetTimeout((SerialUSBDriver *)chp,
                            TIME_MS2I(100)) != STM_TIMEOUT);
        chprintf(chp, "\r\n*** Trace suspended - type ^D or use the "
            "'exit' command to resume trace ***\r\n");
        shellInit();
        shelltp = chThdCreateFromHeap(NULL,
                                      THD_WORKING_AREA_SIZE(4*1024),
                                      "shell", NORMALPRIO + 1,
                                      shellThread,
                                      (void*)&shell_cfg);
        if(shelltp == NULL) {
          chprintf(chp, "\r\n*** Failed to open shell ***\r\n");
          break;
        }
        sdu_chn_state = TERM_SDU_SHELL;
      }
      break;
    } /* End case TERM_SDU_OUT */

    case TERM_SDU_SHELL: {
      /* USB disconnect. */
      if(evtf & CHN_DISCONNECTED) {
        chThdTerminate(shelltp);
        sdu_chn_state = TERM_SDU_EXIT;
        break;
      }
      /* Was shell terminated from CLI? */
      if(chThdTerminatedX(shelltp)) {
          chThdWait(shelltp);
          shelltp = NULL;
          sdu_chn_state = TERM_SDU_OUT;
          chprintf(chp, "\r\n*** Trace resumed by user ***\r\n");
      }
      break;
    } /* End case TERM_SDU_SHELL */

    case TERM_SDU_EXIT: {
      chThdWait(shelltp);
      shelltp = NULL;
      sdu_chn_state = TERM_SDU_START;
      break;
    } /* End case TERM_SDU_EXIT */

    default:
      break;
    } /* End switch. */
}

/*
 *
 */
bool isSDUAvailable(void) {
  /* Return channel connection status of SDU. */
    return (bool)(sdu_chn_state == TERM_SDU_OUT);
}
