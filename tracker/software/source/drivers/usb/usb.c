#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "shell.h"
#include "commands.h"
#include "pktconf.h"


static const ShellConfig shell_cfg = {
	(BaseSequentialStream*)&SDU1,
	commands
};

static con_chn_state_t console_state;

/**
 * @brief   Manage trace output and shell on Serial Over USB.
 * @notes   TRACE output is sent to USB serial.
 * @notes   TRACE output is suspended when any key is pressed on terminal.
 * @notes   A new shell is invoked and remains active until logout.
 * @notes   TRACE output is then resumed.
 *
 * @thread
 */
THD_FUNCTION(pktConsole, arg) {
  BaseAsynchronousChannel *driver = (BaseAsynchronousChannel *)arg;
  event_listener_t con_el;

  thread_t *shelltp;
/*  chEvtRegisterMaskWithFlags(chnGetEventSource(driver),
                      &con_el,
                      CONSOLE_CHANNEL_EVT,
                      CHN_CONNECTED | CHN_DISCONNECTED | CHN_INPUT_AVAILABLE);*/
  chEvtRegister(chnGetEventSource(driver), &con_el, CONSOLE_CHANNEL_EVT);
  console_state = CON_CHN_READY;

  /* Initialisation done. Wait for start from initiator. */
  thread_t *initiator = chMsgWait();
  (void)chMsgGet(initiator);

  /* Release the initiator which then enables the channel. */
  chMsgRelease(initiator, MSG_OK);

  while(true) {
    chEvtWaitAny(EVENT_MASK(CONSOLE_CHANNEL_EVT));
    BaseSequentialStream *chp = (BaseSequentialStream *)driver;
    eventflags_t evtf = chEvtGetAndClearFlags(&con_el);

    switch(console_state) {

    case CON_CHN_READY:
      if(evtf & CHN_CONNECTED) {
        console_state = CON_CHN_OUT;
        chprintf(chp, "\r\n*** Terminal connected ***\r\n");
        break;
      }
      if(evtf & CHN_DISCONNECTED) {
        console_state = CON_CHN_IDLE;
        break;
      }
      break;

    case CON_CHN_IDLE: {
      if(evtf & CHN_CONNECTED) {
        console_state = CON_CHN_OUT;
        chprintf(chp, "\r\n*** Trace output enabled ***\r\n");
      }
      break;
    } /* End case TERM_SDU_IDLE */

    case CON_CHN_OUT: {
      if(evtf & CHN_DISCONNECTED) {
        console_state = CON_CHN_READY;
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
        console_state = CON_CHN_SHELL;
      }
      break;
    } /* End case TERM_SDU_OUT */

    case CON_CHN_SHELL: {
      /* USB disconnect. */
      if(evtf & CHN_DISCONNECTED) {
        chThdTerminate(shelltp);
        console_state = CON_CHN_EXIT;
        break;
      }
      /* Was shell terminated from CLI? */
      if(chThdTerminatedX(shelltp)) {
          chThdWait(shelltp);
          shelltp = NULL;
          console_state = CON_CHN_OUT;
          chprintf(chp, "\r\n*** Trace resumed by user ***\r\n");
      }
      break;
    } /* End case TERM_SDU_SHELL */

    case CON_CHN_EXIT: {
      chThdWait(shelltp);
      shelltp = NULL;
      console_state = CON_CHN_READY;
      break;
    } /* End case TERM_SDU_EXIT */

    default:
      break;
    } /* End switch. */
  }
}

/*
 * TODO: Defer configure of GPIO USB so a disconnect can be signaled on D+.
 * Set D+ (LINE_USB_DP) as pushpull out and low in board.h.
 * Then delay here before setting alternate mode to enable USB IO.
 * Rename this function to pktStartConsole.
 */
msg_t pktStartConsole(void) {

  /* Init and start USB. */
  usbObjectInit(&USBD1);

  usbStart(&USBD1, &usbcfg);

  /* Set SDIS in USB controller. */
  usbDisconnectBus(&USBD1);

  chThdSleep(TIME_MS2I(1000));

  /* Remove SDIS. */
  usbConnectBus(&USBD1);

  /* Init serial over USB. */
  sduObjectInit(&SDU1);

  /* Start the console handler. */
  thread_t *console = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(1024),
              "CON",
              NORMALPRIO - 10,
              pktConsole,
              &SDU1);
  if(console == NULL)
    return MSG_TIMEOUT;

  msg_t smsg = chMsgSend(console, MSG_OK);

  /* Start serial over USB. */
  sduStart(&SDU1, &serusbcfg);
  return smsg;
}

/**
 *
 */
bool isConsoleOutputAvailable(void) {
  /* Return channel connection status of SDU. */
    return (bool)(console_state == CON_CHN_OUT);
}
