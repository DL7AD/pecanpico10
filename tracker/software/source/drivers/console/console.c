#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "shell.h"
#include "commands.h"
#include "pktconf.h"
#include "console.h"


static const ShellConfig shell_cfg = {
	(BaseSequentialStream*)&SDU1,
	commands
};

static con_chn_state_t console_state;

/**
 * @brief   Manage trace output and shell on serial channel.
 * @notes   TRACE output is sent to serial.
 * @notes   TRACE output is suspended when any key is pressed on terminal.
 * @notes   A new shell is invoked and remains active until logout.
 * @notes   TRACE output is then resumed.
 *
 * @thread
 */
THD_FUNCTION(pktConsole, arg) {
  BaseAsynchronousChannel *driver = (BaseAsynchronousChannel *)arg;
  event_listener_t con_el;

  thread_t *shelltp = NULL;
  chEvtRegisterMaskWithFlags(chnGetEventSource(driver),
                      &con_el,
                      CONSOLE_CHANNEL_EVT,
                      CHN_CONNECTED | CHN_DISCONNECTED
                      | CHN_INPUT_AVAILABLE/* | CHN_OUTPUT_EMPTY*/);
  console_state = CON_CHN_IDLE;

  /* Next run the handshake protocol to start the control thread. */

  /*Wait for start from initiator. */
  thread_t *initiator = chMsgWait();
  (void)chMsgGet(initiator);

  /* Release the initiator which next enables the channel. */
  chMsgRelease(initiator, MSG_OK);

  /* Wait for channel to be started. */
  initiator = chMsgWait();
  (void)chMsgGet(initiator);

  /* Release the initiator which then completes. */
  chMsgRelease(initiator, MSG_OK);

  while(true) {
    chEvtWaitAny(CONSOLE_CHANNEL_EVT);
    BaseSequentialStream *chp = (BaseSequentialStream *)driver;
    eventflags_t evtf = chEvtGetAndClearFlags(&con_el);

    switch(console_state) {

    case CON_CHN_IDLE: {
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
    } /* End case CON_CHN_IDLE */

    case CON_CHN_OUT: {
      if(evtf & CHN_DISCONNECTED) {
        console_state = CON_CHN_IDLE;
        break;
      }
      if(evtf & CHN_INPUT_AVAILABLE) {
        chprintf(chp, "\r\n*** Trace suspended - type ^D or use the "
            "'exit' command to resume trace ***\r\n");
        console_state = CON_CHN_FLUSH;
        break;
      }
      break;
    } /* End case CON_CHN_OUT */

    case CON_CHN_SHELL: {
      /* Channel disconnect. */
      if(evtf & CHN_DISCONNECTED) {
        /* Wait for the shell to see EOF from input read and terminate. */
        chThdWait(shelltp);
        shelltp = NULL;
        console_state = CON_CHN_IDLE;
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
    } /* End case CON_CHN_SHELL */

    case CON_CHN_FLUSH: {
      if(shelltp == NULL) {
        shellInit();
        shelltp = chThdCreateFromHeap(NULL,
                                      THD_WORKING_AREA_SIZE(1 * 1024),
                                      "shell", NORMALPRIO + 1,
                                      shellThread,
                                      (void*)&shell_cfg);
        if(shelltp == NULL) {
          chprintf(chp, "\r\n*** Failed to open shell ***\r\n");
          console_state = CON_CHN_OUT;
          break;
        }
        console_state = CON_CHN_SHELL;
      }
      break;
    } /* End case CON_CHN_FLUSH */

    default:
      break;
    } /* End switch. */
  }
}

/*
 *
 */
msg_t pktStartConsole(void) {

  /* Init and start USB. */
  usbObjectInit(&USBD1);

  usbStart(&USBD1, &usbcfg);

  /* Init serial over USB. */
  sduObjectInit(&SDU1);

  /* Start the console handler. */
  thread_t *console = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(1024),
              "CON",
              LOWPRIO + 10,
              pktConsole,
              &SDU1);
  if(console == NULL)
    return MSG_TIMEOUT;

  /* Wait for thread to start. */
  msg_t smsg = chMsgSend(console, MSG_OK);

  /* Start serial over USB. */
  sduStart(&SDU1, &serusbcfg);

  /* Signal thread to enter trace output and shell request monitoring. */
  smsg = chMsgSend(console, MSG_OK);

  /* Set SDIS in USB controller. */
  usbDisconnectBus(&USBD1);

  chThdSleep(TIME_MS2I(1000));

  /* Remove SDIS. */
  usbConnectBus(&USBD1);

  return smsg;
}

/**
 *
 */
bool isConsoleOutputAvailable(void) {
  /* Return console connection status. */
    return (bool)(console_state == CON_CHN_OUT);
}
