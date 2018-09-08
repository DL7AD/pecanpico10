#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "shell.h"
#include "commands.h"
#include "pktconf.h"
#include "console.h"

/*===========================================================================*/
/* Module macros.                  .                                         */
/*===========================================================================*/

#define flushConsoleInputQueue(chp)                                         \
  msg_t c;                                                                  \
  do {                                                                      \
    c = chnGetTimeout((BaseAsynchronousChannel *)chp, TIME_MS2I(100));      \
  } while(c != STM_TIMEOUT && c != STM_RESET);

/*===========================================================================*/
/* Module data structures and arrays.                                        */
/*===========================================================================*/

static const ShellConfig shell_cfg = {
	(BaseSequentialStream*)&SDU1,
	commands
};

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

static con_chn_state_t console_state;
thread_t *shelltp = NULL;

/**
 *
 */
static void pktConsoleConnected(eventid_t id) {
  (void)id;
  switch(console_state) {
  case CON_CHN_INIT:
    console_state = CON_CHN_CONNECT;
    TRACE_DEBUG("CON  > Connect event when in INIT");
    break;

  case CON_CHN_IDLE: {
    console_state = CON_CHN_CONNECT;
    TRACE_DEBUG("CON  > Connect event when in IDLE");
    break;
  }

  case CON_CHN_TRACE: {
    TRACE_ERROR("CON  > Connect event when in TRACE");
    break;
  }

  case CON_CHN_SHELL: {
    TRACE_ERROR("CON  > Connect event when in SHELL");
    break;
  }

  case CON_CHN_FLUSH: {
    TRACE_ERROR("CON  > Connect event when in FLUSH");
    break;
  }

  case CON_CHN_CONNECT: {
    TRACE_ERROR("CON  > Connect event when in CONNECT");
    break;
  }

  case CON_CHN_WAIT: {
    /* Re-establish the shell session. */
    TRACE_DEBUG("CON  > Connected event when in WAIT");
    console_state = CON_CHN_FLUSH;
    break;
  }

  case CON_CHN_TERM: {
    TRACE_ERROR("CON  > Connect event when in TERM");
    /* Ignore input when processing connect. */
    break;
  }

  } /* End switch. */
}

/**
 *
 */
static void pktConsoleDisconnected(eventid_t id) {
  (void)id;
  switch(console_state) {
  case CON_CHN_INIT:
    TRACE_ERROR("CON  > Disconnect event when in INIT");
    break;

  case CON_CHN_IDLE: {
    /* When the USB cable is unplugged it is possible to get a flood of events. */
    TRACE_DEBUG("CON  > Disconnect event when in IDLE");
    break;
  }

  case CON_CHN_TRACE: {
    /* When the USB cable is unplugged it is possible to get a flood of events. */
    TRACE_DEBUG("CON  > Disconnect event when in TRACE");
    console_state = CON_CHN_IDLE;
    break;
  }

  case CON_CHN_SHELL: {
    TRACE_DEBUG("CON  > Disconnect event when in SHELL");

    /*
     *  Wait for shell to time out in serial read.
     *  It will broadcast event and enter thread final state.
     */
    chDbgAssert(shelltp != NULL, "shell thread not assigned");
    while(!chThdTerminatedX(shelltp)) {
      chThdSleep(TIME_MS2I(100));
    }

    /*
     * Set state here.
     * The shell event handler waits for shell exit and unregisters event listener.
     */
    console_state = CON_CHN_TERM;
    break;
  }

  case CON_CHN_FLUSH: {
    TRACE_ERROR("CON  > Disconnect event when in FLUSH");
    break;
  }

  case CON_CHN_CONNECT: {
    TRACE_DEBUG("CON  > Disconnect event when in CONNECT");
    console_state = CON_CHN_IDLE;
    break;
  }

  case CON_CHN_WAIT: {
    TRACE_ERROR("CON  > Disconnect event when in WAIT");
    /* Ignore disconnect event when waiting for connect event. */
    break;
  }

  case CON_CHN_TERM: {
    TRACE_ERROR("CON  > Disconnect event when in TERM");
    /* Ignore input when processing connect. */
    break;
  }
  } /* End switch. */
}

/**
 *
 */
static void pktConsoleInputAvailable(eventid_t id) {
  (void)id;
  switch(console_state) {
  case CON_CHN_INIT:
    TRACE_ERROR("CON  > Input available event when in INIT");
    break;

  case CON_CHN_IDLE: {
    TRACE_ERROR("CON  > Input available event when in IDLE");
    break;
  }

  case CON_CHN_TRACE: {
    TRACE_DEBUG("CON  > Input available event when in TRACE");
    console_state = CON_CHN_FLUSH;
    break;
  }

  case CON_CHN_SHELL: {
    //TRACE_DEBUG("CON  > Input available event when in SHELL");
    break;
  }

  case CON_CHN_FLUSH: {
    TRACE_ERROR("CON  > Input available event when in FLUSH");
    break;
  }

  case CON_CHN_CONNECT: {
    TRACE_ERROR("CON  > Input available event when in CONNECT");
    /* Ignore input when processing connect. */
    break;
  }

  case CON_CHN_WAIT: {
    TRACE_ERROR("CON  > Input available event when in WAIT");
    /* Ignore input when processing connect. */
    break;
  }

  case CON_CHN_TERM: {
    TRACE_DEBUG("CON  > Input available event when in TERM");
    /* Ignore input when disconnecting and terminating shell. */
    break;
  }
  } /* End switch. */
}

/**
 * @brief   Manage trace output and shell on serial channel.
 * @notes   TRACE output is sent to serial.
 * @notes   TRACE output is suspended when any key is pressed on terminal.
 * @notes   A new shell is invoked and remains active until terminated.
 * @notes   TRACE output is then resumed.
 *
 * @thread
 */
THD_FUNCTION(pktConsole, arg) {

  BaseSequentialStream* chp = (BaseSequentialStream *)arg;

  event_listener_t con_el;
  event_listener_t shell_el;

  /*Wait for start permission. */
  thread_t *initiator = chMsgWait();
  (void)chMsgGet(initiator);

  console_state = CON_CHN_INIT;

  /* Signal that basic init is done. */
  chMsgRelease(initiator, MSG_OK);

  /* Wait for serial channel to be started for us. */
  initiator = chMsgWait();
  (void)chMsgGet(initiator);

  /* Attach event listener to serial channel. */
  chEvtRegisterMaskWithFlags(
                      chnGetEventSource((BaseAsynchronousChannel *)chp),
                      &con_el, CONSOLE_CHANNEL_EVT,
                      CHN_CONNECTED | CHN_DISCONNECTED | CHN_INPUT_AVAILABLE);

  static evhandler_t handlers[] = {
                                      pktConsoleConnected,
                                      pktConsoleDisconnected,
                                      pktConsoleInputAvailable,
                                      NULL
  };

  /* Flag that channel setup is done. */
  chMsgRelease(initiator, MSG_OK);

  /* Run. */
  do {
    eventmask_t evt = chEvtWaitAny(CONSOLE_CHANNEL_EVT | CONSOLE_SHELL_EVT);
    eventflags_t evtf = chEvtGetAndClearFlags(&con_el);
    TRACE_DEBUG("CON  > Events %x with flags %x in state %x",
                evt, evtf, console_state);
    if(evt & CONSOLE_CHANNEL_EVT) {
      chEvtDispatch(handlers, evtf);
      switch(console_state) {

      case CON_CHN_FLUSH:
        /* Falls through. */
      case CON_CHN_WAIT:
        /* Falls through. */
      case CON_CHN_CONNECT: {
        /* Flush any garbage input. */
        do {
          /* Flush input events. */
          (void)chEvtWaitAnyTimeout(CONSOLE_CHANNEL_EVT, TIME_MS2I(500));
        } while(chEvtGetAndClearFlags(&con_el) & CHN_INPUT_AVAILABLE);
        /* Flush the input queue. */
        flushConsoleInputQueue(chp);
        chThdSleep(TIME_MS2I(100));
        if(console_state == CON_CHN_CONNECT) {
          /* Put out our welcome message and enable trace. */
          chprintf(chp, "\r\n*** Terminal connected ***\r\n");
          console_state = CON_CHN_TRACE;
          continue;
        }
        /* Else we start or resume shell mode. */
        chprintf(chp, "\r\n*** Trace suspended - type ^D or use the "
                      "'exit' command to resume trace ***\r\n");
        chDbgAssert(shelltp == NULL, "shell thread still assigned");
        shellInit();
        shelltp = chThdCreateFromHeap(NULL,
                                      THD_WORKING_AREA_SIZE(3 * 1024),
                                      "shell", NORMALPRIO + 1,
                                      shellThread,
                                      (void *)&shell_cfg);
        if(shelltp == NULL) {
          console_state = CON_CHN_TRACE;
          TRACE_ERROR("CON  > Failed to create shell");
          continue;
        }
        /* Register for shell terminate event. */
        chEvtRegisterMaskWithFlags(
                            &shell_terminated,
                            &shell_el, CONSOLE_SHELL_EVT,
                            (eventflags_t)0);
        console_state = CON_CHN_SHELL;
        continue;
      } /* End case CON_CHN_CONNECT or CON_CHN_WAIT. */

      default:
        break;
      } /* End switch on console_state. */
    } /* End if(evt & CONSOLE_CHANNEL_EVT). */
    /*
     * Check if the shell has terminated by...
     * - channel disconnect
     * - CLI (^D) or exit command
     * - Thread programmatic request.
     */
    if((bool)(evt & CONSOLE_SHELL_EVT) || (console_state == CON_CHN_TERM)) {
      /*
       * The shell was terminated by CLI or via chThdTerminate(...).
       * Unregister from the shell event source.
       * Wait for the thread to exit.
       * If the channel disconnected go to IDLE else resume TRACE
       */
      TRACE_DEBUG("CON  > Terminating shell thread %x with evt %x & evtf %x in state %x",
                  shelltp, evt, evtf, console_state);
      if(shelltp != NULL) {
        chEvtUnregister(&shell_terminated, &shell_el);
        chThdWait(shelltp);
        shelltp = NULL;
      } else {
        TRACE_ERROR("CON  > Shell not running");
      }
      /* Check for CLI or thread terminate. */
      if(console_state == CON_CHN_SHELL) {
        chprintf(chp, "\r\n*** Trace resumed by user ***\r\n");
        chThdSleep(TIME_MS2I(100));
        console_state = CON_CHN_TRACE;
        continue;
      }
      /* Set next state when coming out of disconnected. */
      console_state = (console_state == CON_CHN_TERM) ? CON_CHN_WAIT
                                                      : CON_CHN_IDLE;
    } /* End of shell event block. */
  } while(true);
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
  thread_t *con_thd = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(1 * 1024),
              "CON",
              LOWPRIO + 10,
              pktConsole,
              &SDU1);
  if(con_thd == NULL)
    return MSG_TIMEOUT;

  /* Wait for thread to initialise. */
  msg_t smsg = chMsgSend(con_thd, MSG_OK);

  /* Start serial over USB. */
  sduStart(&SDU1, &serusbcfg);

  chThdSleep(TIME_MS2I(100));

  /* Signal thread to enter trace output and shell request monitoring. */
  smsg = chMsgSend(con_thd, MSG_OK);

  chThdSleep(TIME_MS2I(100));

  /* Signal soft disconnect to the host (DP pull-up disconnected). */
  usbDisconnectBus(&USBD1);

  chThdSleep(TIME_MS2I(1500));

  /*
   * Notify host we are here.
   * If the cable is connected the host should enumerate USB.
   */
  usbConnectBus(&USBD1);

  return smsg;
}

/**
 *
 */
bool isConsoleOutputAvailable(void) {
  /* Return console connection status. */
    return (bool)(console_state == CON_CHN_TRACE);
}
