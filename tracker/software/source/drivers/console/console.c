#include "ch.h"
#include "hal.h"
#include "usbcfg2.h"
#include "shell.h"
#include "commands.h"
#include "pktconf.h"
#include "console.h"

#if USE_UART_FOR_CONSOLE == TRUE
#warning "Console is set to UART"
#endif

/*===========================================================================*/
/* Module macros.                  .                                         */
/*===========================================================================*/

#define flushConsoleInputQueue(chp)                                         \
  msg_t c;                                                                  \
  do {                                                                      \
    c = chnGetTimeout((BaseAsynchronousChannel *)chp, TIME_MS2I(100));      \
  } while(c == STM_OK);

/*===========================================================================*/
/* Module data structures and arrays.                                        */
/*===========================================================================*/

static ShellConfig shell_cfg = {
	NULL,
	commands
};

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

static con_chn_state_t console_state;
thread_t *shelltp = NULL;

/**
 * Array for looking up console state.
 */
static const char *state[] = {CONSOLE_STATE_NAMES};

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * Get pointer to console state as string
 */
const char *console_state_name(uint8_t index) {
  return (index > CONSOLE_STATE_MAX ? "INVALID" : state[index]);
}

/**
 * Event handlers.
 */
static void pktConsoleConnected(eventid_t id) {
  (void)id;
  switch(console_state) {
  case CON_CHN_INIT:
    console_state = CON_CHN_CONNECT;
    if(CON_DEBUG_TRACE)
      TRACE_DEBUG("CON  > Connect event when in INIT");
    break;

  case CON_CHN_IDLE: {
    console_state = CON_CHN_CONNECT;
    if(CON_DEBUG_TRACE)
      TRACE_DEBUG("CON  > Connect event when in IDLE");
    break;
  }

  case CON_CHN_TRACE: {
    TRACE_DEBUG("CON  > Connect event when in TRACE");
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
    if(CON_DEBUG_TRACE)
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
    if(CON_DEBUG_TRACE)
      TRACE_DEBUG("CON  > Disconnect event when in IDLE");
    break;
  }

  case CON_CHN_TRACE: {
    /* When the USB cable is unplugged it is possible to get a flood of events. */
    if(CON_DEBUG_TRACE)
      TRACE_DEBUG("CON  > Disconnect event when in TRACE");
    console_state = CON_CHN_IDLE;
    break;
  }

  case CON_CHN_SHELL: {
    if(CON_DEBUG_TRACE)
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
     * At shell termination set re-connect state to re-start shell or trace state.
     */
    console_state = CON_CHN_TERM;
    break;
  }

  case CON_CHN_FLUSH: {
    TRACE_ERROR("CON  > Disconnect event when in FLUSH");
    break;
  }

  case CON_CHN_CONNECT: {
    if(CON_DEBUG_TRACE)
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
    if(CON_DEBUG_TRACE)
      TRACE_DEBUG("CON  > Input available event when in TRACE");
    TRACE_INFO("CON  > Enter command mode");
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
    if(CON_DEBUG_TRACE)
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

  BaseSequentialStream* chp = arg;

  /* Even listener objects. */
  event_listener_t con_el;
  event_listener_t shell_el;

  console_state = CON_CHN_TRACE;

  /* Wait for serial channel to be started for us. */
  thread_t *initiator = chMsgWait();
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

  (void)chEvtGetAndClearEvents(CONSOLE_CHANNEL_EVT);
  (void)chEvtGetAndClearFlags(&con_el);

  /* Flag that channel setup is done. */
  chMsgRelease(initiator, MSG_OK);

  /* Run. */
  do {
    eventmask_t evt = chEvtWaitAny(CONSOLE_CHANNEL_EVT | CONSOLE_SHELL_EVT);
    eventflags_t evtf = chEvtGetAndClearFlags(&con_el);
    if(CON_DEBUG_TRACE)
      TRACE_DEBUG("CON  > Events %x with flags %x in state %s",
                evt, evtf, console_state_name(console_state));
    if(evt & CONSOLE_CHANNEL_EVT) {
      /* Execute the event handlers. */
      chEvtDispatch(handlers, evtf);
      /* Process state either current or as updated by event handlers. */
      switch(console_state) {
      /* The next two cases are entered by a channel connect happening. */
      case CON_CHN_WAIT:
      case CON_CHN_CONNECT:
        /* Wait for any garbage input to subside. */
        chThdSleep(TIME_MS2I(1200));
       (void)chEvtGetAndClearEvents(CONSOLE_CHANNEL_EVT);
       (void)chEvtGetAndClearFlags(&con_el);
       /* FALLTHRU  */
      case CON_CHN_FLUSH: {
        /* Flush the input queue. */
        flushConsoleInputQueue(chp);
        chThdSleep(TIME_MS2I(100));
        if(console_state == CON_CHN_CONNECT) {
          /* Put out our welcome message and enable trace. */
          chprintf(chp, "\r\n*** Terminal connected ***\r\n");
          console_state = CON_CHN_TRACE;
          /* Wait for next event. */
          continue;
        }
        /* Else we start or resume shell mode. */
        chprintf(chp, "\r\n*** Trace suspended - type ^D or use the "
                      "'exit' command to resume trace ***\r\n");
        chDbgAssert(shelltp == NULL, "shell thread still assigned");

        /* Set the channel for the shell to use. */
        shell_cfg.sc_channel = chp;
        shellInit();
        shelltp = chThdCreateFromHeap(NULL,
                                      THD_WORKING_AREA_SIZE(3 * 1024),
                                      "shell", LOWPRIO,
                                      shellThread,
                                      (void *)&shell_cfg);
        if(shelltp == NULL) {
          console_state = CON_CHN_TRACE;
          TRACE_ERROR("CON  > Failed to create shell");
          /* Wait for next event. */
          continue;
        }
        /* Register for shell terminate event. */
        chEvtRegisterMaskWithFlags(
                            &shell_terminated,
                            &shell_el, CONSOLE_SHELL_EVT,
                            (eventflags_t)0);
        console_state = CON_CHN_SHELL;
        /* Wait for next event. */
        continue;
      } /* End case CON_CHN_CONNECT, CON_CHN_WAIT or CON_CHN_FLUSH. */

      default:
        /* Check if there was a shell event. */
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
      if(CON_DEBUG_TRACE)
        TRACE_DEBUG("CON  > Terminating shell thread %x with events"
                          " %x & flags %x in state %s",
                  shelltp, evt, evtf, console_state_name(console_state));
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
        TRACE_INFO("CON  > Resume trace mode");
        continue;
      }
      /*
       * Set next state to be used when coming out of disconnected.
       * If the shell is open then re-establish it on re-connect.
      */
      console_state = ((console_state == CON_CHN_TERM) && CON_RESUME_SHELL)
                                                      ? CON_CHN_WAIT
                                                      : CON_CHN_IDLE;
    } /* End of shell event block. */
  } while(true);
}


/*
 *
 */
msg_t pktStartConsole(BaseSequentialStream *ser) {

  /* Start the console handler. */
  thread_t *con_thd = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(1 * 1024),
              "CON",
              LOWPRIO,
              pktConsole,
              ser);
  if(con_thd == NULL)
    return MSG_TIMEOUT;

  /* Wait for thread to initialise. */
  msg_t smsg = chMsgSend(con_thd, MSG_OK);

  return smsg;
}

/**
 *
 */
bool isConsoleOutputAvailable(void) {
  /* Return console connection status. */
    return (bool)(console_state == CON_CHN_TRACE);
}
