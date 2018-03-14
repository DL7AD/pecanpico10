/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    rxpwm.c
 * @brief   PWM data handler for radio.
 * @brief   the ICU driver is used to capture PWM data.
 *
 * @addtogroup channels
 * @details The Radio PWM is a subsystem that will:
 *          - Respond to the CCA (squelch) gated to the radio NIRQ pin.
 *          - Receive PWM format AFSK data from the si446x radio.
 *          - Buffer data in a shared access FIFO posted to the decoder process.
 *          .
 *          The PWM interface is designed to handle multiple sequential transmissions.
 *          A buffer is assigned after CCA is de-glitched.
 *          Radio PWM data is written to a shared queue.
 *          The Radio is the producer side. The decoder is the consumer side.
 *          The demodulator/decoder operates at thread level to decode PWM.<br>
 * @pre     This subsystem requires an extended ICU data structure.
 *          see halconf.h for the configuration.
 * @note
 * @{
 */

#include "pktconf.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/* ICU configuration.
 * TODO: Work out where to put this and manage assigning ICU.
 * There could be multiple radios so there needs to be an assignment method.
 */

const ICUConfig pwm_icucfg = {
  ICU_INPUT_ACTIVE_HIGH,
  ICU_COUNT_FREQUENCY,          /* ICU clock frequency. */
#if defined(LINE_PWM_MIRROR)
  pktRadioICUWidth,             /* ICU width callback. */
#else
  NULL,                         /* ICU width callback. */
#endif
  pktRadioICUPeriod,            /* ICU period callback. */
  PktRadioICUOverflow,          /* ICU overflow callback. */
#if PWM_TIMER_CHANNEL == 0
  ICU_CHANNEL_1,                /* Timer channel 0. */
#elif PWM_TIMER_CHANNEL == 1
  ICU_CHANNEL_2,                /* Timer channel 1. */
#elif
#error PWM_CHANNEL undefined or incorrectly defined
#endif
  0
};
/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/


/**
 * @brief   Initialises and assigns ICU to a Radio.
 * @post    The ICU is configured and started for a specified radio.
 * @post    The ports and timers for CCA input are configured.
 *
 * @param[in]   radio_id   radio being started.
 *
 * @return  Pointer to assigned ICUDriver object.
 *
 * @api
 */
ICUDriver *pktAttachICU(radio_unit_t radio_id) {
  /* For now there is only one radio and a fixed ICU association.
   * TODO: Implement Radio <-> ICU association code and data structure.
   */
  (void)radio_id;

  /*
   * Initialize the RX_DATA capture ICU.
   */

  /* Set the ICU as declared in portab.h. */
  ICUDriver *myICU = &PWM_ICU;
  icuObjectInit(myICU);

  /* The RX_DATA input is routed to ICU timer.
   * Set in portab.h
   */
  pktSetLineModeICU();

  /* If using PWM mirror to output to a diagnostic port. */
  pktSetLineModePWMMirror();

  /* Initialise the timers. */
  chVTObjectInit(&myICU->cca_timer);
  chVTObjectInit(&myICU->icu_timer);
  chVTObjectInit(&myICU->pwm_timer);

  /* Configure ports. */
  pktSetLineModeCCA();

  /* Setup the squelch LED. */
  pktSetLineModeSquelchLED();
  pktWriteSquelchLED(PAL_LOW);

  /* Setup the overflow LED. */
  pktSetLineModeOverflowLED();
  pktWriteOverflowLED(PAL_LOW);

  /* Setup the no FIFO LED. */
  pktSetLineModeNoFIFOLED();
  pktWriteNoFIFOLED(PAL_LOW);

  return myICU;
}

/**
 * @brief   Detaches the Radio ICU channel.
 * @post    The ICU is stopped.
 * @post    The GPIO for CCA input is disabled.
 * @post    The GPIO for LED indicators are disabled.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktDetachICU(ICUDriver *myICU) {

  chDbgAssert(myICU->link != NULL, "no ICU driver");
  /*
   * Stop the ICU.
   */
  icuStop(myICU);

  /* Disable the squelch LED. */
  pktUnsetLineModeSquelchLED();

  /* Disable overflow LED. */
  pktUnsetLineModeOverflowLED();

  /* Disable no FIFO LED. */
  pktUnsetLineModeNoFIFOLED();

  /* If using PWM mirror disable diagnostic port. */
  pktUnsetLineModePWMMirror();
}

/**
 * @brief   Start the Radio ICU channel.
 * @pre     The ICU is stopped.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktICUStart(ICUDriver *myICU) {
  icuStart(myICU, &pwm_icucfg);
}

/**
 * @brief   Terminates the PWM stream from the ICU.
 * @post    The ICU notification (callback) is stopped.
 * @post    An in-band reason code flag is written to the PWM queue.
 * @post    If the queue is full the optional LED is lit.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 * @param[in]   event flags to be set as to why the channel is closed.
 *
 * @api
 */
void pktClosePWMChannelI(ICUDriver *myICU, eventflags_t evt, pwm_code_t reason) {
  /* Stop posting data and write end marker. */
  AFSKDemodDriver *myDemod = myICU->link;
  packet_svc_t *myHandler = myDemod->packet_handler;
  chDbgAssert(myDemod != NULL, "no demod linked");

  chVTResetI(&myICU->pwm_timer);

  /*
   * Turn off the squelch LED.
   */
  pktWriteSquelchLED(PAL_LOW);

  /* Stop the ICU notification (callback). */
  icuDisableNotificationsI(myICU);
  if(myDemod->active_radio_object != NULL) {
    myDemod->active_radio_object->status |= (EVT_PWM_QUEUE_LOCK | evt);
    pktAddEventFlagsI(myHandler, (EVT_PWM_QUEUE_LOCK | evt));
    input_queue_t *myQueue = &myDemod->active_radio_object->radio_pwm_queue;
    /* End of data flag. */
#if USE_12_BIT_PWM == TRUE
    byte_packed_pwm_t pack = {{0, reason, 0}};
#else
    byte_packed_pwm_t pack = {{0, reason}};
#endif
    msg_t qs = pktWritePWMQueueI(myQueue, pack);
    if(qs != MSG_OK) {
      pktWriteOverflowLED(PAL_HIGH);
      myDemod->active_radio_object->status |= EVT_PWM_QUEUE_FULL;
      pktAddEventFlagsI(myHandler, EVT_PWM_QUEUE_FULL);
    }
    /* Release the decoder thread if waiting. */
    chBSemSignalI(&myDemod->active_radio_object->sem);
    /* Remove object reference. */
    myDemod->active_radio_object = NULL;
  } else {
    pktAddEventFlagsI(myHandler, evt);
  }
  myDemod->icustate = PKT_PWM_READY;
}

/**
 * @brief   Opens the PWM stream from the ICU.
 * @post    The ICU notification (callback) is enabled.
 * @post    If an error occurs the PWM is not started and state is unchanged.
 * @post    If the queue is full the yellow LED is lit.
 * @post    If no error occurs the timers associated with PWM are started.
 * @post    The seized FIFO is sent via the queue mailbox.
 * @post    The ICU state is set to active.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 * @param[in]   event flags to be set as to why the channel is opened.
 *
 * @api
 */
void pktOpenPWMChannelI(ICUDriver *myICU, eventflags_t evt) {
  AFSKDemodDriver *myDemod = myICU->link;
  packet_svc_t *myHandler = myDemod->packet_handler;

  /* Turn on the squelch LED. */
  pktWriteSquelchLED(PAL_HIGH);

  /* Turn off the overflow LED. */
  //pktWriteOverflowLED(PAL_LOW);

  if(myDemod->active_radio_object != NULL) {
    /* TODO: Work out correct handling.
     * Shouldn't happen unless CCA has not triggered an EXTI trailing edge.
     * For now just flag that an error condition happened.
     */
    pktClosePWMChannelI(myICU, EVT_RADIO_CCA_FIFO_ERR, PWM_TERM_NO_QUEUE);
    return;
  }
  /* Normal CCA handling. */
  radio_cca_fifo_t *myFIFO = chFifoTakeObjectI(myDemod->pwm_fifo_pool);
  if(myFIFO == NULL) {
    myDemod->active_radio_object = NULL;
    /* No FIFO available.
     * Send an event to any listener.
     * Disable ICU notifications.
     */
    pktAddEventFlagsI(myHandler, EVT_PWM_FIFO_EMPTY);
    icuDisableNotificationsI(myICU);

    /* Turn on the FIFO out LED. */
    pktWriteNoFIFOLED(PAL_HIGH);
    return;
  }

  myDemod->active_radio_object = myFIFO;

  /* Clear event/status bits. */
  myFIFO->status = 0;

  /*
   * Initialize FIFO release control semaphore.
   * The decoder thread waits on the semaphore before releasing  to pool.
   */
  chBSemObjectInit(&myFIFO->sem, true);

  /* Each FIFO entry has an embedded input queue with data buffer. */
  (void)iqObjectInit(&myFIFO->radio_pwm_queue,
                     myFIFO->packed_buffer.pwm_bytes,
                     sizeof(radio_pwm_buffer_t),
                     NULL , NULL);

  /*
   * Set the status of this FIFO.
   * Send the FIFO entry to the decoder thread.
   */
  chFifoSendObjectI(myDemod->pwm_fifo_pool, myFIFO);
  myFIFO->status |= EVT_PWM_FIFO_SENT;

  /*
   * Start the ICU activity timer.
   * After timeout shutdown ICU.
   * This reduces power consumption.
   */
  chVTSetI(&myICU->icu_timer, TIME_S2I(10),
           (vtfunc_t)pktICUInactivityTimeout, myICU);

  /*
   * Start the PWM activity timer.
   * This catches the condition where CCA raises but no RX data appears.
   */
  chVTSetI(&myICU->pwm_timer, TIME_MS2I(50),
           (vtfunc_t)pktPWMInactivityTimeout, myICU);

  icuStartCaptureI(myICU);
  icuEnableNotificationsI(myICU);
  pktAddEventFlagsI(myHandler, evt);
  myFIFO->status |= evt;

  myDemod->icustate = PKT_PWM_ACTIVE;
}

/**
 * @brief   Stops the ICU capture.
 * @notes   Primarily intended to save on overhead/power.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktSleepICUI(ICUDriver *myICU) {
  /* All we do is stop capture. */
  icuStopCaptureI(myICU);
}

/**
 * @brief   Timer callback when ICU has been inactive.
 * @post    The ICU is put to sleep.
 * @post    The next CCA event will re-enable the ICU.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktICUInactivityTimeout(ICUDriver *myICU) {

  /* This will stop ICU to save power.
   * The ICU notifications are enabled and disabled during normal operation.
   * This timer will shutdown the ICU timer after an idle period.
   */
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;
  packet_svc_t *myHandler = myDemod->packet_handler;
  if(myDemod->active_radio_object == NULL) {
    pktSleepICUI(myICU);
    pktAddEventFlagsI(myHandler, EVT_ICU_SLEEP_TIMEOUT);
  }
  chSysUnlockFromISR();
}

/**
 * @brief   Stop all ICU associated timer.
 * @notes   Will be called when the packet channel is stopped.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktStopAllICUTimersI(ICUDriver *myICU) {
  chVTResetI(&myICU->icu_timer);
  chVTResetI(&myICU->cca_timer);
  chVTResetI(&myICU->pwm_timer);
}

/**
 * @brief   Timer callback when no PWM data arises from a CCA open.
 * @post    The PWM channel will be closed
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktPWMInactivityTimeout(ICUDriver *myICU) {
  /* Timeout waiting for PWM data from the radio. */
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;
  if(myDemod->active_radio_object != NULL) {
    pktClosePWMChannelI(myICU, EVT_PWM_NO_DATA, PWM_TERM_ICU_OVERFLOW);
  }
  chSysUnlockFromISR();
}

/**
 * @brief   Timer callback when CCA leading edge de-glitch period expires.
 * @notes   If CCA is still asserted then PWM capture will be enabled.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktRadioCCALeadTimer(ICUDriver *myICU) {
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;
  packet_svc_t *myHandler = myDemod->packet_handler;
  /* CCA de-glitch timer expired. */
  switch(palReadLine(LINE_CCA)) {
    case PAL_LOW: {
        /*
         * CAA has dropped so it is a spike.
         */
      pktAddEventFlagsI(myHandler, EVT_RADIO_CCA_SPIKE);
      break;
      }

    case PAL_HIGH: {
      pktOpenPWMChannelI(myICU, EVT_RADIO_CCA_OPEN);
      break;
    }
  }
  chSysUnlockFromISR();
  return;
}

/**
 * @brief   Timer callback when CCA trailing edge de-glitch period expires.
 * @notes   If CCA is still asserted then PWM capture will continue.
 * @notes   If CCA is not asserted then PWM capture will be closed.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktRadioCCATrailTimer(ICUDriver *myICU) {
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;
  packet_svc_t *myHandler = myDemod->packet_handler;
  /* CCA de-glitch timer for trailing edge expired. */
  switch(palReadLine(LINE_CCA)) {
    case PAL_LOW: {
      /*
       * The decoder operates asynchronously to and usually slower than PWM.
       * When the decoder ends it returns its FIFO object to the pool.
       * Closing PWM sets the FIFO management semaphore.
       */
      pktClosePWMChannelI(myICU, EVT_RADIO_CCA_CLOSE, PWM_TERM_CCA_CLOSE);
      break;
      }

    case PAL_HIGH: {
      /* CCA is active again so leave PWM open. */
      pktAddEventFlagsI(myHandler, EVT_RADIO_CCA_GLITCH);
      break;
    }
  }
  chSysUnlockFromISR();
  return;
}

/**
 * @brief   GPIO callback when CCA edge transitions.
 * @notes   Both edges are de-glitched by the CCA timer.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktRadioCCAInput(ICUDriver *myICU) {
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;

  if(myDemod->icustate == PKT_PWM_STOP) {
    chSysUnlockFromISR();
    return;
  }
  /* CCA changed. */
  switch(palReadLine(LINE_CCA)) {
    case PAL_LOW: {
      if(myDemod->icustate == PKT_PWM_ACTIVE) {
        /* CCA trailing edge glitch handling.
         * Start timer and check if CCA remains low before closing PWM.
         *
         * De-glitch for 8 AFSK bit times.
         */
        chVTSetI(&myICU->cca_timer, TIME_MS2I(7),
                 (vtfunc_t)pktRadioCCATrailTimer, myICU);
      }
      /* Idle state. */
      break;
    } /* End case PAL_LOW. */

    case PAL_HIGH: {
      if(chVTIsArmedI(&myICU->cca_timer)) {
        /* CAA has been re-asserted during trailing edge timer. */
        chVTResetI(&myICU->cca_timer);
        break;
      }
      /* Else this is a leading edge of CCA for a new packet. */
      /* De-glitch for 16 AFSK bit times. */
      chVTSetI(&myICU->cca_timer,
               TIME_MS2I(14),
               (vtfunc_t)pktRadioCCALeadTimer, myICU);
      break;
    }
  } /* End switch. */
  chSysUnlockFromISR();
  return;
}

/**
 * @brief   Width callback from ICU driver.
 * @notes   Called at ISR level.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktRadioICUWidth(ICUDriver *myICU) {
  (void)myICU;
#if defined(LINE_PWM_MIRROR)
  pktWritePWMMirror(PAL_LOW);
#endif
}

/**
 * @brief   Period callback from ICU driver.
 * @notes   Called at ISR level.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktRadioICUPeriod(ICUDriver *myICU) {
  /* ICU data structure is extended with...
   * - a pointer to the decoder control.
   * - timers used in ICU.
   *
   * See halconf.h for the definition.
   */
#if defined(LINE_PWM_MIRROR)
  pktWritePWMMirror(PAL_HIGH);
#endif
  AFSKDemodDriver *myDemod = myICU->link;

  chSysLockFromISR();
  /*
   * On period clear the ICU activity watchdog timer.
   * i.e. Once radio data appears then a "no data" error is invalidated.
   */
  chVTResetI(&myICU->pwm_timer);

  if(myDemod->active_radio_object == NULL) {
    /*
     * Arrive here when we are running but not buffering.
     * The ICU has been stopped and PWM aborted.
     */
    chSysUnlockFromISR();
    return;
  }
  /*
   * Check if decoder has already finished while ICU is still active.
   * The decoder terminates a frame on the first trailing HDLC flag.
   * If CPU is fast (FPU enabled) it might finish decode before ICU stops.
   * A long sequence of trailing HDLC flags or junk after a frame close
   * but before squelch close could cause lingering ICU activity.
   *
   */
  if((myDemod->active_radio_object->status & EVT_AFSK_DECODE_DONE) != 0) {
    pktClosePWMChannelI(myICU, EVT_PWM_STREAM_CLOSED, PWM_TERM_DECODE_ENDED);
    chSysUnlockFromISR();
    return;
  }

  /* Check and write ICU values if OK. */
  msg_t qs = pktQueuePWMDataI(myICU);
  if(qs != MSG_OK) {
    pktWriteOverflowLED(PAL_HIGH);
    pktClosePWMChannelI(myICU, EVT_PWM_QUEUE_FULL, PWM_TERM_QUEUE_FULL);
  }
  chSysUnlockFromISR();
  return;
}

/**
 * @brief   Overflow callback from ICU driver.
 * @notes   Called at ISR level.
 * @notes   This indicates PWM data that is outside AFSK timing bounds.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void PktRadioICUOverflow(ICUDriver *myICU) {
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;
  packet_svc_t *myHandler = myDemod->packet_handler;
  pktAddEventFlagsI(myHandler, EVT_ICU_OVERFLOW);
  if(myDemod->active_radio_object != NULL) {
    pktClosePWMChannelI(myICU, EVT_ICU_OVERFLOW, PWM_TERM_ICU_OVERFLOW);
  } else {
    /* Just stop the ICU notification. */
    icuDisableNotificationsI(myICU);
  }
  chSysUnlockFromISR();
}

/**
 * @brief   Converts ICU data and posts to the PWM queue.
 * @pre     The ICU driver is linked to a demod driver (pointer to driver).
 * @details Byte values of packed PWM data are written into an input queue.
 *          The operation will succeed if sufficient queue space is available.
 *          If the queue will become full then an in-band QOV flag is written.
 *          In that case PWM data will not be queued unless it was an EOD flag.
 *
 * @param[in] myICU      pointer to the ICU driver structure
 *
 * @return              The operation status.
 * @retval MSG_OK       The PWM data has been queued.
 * @retval MSG_TIMEOUT  The queue is already full.
 * @retval MSG_RESET    Queue space would be exhausted so a QOV
 *                      flag is written in place of PWM data.
 *
 * @iclass
 */
msg_t pktQueuePWMDataI(ICUDriver *myICU) {

  chDbgCheckClassI();

  AFSKDemodDriver *myDemod = myICU->link;
  chDbgAssert(myDemod != NULL, "no linked demod driver");

  input_queue_t *myQueue = &myDemod->active_radio_object->radio_pwm_queue;
  chDbgAssert(myQueue != NULL, "no queue assigned");

  byte_packed_pwm_t pack;
  pktConvertICUtoPWM(myICU, &pack);
  return pktWritePWMQueueI(myQueue, pack);
}

/** @} */
