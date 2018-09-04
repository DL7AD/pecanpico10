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
 *          - Respond to the CCA (squelch) interrupt from the radio NIRQ pin.
 *          - Receive PWM format AFSK data from the si446x radio.
 *          - Buffer data in a shared stream between the radio and the decoder.
 *          - Handle multiple sequential streams through a FIFO mechanism.
 *
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
 * @brief   Attaches decoder to radio hardware according radio config.
 * @post    The PWM ICU is configured and started for a specified radio.
 * @post    The ports and timers for CCA input are configured.
 *
 * @param[in]   radio_id   radio being started.
 *
 * @return  Pointer to assigned ICUDriver object.
 *
 * @api
 */
ICUDriver *pktAttachRadio(const radio_unit_t radio) {
  /*
   * Initialize the association between the radio and the PWM IO.
   */
  ICUDriver *myICU = pktLLDradioAttachStream(radio);

  chDbgAssert(myICU != NULL, "no ICU driver");

  icuObjectInit(myICU);

  /* Initialise the ICU PWM timers. */
  chVTObjectInit(&myICU->cca_timer);
  //chVTObjectInit(&myICU->icu_timer);
  chVTObjectInit(&myICU->pwm_timer);

  /* TODO: Implement LLD call to setup indicator LEDs specific to radio. */
  /* Setup the squelch LED. */
  pktSetGPIOlineMode(LINE_SQUELCH_LED, PAL_MODE_OUTPUT_PUSHPULL);
  pktWriteGPIOline(LINE_SQUELCH_LED, PAL_LOW);

  /* Setup the overflow LED. */
  pktSetGPIOlineMode(LINE_OVERFLOW_LED, PAL_MODE_OUTPUT_PUSHPULL);
  pktWriteGPIOline(LINE_OVERFLOW_LED, PAL_LOW);

  /* Setup the no FIFO LED. */
  pktSetGPIOlineMode(LINE_NO_FIFO_LED, PAL_MODE_OUTPUT_PUSHPULL);
  pktWriteGPIOline(LINE_NO_FIFO_LED, PAL_LOW);

  /* If using PWM mirror to output to a diagnostic port. */
  pktSetGPIOlineMode(LINE_PWM_MIRROR, PAL_MODE_OUTPUT_PUSHPULL);

  return myICU;
}

/**
 * @brief   Detaches the Radio from the PWM handlers.
 * @post    The PWM ICU is stopped.
 * @post    The GPIO for CCA input is disabled.
 * @post    The GPIO for LED indicators are disabled.
 *
 * @param[in]   radio   radio attached to this PWM handler
 *
 * @api
 */
void pktDetachRadio(const radio_unit_t radio) {

  packet_svc_t *myHandler = pktGetServiceObject(radio);
  AFSKDemodDriver *myDemod = (AFSKDemodDriver *)myHandler->rx_link_control;

  chDbgAssert(myDemod != NULL, "no demod linked");

  /*
   * Stop the ICU.
   */
  icuStop(myDemod->icudriver);

  /*
   * Detach the radio from the PWM handlers.
   */
  pktLLDradioDetachStream(radio);
  myDemod->icudriver = NULL;

  /* TODO: Implement LLD call to release indicator LEDs specific to radio. */
  /* Disable the squelch LED. */
  pktUnsetGPIOlineMode(LINE_SQUELCH_LED);

  /* Disable overflow LED. */
  pktUnsetGPIOlineMode(LINE_OVERFLOW_LED);

  /* Disable no FIFO LED. */
  pktUnsetGPIOlineMode(LINE_NO_FIFO_LED);

  /* If using PWM mirror disable diagnostic port. */
  pktUnsetGPIOlineMode(LINE_PWM_MIRROR);
}

/**
 * @brief   Enables PWM stream from radio.
 * @post    The ICU is configured and started.
 * @post    The ports and timers for CCA input are configured.
 *
 * @param[in]   radio   radio attached to this PWM handler
 *
 * @api
 */
void pktEnableRadioStream(const radio_unit_t radio) {

  packet_svc_t *myHandler = pktGetServiceObject(radio);

  /* Is the AFSK decoder active? */
/*  if(!((myHandler->state == PACKET_DECODE || myHandler->state == PACKET_PAUSE)
      && myHandler->rx_link_type == MOD_AFSK))
    return;*/

  if(myHandler->rx_state != PACKET_RX_ENABLED)
    return;

  AFSKDemodDriver *myDemod = (AFSKDemodDriver *)myHandler->rx_link_control;
  chDbgAssert(myDemod != NULL, "no link controller");
  chDbgAssert(myDemod->icudriver != NULL, "no ICU driver");

  switch(myDemod->icustate) {
  case PKT_PWM_INIT: {
    /* Enable CCA callback. */
    const ICUConfig *icucfg = pktLLDradioStreamEnable(radio,
                         (palcallback_t)pktRadioCCAInput);

    /* Start ICU and start capture. */
    icuStart(myDemod->icudriver, icucfg);
    icuStartCapture(myDemod->icudriver);
    myDemod->icustate = PKT_PWM_READY;
    return;

  }
  case PKT_PWM_STOP: {
    /* Enable CCA callback. */
    pktLLDradioStreamEnable(radio, (palcallback_t)pktRadioCCAInput);

    /* Start ICU capture. */
    icuStartCapture(myDemod->icudriver);
    myDemod->icustate = PKT_PWM_READY;
    return;
  }

  case PKT_PWM_READY:
    return;

  case PKT_PWM_ACTIVE: {
    chDbgAssert(false, "wrong PWM state");
    return;
  }
  } /* End switch. */
}

/**
 * @brief   Disables PWM stream from radio.
 * @post    The PWM channel is closed.
 * @post    All PWM related timers are stopped.
 * @post    The port for CCA input is disabled.
 * @post    The ICU capture is stopped.
 * @post    The ICU remains ready for capture to be restarted.
 *
 * @param[in]   radio   radio attached to this PWM handler
 *
 * @api
 */
void pktDisableRadioStream(const radio_unit_t radio) {

  packet_svc_t *myHandler = pktGetServiceObject(radio);

  /* Is the AFSK decoder active? */
/*  if(!((myHandler->state == PACKET_DECODE || myHandler->state == PACKET_PAUSE)
      && myHandler->rx_link_type == MOD_AFSK))
    return;*/

  if(myHandler->rx_state != PACKET_RX_ENABLED)
    return;
  AFSKDemodDriver *myDemod = (AFSKDemodDriver *)myHandler->rx_link_control;
  chDbgAssert(myDemod != NULL, "no link controller");
  chDbgAssert(myDemod->icudriver != NULL, "no ICU driver");

  switch(myDemod->icustate) {
  case PKT_PWM_ACTIVE: {
    /* PWM incoming active. */

    chSysLock();
    /* Disable CCA line event. */
    pktLLDradioStreamDisableI(radio);

    /* Stop any timeouts in ICU PWM handling. */
    pktStopAllICUtimersI(myDemod->icudriver);

    /*
     *  Close the PWM stream.
     *  Disable ICU notifications.
     *  Stop ICU capture.
     *  Post in-band PWM message.
     *
     */
    pktClosePWMchannelI(myDemod->icudriver, EVT_NONE, PWM_TERM_PWM_STOP);

    myDemod->icustate = PKT_PWM_STOP;

    /*
     * Reschedule to avoid a "priority order violation".
     */
    chSchRescheduleS();
    chSysUnlock();
    return;
  }

  case PKT_PWM_STOP:
    /* Stream is already stopped. */
    return;

  case PKT_PWM_INIT: {
    chDbgAssert(false, "wrong PWM state");
    return;
  }

  case PKT_PWM_READY: {
    /*
     *  PWM incoming is enabled but not active.
     *  Capture and notifications are not enabled.
     */

    chSysLock();
    /* Disable CCA line event. */
    pktLLDradioStreamDisableI(radio);

    /* Stop any timeouts in ICU PWM handling. */
    pktStopAllICUtimersI(myDemod->icudriver);

    /* Stop ICU capture. */
    icuStopCaptureI(myDemod->icudriver);
    myDemod->icustate = PKT_PWM_STOP;

    chSysUnlock();
    return;
  }

  } /* End switch. */
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
void pktClosePWMchannelI(ICUDriver *myICU, eventflags_t evt, pwm_code_t reason) {
  /* Stop posting data and write end marker. */
  AFSKDemodDriver *myDemod = myICU->link;
  packet_svc_t *myHandler = myDemod->packet_handler;
  chDbgAssert(myDemod != NULL, "no demod linked");

  chVTResetI(&myICU->pwm_timer);

  /*
   * Turn off the squelch LED.
   */
  pktWriteGPIOline(LINE_SQUELCH_LED, PAL_LOW);

  /* Stop the ICU notification (callback). */
  //icuDisableNotificationsI(myICU);

  /* Stop capture (and stop notifications). */
  icuStopCaptureI(myICU);

  /* Close can be called when there is no PWM activity. */
  if(myDemod->active_radio_stream != NULL) {
    pktAddEventFlagsI(myHandler, evt);
    if(reason == PWM_TERM_QUEUE_ERROR) {
      myDemod->active_radio_stream->status |= STA_PWM_QUEUE_ERROR;
    } else {
#if USE_HEAP_PWM_BUFFER == TRUE
      input_queue_t *myQueue =
          &myDemod->active_radio_stream->radio_pwm_queue->queue;
#if USE_CCM_BASED_PWM_HEAP == TRUE
      pktAssertCCMdynamicCheck(myQueue);
#endif
#else
      input_queue_t *myQueue = &myDemod->active_radio_stream->radio_pwm_queue;
#endif
      /* End of data flag. */
#if USE_12_BIT_PWM == TRUE
      byte_packed_pwm_t pack = {{PWM_IN_BAND_PREFIX, reason, 0}};
#else
      byte_packed_pwm_t pack = {{PWM_IN_BAND_PREFIX, reason}};
#endif
      msg_t qs = pktWritePWMQueueI(myQueue, pack);
      if(qs == MSG_TIMEOUT || qs == MSG_ERROR) {
        /*
         * No space to write in-band flag.
         * This may be due to a pending ICU interrupt?
         * In any case flag the error.
         */
        pktWriteGPIOline(LINE_OVERFLOW_LED, PAL_HIGH);

        pktAddEventFlagsI(myHandler, EVT_PWM_QUEUE_ERROR);
        myDemod->active_radio_stream->status |= STA_PWM_QUEUE_ERROR;
      } else {
        myDemod->active_radio_stream->status |= STA_PWM_STREAM_CLOSED;
      }
    } /* End if(reason != PWM_TERM_QUEUE_ERROR). */

    /* Allow the decoder thread to release the stream control object. */
    chBSemSignalI(&myDemod->active_radio_stream->sem);

#if USE_HEAP_PWM_BUFFER == TRUE
    /* Remove the PWM object reference. */
    myDemod->active_radio_stream->radio_pwm_queue = NULL;
#endif
    /* Remove object reference. */
    myDemod->active_radio_stream = NULL;
  } else {
    /* No object. Just send event broadcast. */
    pktAddEventFlagsI(myHandler, evt);
  }
  /* Return to ready state (inactive). */
  myDemod->icustate = PKT_PWM_READY;
}

/**
 * @brief   Opens the PWM stream from the ICU.
 * @post    The ICU notification (callback) is enabled.
 * @post    If an error occurs the PWM is not started and state is unchanged.
 * @post    If the FIFO is empty the "no FIFO object" LED is lit (if assigned).
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
  pktWriteGPIOline(LINE_SQUELCH_LED, PAL_HIGH);

  if(myDemod->active_radio_stream != NULL) {
    /* TODO: Work out correct handling. We should not have an open channel.
     * Shouldn't happen unless CCA has not triggered an EXTI trailing edge.
     * For now just flag that an error condition happened.
     */
    pktClosePWMchannelI(myICU, EVT_PWM_FIFO_REMNANT, PWM_TERM_QUEUE_ERR);
    return;
  }
  /* Normal CCA handling. */
  radio_pwm_fifo_t *myFIFO = chFifoTakeObjectI(myDemod->pwm_fifo_pool);
  if(myFIFO == NULL) {
    myDemod->active_radio_stream = NULL;
    /* No FIFO available.
     * Send an event to any listener.
     * Disable ICU notifications.
     */
    pktAddEventFlagsI(myHandler, EVT_PWM_FIFO_EMPTY);
    icuDisableNotificationsI(myICU);

    /* Turn on the FIFO out LED. */
    pktWriteGPIOline(LINE_NO_FIFO_LED, PAL_HIGH);
    return;
  }

  /* Save the FIFO used for this PWM -> decoder session. */
  myDemod->active_radio_stream = myFIFO;

#if USE_HEAP_PWM_BUFFER == TRUE
  /*
   * The linked PWM queue system buffers PWM in chained queue/buffer pool objects.
   * Once CCA is validated PWM buffering commences.
   * A queue/buffer object is taken from the pool.
   * The object is set as the current radio PWM side object.
   * This will be replaced as PWM arrives and the buffer becomes full.
   *
   * As PWM data arrives the memory pool object buffer is filled with PWM data.
   * When the current buffer is full a new object is obtained from the pool.
   * The embedded queue is initialised and points to the objects internal buffer.
   * The new object is chained to the prior buffer object.
   * The pointer is updated to point to the new object
   *
   * The PWM interrupt handler then continues to fill the new buffer.
   *
   * Each memory pool object contains:
   * 1. An embedded input queue object
   * 2. A buffer associated with the input queue
   * 3. A pointer to the next object (or NULL if none)
   *
   */

  radio_pwm_object_t *pwm_object = chPoolAllocI(&myDemod->pwm_buffer_pool);
  if(pwm_object == NULL) {
    /*
     * Failed to get a PWM buffer object.
     * Post an event and disable ICU.
     */
    chFifoReturnObjectI(myDemod->pwm_fifo_pool, myFIFO);
    myDemod->active_radio_stream = NULL;
    pktAddEventFlagsI(myHandler, EVT_PWM_BUFFER_FAIL);
    icuDisableNotificationsI(myICU);
    /* Turn on the PWM buffer out LED. */
    pktWriteGPIOline(LINE_NO_BUFF_LED, PAL_HIGH);
    return;
  }

#if USE_CCM_BASED_PWM_HEAP == TRUE
  /* Verify object is in CCM. */
  pktAssertCCMdynamicCheck(pwm_object);
#endif

  pktWriteGPIOline(LINE_NO_BUFF_LED, PAL_LOW);

  /* Save this object as the one currently receiving PWM. */
  myFIFO->radio_pwm_queue = pwm_object;
  myFIFO->in_use = 1;
  myFIFO->peak = 0;
  myFIFO->rlsd = 0;
  myFIFO->decode_pwm_queue = pwm_object;
  /*
   * Initialize the queue object.
   * Set the user defined link to NULL.
   * Using the embedded link allows removal of the buffer object link field.
   */
  iqObjectInit(&pwm_object->queue,
                     (*pwm_object).buffer.pwm_bytes,
                     sizeof(radio_pwm_buffer_t),
                     NULL, NULL);

#else /* USE_HEAP_PWM_BUFFER != TRUE */
  /* Non linked FIFOs have an embedded input queue with data buffer. */
  iqObjectInit(&myFIFO->radio_pwm_queue,
                     myFIFO->packed_buffer.pwm_bytes,
                     sizeof(radio_pwm_buffer_t),
                     NULL, NULL);
#endif /* USE_HEAP_PWM_BUFFER == TRUE */

  /*
   * Initialize stream object release control semaphore.
   * The decoder thread waits on the semaphore before releasing to pool.
   */
  chBSemObjectInit(&myFIFO->sem, true);

  /*
   * Set the status of this FIFO.
   * Send the FIFO entry to the decoder thread.
   */
  chFifoSendObjectI(myDemod->pwm_fifo_pool, myFIFO);

  /*
   * Start the PWM activity timer.
   * This catches the condition where CCA raises but no RX data appears.
   */
  chVTSetI(&myICU->pwm_timer, TIME_MS2I(50),
           (vtfunc_t)pktPWMInactivityTimeout, myICU);

  icuStartCaptureI(myICU);
  icuEnableNotificationsI(myICU);
  pktAddEventFlagsI(myHandler, evt);

  /* Clear status bits. */
  myFIFO->status = 0;

  myDemod->icustate = PKT_PWM_ACTIVE;
}

/**
 * @brief   Stop all ICU associated timers.
 * @notes   Will be called when the packet channel is stopped.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @iclass
 */
void pktStopAllICUtimersI(ICUDriver *myICU) {
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
  if(myDemod->active_radio_stream != NULL
      && myDemod->icustate == PKT_PWM_ACTIVE) {
    pktClosePWMchannelI(myICU, EVT_PWM_NO_DATA, PWM_TERM_NO_DATA);
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
  chDbgAssert(myDemod->icudriver != NULL, "no ICU driver");

  packet_svc_t *myHandler = myDemod->packet_handler;
  if(myDemod->icustate == PKT_PWM_STOP) {
    chSysUnlockFromISR();
    return;
  }
  uint8_t cca = pktLLDradioReadCCA(myHandler->radio);
  /* CCA de-glitch timer expired. */
  switch(cca) {
    case PAL_LOW: {
        /*
         * CAA has dropped so it is a spike which is ignored.
         */
      pktAddEventFlagsI(myHandler, EVT_RADIO_CCA_SPIKE);
      break;
      }

    /* CCA still high so open PWM channel now it is validated. */
    case PAL_HIGH: {
      pktOpenPWMChannelI(myICU, EVT_PWM_STREAM_OPEN);
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
  chDbgAssert(myDemod->icudriver != NULL, "no ICU driver");

  if(myDemod->icustate == PKT_PWM_STOP) {
    chSysUnlockFromISR();
    return;
  }

  packet_svc_t *myHandler = myDemod->packet_handler;
  uint8_t cca = pktLLDradioReadCCA(myHandler->radio);
  /* CCA de-glitch timer for trailing edge expired. */
  switch(cca) {
    case PAL_LOW: {
      /*
       * The decoder operates asynchronously to and usually slower than PWM.
       * Hence the decoder is responsible for releasing the PWM FIFO object.
       * Prior to releasing the FIFO the decoder waits on the FIFO semaphore.
       * Closing PWM from here sets the FIFO management semaphore.
       * This caters for the case where the decoder terminates stream processing first.
       * This may happen if noise produces a long string of data.
       */
      pktClosePWMchannelI(myICU, EVT_NONE, PWM_TERM_CCA_CLOSE);
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
 * @notes   Called from ISR level.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @isr
 */
void pktRadioCCAInput(ICUDriver *myICU) {
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;
  chDbgAssert(myDemod->icudriver != NULL, "no ICU driver");

  if(myDemod->icustate == PKT_PWM_STOP) {
    chSysUnlockFromISR();
    return;
  }
  packet_svc_t *myHandler = myDemod->packet_handler;
  uint8_t cca = pktLLDradioReadCCA(myHandler->radio);
  /* CCA changed. */
  switch(cca) {
    case PAL_LOW: {
      if(myDemod->icustate == PKT_PWM_ACTIVE) {
        /* CCA trailing edge glitch handling.
         * Start timer and check if CCA remains low before closing PWM.
         *
         * De-glitch for 8 AFSK bit times.
         */
        chVTSetI(&myICU->cca_timer, TIME_US2I(833 * 8),
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
               TIME_US2I(833 * 16),
               (vtfunc_t)pktRadioCCALeadTimer, myICU);
      break;
    }
  } /* End switch. */
  chSysUnlockFromISR();
  return;
}

#if LINE_PWM_MIRROR != PAL_NOLINE
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
  pktWriteGPIOline(LINE_PWM_MIRROR, PAL_LOW);
}
#endif

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
  pktWriteGPIOline(LINE_PWM_MIRROR, PAL_HIGH);

  AFSKDemodDriver *myDemod = myICU->link;
  chDbgAssert(myDemod->icudriver != NULL, "no ICU driver");

  if(myDemod->icustate != PKT_PWM_ACTIVE)
    return;

  chSysLockFromISR();
  /*
   * On period clear the ICU activity watchdog timer.
   * i.e. Once radio data appears a "no data" timeout is invalidated.
   */
  chVTResetI(&myICU->pwm_timer);

  if(myDemod->active_radio_stream == NULL) {
    /*
     * Arrive here when we are running but not buffering.
     * The ICU has been stopped and PWM aborted.
     */
    chSysUnlockFromISR();
    return;
  }
  /*
   * Check if decoding has already finished while ICU is still active.
   * The decoder terminates a frame on the first trailing HDLC flag.
   * If CPU is fast (FPU enabled) it might finish decode before PWM stops.
   * A long sequence of trailing HDLC flags or junk after a frame close
   *  flag may cause trailing PWM activity.
   *
   */
  if((myDemod->active_radio_stream->status & STA_AFSK_DECODE_DONE) != 0) {
    pktClosePWMchannelI(myICU, EVT_NONE, PWM_ACK_DECODE_END);
    chSysUnlockFromISR();
    return;
  }

  /*
   * Check if the the decoder encountered an error condition.
   * This will happen when no AX25 buffer is available or overflows.
   * Close the PWM stream and wait for next radio CCA.
   */
  if((myDemod->active_radio_stream->status & STA_AFSK_DECODE_RESET) != 0) {
    pktClosePWMchannelI(myICU, EVT_NONE, PWM_ACK_DECODE_ERROR);
    chSysUnlockFromISR();
    return;
  }

  /*
   * Check if impulse ICU value is zero and thus invalid.
   */
  if(icuGetWidthX(myICU) == 0) {
    pktClosePWMchannelI(myICU, EVT_NONE, PWM_TERM_ICU_ZERO);
    chSysUnlockFromISR();
    return;
  }

  /* Write ICU data to PWM queue. */
  msg_t qs = pktQueuePWMDataI(myICU);

  /* Switch on PWM write result. */
  switch(qs) {
  case MSG_OK: {
    /* PWM write OK. */
    chSysUnlockFromISR();
    return;
  }

  case MSG_RESET: {
#if USE_HEAP_PWM_BUFFER == TRUE
    /* Get another queue/buffer object. */
    radio_pwm_object_t *pwm_object = chPoolAllocI(&myDemod->pwm_buffer_pool);
    if(pwm_object != NULL) {
#if USE_CCM_BASED_PWM_HEAP == TRUE
      pktAssertCCMdynamicCheck(pwm_object);
#endif
      /*
       *  Initialise the new queue/buffer object.
       * The next link is set to NULL.
       */
      iqObjectInit(&pwm_object->queue,
                         (*pwm_object).buffer.pwm_bytes,
                         sizeof(radio_pwm_buffer_t),
                         NULL, NULL);

      /* Link the new object in read sequence after the prior object. */
      radio_pwm_object_t *myObject =
          myDemod->active_radio_stream->radio_pwm_queue;

      qSetLink(&myObject->queue, pwm_object);

      /* Update statistics. */
      myDemod->active_radio_stream->in_use++;
      uint8_t out = (myDemod->active_radio_stream->in_use
          - myDemod->active_radio_stream->rlsd);
      if(out > myDemod->active_radio_stream->peak)
        myDemod->active_radio_stream->peak = out;

      /* Write the in-band queue swap message to the current object. */

#if USE_12_BIT_PWM == TRUE
      byte_packed_pwm_t pack = {{PWM_IN_BAND_PREFIX, PWM_INFO_QUEUE_SWAP, 0}};
#else
      byte_packed_pwm_t pack = {{PWM_IN_BAND_PREFIX, PWM_INFO_QUEUE_SWAP}};
#endif

      /* Write the swap message to the old queue. */
      msg_t qs = pktWritePWMQueueI(&myObject->queue, pack);

      /* Set the new object as the active PWM queue/buffer. */
      myDemod->active_radio_stream->radio_pwm_queue = pwm_object;

      /* Write the PWM data to the new buffer. */
      qs = pktQueuePWMDataI(myICU);
      chDbgAssert(qs == MSG_OK, "PWM initial write to empty buffer failed");
      chSysUnlockFromISR();
      return;
    } /* End pwm_object != NULL. */
    /* No next PWM stream buffer object available. */
#endif /* USE_HEAP_PWM_BUFFER == TRUE */
    /*
     * No Next PWM stream buffer available.
     * Queue has space for one entry only.
     * Close channel and write in-band message indicating queue full.
     */
    pktWriteGPIOline(LINE_OVERFLOW_LED, PAL_HIGH);
    pktClosePWMchannelI(myICU, EVT_PWM_QUEUE_FULL, PWM_TERM_QUEUE_FULL);
    chSysUnlockFromISR();
    return;
  } /* End case. */

  case MSG_TIMEOUT: {
    chDbgAssert(qs != MSG_OK, "PWM queue unexpectedly full");
    pktClosePWMchannelI(myICU, EVT_PWM_QUEUE_ERROR, PWM_TERM_QUEUE_ERROR);
    chSysUnlockFromISR();
    return;
  } /* End case. */

  case MSG_ERROR: {
    chDbgAssert(qs != MSG_OK, "PWM initial write to empty buffer failed");
    chSysUnlockFromISR();
    return;
  } /* End case. */

  default:
    chDbgAssert(false, "PWM invalid return code from PWM queue write");
  } /* End switch. */
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
void pktRadioICUOverflow(ICUDriver *myICU) {
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;
/*  packet_svc_t *myHandler = myDemod->packet_handler;
  pktAddEventFlagsI(myHandler, EVT_ICU_OVERFLOW);*/
  if(myDemod->active_radio_stream != NULL) {
    /* Close the channel and stop ICU notifications. */
    pktClosePWMchannelI(myICU, EVT_NONE, PWM_TERM_ICU_OVERFLOW);
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
 *
 * @param[in] myICU      pointer to the ICU driver structure
 *
 * @return              The operation status.
 * @retval MSG_OK       The PWM data has been queued.
 * @retval MSG_TIMEOUT  The queue is already full.
 * @retval MSG_RESET    Queue has one slot left and the data is not an in-band.
 * @retval MSG_ERROR    The PWM queue chunk size is incorrect.
 *
 * @iclass
 */
msg_t pktQueuePWMDataI(ICUDriver *myICU) {

  chDbgCheckClassI();

  AFSKDemodDriver *myDemod = myICU->link;
  chDbgAssert(myDemod != NULL, "no linked demod driver");

#if USE_HEAP_PWM_BUFFER == TRUE
  input_queue_t *myQueue =
      &myDemod->active_radio_stream->radio_pwm_queue->queue;
#if USE_CCM_BASED_PWM_HEAP == TRUE
  pktAssertCCMdynamicCheck(myQueue);
#endif
#else
  input_queue_t *myQueue = &myDemod->active_radio_stream->radio_pwm_queue;
#endif
  chDbgAssert(myQueue != NULL, "no queue assigned");

  byte_packed_pwm_t pack;
  pktConvertICUtoPWM(myICU, &pack);
  return pktWritePWMQueueI(myQueue, pack);
}


/**
 * @brief   Write PWM data into input queue.
 * @note    This function deals with PWM data packed in sequential bytes.
 *
 * @param[in] queue     pointer to an input queue object.
 * @param[in] pack      PWM packed data object.
 *
 * @return              The operation status.
 * @retval MSG_OK       The PWM entry has been queued.
 * @retval MSG_RESET    One slot remains which is reserved for an in-band signal.
 * @retval MSG_TIMEOUT  The queue is full for normal PWM data writes.
 * @retval MSG_ERROR    The queue chunk size has become incorrect.
 *
 *
 * @api
 */
msg_t pktWritePWMQueueI(input_queue_t *queue,
                                     byte_packed_pwm_t pack) {

  size_t empty = iqGetEmptyI(queue);

  if(empty % sizeof(byte_packed_pwm_t) != 0) {
    chDbgAssert(false, "invalid PWM chunk size");
    pktWriteGPIOline(LINE_PWM_ERROR_LED, PAL_HIGH);
    return MSG_ERROR;
  }

  /* Check if there is only one slot left. */
  if(empty == sizeof(byte_packed_pwm_t)) {
    array_min_pwm_counts_t data;
    pktUnpackPWMData(pack, &data);
    if(data.pwm.impulse != PWM_IN_BAND_PREFIX)
      return MSG_RESET;
  }

  if(empty < sizeof(byte_packed_pwm_t))
    return MSG_TIMEOUT;

  /* Data is normal PWM or an in-band. */
  for(uint8_t b = 0; b < sizeof(pack.bytes); b++) {
    iqPutI(queue, pack.bytes[b]);
  }
  return MSG_OK;
}
/** @} */
