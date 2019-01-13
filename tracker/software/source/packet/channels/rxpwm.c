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
#include "pktradio.h"

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
 * @param[in]   radio   radio being started.
 *
 * @return  Pointer to assigned ICUDriver object.
 *
 * @api
 */
ICUDriver *pktAttachRadio(const radio_unit_t radio) {
  /*
   * Initialize the association between the radio and the PWM IO.
   * The ICU is started (notifications are not enabled).
   */
  ICUDriver *myICU = pktLLDradioAttachStream(radio);

  chDbgAssert(myICU != NULL, "no ICU driver");

  //icuObjectInit(myICU);

  /* Initialise the ICU PWM timers. */
  chVTObjectInit(&myICU->cca_timer);
  chVTObjectInit(&myICU->pwm_timer);
  chVTObjectInit(&myICU->pkt_timer);
  chVTObjectInit(&myICU->jam_timer);

  /* Setup the squelch LED. */
  pktLLDradioConfigIndicator(radio, PKT_INDICATOR_SQUELCH);
  pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_SQUELCH, PAL_LOW);


  /* Setup the overflow LED. */
  pktLLDradioConfigIndicator(radio, PKT_INDICATOR_OVERFLOW);
  pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_OVERFLOW, PAL_LOW);


  /* Setup the no FIFO LED. */
  pktLLDradioConfigIndicator(radio, PKT_INDICATOR_FIFO);
  pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_FIFO, PAL_LOW);


  /* Setup the no buffer LED. */
  pktLLDradioConfigIndicator(radio, PKT_INDICATOR_NO_BUFF);
  pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_NO_BUFF, PAL_LOW);

  /* Setup the PWM error LED. */
  pktLLDradioConfigIndicator(radio, PKT_INDICATOR_PWM_ERROR);
  pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_PWM_ERROR, PAL_LOW);

  /* If using PWM mirror to output to a diagnostic port. */
  //pktSetGPIOlineMode(LINE_PWM_MIRROR, PAL_MODE_OUTPUT_PUSHPULL);

  return myICU;
}

/**
 * @brief   Detaches the Radio from the PWM handlers.
 * @notes   Called by the decoder only.
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
  //icuStop(myDemod->icudriver);

  /*
   * Detach the radio from the PWM handlers.
   * Detach stops the ICU.
   * Then unlink the driver from the ICU.
   * Forget the ICU driver.
   */
  pktLLDradioDetachStream(radio);
  myDemod->icudriver->link = NULL;
  myDemod->icudriver = NULL;

  /* Disable the squelch LED. */
  pktLLDradioDeconfigIndicator(radio, PKT_INDICATOR_SQUELCH);

  /* Disable overflow LED. */
  pktLLDradioDeconfigIndicator(radio, PKT_INDICATOR_OVERFLOW);

  /* Disable no FIFO LED. */
  pktLLDradioDeconfigIndicator(radio, PKT_INDICATOR_FIFO);

  /* If using PWM mirror disable diagnostic port. */
  //pktUnsetGPIOlineMode(LINE_PWM_MIRROR);
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
static void pktRadioJammingReset(ICUDriver *myICU) {
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;
  chDbgAssert(myDemod->icudriver != NULL, "no ICU driver");

  if(myDemod->icustate == PKT_PWM_STOP) {
    chSysUnlockFromISR();
    return;
  }

  packet_svc_t *myHandler = myDemod->packet_handler;
  /* Send event broadcast. */
  pktAddEventFlagsI(myHandler, EVT_PWM_JAMMING_RESET);
  chSysUnlockFromISR();
  return;
}

/**
 * @brief   Terminates the PWM stream from the ICU.
 * @post    The ICU notification (callback) is stopped.
 * @post    An in-band reason code flag is written to the PWM queue.
 * @post    If the queue is full the optional LED is lit.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 * @param[in]   sta     flags to be set as to why the channel is closed.
 * @param[in]   evt     event to be broadcast as to why the channel is closed.
 * @param[in]   reason  in-band reason code for closing the queue
 *
 * @api
 */
static void pktClosePWMStreamI(ICUDriver *myICU,
                        statusflags_t sta,
                        eventflags_t evt,
                        pwm_code_t reason) {
  /* Stop posting data and write end marker. */
  AFSKDemodDriver *myDemod = myICU->link;
  packet_svc_t *myHandler = myDemod->packet_handler;
  chDbgAssert(myDemod != NULL, "no demod linked");

  radio_unit_t radio = myHandler->radio;

  /* Stop the PWM activity check timer. */
  chVTResetI(&myICU->pwm_timer);

  /*
   * Turn off the squelch LED.
   */
  pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_SQUELCH, PAL_LOW);

  /* Stop notifications. Capture is left enabled so ICU remains synced
     to PWM stream from radio. */

  icuDisableNotificationsI(myICU);
  //icuStopCaptureI(myICU);

  /* Close can be called when there is no PWM activity. */
  if (myDemod->active_radio_stream != NULL) {
    pktAddEventFlagsI(myHandler, evt);
    if (reason != PWM_TERM_QUEUE_ERROR) {
      input_queue_t *myQueue =
          &myDemod->active_radio_stream->radio_pwm_queue->queue;

      /* End of data flag. */
      msg_t qs = pktWritePWMinBandMessageI(myQueue, reason);
      if (qs == MSG_TIMEOUT || qs == MSG_ERROR) {
        /*
         * No space to write in-band flag.
         * This should not happen as any write should check status.
         * In any case flag the error.
         */
        if (qs == MSG_ERROR)
          pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_PWM_ERROR, PAL_HIGH);

        if (qs == MSG_TIMEOUT)
          pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_OVERFLOW, PAL_HIGH);

        pktAddEventFlagsI(myHandler, EVT_PWM_QUEUE_ERROR);
        sta |= STA_PWM_QUEUE_ERROR;
      } /* End if (qs == MSG_TIMEOUT || qs == MSG_ERROR). */
      sta |= STA_PWM_STREAM_CLOSED;
    } /* End if(reason != PWM_TERM_QUEUE_ERROR). */
    myDemod->active_radio_stream->status |= sta;
    /* Allow the decoder thread to proceed in RESET state. */
    chBSemSignalI(&myDemod->active_radio_stream->sem);

    /* Remove the PWM object reference. */
    myDemod->active_radio_stream->radio_pwm_queue = NULL;
    /* Remove object reference. */
    myDemod->active_radio_stream = NULL;
  }
  /* Send event broadcast. */
  pktAddEventFlagsI(myHandler, (evt | EVT_RAD_STREAM_CLOSE));

  /* Return to ready state (inactive) if not stopped. */
  myDemod->icustate = myDemod->icustate == PKT_PWM_STOP ? PKT_PWM_STOP : PKT_PWM_READY;
}

/**
 * @brief   Timer callback when no PWM data arises from a CCA open.
 * @post    The PWM stream will be closed
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
static void pktPWMInactivityTimeout(ICUDriver *myICU) {
  /* Timeout waiting for PWM data from the radio. */
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;
  if(myDemod->active_radio_stream != NULL
      && myDemod->icustate == PKT_PWM_WAITING) {
    pktClosePWMStreamI(myICU, STA_PWM_NO_RADIO_DATA,
                       EVT_PWM_NO_DATA, PWM_TERM_NO_DATA);
  }
  chSysUnlockFromISR();
}

/**
 * @brief   Opens the PWM stream from the ICU.
 * @post    The ICU notification (callback) is enabled.
 * @post    If an error occurs the PWM is not started and state is unchanged.
 * @post    If the FIFO is empty the "no FIFO object" LED is lit (if assigned).
 * @post    If no error occurs the timers associated with PWM are started.
 * @post    The acquired FIFO is sent via the queue mailbox.
 * @post    The ICU state is set to active.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 * @param[in]   event flags to be set as to why the channel is opened.
 *
 * @api
 */
static void pktOpenPWMStreamI(ICUDriver *myICU, eventflags_t evt) {
  AFSKDemodDriver *myDemod = myICU->link;
  packet_svc_t *myHandler = myDemod->packet_handler;
  radio_unit_t radio = myHandler->radio;

  /* Turn on the squelch LED. */
  pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_SQUELCH, PAL_HIGH);

  if(myDemod->active_radio_stream != NULL) {
    /* A stream is already open. */
#if PKT_USE_CCA_DEGLITCH == TRUE
    pktClosePWMStreamI(myICU, STA_PWM_QUEUE_ERROR,
                       PWM_FIFO_ORDER, PWM_TERM_QUEUE_ERR);

    return;
#else
    pktClosePWMStreamI(myICU, STA_CCA_RADIO_CONTINUE,
                       EVT_RAD_STREAM_CLOSE, PWM_TERM_STREAM_CLOSE);
#endif
  }
  /* Normal CCA handling.
   * TODO: Check buffer availability first before taking FIFO object?
   */
  radio_pwm_fifo_t *myFIFO = chFifoTakeObjectI(myDemod->pwm_fifo_pool);
  if(myFIFO == NULL) {
    myDemod->active_radio_stream = NULL;
    /* No FIFO available.
     * Send an event to any listener.
     * Disable ICU notifications.
     */
    pktAddEventFlagsI(myHandler, EVT_PWM_FIFO_EMPTY);
    icuDisableNotificationsI(myICU);

    /*
     * This could be noise/jamming which consumes all FIFO objects.
     * Wait for a timeout before allowing new CAA detection.
     * TODO: Keep data on jamming (decode qualified) and adjust RSSI threshold
     *       If it is just jamming then decodes won't succeed (over x events).
     *       In that case do adjust RSSI as we are listening to noise.
     *       If there are successful decodes don't adjust RSSI.
     *       This should be handled in pktSwitchPWMStreamI (when that gets operational.
     */
    chVTSetI(&myICU->jam_timer, PWM_JAMMING_TIMEOUT,
             (vtfunc_t)pktRadioJammingReset, myICU);

    /* Turn on the FIFO out LED. */
    pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_FIFO, PAL_HIGH);
    return;
  }

  /* Save the FIFO used for this PWM -> decoder session. */
  myDemod->active_radio_stream = myFIFO;

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
     * The jamming timer should have been set in ICU period so no need here.
     */
    chFifoReturnObjectI(myDemod->pwm_fifo_pool, myFIFO);
    myDemod->active_radio_stream = NULL;
    pktAddEventFlagsI(myHandler, EVT_PWM_BUFFER_FAIL);
    icuDisableNotificationsI(myICU);
    /* Turn on the PWM buffer out LED. */
    pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_OVERFLOW, PAL_HIGH);
    return;
  }

  pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_NO_BUFF, PAL_HIGH);

  /* Save this object as the one currently receiving PWM. */
  myFIFO->radio_pwm_queue = pwm_object;
#if TRACE_PWM_BUFFER_STATS == TRUE
  myFIFO->in_use = 1;
  myFIFO->sync = 1;
  myFIFO->peak = 0;
  myFIFO->rlsd = 0;
#endif
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

  /* Sequence stamp of PWM FIFO happens when ICU is active. */
  myFIFO->seq_num = 0;

  /* Clear PWM session status bits. */
  myFIFO->status = 0;

#if PKT_RSSI_CAPTURE == TRUE
  myFIFO->rssi = 0;
#endif
#if LINE_PWM_MIRROR != PAL_NOLINE
  pktWriteGPIOline(LINE_PWM_MIRROR, PAL_HIGH);
#endif
  myDemod->icustate = PKT_PWM_WAITING;
}

/**
 * @brief   Switches the PWM stream where ICU data is continuous.
 * @notes   The purpose is to handle the case were CCA does not drop.
 *          The PWM side will re-use the existing stream FIFO.
 *          The decoder will be signalled via a status to re-use the FIFO.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 * @param[in]   sta     additional flags to be set.
 * @param[in]   evt     additional event to be broadcast.
 *
 * @api
 */
static void pktSwitchPWMStreamI(ICUDriver *myICU,
                        statusflags_t sta,
                        eventflags_t evt) {

  AFSKDemodDriver *myDemod = myICU->link;
  packet_svc_t *myHandler = myDemod->packet_handler;
  chDbgAssert(myDemod != NULL, "no demod linked");

  myDemod->active_radio_stream->status |= (sta | STA_PWM_STREAM_SWITCH);
  /* Allow the decoder thread to proceed in RESET state. */
  chBSemSignalI(&myDemod->active_radio_stream->sem);
  /* Send event broadcast. */
  pktAddEventFlagsI(myHandler, (evt | EVT_RAD_STREAM_SWITCH));

  /*
   * Start the PWM activity timer.
   * This catches the condition where PWM does not continue.
   */
  chVTSetI(&myICU->pwm_timer, TIME_MS2I(50),
           (vtfunc_t)pktPWMInactivityTimeout, myICU);

  /* Return to waiting state. ICU width updates receive sequence number
     and triggers RSSI capture. */
  myDemod->icustate = PKT_PWM_WAITING;
}

/**
 * @brief   Enables PWM stream from radio for DSP.
 * @post    The ICU capture only is started (notifications are not changed).
 * @post    The ports and timers for CCA input are configured.
 *
 * @param[in]   radio   radio attached to this PWM handler
 *
 * @api
 */
void pktEnableRadioStream(const radio_unit_t radio) {

  packet_svc_t *myHandler = pktGetServiceObject(radio);

  /*
   *  Is the AFSK decoder active?
   *  TODO: Need to rationalise the tests.
   *  This should be AFSK or other DSP based modulation only.
   */

  radio_mod_t mod = myHandler->rx_link_type;

  switch(mod) {

  case MOD_AFSK: {
    AFSKDemodDriver *myDemod = (AFSKDemodDriver *)myHandler->rx_link_control;
    chDbgAssert(myDemod != NULL, "no link controller");
    chDbgAssert(myDemod->icudriver != NULL, "no ICU driver");

    switch(myDemod->icustate) {
#if 0
    case PKT_PWM_INIT: {

      /* Enable CCA callback. */
      const ICUConfig *icucfg = pktLLDradioCCAEnable(radio, mod,
                                      (palcallback_t)pktRadioCCAInput);

      /* Start ICU and start capture. */
      icuStart(myDemod->icudriver, icucfg);
      icuStartCapture(myDemod->icudriver);
      myDemod->icustate = PKT_PWM_READY;
      return;

    } /* End case PKT_PWM_INIT. */
#endif
    /* The PWM handling is currently stopped. */
    case PKT_PWM_STOP: {
      /* Enable CCA callback from GPIO. */
      pktLLDradioCCAEnable(radio, mod, (palcallback_t)pktRadioCCAInput);

      /* Start ICU capture. The ICU will sync to the next edge. Notifications
         are enabled by CCA interrupt. */
      icuStartCapture(myDemod->icudriver);
      myDemod->icustate = PKT_PWM_READY;
      return;
    } /* End case PKT_PWM_STOP. */

    /* The PWM handling is currently ready. */
    case PKT_PWM_READY:
      /* Arriving here means enable is being called without prior disable. */
      chDbgAssert(false, "wrong PWM state");
      return;

      /* The PWM handler is activated to handle a stream. */
    case PKT_PWM_WAITING:
    case PKT_PWM_ACTIVE: {
      /* Should not be attempting to enable PWM in this state. */
      chDbgAssert(false, "wrong PWM state");
      return;
    } /* End case PKT_PWM_WAITING or PKT_PWM_ACTIVE. */
    } /* End switch on ICU state. */
  } /* End case MOD_AFSK. */

  case MOD_NONE:
  case MOD_CW:
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2: {
    chDbgAssert(false, "wrong PWM state");
    return;
  }
  }
}

/**
 * @brief   Disables packet data stream from radio.
 * @details Actions will depend on the modulation in effect.
 * @pre     Radio must be locked prior to calling.
 * @notes   For AFSK...
 * @post    Any active PWM stream is closed (stops notifications).
 * @post    All PWM related timers are stopped.
 * @post    The port for CCA input is disabled.
 * @post    The ICU capture is stopped.
 * @post    The ICU remains ready for capture to be restarted.
 * @post    The radio is put into standby mode.
 * @notes   For other modulations...
 * @post    TBD
 *
 * @param[in]   radio   radio unit ID
 *
 * @notapi
 */
void pktDisableRadioStream(const radio_unit_t radio) {

  packet_svc_t *myHandler = pktGetServiceObject(radio);

  /* Is the receiver active? */
  if(!pktIsReceiveEnabled(radio)) {
    return;
  }
  radio_mod_t mod = myHandler->rx_link_type;

  switch (mod) {

  case MOD_AFSK: {

    AFSKDemodDriver *myDemod = (AFSKDemodDriver *)myHandler->rx_link_control;
    chDbgAssert(myDemod != NULL, "no link controller");
    chDbgAssert(myDemod->icudriver != NULL, "no ICU driver");

    chSysLock();

    switch(myDemod->icustate) {
    case PKT_PWM_WAITING:
    case PKT_PWM_ACTIVE: {
      /* PWM incoming active. */


      /* Disable CCA line event. */
      pktLLDradioCCADisableI(radio, mod);

      /* Stop any timeouts in ICU PWM handling. */
      pktStopAllICUtimersI(myDemod->icudriver);

      /*
       *  Close the PWM stream.
       *  Disable ICU notifications.
       *  Post in-band PWM message.
       */
      pktClosePWMStreamI(myDemod->icudriver, STA_PWM_STREAM_DISABLE,
                                       EVT_NONE, PWM_TERM_PWM_STOP);

      /* Stop ICU capture. When ICU capture is started it will
         sync to the next PWM edge. */
      icuStopCaptureI(myDemod->icudriver);

      myDemod->icustate = PKT_PWM_STOP;

      /* Reschedule to avoid a "priority order violation". */
      chSchRescheduleS();
      chSysUnlock();

      /* Putting radio in standby stops all output from radio. */
      pktLLDradioStandby(radio);
      return;
    } /* End case PKT_PWM_WAITING or PKT_PWM_ACTIVE */

    case PKT_PWM_STOP:
      /* Stream is stopped. This can be done from PWMClose. */
      //chDbgAssert(false, "wrong PWM state");
      chSysUnlock();
      return;
#if 0
    case PKT_PWM_INIT: {

      /* TX threads can call this when PWM is not enabled... */
      //chDbgAssert(false, "wrong PWM state");
      chSysUnlock();
      return;

    }
#endif
    case PKT_PWM_READY: {

      /*
       *  PWM incoming is enabled.
       *  There is no active stream.
       */

      /* Disable CCA line event. */
      pktLLDradioCCADisableI(radio, mod);

      /* Stop any timeouts in ICU PWM handling. */
      pktStopAllICUtimersI(myDemod->icudriver);

      /* Stop ICU capture. Notifications can't happen now. */
      icuStopCaptureI(myDemod->icudriver);
      myDemod->icustate = PKT_PWM_STOP;

      chSysUnlock();

      /* Put radio in standby. */
      pktLLDradioStandby(radio);
      return;
    } /* End case PKT_PWM_READY. */

    } /* End switch on ICU state. */
  } /* End case AFSK. */

  case MOD_NONE:
  case MOD_CW:
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2: {
    chSysUnlock();
    chDbgAssert(false, "wrong PWM state");
    return;
  }
  default:
    chSysUnlock();
    chDbgAssert(false, "wrong PWM state");
    return;
  } /* End switch on link type. */
}

/**
 * @brief   Stop all ICU associated timers.
 * @notes   Will be called when a packet stream is closed.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @iclass
 */
void pktStopAllICUtimersI(ICUDriver *myICU) {
  chVTResetI(&myICU->cca_timer);
  chVTResetI(&myICU->pwm_timer);
  chVTResetI(&myICU->pkt_timer);
  chVTResetI(&myICU->jam_timer);
}

/**
 * @brief   Timer callback when PWM data stops after stream is open.
 * @post    The PWM stream will be closed.
 *
 * @param[in]   myICU   pointer to a @p ICUDriver structure
 *
 * @api
 */
void pktPWMActivityTimeout(ICUDriver *myICU) {
  /* Timeout when PWM stops from radio. */
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;
  if(myDemod->active_radio_stream != NULL
      && myDemod->icustate == PKT_PWM_ACTIVE) {
    pktClosePWMStreamI(myICU, STA_PWM_RADIO_STOP,
                       EVT_PWM_RADIO_TIMEOUT, PWM_TERM_PWM_TIMEOUT);
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
  if(myDemod->icustate == PKT_PWM_STOP || chVTIsArmedI(&myICU->jam_timer)) {
    chSysUnlockFromISR();
    return;
  }
  uint8_t cca = pktLLDradioReadCCAlineI(myHandler->radio);
  /* CCA de-glitch timer expired. */
  switch(cca) {
    /* CAA has dropped so it is a spike which is ignored. */
    case PAL_LOW: {
      pktAddEventFlagsI(myHandler, EVT_RADIO_CCA_SPIKE);
      break;
      }

    /* CCA still high. Start stream if not already open. */
    case PAL_HIGH: {
      if(myDemod->active_radio_stream == NULL)
        pktOpenPWMStreamI(myICU, EVT_RAD_STREAM_OPEN);
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

  if(myDemod->icustate == PKT_PWM_STOP || chVTIsArmedI(&myICU->jam_timer)) {
    chSysUnlockFromISR();
    return;
  }

  packet_svc_t *myHandler = myDemod->packet_handler;
  uint8_t cca = pktLLDradioReadCCAlineI(myHandler->radio);
  /* CCA de-glitch timer for trailing edge expired. */
  switch(cca) {
    case PAL_LOW: {
      /*
       * CCA has dropped. The trailing edge is de-glitched by a VT.
       * Arrive here when the VT expires.
       * If CCA is still low close the PWM stream.
       */
      pktClosePWMStreamI(myICU, STA_CCA_RADIO_DROP,
                         EVT_RADIO_CCA_DROP, PWM_TERM_STREAM_CLOSE);
      /* TODO: Either send a RX re-set event to the radio from here
       *  Or preferably...
       *  Have radio monitor CCA and re-set itself.
       *  This is an option to enable si446x to use gear switching in RX
       */
      break;
      }

    case PAL_HIGH: {
      /* CCA is active again so leave PWM stream open. */
      //pktAddEventFlagsI(myHandler, EVT_RADIO_CCA_GLITCH);
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

  if(myDemod->icustate == PKT_PWM_STOP || chVTIsArmedI(&myICU->jam_timer)) {
    chSysUnlockFromISR();
    return;
  }
  packet_svc_t *myHandler = myDemod->packet_handler;

  uint8_t cca = pktLLDradioReadCCAlineI(myHandler->radio);

  /* CCA changed. */
  switch (cca) {

#if PKT_USE_CCA_LEADING_ONLY == TRUE
  case PAL_HIGH: {
    if(myDemod->active_radio_stream == NULL)
      pktOpenPWMStreamI(myICU, EVT_RAD_STREAM_OPEN);
    break;
  }
#else
    case PAL_LOW: {
      if (myDemod->icustate == PKT_PWM_ACTIVE) {
        /* CCA trailing edge handling.
         * The ICU is active
         * Start timer and check if CCA remains low before closing PWM stream.
         *
         * De-glitch for 1 AFSK bit times.
         */
#if PKT_USE_CCA_DEGLITCH == TRUE
        chVTSetI(&myICU->cca_timer, TIME_US2I(833 * 1),
                 (vtfunc_t)pktRadioCCATrailTimer, myICU);
#else
        pktClosePWMStreamI(myICU, STA_CCA_RADIO_DROP,
                           EVT_RADIO_CCA_DROP, PWM_TERM_STREAM_CLOSE);
#endif
        break;
      }
      if (myDemod->icustate == PKT_PWM_WAITING) {
        /* CCA trailing edge glitch handling.
         * ICU has not processed any PWM yet.
         */
#if PKT_USE_CCA_DEGLITCH == TRUE
        /*
         * Start timer and check if CCA remains low before closing PWM.
         * De-glitch for 8 AFSK bit times.
         */
        chVTSetI(&myICU->cca_timer, chTimeUS2I(833 * 8),
                 (vtfunc_t)pktRadioCCATrailTimer, myICU);
#else
        /*
         * CCA has dropped. Close the PWM stream.
         */
        pktClosePWMStreamI(myICU, STA_CCA_RADIO_SPIKE,
                           EVT_RADIO_CCA_SPIKE, PWM_TERM_STREAM_CLOSE);
#endif
      }
      /* other state. */
      break;
    } /* End case PAL_LOW. */

    case PAL_HIGH: {
#if PKT_USE_CCA_DEGLITCH == TRUE

      if(chVTIsArmedI(&myICU->cca_timer)) {
        /* CAA has been re-asserted during trailing edge timer. */
        chVTResetI(&myICU->cca_timer);
        break;
      }
      /* Else this is a leading edge of CCA for a new packet. */
      /* Delay for 1 AFSK bit time to allow AGC to settle. */
      chVTSetI(&myICU->cca_timer, chTimeUS2I(833 * 1),
               (vtfunc_t)pktRadioCCALeadTimer, myICU);
      break;
#else
      pktOpenPWMStreamI(myICU, EVT_RAD_STREAM_OPEN);
      break;
#endif
    }
#endif
  } /* End switch. */
  chSysUnlockFromISR();
  return;
}

/**
 * Callback after radio manager gets RSSI from radio
 */
static void pktRadioRSSIreadCB(radio_task_object_t *rt) {
  AFSKDemodDriver *myDemod = rt->handler->rx_link_control;

  radio_pwm_fifo_t *myFIFO = myDemod->active_radio_stream;
  if(myFIFO == NULL)
    return;
  /*
   * Is the callback still good for current RX sequence?
   * Can be out of sync if the radio manager is delayed or PWM is jittery.
   */
  if(myFIFO->seq_num == rt->radio_dat.seq_num)
    /* Set the RSSI or flag unable to read. */
    myFIFO->rssi = (rt->result == MSG_OK) ? rt->rssi : 0xFF;
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
static msg_t pktICUQueueAsPWMDataI(ICUDriver *myICU) {

  chDbgCheckClassI();

  AFSKDemodDriver *myDemod = myICU->link;
  chDbgAssert(myDemod != NULL, "no linked demod driver");

  input_queue_t *myQueue =
      &myDemod->active_radio_stream->radio_pwm_queue->queue;

  chDbgAssert(myQueue != NULL, "no queue assigned");

  byte_packed_pwm_t pack;
  pktConvertICUtoPWM(myICU, &pack);
  return pktWritePWMQueueI(myQueue, pack);
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
  AFSKDemodDriver *myDemod = myICU->link;
  chDbgAssert(myDemod->icudriver != NULL, "no ICU driver");

  packet_svc_t *myHandler = myDemod->packet_handler;
  chDbgAssert(myHandler != NULL, "no packet handler");

  chSysLockFromISR();
#if LINE_PWM_MIRROR != PAL_NOLINE
  pktWriteGPIOline(LINE_PWM_MIRROR, PAL_LOW);
#endif
  if(myDemod->active_radio_stream == NULL) {
    /*
     * Arrive here if ICU is running but not buffering.
     * The PWM stream is not active.
     */
    chSysUnlockFromISR();
    return;
  }

  switch (myDemod->icustate) {
  case PKT_PWM_WAITING: {
    /* TODO: The stream could be opened here rather than in CCA handlers.
     * Probably an infrequent case where PWM does not happen but probably
     * more sensible to open the PWM stream here in any case.
     */
    /* Increment receive session count. */
    myHandler->radio_rx_config.seq_num++;

    /* Cancel the PWM inactivity timer. */
    chVTResetI(&myICU->pwm_timer);

#if PKT_RSSI_CAPTURE == TRUE
    /* Queue a radio task to read RSSI in radio.
       Radio registers can't be read at ISR level.  */

    radio_params_t rp = myHandler->radio_rx_config;

    /* Sequence stamp the RT and PWM FIFO objects. Clear RSSI value. */
    rp.seq_num = myHandler->radio_rx_config.seq_num;
    myDemod->active_radio_stream->seq_num = rp.seq_num;
    rp.rssi = 0;

    /* Send RSSI read as priority radio task (front of the queue). */
    msg_t msg = pktQueuePriorityRadioCommandI(myHandler->radio,
                                              PKT_RADIO_RX_RSSI,
                                              &rp,
                                              pktRadioRSSIreadCB);

    if(msg == MSG_TIMEOUT) {
      /* Indicate RSSI was not captured. */
      myDemod->active_radio_stream->rssi = 0xFF;
    }
#endif /* PKT_RSSI_CAPTURE == TRUE */
    pktAddEventFlagsI(myHandler, EVT_PWM_ACTIVE);
    myDemod->icustate = PKT_PWM_ACTIVE;
    break;
  } /* End case PKT_PWM_WAITING */

  case PKT_PWM_ACTIVE: {
    /*
     * Check if decoding has been reset while ICU is still active.
     * The decoder always resets after processing a stream (success or fail).
     *
     * Situations where PWM stream may still be incoming and a
     *   switch should be made...
     * 1. If CPU is fast (FPU enabled) it might finish decode before PWM stops.
     *    This can also happen if the packet transmission has a long tail.
     *
     * 2. Overlapping or continuous packet traffic (CCA stays asserted).
     *
     * 3. RSSI threshold is set too low and CCA remains asserted.
     */
    if((myDemod->active_radio_stream->status & STA_AFSK_DECODE_RESET) != 0)
      pktSwitchPWMStreamI(myICU, STA_NONE, EVT_NONE);
    break;
  } /* End case PKT_PWM_ACTIVE */

  default:
    break;
  } /* End switch. */

  /* Check if impulse ICU value is zero and thus invalid. */
  if(icuGetWidthX(myICU) == 0) {
    /* Post an in-band message to the stream if one is allocated.
      The EVT_PWM_ICU_ZERO event would be broadcast by the decoder. */
    pktClosePWMStreamI(myICU, STA_PWM_ICU_ZERO, EVT_NONE, PWM_TERM_ICU_ZERO);
  }
  chSysUnlockFromISR();
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

  AFSKDemodDriver *myDemod = myICU->link;
  chDbgAssert(myDemod->icudriver != NULL, "no ICU driver");
  packet_svc_t *myHandler = myDemod->packet_handler;
  chDbgAssert(myHandler != NULL, "no packet handler");
  chSysLockFromISR();

#if LINE_PWM_MIRROR != PAL_NOLINE
  pktWriteGPIOline(LINE_PWM_MIRROR, PAL_HIGH);
#endif
  if(myDemod->active_radio_stream == NULL
      || myDemod->icustate != PKT_PWM_ACTIVE) {
    /*
     * Arrive here if ICU is running but not buffering.
     * The PWM stream is not active.
     */
    chSysUnlockFromISR();
    return;
  }
#if 0
  /*
   * Check if decoding has already finished while ICU is still active.
   * The decoder terminates a frame on the first valid trailing HDLC flag.
   * If CPU is fast (FPU enabled) it might finish decode before PWM stops.
   * A long sequence of trailing HDLC flags or junk after a frame close
   *  flag may cause trailing PWM activity.
   *
   */
  if((myDemod->active_radio_stream->status & STA_AFSK_DECODE_DONE) != 0) {
    pktClosePWMStreamI(myICU, STA_PWM_DECODE_DONE,
                       EVT_NONE, PWM_ACK_DECODE_END);
    chSysUnlockFromISR();
    return;
  }

  /*
   * Check if the the decoder encountered an error condition.
   * This will happen when no AX25 buffer is available or overflows.
   * Close the PWM stream and wait for next radio CCA.
   */
  if((myDemod->active_radio_stream->status & STA_AFSK_DECODE_RESET) != 0) {
    pktClosePWMStreamI(myICU, STA_PWM_DECODE_RESET,
                       EVT_NONE, PWM_ACK_DECODE_ERROR);
    chSysUnlockFromISR();
    return;
  }
#endif
  /* Write ICU data to PWM queue. */
  msg_t qs = pktICUQueueAsPWMDataI(myICU);

  /* Switch on PWM write result. */
  switch(qs) {
  case MSG_OK: {
#if 0
    /* PWM write OK. If ICU has produced first period count cancel timer
       and change state. */
    if(myDemod->icustate == PKT_PWM_WAITING) {
#if PKT_USE_CCA_LEADING_ONLY != TRUE
      /* Cancel the PWM inactivity timer. */
      chVTResetI(&myICU->pwm_timer);
#endif
      pktAddEventFlagsI(myHandler, EVT_PWM_ACTIVE);
      myDemod->icustate = PKT_PWM_ACTIVE;
    }
#endif
#if PKT_USE_CCA_LEADING_ONLY == TRUE
    /*
     * The PWM timer now looks for activity.
     * If it times out the stream is closed.
     * The timer is stopped in pktClosePWMStreamI(...).
     */

    chVTSetI(&myICU->pwm_timer, TIME_US2I(833 * PKT_TRAILING_SYMBOL_TIMEOUT),
             (vtfunc_t)pktPWMActivityTimeout, myICU);
#endif
    chSysUnlockFromISR();
    return;
  }

  case MSG_RESET: {
    /* Space remaining in current queue for in-band message only. */
    /* Get another queue/buffer object. */
    radio_pwm_object_t *pwm_object = chPoolAllocI(&myDemod->pwm_buffer_pool);
    if(pwm_object != NULL) {
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
#if TRACE_PWM_BUFFER_STATS == TRUE
      /* Update statistics. */
      myDemod->active_radio_stream->in_use++;
      uint8_t out = (myDemod->active_radio_stream->in_use
          - myDemod->active_radio_stream->rlsd);
      if(out > myDemod->active_radio_stream->peak)
        myDemod->active_radio_stream->peak = out;
#endif
      /* Write the in-band queue swap message to the current object. */
      msg_t qs = pktWritePWMinBandMessageI(&myObject->queue, PWM_INFO_QUEUE_SWAP);

      chDbgAssert(qs == MSG_OK, "PWM write of in-band swap message failed");
      /* Set the new object as the active PWM queue/buffer. */
      myDemod->active_radio_stream->radio_pwm_queue = pwm_object;

      /* Write the PWM data to the new buffer. */
      qs = pktICUQueueAsPWMDataI(myICU);
      chDbgAssert(qs == MSG_OK, "PWM initial write to empty buffer failed");
#if PKT_USE_CCA_LEADING_ONLY == TRUE
    /*
     * The PWM timer is checking activity.
     * If it times out the stream is closed.
     * The timer is stopped in pktClosePWMStreamI(...).
     */
    chVTSetI(&myICU->pwm_timer, TIME_US2I(833 * PKT_TRAILING_SYMBOL_TIMEOUT),
             (vtfunc_t)pktPWMActivityTimeout, myICU);
#endif
      chSysUnlockFromISR();
      return;
    } /* End pwm_object != NULL. */
    /* No next PWM stream buffer object available. */
    /*
     * No Next PWM stream buffer available.
     * Queue has space for one entry only.
     * Close channel and write in-band message indicating queue full.
     */
    radio_unit_t radio = myDemod->packet_handler->radio;

    pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_OVERFLOW, PAL_HIGH);
    pktClosePWMStreamI(myICU, STA_PWM_BUFFER_FULL,
                       EVT_PWM_QUEUE_FULL, PWM_TERM_QUEUE_FULL);
    /*
     * This looks like noise/jamming.
     * Wait for a timeout before allowing new CAA detection.
     * TODO: Keep data on jamming and adjust RSSI threshold?
     */
    chVTSetI(&myICU->jam_timer, PWM_JAMMING_TIMEOUT,
             (vtfunc_t)pktRadioJammingReset, myICU);
    chSysUnlockFromISR();
    return;
  } /* End case. */

  case MSG_TIMEOUT: {
    /*
     *  The PWM queue reported full.
     *  But prior write should have reported "one slot left".
     */
    chDbgAssert(qs != MSG_OK, "PWM queue unexpectedly full");
    pktClosePWMStreamI(myICU, STA_PWM_QUEUE_ERROR,
                       EVT_PWM_QUEUE_ERROR, PWM_TERM_QUEUE_ERROR);
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
#if PKT_CATCH_ICU_OVERFLOW != TRUE
  return;
#endif
  chSysLockFromISR();
  AFSKDemodDriver *myDemod = myICU->link;
  /* Is the ICU active? */
  if(myDemod->icustate != PKT_PWM_ACTIVE) {
    chSysUnlockFromISR();
    return;
  }

  /* Is there a stream attached? */
  if(myDemod->active_radio_stream != NULL) {
    /* Close the channel and stop ICU notifications. */
    pktClosePWMStreamI(myICU, STA_PWM_ICU_OVERFLOW,
                       EVT_PWM_ICU_OVERFLOW, PWM_TERM_ICU_OVERFLOW);
  } else {
    /* Just stop the ICU notification. */
    icuDisableNotificationsI(myICU);
  }
  chSysUnlockFromISR();
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
 * @retval MSG_TIMEOUT  The queue is full.
 * @retval MSG_ERROR    The queue chunk size has become incorrect.
 *
 *
 * @iclass
 */
msg_t pktWritePWMQueueI(input_queue_t *queue, byte_packed_pwm_t pack) {

  size_t empty = iqGetEmptyI(queue);

  if(empty % sizeof(byte_packed_pwm_t) != 0) {
    chDbgAssert(false, "invalid PWM chunk size");
    return MSG_ERROR;
  }

  /* Check if the queue is full. */
  if(empty < sizeof(byte_packed_pwm_t))
    return MSG_TIMEOUT;

  /* If there is only one slot left reserve it for an in-band message. */
  if(empty == sizeof(byte_packed_pwm_t)) {
    array_min_pwm_counts_t data;
    pktUnpackPWMData(pack, &data);
    if(data.pwm.impulse != PWM_IN_BAND_PREFIX)
      return MSG_RESET;
  }

  /* Data is normal PWM or an in-band. */
  for(uint8_t b = 0; b < sizeof(pack.bytes); b++) {
    iqPutI(queue, pack.bytes[b]);
  }
  return MSG_OK;
}
/** @} */
