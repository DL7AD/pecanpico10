/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file        pktradio.c
 * @brief       Radio manager.
 *
 * @addtogroup  managers
 * @{
 */

#include "pktconf.h"
#include "radio.h"

#include "si446x.h"
#include "debug.h"
#include "geofence.h"
#include "ov5640.h"
#include "tcxo.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

const radio_band_t band_2m = {
  .start    = BAND_MIN_2M_FREQ,
  .end      = BAND_MAX_2M_FREQ,
  .step     = BAND_STEP_2M_HZ,
};

const radio_band_t band_70cm = {
  .start    = BAND_MIN_70CM_FREQ,
  .end      = BAND_MAX_70CM_FREQ,
  .step     = BAND_STEP_70CM_HZ,
};

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/* Radio HAL functions. */

/**
 *
 */
static bool pktLLDradioInit(const radio_unit_t radio) {
  /* TODO: Implement hardware mapping. */
  /*
   * NOTE: RADIO_CS and RADIO_SDN pins are configured in board.h
   * RADIO_SDN is configured to open drain pullup.
   * It is also pulled up on PCB by 100K.
   * The radio powers up in SDN mode.
   *
   * CS is set as push-pull and initialized to HIGH.
   */
  return Si446x_conditional_init(radio);
}

/**
 *
 */
static void pktLLDradioShutdown(const radio_unit_t radio) {
  /* TODO: Implement hardware mapping. */

  /*
   * Put radio in shutdown mode.
   * All registers are lost.
   */
  Si446x_radioShutdown(radio);
}

/**
 *
 */
static void pktLLDradioStandby(const radio_unit_t radio) {
  /* TODO: Implement hardware mapping. */

  /*
   * Put radio in standby (low power) mode.
   * All registers are retained.
   */
  Si446x_radioStandby(radio);
}

/**
 * @brief   Send packet(s) on radio.
 * @notes   This is the API interface to the radio LLD.
 * @notes   Currently just map directly to 446x driver.
 * @notes   In future would implement a lookup and VMT to access radio methods.
 *
 * @param[in] rto radio task object pointer.
 *
 * @notapi
 */
static bool pktLLDradioSendPacket(radio_task_object_t *const rto) {
  bool status;
  /* TODO: Implement VMT to functions per radio type. */
#if PKT_RTO_HAS_INNER_CB == TRUE
  switch(rto->radio_dat.type) {
#else
  switch(rto->type) {
#endif
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2:
    status = Si446x_blocSend2FSK(rto);
    break;

  case MOD_AFSK:
    status = Si446x_blocSendAFSK(rto);
    break;

  case MOD_CW:
    status = Si446x_blocSendCW(rto);
    break;

  case MOD_NONE:
    status = false;
  } /* End switch on task_object->type. */
  return status;
}

/**
 * @brief   Start receive.
 * @notes   This is the HAL API to the radio LLD.
 * @notes   Currently just map directly to 446x driver.
 * @notes   In future a VMT lookup would access the relevant radio methods.
 *
 * @param[in] radio radio unit ID.
 * @param[in] rto   pointer to radio task object
 *
 * @return  status of the operation
 * @retval  true    operation succeeded.
 * retval   false   operation failed.
 *
 * @notapi
 */
static bool pktLLDradioStartReceive(const radio_unit_t radio,
                         radio_task_object_t *const rto) {
  packet_svc_t *handler = pktGetServiceObject(radio);

  if(handler == NULL)
    return false;
#if PKT_RTO_HAS_INNER_CB == TRUE
  if(!Si4464_enableReceive(radio,
                            rto->radio_dat.base_frequency,
                            rto->radio_dat.step_hz,
                            rto->radio_dat.channel,
                            rto->radio_dat.rssi,
                            rto->radio_dat.type)) {
    return false;
  }
#else
  if(!Si4464_enableReceive(radio,
                            rto->base_frequency,
                            rto->step_hz,
                            rto->channel,
                            rto->squelch,
                            rto->type)) {
    return false;
  }
#endif
  return pktLLDradioAttachStream(radio);
}

/**
 * Disable receive when closing packet receive for the channel.
 */
static void pktLLDradioStopReceive(const radio_unit_t radio) {
  /* TODO: Implement hardware mapping. */
  pktLLDradioDetachStream(radio);
  Si446x_disableReceive(radio);
}

#if 0
/**
 * @brief   Resume reception paused by transmit task.
 * @notes   This is the API interface to the radio LLD.
 * @notes   Currently just map directly to 446x driver.
 * @notes   In future would implement a lookup and VMT to access radio methods.
 * @post    The radio is active.
 * @post    External feed of PWM for AFSK has to be resumed by caller.
 *
 * @param[in] radio radio unit ID.
 *
 * @return  status of the operation
 * @retval  true    operation succeeded.
 * @retval  false   operation failed.
 *
 * @notapi
 */
static bool pktLLDradioResumeReceive(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);

  radio_freq_hz_t freq = handler->radio_rx_config.base_frequency;
  radio_chan_hz_t step = handler->radio_rx_config.step_hz;
  radio_ch_t chan = handler->radio_rx_config.channel;
#if PKT_RTO_USE_SETTING == TRUE
  radio_squelch_t rssi = handler->radio_rx_config.rssi;
#else
    radio_squelch_t rssi = handler->radio_rx_config.squelch;
#endif
  radio_mod_t mod = handler->radio_rx_config.type;
  bool result = Si4464_enableReceive(radio, freq, step, chan, rssi, mod);
  return result;
}
#endif
/**
 * @brief   Captures the current signal strength from the radio.
 * @notes   This is the API interface to the radio LLD.
 * @notes   Currently just map directly to 446x driver.
 * @notes   In future would implement a lookup and VMT to access radio methods.
 * @notes   The function should be called while the RX front end handler is active.
 * @notes   Calling from a decoder level will not capture the instantaneous level.
 *
 * @param[in] radio radio unit ID.
 *
 * @notapi
 */
static radio_signal_t pktLLDradioCaptureRSSI(const radio_unit_t radio) {
  return Si446x_getCurrentRSSI(radio);
}

/**
 * @brief Send the latest main clock value to the radio.
 * @notes The radio may update its local Xtal/Osc frequency.
 * @notes The decision to update or not would based on the drift magnitude.
 *
 * @return  status from the radio driver
 * @retval  true if the change was applied
 * @retval  false if the change was not applied
 */
bool pktLLDradioOscUpdate(const radio_unit_t radio, const xtal_osc_t freq) {
  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only one at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */

  return Si446x_updateClock(radio, freq);
}


/**
 * @brief   Open radio receive.
 * @pre     The packet service and receive chain is opened and initialised.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   rto     pointer to radio task object
 * @param[in] timeout   the number of ticks before the operation times outs
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return  Status of operation
 * @retval  MSG_OK      if receive started
 * @retval  MSG_TIMEOUT if timeout waiting to lock radio
 * @retval  MSG_RESET   if the radio can not be used due to a system abort.
 * @retval  MSG_ERROR   if there was an error starting the radio receive
 *
 * @api
 */
static msg_t pktOpenRadioReceive(const radio_unit_t radio,
                          radio_task_object_t *const rto,
                          const sysinterval_t timeout) {

  packet_svc_t *handler = rto->handler;
#if 0
#if USE_POOL_RX_BUFFER_OBJECTS == TRUE
  /* Create the packet management services. */
  if(pktIncomingBufferPoolCreate(radio) == NULL) {
    pktAddEventFlags(rto->handler, (EVT_PKT_BUFFER_MGR_FAIL));
    return MSG_ERROR;
  }
#endif
#endif

  /*
   * Initialize the outstanding callback count.
   */
  handler->rxcb_ref_count = 0;
  /* Switch on modulation type. */
#if PKT_RTO_HAS_INNER_CB == TRUE
  switch(rto->radio_dat.type) {
#else
  switch(rto->type) {
#endif
  case MOD_AFSK: {
    /*
     *  Lock the radio.
     */
    msg_t msg = pktLockRadio(radio, RADIO_RX, timeout);
    if(msg != MSG_OK)
      return msg;

    /* Create the AFSK decoder (includes PWM, filters, etc.). */
    AFSKDemodDriver *driver = pktCreateAFSKDecoder(radio);

    /*
     *  If AFSK start failed send event.
     */
    if(driver == NULL) {
#if 0
      pktIncomingBufferPoolRelease(handler);
#endif
      pktAddEventFlags(handler, (EVT_AFSK_START_FAIL));
      handler->rx_link_type = MOD_NONE;
      handler->rx_link_control = NULL;
      pktUnlockRadio(radio, RADIO_RX);
      return MSG_ERROR;
    }
    /* Else AFSK receive decoder started. */
    handler->rx_link_control = driver;
    handler->rx_link_type = MOD_AFSK;
    handler->rx_state = PACKET_RX_OPEN;
    pktUnlockRadio(radio, RADIO_RX);
    return MSG_OK;
  } /* End case MOD_AFSK. */

  case MOD_NONE:
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2: {
    return MSG_ERROR;
  }

  case MOD_CW:
    return MSG_ERROR;

  default:
    return MSG_ERROR;
  } /* End switch on modulation type. */
} /* End function. */

/**
 * @brief   Close radio receive.
 * @pre     The packet service and receive chain is closed.
 * @post    If successful the radio is left locked.
 *
 * @param[in] radio     radio unit ID.
 * @param[in] rto       pointer to radio task object
 * @param[in] timeout   the number of ticks before the operation times outs
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return  Status of operation.
 * @retval  MSG_OK      if receive service closed.
 * @retval  MSG_TIMEOUT if timeout waiting to lock radio.
 * @retval  MSG_RESET   if the radio can not be used due to a lock reset.
 * @retval  MSG_ERROR   if there was an error closing the radio receive.
 *
 * @api
 */
static msg_t pktCloseRadioReceive(const radio_unit_t radio,
                          radio_task_object_t *const rto,
                          const sysinterval_t timeout) {

  packet_svc_t *handler = rto->handler;

  //event_listener_t el;
  //event_source_t *esp;
  //thread_t *decoder = NULL;

#if PKT_RTO_HAS_INNER_CB == TRUE
  switch(rto->radio_dat.type) {
#else
  switch(rto->type) {
#endif

  case MOD_AFSK: {
    /* Stop receive. */
    msg_t msg = pktLockRadio(radio, RADIO_RX, timeout);
    if(msg == MSG_TIMEOUT)
      return msg;
    /* Stop the receiver and stream. */
    pktLLDradioStopReceive(radio);
    /* TODO: This should be a function back in pktservice or rxafsk. */
#if 0
    esp = pktGetEventSource((AFSKDemodDriver *)handler->rx_link_control);
    pktRegisterEventListener(esp, &el, USR_COMMAND_ACK, DEC_CLOSE_EXEC);
    thread_t *decoder =
        ((AFSKDemodDriver *)handler->rx_link_control)->decoder_thd;

    /* Send event to release AFSK resources and terminate thread. */
    chEvtSignal(decoder, DEC_COMMAND_CLOSE);

    /* Wait for the decoder to stop. */
    eventflags_t evt;
    do {
      chEvtWaitAny(USR_COMMAND_ACK);

      /* Wait for correct event at source.
       */
      evt = chEvtGetAndClearFlags(&el);
    } while (evt != DEC_CLOSE_EXEC);
    pktUnregisterEventListener(esp, &el);
#endif

    //esp = pktGetEventSource((AFSKDemodDriver *)handler->rx_link_control);
    //pktRegisterEventListener(esp, &el, USR_COMMAND_ACK, DEC_CLOSE_EXEC);

    thread_t *decoder =
        ((AFSKDemodDriver *)handler->rx_link_control)->decoder_thd;
    chEvtSignal(decoder, DEC_COMMAND_CLOSE);

    /*
     *  Release decoder thread heap when it terminates.
     */
    rto->result = chThdWait(decoder);
    break;
  } /* End case MOD_AFSK */

  case MOD_NONE:
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2: {
    return MSG_ERROR;
  } /* End case MOD_2FSK_xxxx . */

  case MOD_CW:
    return MSG_ERROR;

  default:
    return MSG_ERROR;
  } /* End switch on link_type. */

#if USE_POOL_RX_BUFFER_OBJECTS == TRUE
  /*
   * Release packet services.
   * Check for outstanding callbacks done earlier so this is safe.
   */
  pktIncomingBufferPoolRelease(handler);
#endif
  handler->rx_state = PACKET_RX_IDLE;
  return MSG_OK;
}


/**
 * @brief   Enables a packet decoder.
 * @pre     The packet channel must have been opened.
 * @post    The packet decoder is running.
 *
 * @param[in]   radio unit ID.
 * @param[in] timeout   the number of ticks before the operation times outs
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return  result of operation
 * @retval  MSG_OK      Decoder started.
 * @retval  MSG_TIMEOUT Timeout waiting for start.
 *
 * @api
 */
static msg_t pktStartRadioDecoder(const radio_unit_t radio,
                                  sysinterval_t timeout) {
  /*
   */
  packet_svc_t *handler = pktGetServiceObject(radio);

  if(!pktIsReceiveReady(radio)) {
    /* Wrong state. */
    chDbgAssert(false, "wrong state for decoder start");
    return MSG_ERROR;
  }
  event_listener_t el;
  event_source_t *esp;

  switch(handler->radio_rx_config.type) {
  case MOD_AFSK: {

    esp = pktGetEventSource((AFSKDemodDriver *)handler->rx_link_control);

    pktRegisterEventListener(esp, &el, USR_COMMAND_ACK, DEC_START_EXEC);

    thread_t *the_decoder =
        ((AFSKDemodDriver *)handler->rx_link_control)->decoder_thd;
    chEvtSignal(the_decoder, DEC_COMMAND_START);
    /* Wait for the decoder to start. */
    eventmask_t evm = chEvtWaitAnyTimeout(USR_COMMAND_ACK, timeout);
    pktUnregisterEventListener(esp, &el);
    return (evm == 0) ? MSG_TIMEOUT : MSG_OK;
  } /* End case. */

  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2: {
    return MSG_ERROR;
  }

  case MOD_CW: {
    return MSG_ERROR;
  }

  default:
    return MSG_ERROR;
  } /* End switch. */
}

/**
 * @brief   Disables a packet decoder.
 * @pre     The packet channel must be running.
 * @post    The packet decoder is stopped.
 * @post    The radio is placed in standby mode.
 *
 * @param[in]   radio unit ID.
 * @param[in] timeout   the number of ticks before the operation times outs
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @returns     status of operation.
 *
 * @api
 */
static msg_t pktStopRadioDecoder(const radio_unit_t radio,
                                 sysinterval_t timeout) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  if(!pktIsReceiveEnabled(radio)) {
    /* Wrong state. */
    chDbgAssert(false, "wrong state for decoder stop");
    return MSG_ERROR;
  }
  event_listener_t el;
  event_source_t *esp;

  switch(handler->radio_rx_config.type) {
  case MOD_AFSK: {
    esp = pktGetEventSource((AFSKDemodDriver *)handler->rx_link_control);

    pktRegisterEventListener(esp, &el, USR_COMMAND_ACK, DEC_STOP_EXEC);

    thread_t *the_decoder =
        ((AFSKDemodDriver *)handler->rx_link_control)->decoder_thd;
    chEvtSignal(the_decoder, DEC_COMMAND_STOP);
    /* Wait for the decoder to stop. */
    eventmask_t evm = chEvtWaitAnyTimeout(USR_COMMAND_ACK, timeout);
    pktUnregisterEventListener(esp, &el);
    if(evm != 0)
      pktLLDradioStandby(radio);
    return (evm == 0) ? MSG_TIMEOUT : MSG_OK;
  } /* End case. */

  case MOD_NONE:
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2: {
    return MSG_ERROR;
  }

  case MOD_CW:
    return MSG_ERROR;
  } /* End switch. */
  return MSG_ERROR;
}

/**
 * @brief   Start radio receive.
 * @pre     The packet service and receive chain should be open.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   rto     pointer to radio task object
 * @param[in] timeout   the number of ticks before the operation times out
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return  Status of operation
 * @retval  MSG_OK      if receive started
 * @retval  MSG_TIMEOUT if timeout waiting to lock radio
 * @retval  MSG_RESET   if the radio can not be used due to a system abort.
 * @retval  MSG_ERROR   if there was an error starting the radio receive
 *
 * @api
 */
static msg_t pktStartRadioReceive(const radio_unit_t radio,
                                  radio_task_object_t *const rto,
                                  const sysinterval_t timeout) {
  packet_svc_t *handler = rto->handler;

  /* Hold any radio requests. */
  msg_t msg;
  if((msg = pktLockRadio(radio, RADIO_RX, timeout)) != MSG_OK) {
    return msg;
  }

  /* Configure physical radio for receive. */
  if(!pktLLDradioStartReceive(radio, rto)) {
    pktUnlockRadio(radio, RADIO_RX);
    return MSG_ERROR;
  }

  /*
   * Start the external decoder (for PWM AFSK).
   * The decoder attaches the packet stream and waits for data.
   */
  msg = pktStartRadioDecoder(radio, timeout);

  if(msg == MSG_OK)
    /* Set state. */
    handler->rx_state = PACKET_RX_ENABLED;
  pktUnlockRadio(radio, RADIO_RX);
  return msg;
}

/**
 * @brief   Stop radio receive.
 * @pre     The packet service is open and with receive chain active.
 *
 * @param[in] radio     radio unit ID.
 * @param[in] lock      the number of ticks before the radio lock times out
 * @param[in] lock      the number of ticks before the decoder stop times out
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return  Status of operation
 * @retval  MSG_OK      if receive was stopped
 * @retval  MSG_TIMEOUT if timed out waiting for a radio lock
 * @retval  MSG_RESET   if a semaphore reset released the radio lock
 * @retval  MSG_ERROR   decoder failed to stop.
 *
 * @api
 */
static msg_t pktStopRadioReceive(const radio_unit_t radio,
                          radio_task_object_t *rto,
                          sysinterval_t lock_to,
                          sysinterval_t decode_to) {

  /* Hold any transmit requests. */
  msg_t msg = pktLockRadio(radio, RADIO_RX, lock_to);
  if(msg != MSG_OK)
    return msg;
  msg = pktStopRadioDecoder(radio, decode_to);
  if(msg == MSG_OK) {
    rto->handler->rx_state = PACKET_RX_OPEN;
  }
  pktUnlockRadio(radio, RADIO_RX);
  return msg;
}


/**
 * @brief   Set receive data stream active.
 * @notes   The receive must be in enabled state.
 * @post    The radio receive is resumed.
 *
 * @param[in] radio     Radio unit ID.
 * @param[in] timeout   the number of ticks before the operation times out.
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return  status of request
 * @retval  MSG_IDLE        receive is not enabled.
 * @retval  MSG_OK          receive stream resumed.
 * @retval  MSG_TIMEOUT     radio was not locked within timeout.
 * @retval  MSG_RESET       radio lock semaphore was reset.
 * @retval  MSG_ERROR       an error occurred in managing the radio.
 *
 * @api
 */
static msg_t pktSetReceiveStreamActive(const radio_unit_t radio,
                            radio_task_object_t *const rto,
                            const sysinterval_t timeout) {
  if(pktIsReceiveEnabled(radio)) {
      packet_svc_t *handler = rto->handler;

      /* Must use the RX link type here. */
      switch(handler->rx_link_type) {
      case MOD_AFSK: {
        msg_t msg = pktLockRadio(radio, RADIO_RX, timeout);
        if(msg == MSG_TIMEOUT)
          return msg;
        if(!pktLLDradioStartReceive(radio, rto)) {
          TRACE_ERROR("RAD  > Receive on radio %d failed to "
              "resume after pause", radio);
          pktUnlockRadio(radio, RADIO_RX);
          return MSG_ERROR;
        }
        pktEnableRadioStream(radio);
        pktUnlockRadio(radio, RADIO_RX);
        return MSG_OK;
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
      case MOD_2FSK_115k2:
        return MSG_ERROR;
      } /* End switch on mod type. */
  } /* End test of receive disabled. */
  return MSG_IDLE;
}
#if 0
#if PKT_RTO_HAS_INNER_CB == TRUE
/**
 * Outer level CB when an active receive osc update completes.
 */
static bool pktReceiveOscDone(radio_task_object_t *rt) {
  (void)rt;
}

/**
 * TODO: The stop/restart process should be handled as an internal RM task.
 * The TCXO update request is an outer level command (even though the RM initiates it).
 * Then the RM should handle the pause/resume as an internal sequence.
 */
static void pktReceiveOscUpdate(radio_task_object_t *rt) {
  packet_svc_t *handler = rt->handler;
  const radio_unit_t radio = handler->radio;
  xtal_osc_t tcxo = pktGetCurrentTCXO();
  /* We are in a callback and have the radio locked.
   * Apply the oscillator update.
   */
  (void)pktLLDradioOscUpdate(radio, tcxo);
  pktUnlockRadio(radio, RADIO_RX);
  handler->xtal = tcxo;
  rt->command = PKT_RADIO_RX_START_UNLOCK;
  pktSubmitPriorityRadioTask(radio, rt, NULL);
  TRACE_DEBUG("RAD  > Resuming receive on radio %d", radio);
  /*
   * The RTO is being re-used.
   * It will be released after the RM task is done.
   */
  return true;
}

#else
/**
 *
 */
static bool pktReceiveOscUpdate(radio_task_object_t *rt) {
  packet_svc_t *handler = rt->handler;
  const radio_unit_t radio = handler->radio;
  xtal_osc_t tcxo = pktGetCurrentTCXO();
  /* We are in a callback and can't block the RM thread. */
  msg_t msg = pktLockRadio(radio, RADIO_RX, TIME_MS2I(10));
  if(msg == MSG_OK) {
    /*
     * Radio is locked.
     * Apply the oscillator update.
     */
    TRACE_DEBUG("RAD  > Sending new TCXO %d Hz to radio %d", tcxo, radio);
    (void)pktLLDradioOscUpdate(radio, tcxo);
    pktUnlockRadio(radio, RADIO_RX);
    handler->xtal = tcxo;
    handler->xo_update = false;
  }

  /*
   *  Now resume receive.
   *  A re-schedule may have allowed another task to be queued.
   *  The existing object is posted ahead by using priority command.
   *  The radio is locked during receive start actions.
   */
  rt->command = PKT_RADIO_RX_START;
  pktSubmitPriorityRadioTask(radio, rt, NULL);
  TRACE_DEBUG("RAD  > Resuming receive on radio %d", radio);

  /*
   *  Task object passed in has been re-posted.
   *  Indicate not to be freed upon return from this callback.
   */
  return true;
}
#endif
#endif
/**
 * @brief   Process radio task requests.
 * @notes   Task objects posted to the queue are processed per radio.
 * @notes   The queue is blocked while the radio driver functions execute.
 * @notes   Receive tasks start the receive/decode system which are threads.
 * @notes   Transmit tasks should be handled in threads (and are in 446x).
 *
 * @param[in] arg pointer to a @p packet service object for this radio.
 *
 * @return  status (MSG_OK) on exit.
 *
 * @notapi
 */
THD_FUNCTION(pktRadioManager, arg) {

  /* When waiting for TX tasks to complete if manager is terminating. */
#define PKT_RADIO_TASK_RESUBMIT_WAIT        TIME_MS2I(100)
  /* Poll for TCXO changes. */
#define PKT_RADIO_TCXO_POLL                 TIME_S2I(10)

  packet_svc_t *handler = arg;

  dyn_objects_fifo_t *the_radio_fifo = handler->the_radio_fifo;

  chDbgCheck(arg != NULL);

  objects_fifo_t *radio_queue = chFactoryGetObjectsFIFO(the_radio_fifo);

  chDbgAssert(radio_queue != NULL, "no queue in radio manager FIFO");

  const radio_unit_t radio = handler->radio;

  /* Clear the XO frequency. */
  handler->xtal = 0;
  handler->xo_update = false;
  xtal_osc_t tcxo;

  /*
   * Collective status of initialisations.
   * Any init that fails will set true.
   */
  bool init_fail = false;

#if USE_POOL_RX_BUFFER_OBJECTS == TRUE
  handler->packet_heap = pktIncomingBufferPoolCreate(radio);
  if(handler->packet_heap == NULL)
    init_fail |= true;
#endif /* USE_POOL_RX_BUFFER_OBJECTS == TRUE */

  handler->active_packet_object = NULL;

  /* Take radio out of shutdown and initialize base registers. */
  init_fail |= !pktLLDradioInit(radio);

  thread_t *initiator = chMsgWait();
  chMsgGet(initiator);
  if(init_fail) {
    /* Failed to initialise. */
    chMsgRelease(initiator, MSG_ERROR);
    chThdExit(MSG_OK);
  }
  /* Tell initiator all is OK with radio init. */
  chMsgRelease(initiator, MSG_OK);
  /*
   * Run unless error based close request.
   * Otherwise process tasks until closed by command.
   */
  while(!chThdShouldTerminateX()) {
    /* Check for task requests. */
    radio_task_object_t *task_object;
    msg_t msg = chFifoReceiveObjectTimeout(radio_queue,
                         (void *)&task_object, PKT_RADIO_TCXO_POLL);
    if(msg == MSG_TIMEOUT) {
      /*
       * Update the radio clock if TCXO has changed.
       * This task could be initiated from another thread.
       * For now it is conveniently located here.
       */
      tcxo = pktCheckUpdatedTCXO(handler->xtal);
      if(tcxo != 0 && !handler->xo_update) {
#if PKT_RTO_HAS_INNER_CB == TRUE
        radio_params_t rp = handler->radio_rx_config;
        /* RX stop will process after any packet being decoded is complete. */

        msg = pktQueueRadioCommand(radio, PKT_RADIO_TCXO_UPDATE,
                                           &rp, TIME_MS2I(10),
                                           NULL);
#else
        radio_task_object_t rt = handler->radio_rx_config;
        rt.command = PKT_RADIO_TCXO_UPDATE;
        msg = pktQueueRadioCommand(radio, &rt, TIME_MS2I(10), NULL);
#endif
        if(msg == MSG_OK)
          handler->xo_update = true;
      } /* End (tcxo != 0 && !handler->xo_update). */
      continue;
    }
    /*
     * Queued task object to handle.
     * Process command.
     */
/*    TRACE_DEBUG("RAD  > Radio task object 0x%x (%d) processing on radio %d",
                task_object, task_object->command, radio);*/

    switch(task_object->command) {

    case PKT_RADIO_TCXO_UPDATE: {
      /*
       * The RM thread loop has detected a change of TCXO and posted a task.
       * The update process is:
       * - Lock radio and set flag for TXCO update in process.
       * - Queue a TCXO update task.
       * - The task executes and restarts the radio with new XO value.
       * - The local copy of TCXO value is updated and the TCXO flag reset.
       * - The radio is restored to prior RX state.
       */
      msg_t msg = pktLockRadio(radio, RADIO_RX, TIME_MS2I(10));
      if(msg == MSG_TIMEOUT) {
        /*
         * The radio has not been locked.
         * Repost inner task, let the FIFO be processed and check again.
         */

#if PKT_RTO_HAS_INNER_CB == TRUE
        pktSubmitRadioTask(radio, task_object, NULL);
#else
        pktSubmitRadioTask(radio, task_object, task_object->user_cb);
#endif
        continue;
      } /* Else MSG_OK or MSG_RESET */
      if(msg == MSG_RESET) {
        /* Radio lock reset. */
        handler->xo_update = false;
        /* Can't execute update. End task. */
        break;
      }

      /*
       * Set stream inactive.
       * RX state is left as enabled.
       * Radio lock is not touched.
       */
      msg_t imsg = pktSetReceiveStreamInactive(radio, task_object,
                                               TIME_MS2I(10));
      if(imsg == MSG_ERROR) {
        handler->xo_update = false;
        pktUnlockRadio(radio, RADIO_RX);
        TRACE_ERROR("RAD  > Unable to deactivate RX stream "
            "for TCXO update on radio %d", radio);
        break;
      }

      /*
       * Radio is now set inactive if it was enabled.
       * Apply the TCXO update.
       * Reactivate stream if it was stopped.
       */
      TRACE_DEBUG("RAD  > Sending new TCXO %d Hz to radio %d", tcxo, radio);
      (void)pktLLDradioOscUpdate(radio, tcxo);
      pktUnlockRadio(radio, RADIO_RX);
      /* Radio receive not enabled if MSG_IDLE, resumed if MSG_OK. */
      task_object->radio_dat = handler->radio_rx_config;
      msg = pktSetReceiveStreamActive(radio, task_object, TIME_MS2I(100));
      /* Resume of receive failed. */
      if(msg == MSG_ERROR) {
        TRACE_ERROR("RAD  > Error when reactivating receive "
            "after TCXO update on radio %d", radio);
      }

      handler->xtal = tcxo;
      handler->xo_update = false;
      task_object->result = msg;
      break;
    } /* End case PKT_RADIO_TCXO_UPDATE */

    /**
     * Close this radio manager
     */
    case PKT_RADIO_MGR_CLOSE: {
      /*
       * Radio manager close is sent as a task object.
       * Check RX callback and TX task objects tasks outstanding.
       * If all done release the FIFO and terminate.
       *
       * TODO: This is open to a race condition.
       * There should be a mechanism blocking new tasks and rejecting queued ones.
       * - In the command queue function test closing semaphore.
       *
       * The task initiator waits with chThdWait(...).
       * TODO: Refactor what buffers, FIFOs are closed here.
       * chSysLock();
       * if(chBSemGetStateI(&handler->close_sem)) {
       *    msg_t msg = chBSemWaitS(&handler->close_sem);
       *
       *    }
       * ...
       */
      if(handler->txrto_ref_count == 0 && handler->rxcb_ref_count == 0) {
        msg_t msg = pktLockRadio(radio, RADIO_RX, TIME_MS2I(100));
        if(msg == MSG_TIMEOUT) {
          /*
           * The radio has not been locked.
           * Repost task in normal order.
           * Let the FIFO be processed and check again.
           * TODO: Add a retry limit or just force out with a sem reset.
           * - But we don't know if ov5640 has been locked more logic
           */
#if PKT_RTO_HAS_INNER_CB == TRUE
          pktSubmitRadioTask(radio, task_object, NULL);
#else
          pktSubmitRadioTask(radio, task_object, task_object->user_cb);
#endif
          continue;
        } /* Else MSG_OK or MSG_RESET */
        pktLLDradioShutdown(radio);
#if USE_POOL_RX_BUFFER_OBJECTS == TRUE
        pktIncomingBufferPoolRelease(handler);
#endif
        chFactoryReleaseObjectsFIFO(handler->the_radio_fifo);
        if(msg != MSG_RESET)
          /* If the radio semaphore was reset then we did not get lock. */
          pktUnlockRadio(radio, RADIO_RX);
        chThdExit(MSG_OK);
        /* We never arrive here. */
      }
      /*
       * There are still TX sessions running or RX callback active.
       * Wait, repost task, let the FIFO be processed and check again.
       * TODO: RX open should also be handled in some way?
       */
      chThdSleep(PKT_RADIO_TASK_RESUBMIT_WAIT);
#if PKT_RTO_HAS_INNER_CB == TRUE
      pktSubmitRadioTask(radio, task_object, NULL);
#else
      pktSubmitRadioTask(radio, task_object, task_object->user_cb);
#endif
      continue;
    } /* End case PKT_RADIO_MGR_CLOSE */

    /**
     * Request to capture RSSI from receiver.
     * The capture has to be done at thread level as SPI has to be used.
     */
    case PKT_RADIO_RX_RSSI: {
      /* Read RSSI radio task. */
      msg_t msg = pktLockRadio(radio, RADIO_RX, TIME_MS2I(10));
      task_object->result = msg;
      if(msg == MSG_OK) {
        task_object->rssi = pktLLDradioCaptureRSSI(radio);
        pktUnlockRadio(radio, RADIO_RX);
      }
      break;
    } /* End case PKT_RADIO_RX_RSSI */

    /**
     * Open a receive session ready for reception. Receive is not started.
     */
    case PKT_RADIO_RX_OPEN: {
      /*
       *  pktStartRadioReceive checks modulation type and handles accordingly.
       *  It locks the radio and leaves it locked on success (MSG_OK).
       *
       */

      msg_t msg = pktOpenRadioReceive(radio, task_object, TIME_MS2I(10));
      if(msg == MSG_TIMEOUT) {
        /*
         * Time out waiting to lock radio.
         * Repost task, let the FIFO be processed and check again.
         */
#if PKT_RTO_HAS_INNER_CB == TRUE
        task_object->command = PKT_RADIO_RX_OPEN;
        pktSubmitRadioTask(radio, task_object, NULL);
#else
        pktSubmitRadioTask(radio, task_object,
                           task_object->user_cb);
#endif
        continue;
      }
      /* Get here on MSG_OK, MSG_RESET or MSG_ERROR. */
      if(msg != MSG_OK) {
        TRACE_DEBUG("RAD  > Radio receive open on radio %d failed (%d)",
                    radio, msg);
      }
      task_object->result = msg;
      break;
    } /* End case PKT_RADIO_RX_OPEN. */

    /**
     * Start receive on an open session.
     */
    case PKT_RADIO_RX_START: {

      /*
       *  pktStartRadioReceive checks modulation type and handles accordingly.
       *
       */
      msg_t msg = pktStartRadioReceive(radio, task_object, TIME_MS2I(10));
      if(msg == MSG_TIMEOUT) {
        /*
         * Time out waiting to lock radio.
         * Repost task, let the FIFO be processed and check again.
         */
#if PKT_RTO_HAS_INNER_CB == TRUE
        task_object->command = PKT_RADIO_RX_START;
        pktSubmitRadioTask(radio, task_object, NULL);
#else
        pktSubmitRadioTask(radio, task_object,
                                   task_object->user_cb);
#endif
        continue;
      }

      /* Get here on MSG_OK, MSG_RESET or MSG_ERROR. */
      if(msg != MSG_OK) {
        TRACE_DEBUG("RAD  > Radio receive start on radio %d failed (%d)",
                        radio, msg);
      }
      task_object->result = msg;
      break;
    } /* End case PKT_RADIO_RX_START. */

    /**
     * Stop receive.
     * The session is left open.
     * The radio is unlocked after stop executed.
     */
    case PKT_RADIO_RX_STOP: {
      /*
       *  Stop the radio reception.
       *  If success the radio is left locked.
       */
      msg_t msg = pktStopRadioReceive(radio, task_object,
                                      TIME_MS2I(10),
                                      TIME_S2I(5));
      if(msg == MSG_TIMEOUT) {
        /*
         * Time out waiting to lock radio.
         * Repost task, let the FIFO be processed and check again.
         */

#if PKT_RTO_HAS_INNER_CB == TRUE
      pktSubmitRadioTask(radio, task_object, NULL);
#else
      pktSubmitRadioTask(radio, task_object,
                                 task_object->user_cb);
#endif
        continue;
      }
      /* Get here on MSG_OK or MSG_RESET. */
      if(msg != MSG_OK) {
        TRACE_DEBUG("RAD  > Radio receive stop on radio %d failed (%d)",
                        radio, msg);
      }
      task_object->result = msg;
      break;
    } /* End case PKT_RADIO_RX_STOP. */

    /**
     * Check decode finish as task after PKT_RADIO_RX_STOP.
     * TBD if this gets implemented.
     * Right now there is a generous wait in RX_STOP.
     * This is is blocking other tasks up to the T/O period.
     * To implement RX_STOP would just post the decoder stop request.
     * Then retry and wait with a sliced T/O for the stop event in RX_DECODE.
     */
    case PKT_RADIO_RX_DECODE: {
      break;
    } /* End case PKT_RADIO_RX_DECODE. */

    /**
     * Send packet(s) on radio.
     */
    case PKT_RADIO_TX_SEND: {

      /*
       * Queue transmission.
       * This is non blocking as each radio send runs in a thread.
       * The transmit task on the radio is responsible for pausing receive.
       * This is done with pktSetReceiveInactive() once the TX task starts.
       * The pause will wait for in-progress receive to complete or timeout.
       */
      if(pktLLDradioSendPacket(task_object)) {

        /*
         * Keep count of active sends.
         * Shutdown or resume receive is handled in TX terminate when all done.
         */
        handler->txrto_ref_count++;

        /* Send Successfully enqueued.
         * The task object is held by the TX process until complete.
         * The radio task object is released through a TX thread release task.
         */
        continue;
      }
      /*
       *  Send queueing failed.
       *  Release send packet object(s) and task object.
       *  The callback (if set) will get the result in RTO->result.
       *
       *  Actual result of transmit will be returned by radio.
       *  TODO: Delineate between failed submit and failed TX.
       */
#if PKT_RTO_HAS_INNER_CB == TRUE
      packet_t pp = task_object->radio_dat.packet_out;
#else
      packet_t pp = task_object->packet_out;
#endif
      pktReleaseBufferChain(pp);
      task_object->result = MSG_ERROR;
      break;
      } /* End case PKT_RADIO_TX_SEND. */

    /**
     * Close the receive session. The decoder thread and data is released.
     */
    case PKT_RADIO_RX_CLOSE: {
      /*
       *  Close the radio reception.
       *  If success the radio is left locked.
       */
      msg_t msg = pktCloseRadioReceive(radio,
                                task_object,
                                TIME_MS2I(10));
      if(msg == MSG_TIMEOUT) {
        /*
         * Time out waiting to lock radio.
         * Repost task, let the FIFO be processed and check again.
         */

#if PKT_RTO_HAS_INNER_CB == TRUE
      pktSubmitRadioTask(radio, task_object, NULL);
#else
      pktSubmitRadioTask(radio, task_object,
                                 task_object->user_cb);
#endif
        continue;
      }
      /* Get here on MSG_OK or MSG_RESET. */
      if(msg != MSG_OK) {
        TRACE_DEBUG("RAD  > Radio close receive on radio %d failed (%d)",
                        radio, msg);
      } else {
          pktUnlockRadio(radio, RADIO_RX);
      }
      task_object->result = msg;
      break;
      } /* End case PKT_RADIO_RX_CLOSE. */

    case PKT_RADIO_TX_DONE: {
      /* Get thread exit code and free memory. */
      msg_t msg = chThdWait(task_object->thread);
      task_object->result = MSG_ERROR;
      if(msg == MSG_TIMEOUT) {
        TRACE_ERROR("RAD  > Transmit timeout on radio %d", radio);
      }
      if(msg == MSG_RESET) {
        TRACE_ERROR("RAD  > Transmit failed to start on radio %d", radio);
      }
      /* Set status WRT TX. */
      task_object->result = MSG_OK;

      /* If no transmissions pending then enable RX or enter standby. */
      if(--handler->txrto_ref_count == 0) {
#if PKT_RTO_HAS_INNER_CB == TRUE
        task_object->radio_dat = handler->radio_rx_config;
#else
        task_object = handler->radio_rx_config;
#endif
        msg = pktSetReceiveStreamActive(radio, task_object, TIME_MS2I(10));
      }
      if(msg != MSG_IDLE)
        /* Receive is enabled or an error or timeout occurred. */
        break;
      /*
       * Enter standby state (low power).
       * TODO: Create new standby task that can be re-submitted.
       */
      TRACE_DEBUG("RAD  > Radio %d entering standby", radio);
      msg = pktLockRadio(radio, RADIO_RX, TIME_IMMEDIATE);
      if(msg != MSG_OK)
        break;
      pktLLDradioStandby(radio);
      pktUnlockRadio(radio, RADIO_RX);
      /* Execute any callback then release the RTO. */
      break;
    } /* End case PKT_RADIO_TX_DONE */

    } /* End switch on RTO command. */

#if PKT_RTO_HAS_INNER_CB == TRUE
    /* Perform radio manager callback if specified. */
    if(task_object->mgr_cb != NULL) {
      /*
       * Perform the callback.
       * The callback should be brief and non-blocking (no spinning hard loops).
       * The callback returns true if more internal RM processing is required.
       * If so then continue with this RTO for internal processing.
       */
      if(task_object->mgr_cb(task_object))
        /* Another RM callback has been set by the CB using the current RTO.
         * Go around again.
         */
        continue;
    }
    /* Perform radio task callback if specified. */
    if(task_object->user_cb != NULL) {
      /*
       * Perform the callback.
       * The callback should be brief and non-blocking (no spinning hard loops).
       */
      task_object->user_cb(task_object);
    }
    /* Return task object to free list. */
    chFifoReturnObject(radio_queue, (radio_task_object_t *)task_object);
#else
    /* Perform radio task callback if specified. */
    if(task_object->user_cb != NULL) {
      /*
       * Perform the callback.
       * The callback should be brief and non-blocking (no spinning hard loops).
       * The callback returns true if it has re-used the RT object.
       * If not then the object is freed now.
       */
      if(task_object->user_cb(task_object))
        /* Task object has been re-used.
         * Don't return radio task object to free list.
         */
        continue;
    }
    /* Return task object to free list. */
    chFifoReturnObject(radio_queue, (radio_task_object_t *)task_object);
#endif
/*    TRACE_DEBUG("RAD  > Radio task object 0x%x (%d) freed on radio %d",
                task_object, task_object->command, radio);*/

  } /* End while. */
  /*
   *  The loop is terminated if the thread terminate request is set.
   *  This method of termination is used for init error conditions only.
   */
  chThdExit(MSG_ERROR);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Create a radio task manager.
 * @notes   The task manager will initialise the radio.
 *
 * @param[in]   radio   radio unit ID.
 *
 * @return   The thread will exit and return a result...
 * @retval   MSG_OK when terminated normally by api
 * @retval   MSG_ERROR when terminated due to a stratup error
 *
 * @api
 */
thread_t *pktRadioManagerCreate(const radio_unit_t radio) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  /* Create the radio manager name. */
  chsnprintf(handler->rtask_name, sizeof(handler->rtask_name),
             "%s%02i", PKT_RADIO_TASK_QUEUE_PREFIX, radio);

  /* Create the radio task object FIFO. */
  dyn_objects_fifo_t *the_radio_fifo =
      chFactoryCreateObjectsFIFO(handler->rtask_name,
      sizeof(radio_task_object_t),
      RADIO_TASK_QUEUE_MAX, sizeof(msg_t));

  chDbgAssert(the_radio_fifo != NULL, "unable to create radio task queue");

  if(the_radio_fifo == NULL)
    return NULL;

  handler->the_radio_fifo = the_radio_fifo;

  TRACE_INFO("PKT  > radio manager thread created. FIFO @ 0x%x",
            the_radio_fifo);

  /* Start the radio manager thread. */
  handler->radio_manager = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(PKT_RADIO_MANAGER_WA_SIZE),
              handler->rtask_name,
              NORMALPRIO + 20,
              pktRadioManager,
              handler);

  chDbgAssert(handler->radio_manager != NULL,
              "unable to create radio task thread");

  if(handler->radio_manager == NULL) {
    return NULL;
  }
  msg_t init = chMsgSend(handler->radio_manager, MSG_OK);
  if(init == MSG_OK)
    return handler->radio_manager;

  /* Radio init failed so clean up. */
  chThdTerminate(handler->radio_manager);

  /* Wait for the radio manager thread to terminate. */
  chThdWait(handler->radio_manager);

  /* Release the RM FIFO. */
  chFactoryReleaseObjectsFIFO(the_radio_fifo);

  /* Forget the references. */
  handler->radio_manager = NULL;
  return NULL;
}

/**
 * @brief   Release the radio task manager.
 * @pre     The packet session is stopped so new TX or RX requests are blocked.
 * @notes   Any outstanding TX tasks are allowed to complete.
 * @post    The radio task manager is terminated and released.
 * @post    The radio manager FIFO is released.
 *
 * @param[in]   radio   radio unit ID.
 *
 * @api
 */
msg_t pktRadioManagerRelease(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  /*
   * Get a task object to send to the manager.
   * The radio manager thread will terminate.
   * The FIFO is released in the manager thread before terminating.
   */
#if PKT_RTO_HAS_INNER_CB == TRUE
  msg_t msg = pktQueueRadioCommand(radio,
                      PKT_RADIO_MGR_CLOSE,
                      NULL,
                      TIME_INFINITE,
                      NULL);
  if(msg != MSG_OK)
    return msg;
#else
  radio_task_object_t rto;
  rto.command = PKT_RADIO_MGR_CLOSE;
  msg_t msg = pktQueueRadioCommand(radio,
                      &rto,
                      TIME_INFINITE,
                      NULL);
  if(msg != MSG_OK)
    return msg;
#endif
  return chThdWait(handler->radio_manager);
}


/**
 * @brief   Get a radio command task object.
 * @pre     Called from ISR level.
 * @post    A task object is returned ready for filling and submission.
 *
 * @param[in]   radio   radio unit ID.
 * @param[out]  rt      pointer to a task object.
 *
 * @return  Status of the operation.
 * @retval  MSG_TIMEOUT an object could not be obtained.
 * @retval  MSG_OK      an object has been fetched.
 *
 * @iclass
 */
msg_t pktGetRadioTaskObjectI(const radio_unit_t radio,
#if PKT_RTO_HAS_INNER_CB == TRUE
                            const radio_params_t *cfg,
#endif
                            radio_task_object_t **rt) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  dyn_objects_fifo_t  *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /* Test if task manager is closing. */
  if(chBSemGetStateI(&handler->close_sem))
    return MSG_RESET;

  *rt = chFifoTakeObjectI(task_queue);

  if(*rt == NULL) {
    /* No object available. */
    return MSG_TIMEOUT;
  }
  /* Clear the object then add base data. */
  memset(*rt, 0, sizeof(radio_task_object_t));
  (*rt)->handler = handler;
  (*rt)->result = MSG_OK;
#if PKT_RTO_HAS_INNER_CB == TRUE
  (*rt)->mgr_cb = NULL;
  if(cfg != NULL)
    (*rt)->radio_dat = *cfg;
#endif
  return MSG_OK;
}

/**
 * @brief   Submit a priority radio command to the task manager.
 * @notes   Called from ISR level.
 * @post    A task object is populated and submitted to the radio manager.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   object  radio task object to be submitted.
 * @param[in]   cb      function to call with result (can be NULL).
 *
 * @api
 */
void pktSubmitPriorityRadioTaskI(const radio_unit_t radio,
                         radio_task_object_t *object,
#if PKT_RTO_HAS_INNER_CB == TRUE
                         const radio_mgr_cb_t cb) {
#else
                         const radio_task_cb_t cb) {
#endif

  packet_svc_t *handler = pktGetServiceObject(radio);

  dyn_objects_fifo_t *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /* Populate the object with information from request. */


#if PKT_RTO_HAS_INNER_CB == TRUE
  object->mgr_cb = cb;
  /* The user CB is set when the RTO is created. */
#else
  //object->handler = handler; // TODO: Deprecate
  object->user_cb = cb;
#endif
  /*
   * Submit the task to the queue.
   * The radio manager thread will process the request.
   */
  chFifoSendObjectAheadI(task_queue, object);
}

/**
 * @brief   Get a radio command task object.
 * @post    A task object is returned ready for filling and submission.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   timeout maximum time to wait for a task to be submitted.
 * @param[in]   rt      pointer to a task object pointer.
 *
 * @return  Status of the operation.
 * @retval  MSG_OK      an object has been fetched.
 * @retval  MSG_TIMEOUT an object could not be obtained within the timeout.
 * @retval  MSG_RESET   the radio manager is closing so no object available.
 *
 * @api
 */
msg_t pktGetRadioTaskObject(const radio_unit_t radio,
                            const sysinterval_t timeout,
#if PKT_RTO_HAS_INNER_CB == TRUE
                            const radio_params_t *rp,
#endif
                            radio_task_object_t **rt) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  dyn_objects_fifo_t *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /*
   * Wait on RM close semaphore.
   * If we get RESET or TIMEOUT then exit.
   */
  chSysLock();
  msg_t msg = chBSemWaitS(&handler->close_sem);
  if(msg == MSG_RESET) {
    chSysUnlock();
    return msg;
  }

  /* Request an RT object. */
  *rt = chFifoTakeObjectTimeoutS(task_queue, timeout);

  if(*rt == NULL) {
    /* Timeout waiting for object. */
    chBSemSignalI(&handler->close_sem);
    chSysUnlock();
    return MSG_TIMEOUT;
  }
  /* Clear the object then add base data. */
  memset(*rt, 0, sizeof(radio_task_object_t));
  /* Set defaults in RT object. */
  (*rt)->handler = handler;
  (*rt)->result = MSG_OK;
#if PKT_RTO_HAS_INNER_CB == TRUE
  (*rt)->mgr_cb = NULL;
  if(rp != NULL)
    (*rt)->radio_dat = *rp;
#endif
  chBSemSignalI(&handler->close_sem);
  chSysUnlock();
  return MSG_OK;
}

/**
 * @brief   Submit a radio command to the task manager.
 * @post    A task object is populated and submitted to the radio manager.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   object  radio task object to be submitted.
 * @param[in]   cb      function to call with result (can be NULL).
 *
 * @api
 */
void pktSubmitRadioTask(const radio_unit_t radio,
                         radio_task_object_t *object,
#if PKT_RTO_HAS_INNER_CB == TRUE
                         const radio_mgr_cb_t cb) {
#else
                         const radio_task_cb_t cb) {
#endif

  packet_svc_t *handler = pktGetServiceObject(radio);

  dyn_objects_fifo_t *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /* Update object information. */

#if PKT_RTO_HAS_INNER_CB == TRUE
  object->mgr_cb = cb;
#else
  object->user_cb = cb;
#endif
  /*
   * Submit the task to the queue.
   * Submit the task to the queue.
   * The radio manager thread will process the request.
   */
  chFifoSendObject(task_queue, object);
}

/**
 * @brief   Submit a priority radio command to the task manager.
 * @post    A task object is populated and submitted to the radio manager.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   object  radio task object to be submitted.
 * @param[in]   cb      function to call with result (can be NULL).
 *
 * @api
 */
void pktSubmitPriorityRadioTask(const radio_unit_t radio,
                         radio_task_object_t *object,
#if PKT_RTO_HAS_INNER_CB == TRUE
                         const radio_mgr_cb_t cb) {
#else
                         const radio_task_cb_t cb) {
#endif

  packet_svc_t *handler = pktGetServiceObject(radio);

  dyn_objects_fifo_t *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /* Populate the object with information from request. */

#if PKT_RTO_HAS_INNER_CB == TRUE
  object->mgr_cb = cb;
#else
  object->user_cb = cb;
#endif

  /*
   * Submit the task to the queue.
   * The radio manager thread will process the request.
   */
  chFifoSendObjectAhead(task_queue, object);
}

#if 0
/**
 *
 */
static bool pktCheckRadioLockS(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  return chBSemGetStateI(&handler->radio_sem);
}
#endif
/**
 * @brief   Lock resources.
 * @notes   Used to lock radio and optionally PDCMI when...
 * @notes   a) transmitting or
 * @notes   b) making changes to radio configuration.
 * @pre     Receive should be paused if it is active.
 *
 * @param[in] radio     radio unit ID.
 * @param[in] mode      locking mode.
 * @param[in] timeout   time to wait for acquisition.
 *
 * @return              A message specifying the result.
 * @retval MSG_OK       if the resources have been successfully acquired.
 * @retval MSG_TIMEOUT  if all locks could not be acquired within specified time.
 * @retval MSG_RESET    if a resource can not be used due to a system abort.
 *
 * @api
 */
msg_t pktLockRadio(const radio_unit_t radio, const radio_mode_t mode,
                                             const sysinterval_t timeout) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  msg_t msg;
  switch(mode) {
  case RADIO_TX: {
    systime_t now = chVTGetSystemTimeX();
    sysinterval_t remainder;
    if((msg = chBSemWaitTimeout(&handler->radio_sem, timeout)) == MSG_OK) {
      if(timeout == TIME_IMMEDIATE || timeout == TIME_INFINITE)
        remainder = timeout;
      else
        remainder = timeout - chTimeDiffX(now, chVTGetSystemTimeX());
      if((msg = OV5640_PDCMIlock(remainder)) != MSG_OK)
        /* If PDCMI lock failed then release the radio lock. */
        chBSemSignal(&handler->radio_sem);
    }
    break;
  }

  case RADIO_RX:
    msg = chBSemWaitTimeout(&handler->radio_sem, timeout);
    break;
  } /* End switch. */
  return msg;
}

/**
 * @brief   Unlock radio.
 * @notes   Returns when radio unit is unlocked.
 * @pre     Receive should be resumed by calling routine if it was active and mode is TX.
 *
 * @param[in] radio    radio unit ID.
 * @param[in] mode     radio locking mode.
 * @api
 */
void pktUnlockRadio(const radio_unit_t radio, const radio_mode_t mode) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  switch(mode) {
  case RADIO_TX: {
    OV5640_PDCMIunlock();
    chBSemSignal(&handler->radio_sem);
    break;
  }

  case RADIO_RX:
    chBSemSignal(&handler->radio_sem);
    break;
  } /* End switch on mode. */
}

/**
 * @brief   Reset radio lock.
 * @notes   Used to release any thread waiting on radio lock.
 * @post    Threads are released with a MSG_RESET which they should handle.
 * @post    The semaphore is set to the state of taken.
 *
 * @param[in] radio    radio unit ID.
 * @api
 */
void pktResetRadioLock(const radio_unit_t radio, const bool taken) {
  packet_svc_t *handler = pktGetServiceObject(radio);
    chBSemReset(&handler->radio_sem, taken);
}

/**
 * @brief   Return pointer to radio object array for this board.
 *
 * @param[in] radio    radio unit ID.
 *
 * @api
 */
const radio_config_t *pktGetRadioList(void) {
  return radio_list;
}

/**
 * @brief   Return number of radios for this board.
 *
 * @return  Number of radios.
 *
 * @api
 */
uint8_t pktGetNumRadios(void) {
  uint8_t i = 0;
  while(radio_list[i++].unit != PKT_RADIO_NONE);
  return --i;
}

/**
 * @brief   Text rendering of frequency code or absolute frequency.
 *
 * @param[in] buf  Pointer to character buffer.
 * @param[in] size Size of buffer.
 *
 * @return  Number of characters added to buffer.
 *
 * @api
 */
int pktDisplayFrequencyCode(const radio_freq_hz_t code, char *buf, size_t size) {
  char* str = NULL;
  switch(code) {
  case FREQ_INVALID:
    str = "No Code";
    break;

  case FREQ_GEOFENCE:
    str = "APRS Geofence frequency";
    break;

  case FREQ_SCAN:
    str = "APRS Scan channel";
    break;

  case FREQ_RX_APRS:
    str = "APRS Receive frequency";
    break;

  case FREQ_RX_CMDC:
    str = "CNC Receive frequency";
    break;

  case FREQ_DEFAULT:
    str = "APRS Default frequency";
    break;

  case FREQ_CODES_END:
    str = "Invalid Code";
    break;

  default:
    break;
  }
  if(str != NULL)
    return chsnprintf(buf, size, "%s", str);
  else
    return chsnprintf(buf, size, "%d.%03d MHz",
                      code/1000000, (code%1000000)/1000);
}

/**
 * @brief   Get a default operating frequency.
 * @notes   If a valid default is set in configuration use it
 * @notes   Else if a valid frequency is in the radio configuration use it
 * @notes   Else fall back to #defined DEFAULT_OPERATING_FREQUENCY
 * @notes   Else return FREQ_INVALID
 *
 * @param[in] radio         Radio unit ID.
 *
 * @return      Operating frequency
 * @retval      Operating frequency if it is valid for this radio
 * @retval      Default from system configuration if set and supported.
 * @retval      Default for radio if set and supported.
 * @retval      DEFAULT_OPERATING_FREQ if supported on this radio.
 * @retval      FREQ_INVALID otherwise.
 *
 * @api
 */
radio_freq_hz_t pktGetDefaultOperatingFrequency(const radio_unit_t radio) {

  /* Check the system default. */
  radio_band_t *band = pktCheckAllowedFrequency(radio, conf_sram.freq);
  if(band != NULL)
    return conf_sram.freq;

  /* System config frequency not supported by this radio. */
  const radio_config_t *radio_data = pktGetRadioData(radio);

  /*
   * Check if the radio has a valid default set.
   * Could be set to 0 or maybe there is an incorrect configuration.
  */
  if(pktCheckAllowedFrequency(radio, radio_data->def_aprs))
    /* Use default APRS frequency in radio configuration. */
    return radio_data->def_aprs;

  /* Fall back to defined default as last resort. */
  if(pktCheckAllowedFrequency(radio, DEFAULT_OPERATING_FREQ))
    /* Use default APRS frequency in radio configuration. */
    return DEFAULT_OPERATING_FREQ;

  /* None of the options are supported on this radio. */
  return FREQ_INVALID;
}

/**
 * @brief   Get current receive operating frequency.
 *
 * @param[in] radio         Radio unit ID.
 *
 * @return    Actual operating frequency or special code.
 * @retval    Absolute receive frequency or code if receive is active.
 * @retval    Default operating frequency if receive not active.
 *
 * @notapi
 */
radio_freq_hz_t pktGetReceiveOperatingFrequency(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  radio_freq_hz_t op_freq;
  if(pktIsReceiveEnabled(radio)) {
    if(handler->radio_rx_config.base_frequency < FREQ_CODES_END)
      /* Frequency code. */
      return handler->radio_rx_config.base_frequency;
    /* Normal frequency. */
    op_freq = handler->radio_rx_config.base_frequency
        + (handler->radio_rx_config.step_hz * handler->radio_rx_config.channel);
    return op_freq;
  }
  /* Receive is not active so return default operating frequency. */
  return pktGetDefaultOperatingFrequency(radio);
}

/**
 * @brief   Validate an operating frequency in Hz.
 * @notes   Checks absolute frequencies only.
 * @notes   Resolve special frequency codes before calling.
 *
 * @param[in] radio    Radio unit ID.
 * @param[in] freq     Radio frequency in Hz.
 *
 * @return    band object reference
 * @retval    pointer to band object
 * @retval    NULL if frequency not valid
 *
 * @api
 */
radio_band_t *pktCheckAllowedFrequency(const radio_unit_t radio,
                                      const radio_freq_hz_t freq) {
  /* Check validity. */
  uint8_t radios = pktGetNumRadios();
  const radio_config_t *list = pktGetRadioList();
  for(uint8_t i = 0; i < radios; i++) {
    if(list->unit == radio) {
      uint8_t x = 0;
      while(list->bands[x] != NULL) {
        if(list->bands[x]->start <= freq
            && freq < list->bands[x]->end)
          return list->bands[x];
        /* Next band. */
        x++;
      } /* End for bands */
    } /* if(!unit == radio) */
    /* Next radio. */
    list++;
  } /* End for radios*/
  return NULL;
}

/**
 * @brief   Select a radio operable on the required frequency.
 * @notes   Resolves special frequency codes to absolute frequencies.
 *
 * @param[in] freq  Radio frequency or code in Hz.
 * @param[in] step  Step size for radio in Hz.
 * @param[in] chan  Channel for radio.
 * @param[in] mode  Determines which special codes can be resolved
 *
 * @return    radio unit
 * @retval    an enumerated radio ID from PKT_RADIO_1 onwards
 * @retval    PKT_RADIO_NONE if no radio available for the frequency
 *
 * @api
 */
radio_unit_t pktSelectRadioForFrequency(const radio_freq_hz_t freq,
                                        const radio_chan_hz_t step,
                                        const radio_ch_t chan,
                                        const radio_mode_t mode) {
  /* Check for a radio able to operate on the resolved frequency. */
  const radio_config_t *radio_data = pktGetRadioList();
  while(radio_data->unit != PKT_RADIO_NONE) {
    /* Resolve any special codes. */
    radio_freq_hz_t op_freq = pktComputeOperatingFrequency(radio_data->unit,
                                                        freq,
                                                        step,
                                                        chan,
                                                        mode);
    if(pktCheckAllowedFrequency(radio_data->unit, op_freq)) {
      return radio_data->unit;
    }
    radio_data++;
  } /* End for radios*/
  return PKT_RADIO_NONE;
}

/**
 * Get radio data.
 * TODO: refactor to use num radios in loop?
 */
const radio_config_t *pktGetRadioData(const radio_unit_t radio) {
  const radio_config_t *radio_list = pktGetRadioList();
  uint8_t i = 0;
  while(radio_list[i].unit != PKT_RADIO_NONE) {
	  if(radio_list[i].unit == radio)
		  return &radio_list[i];
	  i++;
  }
  return NULL;
}

/**
 *
 */
bool pktLookupModParameters(const radio_unit_t radio, mod_params_t *mp) {
  (void)radio;
  switch(mp->type) {
    case MOD_2FSK_300:    mp->tx_speed = 300;    mp->tx_dev = 200;    break;
    case MOD_2FSK_9k6:    mp->tx_speed = 9600;   mp->tx_dev = 1300;   break;
    case MOD_2FSK_19k2:   mp->tx_speed = 19200;  mp->tx_dev = 1300;   break;
    case MOD_2FSK_38k4:   mp->tx_speed = 38400;  mp->tx_dev = 1300;   break;
    case MOD_2FSK_57k6:   mp->tx_speed = 57600;  mp->tx_dev = 1300;   break;
    case MOD_2FSK_76k8:   mp->tx_speed = 76800;  mp->tx_dev = 1300;   break;
    case MOD_2FSK_96k:    mp->tx_speed = 96000;  mp->tx_dev = 1300;   break;
    case MOD_2FSK_115k2:  mp->tx_speed = 115200; mp->tx_dev = 1300;   break;
    case MOD_AFSK:        mp->tx_speed = 1200;   mp->tx_dev = 2000;   break;
    case MOD_CW:          mp->tx_speed = 0;      mp->tx_dev = 0;      break;
    default:                                                          return false;
  }
  return true;
}

/**
 * @brief   Set receive inactive.
 * @notes   Will wait a timeout for receive in progress before setting inactive.
 * @notes   This function is called by radio transmit threads.
 * @notes   Transmit threads can lock the radio before calling this function.
 * @notes   Receive completion relates with the receiver front end.
 * @notes   The receive will be stopped if activity continues past timeout.
 * @notes   Decoding of currently buffered data will continue and may complete.
 * @post    The radio receive is stopped.
 * @post    The radio manager will resume RX after TX completes.
 *
 * @param[in] radio     Radio unit ID.
 * @param[in] timeout   the number of ticks before the operation times out.
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return  status of request
 * @retval  MSG_IDLE        receive is not enabled.
 * @retval  MSG_OK          receive is enabled but was not active.
 * @retval  MSG_TIMEOUT     receive completed during allowed timeout.
 * @retval  MSG_RESET       receive was stopped as it did not cease within t/o.
 * @retval  MSG_ERROR       invalid receive modulation or other parameter error.
 *
 * @api
 */
msg_t pktSetReceiveStreamInactive(const radio_unit_t radio,
                                  const radio_task_object_t *rto,
                                  const sysinterval_t timeout) {
  if(!pktIsReceiveEnabled(radio))
    return MSG_IDLE;
  msg_t msg = MSG_OK;
  if(pktRadioGetInProgress(radio)) {
    packet_svc_t *handler = rto->handler;

    /*
     * This function is called from TX threads so don't use RTO link type.
     * Use the RX link type from the handler (RPKTDx).
     */
    switch(handler->rx_link_type) {
    case MOD_AFSK: {
      msg = MSG_RESET;
      if(timeout != TIME_IMMEDIATE) {
        event_source_t *esp = pktGetEventSource((packet_svc_t *)handler);
        /* Register for EVT_PWM_STREAM_CLOSE event. */
        event_listener_t el;
        pktRegisterEventListener(esp, &el, GTE_RECEIVE_INACTIVE,
                                 EVT_RAD_STREAM_CLOSE);
        systime_t start = chVTGetSystemTime();
        if(chEvtWaitAnyTimeout(GTE_RECEIVE_INACTIVE, timeout) == 0) {
          msg = MSG_RESET;
          TRACE_DEBUG("RAD  > Timed out in %d ms waiting for in progress receive",
                      chTimeI2MS(timeout));
        } else {
          systime_t end = chVTGetSystemTime();
          TRACE_DEBUG("RAD  > Waited %d ms for in progress receive",
                      chTimeI2MS(end - start));
          msg = MSG_TIMEOUT;
        }
        pktUnregisterEventListener(esp, &el);
      } /* End test on timeout. */
      /*
       * Stop transport layer stream data.
       * The decoder will process buffered data from the radio.
       * If the frame is incomplete the decoder will see an in-stream stop message.
       * In that case the packet is dropped and the decoder resets.
       * Otherwise the decoder can complete processing of a buffered packet.
       */
      pktDisableRadioStream(radio);
      return msg;
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
    case MOD_2FSK_115k2:
      msg = MSG_ERROR;
      break;
    } /* End switch on mod type. */
  } /* End test on receive in progress. */
  return msg;
}


/**
 * @brief   Compute an operating frequency.
 * @notes   All special frequency codes are resolved to a frequency in Hz.
 *
 * @param[in] radio         Radio unit ID.
 * @param[in] base_freq     Radio base frequency in Hz.
 * @param[in] step          Radio channel step size in Hz.
 * @param[in] chan          Radio channel number.
 *
 * @return  operating frequency in Hz.
 * @retval  an absolute operating frequency in Hz.
 * @retval  FREQ_INVALID if frequency or radio ID is invalid.
 *
 * @api
 */
radio_freq_hz_t pktComputeOperatingFrequency(const radio_unit_t radio,
                                          radio_freq_hz_t base_freq,
                                          radio_chan_hz_t step,
                                          radio_ch_t chan,
                                          const radio_mode_t mode) {

  if((base_freq == FREQ_RX_APRS || base_freq == FREQ_SCAN)
                   && mode == RADIO_TX) {
    /* Get current RX frequency (or default) and use that. */
    step = 0;
    chan = 0;
    /* FIXME: Should switch on all special codes for error check. */
    base_freq = pktGetReceiveOperatingFrequency(radio);
  }

  /*
   * Check for dynamic frequency determination.
   * Dynamic can return an absolute frequency or a further special code.
   */
  if(base_freq == FREQ_GEOFENCE) {
    /*
     * Get frequency by geofencing.
     * Geofencing can return special code FREQ_APRS_DEFAULT.
     */
    base_freq = getAPRSRegionFrequency();
    step = 0;
    chan = 0;
  }

  if(base_freq == FREQ_INVALID) { // Geofence not resolved
    base_freq = pktGetDefaultOperatingFrequency(radio);
    step = 0;
    chan = 0;
  }

  /* Calculate operating frequency. */
  radio_freq_hz_t op_freq = base_freq + (step * chan);

  if(pktCheckAllowedFrequency(radio, op_freq) != NULL) {
    return op_freq;
  }
  return FREQ_INVALID;
}

/**
 * @brief   Called by transmit threads to schedule release after completing.
 * @post    A thread release task is posted to the radio manager queue.
 *
 * @param[in]   rto     reference to radio task object.
 * @param[in]   thread  thread reference of thread terminating.
 *
 * @api
 */
void pktRadioSendComplete(radio_task_object_t *const rto,
                          thread_t *const thread) {

  radio_unit_t radio = rto->handler->radio;
  /*
   *  The TX thread to be terminated is set in returned object.
   *  TODO: Just use the result field in RM.
   *  The TX thread can self terminate using idle terminator.
   *  The RTO will be freed by RM.
   */
  rto->command = PKT_RADIO_TX_DONE;
  rto->thread = thread;
#if  PKT_RTO_HAS_INNER_CB == TRUE
  /* Submit guaranteed to succeed by design. */
  pktSubmitPriorityRadioTask(radio, rto, NULL);
#else
  /* Submit guaranteed to succeed by design. */
  pktSubmitPriorityRadioTask(radio, rto, rto->user_cb);
#endif
}

/**
 * Check if a packet is being received by the radio.
 *
 */
bool pktRadioGetInProgress(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);

  if(!pktIsReceiveEnabled(radio))
    return false;

  switch(handler->rx_link_type) {
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2:
    return false;

  case MOD_AFSK: {
    /* TODO: Put a macro in rxpwm.c */
    AFSKDemodDriver *myDemod = handler->rx_link_control;
    chDbgAssert(myDemod != NULL, "no demod driver");

    return myDemod->icustate == PKT_PWM_ACTIVE;
  }

  case MOD_CW:
    /* TODO: Implement check of RSSI threshold. */
    return false;

  case MOD_NONE:
    return false;
  } /* End switch on rx_link_type. */

  return false;
}

/**
 *
 */
ICUDriver *pktLLDradioAttachStream(const radio_unit_t radio) {
  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only one at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */
  return Si446x_attachPWM(radio);
}

/**
 * @brief Read the CCA line when in AFSK PWM receive mode.
 * @notes Used to read a port where CCA is mapped.
 * @notes Must be useable from ISR level so use GPIO read only.
 */
uint8_t pktLLDradioReadCCAlineI(const radio_unit_t radio) {
  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only one at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */
  packet_svc_t *handler = pktGetServiceObject(radio);

  return Si446x_readCCAlineForRX(radio, handler->rx_link_type);
}


/**
 * @brief   Configure indicator for a radio.
 * @notes   A radio can have more than one output type per indicator.
 */
void pktLLDradioConfigIndicator(const radio_unit_t radio,
                                const indicator_t ind) {
  const radio_config_t *data = pktGetRadioData(radio);
  indicator_io_t *inds = data->ind_set;
  if(inds == NULL)
    /* Radio has no indicators. */
    return;
  do {
    if(inds->ind != ind)
      /* This is not the indicator specified. */
      continue;
    switch(inds->type) {
    case PKT_IND_GPIO_LINE: {
      if(inds->address.line != PAL_NOLINE)
        palSetLineMode(inds->address.line, inds->driver.mode);
      continue;
    }

    case PKT_IND_EXT_I2C:
      continue;

    case PKT_IND_EXT_SPI:
      continue;
    }
  } while((inds++)->ind != PKT_INDICATOR_NONE);
}

/**
 * @brief   De-configure indicator output(s) for a radio.
 * @notes   A radio can have more than one output type per indicator.
 */
void pktLLDradioDeconfigIndicator(const radio_unit_t radio,
                                  const indicator_t ind) {
  const radio_config_t *data = pktGetRadioData(radio);
  indicator_io_t *inds = data->ind_set;
  if(inds == NULL)
    return;
  do {
    if(inds->ind != ind)
      /* This is not the indicator specified. */
      continue;
    switch(inds->type) {
    case PKT_IND_GPIO_LINE: {
      if(inds->address.line != PAL_NOLINE)
        palSetLineMode(inds->address.line, PAL_MODE_INPUT);
      continue;
    }

    case PKT_IND_EXT_I2C:
      continue;

    case PKT_IND_EXT_SPI:
      continue;
    }
  } while((inds++)->ind != PKT_INDICATOR_NONE);
}

/**
 * @brief   Update an indicator for a radio.
 * @notes   A radio can have more than one output type per indicator.
 * TODO: Implement so that value can be a scalar or pointer.
 * Then an indicator could get an object or string reference.
 */
void pktLLDradioUpdateIndicator(const radio_unit_t radio,
                                const indicator_t ind,
                                const indicator_msg_t val) {
  const radio_config_t *data = pktGetRadioData(radio);
  const indicator_io_t *inds = data->ind_set;
  if(inds == NULL)
    /* Radio has no indicators. */
    return;
  do {
    if(inds->ind != ind)
      /* This is not the indicator specified. */
      continue;
    switch(inds->type) {
    /* Switch on output type. */
    case PKT_IND_GPIO_LINE: {
      if(inds->address.line == PAL_NOLINE)
        continue;
      if(val == PAL_TOGGLE)
        palToggleLine(inds->address.line);
      else
        palWriteLine(inds->address.line, val);
      return;
    }

    case PKT_IND_EXT_I2C:
      continue;

    case PKT_IND_EXT_SPI:
      continue;
    }
  } while((inds++)->ind != PKT_INDICATOR_NONE);
}


/**
 *
 */
void pktLLDradioDetachStream(const radio_unit_t radio) {
  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only one at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */
  Si446x_detachPWM(radio);
}

/**
 *
 */
const ICUConfig *pktLLDradioStreamEnable(const radio_unit_t radio,
                                         const radio_mod_t mod,
                                         const palcallback_t cb) {
  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only type at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */

  return Si446x_enablePWMevents(radio, mod, cb);
}

/**
 *
 */
void pktLLDradioStreamDisableI(const radio_unit_t radio,
                               const radio_mod_t mod) {
  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only one at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */
  Si446x_disablePWMeventsI(radio, mod);
}

/** @} */
