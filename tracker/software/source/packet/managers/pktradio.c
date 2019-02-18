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
 * LLD init radio.
 * returns false on success, true on fail.
 */
static bool pktLLDradioInit(const radio_unit_t radio) {
  /* TODO: Implement hardware mapping to different radio types. */

  return Si446x_driverInit(radio);
}

/**
 * Called when the radio manager is closing.
 * Release any dynamic memory and the IRQ dispatcher thread.
 */
static void pktLLDradioShutdown(const radio_unit_t radio) {
  /* TODO: Implement hardware mapping. */
  /* TODO: Add new function to 446x to stop this radio IRQ thread.
     This will be done in a critical section. */
  Si446x_radioShutdown(radio);
}

/**
 * Called when the radio has no active tasks.
 */
void pktLLDradioStandby(const radio_unit_t radio) {
  /* TODO: Implement hardware mapping. */

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

  switch(rto->radio_dat.type) {
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
 * @brief   Start physical receiver.
 * @notes   This is the HAL API to the radio LLD.
 * @notes   Currently just maps directly to 446x driver.
 * @notes   In future a VMT lookup would access the relevant radio methods.
 *
 * @param[in] radio radio unit ID.
 *
 * @return  status of the operation
 * @retval  true    operation succeeded.
 * @retval  false   operation failed.
 *
 * @notapi
 */
static bool pktLLDradioStartReceiver(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);

  if (!Si4464_enableReceive(radio,
                            handler->radio_rx_config.base_frequency,
                            handler->radio_rx_config.step_hz,
                            handler->radio_rx_config.channel,
                            handler->radio_rx_config.rssi,
                            handler->radio_rx_config.type)) {
    return false;
  }

  (void)pktLLDradioAttachStream(radio);
  return true;
}

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
 * Returns true if radio is locked.
 */
static bool pktGetRadioLockStatus(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  chSysLock();
  bool s = chBSemGetStateI(&handler->radio_sem);
  chSysUnlock();
  return s;
}

/**
 * @brief   Open radio receive.
 * @pre     The packet service and receive chain is opened and initialised.
 * @pre     The radio parameters are set in the service object.
 * @post    The radio is unlocked.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in] timeout   the number of ticks before the operation times outs
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return  Status of operation
 * @retval  MSG_OK      if receive started.
 * @retval  MSG_TIMEOUT if timeout waiting to lock radio.
 * @retval  MSG_RESET   if the radio can not be used due to a semaphore reset.
 * @retval  MSG_ERROR   if there was an error starting the radio receive.
 *
 * @api
 */
static msg_t pktOpenRadioReceive(const radio_unit_t radio,
                          const sysinterval_t timeout) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  /*
   * Initialize the outstanding callback count.
   */
  handler->rxcb_ref_count = 0;
  /* Switch on modulation type. */

  switch (handler->radio_rx_config.type) {
  case MOD_AFSK: {
    /*
     *  Lock the radio.
     */
    msg_t msg = pktLockRadio(radio, RADIO_RX, timeout);
    if (msg != MSG_OK)
      return msg;

    /* Create the AFSK decoder (includes ICU PWM, filters, etc.). */
    AFSKDemodDriver *driver = pktCreateAFSKDecoder(radio);

    /*
     *  If AFSK start failed send event.
     */
    if (driver == NULL) {
      pktAddEventFlags(handler, (EVT_AFSK_START_FAIL));
      handler->rx_link_type = MOD_NONE;
      handler->rx_link_control = NULL;
      pktUnlockRadio(radio);
      return MSG_ERROR;
    }
    /* Else AFSK receive decoder initialised and is ready. */
    handler->rx_link_control = driver;
    handler->rx_link_type = MOD_AFSK;

    /* Set state. */
    handler->rx_state = PACKET_RX_OPEN;
    pktUnlockRadio(radio);
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
  switch(rto->radio_dat.type) {
  case MOD_AFSK: {
    /* Stop receive. */
    msg_t msg = pktLockRadio(radio, RADIO_RX, timeout);
    if (msg == MSG_TIMEOUT)
      return msg;
    /* Stop the receiver and stream. */
    msg = pktSetReceiveStreamStandby(radio, TIME_S2I(10));
    switch (msg) {
    case MSG_TIMEOUT:
    case MSG_RESET:
    case MSG_OK:
      break;

    case MSG_IDLE:
      /* Receive was not enabled. */
      pktUnlockRadio(radio);
      return msg;

    case MSG_ERROR:
      pktUnlockRadio(radio);
      return msg;

    }

    /* Send close event to decoder. */
    thread_t *decoder =
        ((AFSKDemodDriver *)handler->rx_link_control)->decoder_thd;
    chEvtSignal(decoder, DEC_COMMAND_CLOSE);
    handler->rx_state = PACKET_RX_CLOSE;

    /*
     *  Release decoder thread heap when it terminates.
     */
    (void)chThdWait(decoder);
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

  /*
   * Release packet services.
   * TODO: Is there a check for outstanding call backs done earlier so is this safe?
   */
  pktIncomingBufferPoolRelease(handler);
  handler->rx_state = PACKET_RX_IDLE;
  return MSG_OK;
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
 * @retval  MSG_OK      if receive started.
 * @retval  MSG_IDLE    if receive enabled but stream start deferred due to outstanding TX sessions.
 * @retval  MSG_TIMEOUT if timeout waiting to lock radio.
 * @retval  MSG_RESET   if the radio can not be used due to a system abort.
 * @retval  MSG_ERROR   if there was an error starting the radio receive
 *
 * @api
 */
static msg_t pktStartRadioReceive(const radio_unit_t radio,
                                  const sysinterval_t timeout) {

  packet_svc_t *handler = pktGetServiceObject(radio);
  /* Hold any radio requests. */
  msg_t msg;
  if ((msg = pktLockRadio(radio, RADIO_RX, timeout)) != MSG_OK) {
    return msg;
  }

  if (handler->rx_state != PACKET_RX_OPEN) {
    chDbgAssert(false, "wrong state");
    pktUnlockRadio(radio);
    return MSG_ERROR;
  }

  /* Set ENABLED state. */
  handler->rx_state = PACKET_RX_ENABLED;
  /* If there are outstanding TX tasks defer stream activation. */
  //if (handler->txrto_ref_count == 0) {
    msg = pktSetReceiveStreamReady(radio);
    switch (msg) {
    case MSG_OK:
      break;

    case MSG_TIMEOUT: /* Receive start deferred due TX queued. */
      msg = MSG_IDLE;
      break;

    case MSG_IDLE:
    case MSG_ERROR:
      chDbgAssert(false, "parameter or function error");
      break;
    }
  //}
  pktUnlockRadio(radio);
  return msg;
}

/**
 * @brief   Stop radio receive.
 * @pre     The packet service is open with receive chain active.
 *
 * @param[in] radio     radio unit ID.
 * @param[in] lock_to   the number of ticks before the radio lock times out
 * @param[in] decode_to the number of ticks for decode in progress to time out
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return  Status of operation
 * @retval  MSG_IDLE    receive is not enabled.
 * @retval  MSG_OK      if receive was stopped
 * @retval  MSG_TIMEOUT if timed out waiting for a radio lock
 * @retval  MSG_RESET   if a semaphore reset released the radio lock
 * @retval  MSG_ERROR   decoder failed to stop.
 *
 * @api
 */
static msg_t pktStopRadioReceive(const radio_unit_t radio,
                          sysinterval_t lock_to,
                          sysinterval_t decode_to) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  /*
   * Lock radio to hold any transmit requests.
   */
  msg_t msg = pktLockRadio(radio, RADIO_RX, lock_to);
  if (msg == MSG_OK) {
    msg = pktSetReceiveStreamStandby(radio, decode_to);
    if (msg == MSG_OK) {
      /* Set state back to open versus active. */
      handler->rx_state = PACKET_RX_OPEN;
    }
    pktUnlockRadio(radio);
  }
  return msg;
}

/**
 * @brief   Set receive data streaming ready.
 * @pre     Radio must be locked prior to calling this function.
 * @notes   The receive must be in enabled state.
 * @post    The receive stream is readied if no transmit tasks are outstanding.
 *
 * @param[in] radio     Radio unit ID.
 * @param[in] rto       Pointer to @p radioTask object.
 *
 * @return  status of request
 * @retval  MSG_IDLE        receive is not enabled.
 * @retval  MSG_OK          receive streaming ready. CCA will open a stream.
 * @retval  MSG_TIMEOUT     stream is deferred due to outstanding TX sessions.
 * @retval  MSG_ERROR       radio is not locked or other error.
 *
 * @api
 */
msg_t pktSetReceiveStreamReady(const radio_unit_t radio) {
  chDbgAssert(pktGetRadioLockStatus(radio), "radio not locked");
  if (!pktGetRadioLockStatus(radio))
    return MSG_ERROR;
  if (pktIsReceiveEnabled(radio)) {
      packet_svc_t *handler = pktGetServiceObject(radio);
      if (handler->txrto_ref_count > 0) {
        return MSG_TIMEOUT;
      }
      /* Must use the RX link type here. */
      switch(handler->rx_link_type) {
      case MOD_AFSK: {
        /* Start the physical receiver. */
        if (!pktLLDradioStartReceiver(radio)) {
          TRACE_ERROR("RAD  > Receive on radio %d failed to "
              "resume after pause", radio);
          return MSG_ERROR;
        }

        /* Enable CCA and ICU. Radio CCA will initiate a stream. */
        pktEnableRadioStreamProcessing(radio);
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

/**
 * @brief   Process radio task requests.
 * @detail Task objects posted to the queue are processed per radio.
 *         The queue is blocked while the radio driver functions execute.
 *         Receive tasks start the receive/decode system which are threads.
 *         Transmit tasks are handled in threads (and are in 446x).
 *
 * @param[in] arg pointer to the @p packet_svc_t service object for this radio.
 *
 * @return  status (MSG_OK) on exit.
 *
 * @notapi
 */
THD_FUNCTION(pktRadioManager, arg) {

  packet_svc_t *handler = arg;
  chDbgAssert(handler != NULL, "no handler object");
  const radio_unit_t radio = handler->radio;

  /* Setup for initialisation handshake. */
  thread_t *initiator = chMsgWait();
  (void)chMsgGet(initiator);
#if PKT_RADIO_MANAGER_INIT
  /* Create the radio manager name. */
  chsnprintf(handler->rtask_name, sizeof(handler->rtask_name),
             "%s%02i", PKT_RADIO_TASK_QUEUE_PREFIX, radio);

  /*
   * Create the radio task object FIFO.
   * TODO: Move into thread init.
   */
  dyn_objects_fifo_t *the_radio_fifo =
      chFactoryCreateObjectsFIFO(handler->rtask_name,
      sizeof(radio_task_object_t),
      RADIO_TASK_QUEUE_MAX, sizeof(msg_t));

  chDbgAssert(the_radio_fifo != NULL, "unable to create radio task queue");

  if (the_radio_fifo == NULL) {
    /* Failed to create FIFO. */
    chMsgRelease(initiator, MSG_ERROR);
    chThdExit(MSG_OK);
  }
  handler->the_radio_fifo = the_radio_fifo;
#else
  dyn_objects_fifo_t *the_radio_fifo = handler->the_radio_fifo;

  chDbgCheck(arg != NULL);
#endif
  objects_fifo_t *radio_queue = chFactoryGetObjectsFIFO(the_radio_fifo);

  chDbgAssert(radio_queue != NULL, "no queue in radio manager FIFO");

  /* Create an incoming AX25 packet buffer pool for this radio. */
  handler->packet_heap = pktIncomingBufferPoolCreate(radio);
  if (handler->packet_heap == NULL) {
    /* TODO: Release RTO FIFO. */
#if PKT_RADIO_MANAGER_INIT
    chFactoryReleaseObjectsFIFO(the_radio_fifo);

    /* Forget the references. */
    handler->radio_manager = NULL;
#endif
    /* Failed to initialise/get buffer pool. */
    chMsgRelease(initiator, MSG_ERROR);
    chThdExit(MSG_OK);
  }

  handler->active_packet_object = NULL;

  /* Take radio out of shutdown and initialize base registers. */
  if (pktLLDradioInit(radio)) {
    /* Failed to initialise radio. */
#if PKT_RADIO_MANAGER_INIT
    /* Release the FIFO. */
    chFactoryReleaseObjectsFIFO(the_radio_fifo);

    /* Forget the references. */
    handler->radio_manager = NULL;
#endif
    pktIncomingBufferPoolRelease(handler);
    chMsgRelease(initiator, MSG_ERROR);
    chThdExit(MSG_OK);
  }

  /* Set the initial XO frequency. */
  handler->xtal = pktGetCurrentTCXO();

  /* Tell service manager all is OK with RM init. */
  chMsgRelease(initiator, MSG_OK);

  /* Release the FIFO for allocation. Release the radio for use.*/
  chBSemSignal(&handler->mgr_sem);
  chBSemSignal(&handler->radio_sem);

  /* Now process tasks until closed by command.*/
  while (true) {

    radio_task_object_t *task_object;
    /* Wait for task requests. */
    (void)chFifoReceiveObjectTimeout(radio_queue,
                         (void *)&task_object, TIME_INFINITE);

    /* Process command. */
    switch (task_object->command) {

    case PKT_RADIO_TCXO_UPDATE: {

      /* The TCXO manager has measured an updated TCXO frequency and
          posted a command to notify the RM.  */
      msg_t msg = pktLockRadio(radio, RADIO_RX, TIME_MS2I(10));
      if (msg == MSG_TIMEOUT) {

        /*
         * The radio has not been locked.
         * Repost inner task, let the FIFO be processed and check again.
         */
        pktSubmitRadioTask(radio, task_object, NULL);
        continue;
      } /* Else MSG_OK or MSG_RESET */
      if (msg == MSG_RESET) {
        /* Radio lock semaphore has been reset. Can't execute update. */
        break;
      }
#if 0 /* TODO: Implement PLL update in radio driver for current RX session. */
      /*
       * Set stream inactive.
       * RX state is left as enabled.
       * Radio lock is not touched.
       */
      msg_t imsg = pktSetReceiveStreamStandby(radio, TIME_MS2I(10));
      if (imsg == MSG_ERROR) {
        chDbgAssert(false, "parameter error");
        pktUnlockRadio(radio);
        TRACE_ERROR("RAD  > Parameter error on radio %d", radio);
        break;
      }

      /*
       * Radio is now inactive if it was enabled.
       * Apply the TCXO update.
       * Reactivate stream if it was stopped.
       */
#endif
      TRACE_DEBUG("RAD  > Sending new TCXO %d Hz to radio %d",
                                            handler->xtal, radio);
      bool update = pktLLDradioOscUpdate(radio, handler->xtal);
#if 0
      /* Radio receive not enabled if MSG_IDLE, resumed if MSG_OK. */
      msg = pktSetReceiveStreamReady(radio);
      /* Resume of receive failed. */
      if (msg == MSG_ERROR) {
        chDbgAssert(false, "parameter error");
        TRACE_ERROR("RAD  > Error when reactivating receive "
            "after TCXO update on radio %d", radio);
      }
#endif
      task_object->result = msg;
      pktUnlockRadio(radio);
      TRACE_DEBUG("RAD  > Radio %d %s TCXO update", radio,
                  update ? "accepted" : "ignored");
      break;
    } /* End case PKT_RADIO_TCXO_UPDATE */

    /**
     * Close this radio manager
     */
    case PKT_RADIO_MGR_CLOSE: {
      /*
       * Radio manager close is sent as a task object.
       * Lock out new FIFO object requests.
       * Change to shutdown state and monitor outstanding objects.
       */
      chBSemReset(&handler->mgr_sem, true);

      /* Set internal state for shutdown. */
      task_object->command = PKT_RADIO_MGR_SHUTDOWN;
      pktSubmitRadioTask(radio, task_object, NULL);
      continue;
    } /* End case PKT_RADIO_MGR_CLOSE */

    /**
     * Carrying out the internal processing of the close command
     */
    case PKT_RADIO_MGR_SHUTDOWN: {
      msg_t msg = pktLockRadio(radio, RADIO_RX, TIME_MS2I(100));
      if (msg == MSG_TIMEOUT) {
        /*
         * The radio has not been locked.
         * Repost task in normal order.
         * Let the FIFO be processed and check again.
         */

        pktSubmitRadioTask(radio, task_object, NULL);
        continue;
      } /* Else MSG_OK or MSG_RESET */

      /* The radio is locked. */
      if (handler->txrto_ref_count == 0 && handler->rxcb_ref_count == 0) {

        /* Shutdown radio, release RX buffers and FIFO.  */
        pktLLDradioShutdown(radio);
        pktIncomingBufferPoolRelease(handler);
        chFactoryReleaseObjectsFIFO(handler->the_radio_fifo);
        /* The radio is left locked and the RM semaphore is left held. */
        chThdExit(MSG_OK);
        /* We never arrive here. */
      }

      /* There are still FIFO or RX packet objects outstanding. */
      pktUnlockRadio(radio);

      /* Resubmit shutdown task. */
      pktSubmitRadioTask(radio, task_object, NULL);
      continue;
    } /* End case PKT_RADIO_MGR_SHUTDOWN */

    /**
     * Request to capture RSSI from receiver.
     * The capture has to be done at thread level as SPI has to be used.
     */
    case PKT_RADIO_RX_RSSI: {
      /* Read RSSI radio task. */
      msg_t msg = pktLockRadio(radio, RADIO_RX, TIME_MS2I(10));
      task_object->result = msg;
      if (msg == MSG_OK) {
        task_object->rssi = pktLLDradioCaptureRSSI(radio);
        pktUnlockRadio(radio);
      }
      break;
    } /* End case PKT_RADIO_RX_RSSI */

    /**
     * Open a receive session ready for reception.
     */
    case PKT_RADIO_RX_OPEN: {
      /*
       *
       */

      msg_t msg = pktOpenRadioReceive(radio, TIME_MS2I(10));
      if (msg == MSG_TIMEOUT) {

        /*
         * Time out waiting to lock radio.
         * Repost task, let the FIFO be processed and check again.
         */
        pktSubmitRadioTask(radio, task_object, NULL);
        continue;
      }
      /* Get here on MSG_OK, MSG_RESET or MSG_ERROR. Radio is unlocked. */
      if (msg != MSG_OK) {
        TRACE_DEBUG("RAD  > Radio receive open on radio %d failed (%d)",
                    radio, msg);
        task_object->result = msg;
        break;
      }

      /*
       * Service now has receive open state.
       * Radio is unlocked.
       * Next start receive as next queued task.
       */
      task_object->command = PKT_RADIO_RX_START;
      pktSubmitPriorityRadioTask(radio, task_object, NULL);
      continue;
    } /* End case PKT_RADIO_RX_OPEN. */

    /**
     * Start receive on an open session.
     */
    case PKT_RADIO_RX_START: {

      /*
       *  pktStartRadioReceive checks modulation type and handles accordingly.
       *
       */
      msg_t msg = pktStartRadioReceive(radio, TIME_MS2I(10));
      switch (msg) {
      case MSG_TIMEOUT:

        /*
         * Time out waiting to lock radio.
         * Repost task as normal priority.
         * Let the FIFO be processed and check again.
         */
        pktSubmitRadioTask(radio, task_object, NULL);
        continue;

      case MSG_OK:
      case MSG_IDLE: /* Receive start deferred due TX queued. */
        break;

      case MSG_ERROR: {
        TRACE_DEBUG("RAD  > Radio receive start on radio %d failed (%d)",
                        radio, msg);
        break;
      }

      case MSG_RESET: {
          TRACE_DEBUG("RAD  > Radio %d semaphore released by reset", radio);
        break;
      }

      } /* End switch. */
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
       *  The radio is left unlocked.
       */
      msg_t msg = pktStopRadioReceive(radio,
                                      TIME_MS2I(10),
                                      TIME_S2I(5));
      switch (msg) {
      case MSG_OK:
        /* Receive was stopped. */
        break;

      case MSG_IDLE:
        /* Receive was not enabled. */
        break;

      case MSG_RESET: {
          TRACE_DEBUG("RAD  > Radio %d semaphore released by reset", radio);
          break;
        }

      case MSG_TIMEOUT:
        /*
         * Time out waiting to lock radio.
         * Repost task, let the FIFO be processed and check again.
         */
        pktSubmitRadioTask(radio, task_object, NULL);
        continue;
      }
      /* Execute callbacks and release RTO. */
      task_object->result = msg;
      break;
    } /* End case PKT_RADIO_RX_STOP. */

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
      if (pktLLDradioSendPacket(task_object)) {

        /*
         * Keep count of active sends.
         * Shutdown or resume receive is handled in TX terminate when all done.
         */
        handler->txrto_ref_count++;

        /* Send Successfully enqueued.
         * The task object is held by the TX process until complete.
         * The radio task object is released through a TX done task.
         */
        continue;
      }
      /*
       *  Send queueing failed.
       *  Release send packet object(s) and task object.
       *  The callback (if set) will get the result in RTO->result.
       */

      packet_t pp = task_object->radio_dat.pkt.packet_out;
      pktReleaseBufferChain(pp);
      task_object->result = MSG_ERROR;
      /* Complete command. Callback will be executed and RTO released. */
      break;
      } /* End case PKT_RADIO_TX_SEND. */

    /**
     * Close the receive session. The decoder thread and data is released.
     */
    case PKT_RADIO_RX_CLOSE: {

      /*
       *  Close the radio reception.
       *  TODO: Check outstanding RX call backs termination safety.
       *  If success the radio is left locked.
       */
      msg_t msg = pktCloseRadioReceive(radio,
                                task_object,
                                TIME_MS2I(10));
      switch (msg) {
      case MSG_TIMEOUT:

        /*
         * Time out waiting to lock radio.
         * Repost task, let the FIFO be processed and check again.
         */
        pktSubmitRadioTask(radio, task_object, NULL);
        continue;

      case MSG_OK:
        /* Radio is locked. Receive is shutdown. */
        pktUnlockRadio(radio);
        break;

      case MSG_RESET:
        /* Semaphore has been reset by another thread. */
        break;

      case MSG_ERROR:
        /* Parameter error. */
        chDbgAssert(false, "parameter error");
        break;
      }
      task_object->result = msg;
      break;
      } /* End case PKT_RADIO_RX_CLOSE. */

    /**
     *
     */
    case PKT_RADIO_TX_DONE: {
      msg_t msg = pktLockRadio(radio, RADIO_RX, TIME_MS2I(10));
      if (msg == MSG_TIMEOUT) {
        /*
         * Time out waiting to lock radio.
         * Repost task, let the FIFO be processed and check again.
         */
        pktSubmitRadioTask(radio, task_object, NULL);
        continue;
      }
      if (msg == MSG_RESET) {
        /* Radio semaphore has been reset by another thread. */
        /* Mark TX thread has completed. */
        --handler->txrto_ref_count;
        break;
      }
      /* Radio is locked. Decrease TX ref count. */
#if 0
      --handler->txrto_ref_count;
#else
      if (--handler->txrto_ref_count > 0) {
        /* More TX tasks outstanding. Unlock radio. Process call backs
           and release RTO. */
        pktUnlockRadio(radio);
        break;
      }

      /* No transmissions pending. Resume RX if enabled. */
#endif
      msg = pktSetReceiveStreamReady(radio);
      switch (msg) {
      case MSG_ERROR:   /* Parameter error. */
        chDbgAssert(false, "parameter error");

      case MSG_OK:      /* Receive now active. */
      case MSG_TIMEOUT: /* Receive start deferred due TX queued. */
      case MSG_IDLE:    /* Receive not enabled. */
        break;
      } /* End switch. */

      /* Release radio. */
      pktUnlockRadio(radio);
      /* Execute any callback then release the RTO. */
      break;
    } /* End case PKT_RADIO_TX_DONE */

    } /* End switch on RTO command. */

    /* Task has completed. Perform internal callback if specified. */
    if (task_object->mgr_cb != NULL) {
      /*
       * Perform the callback.
       * The callback should be brief and non-blocking (no spinning hard loops).
       * The callback returns true if more internal RM processing is required.
       * If so then continue with this RTO for internal processing.
       */
      if (task_object->mgr_cb(task_object))
        /* Another RM callback has been set by the CB using the current RTO.
         * Go around again.
         */
        continue;
    }

    /* Perform radio task callback if specified. */
    if (task_object->user_cb != NULL) {
      /*
       * Perform the callback.
       * The callback should be brief, non-blocking and not use suspend.
       * TODO: This could be done as a thread like RX CB but potential
       * exists for a race condition where both CB and suspend are used
       * by a calling thread and it assumes CB completes before wake up.
       */
      task_object->user_cb(task_object);
    }

    /* Wake up calling thread if RM task status was requested. */
    if (task_object->thread != NULL) {
      chThdResume(&(task_object)->thread, task_object->result);
    }

    /* Return task object to free list. */
    chFifoReturnObject(radio_queue, (radio_task_object_t *)task_object);

    /* Decrease the FIFO ref count. */
    chFactoryReleaseObjectsFIFO(handler->the_radio_fifo);

  } /* End while. */
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
#if  PKT_RADIO_MANAGER_INIT != TRUE
  /* Create the radio manager name. */
  chsnprintf(handler->rtask_name, sizeof(handler->rtask_name),
             "%s%02i", PKT_RADIO_TASK_QUEUE_PREFIX, radio);

  /*
   * Create the radio task object FIFO.
   * TODO: Move into thread init.
   */
  dyn_objects_fifo_t *the_radio_fifo =
      chFactoryCreateObjectsFIFO(handler->rtask_name,
      sizeof(radio_task_object_t),
      RADIO_TASK_QUEUE_MAX, sizeof(msg_t));

  chDbgAssert(the_radio_fifo != NULL, "unable to create radio task queue");

  if (the_radio_fifo == NULL)
    return NULL;

  handler->the_radio_fifo = the_radio_fifo;
#endif
  /* Start the radio manager thread. */
  handler->radio_manager = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(PKT_RADIO_MANAGER_WA_SIZE),
              handler->rtask_name,
              NORMALPRIO + 20,
              pktRadioManager,
              handler);

  chDbgAssert(handler->radio_manager != NULL,
              "unable to create radio task thread");

  if (handler->radio_manager == NULL) {
    return NULL;
	}
  TRACE_INFO("PKT  > radio manager thread created for radio %d", radio);

  msg_t init = chMsgSend(handler->radio_manager, MSG_OK);
  if (init == MSG_OK)
    return handler->radio_manager;

  /* Radio init failed so clean up. */
  //chThdTerminate(handler->radio_manager);

  /* Wait for the radio manager thread to terminate. */
  chThdWait(handler->radio_manager);
#if  PKT_RADIO_MANAGER_INIT != TRUE
  /*
   * Release the RM FIFO.
   * TODO: Moves when FIFO creation is in thread init.
   */
  chFactoryReleaseObjectsFIFO(the_radio_fifo);
#endif
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
  msg_t result;
  msg_t msg = pktQueueRadioCommand(radio,
                      PKT_RADIO_MGR_CLOSE,
                      NULL,
                      TIME_INFINITE,
                      &result, NULL);
  if (msg == MSG_OK) {
    if (result == MSG_OK)
      /* Clean up the thread memory. */
      return chThdWait(handler->radio_manager);
  }
  return msg;
}


/**
 * @brief   Get a radio command task object.
 * @pre     Called from ISR level.
 * @post    A task object is returned ready for filling and submission.
 * @note    The radio manager will lock all RT objects when closing.
 *
 * @param[in]   radio   radio unit ID.
 * @param[out]  rt      pointer to a task object.
 *
 * @return  Status of the operation.
 * @retval  MSG_TIMEOUT an object is not available.
 * @retval  MSG_OK      an object has been fetched.
 * @retval  MSG_RESET   the radio manager is closed or closing.
 *
 * @iclass
 */
msg_t pktGetRadioTaskObjectI(const radio_unit_t radio,
                            const radio_params_t *cfg,
                            radio_task_object_t **rt) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  dyn_objects_fifo_t  *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /* FIFO objects are not to be taken. The service is being closed. */
  if (chBSemGetStateI(&handler->mgr_sem)) {
    return MSG_RESET;
  }
  if ((*rt = chFifoTakeObjectI(task_queue)) == NULL) {
    /* No object available. */
      return MSG_TIMEOUT;
  }

  /* Increment the FIFO ref count. */
  chFactoryDuplicateReference(&task_fifo->element);

  /* Clear the object then add base data. */
  memset(*rt, 0, sizeof(radio_task_object_t));
  (*rt)->handler = handler;
  (*rt)->result = MSG_OK;
  (*rt)->mgr_cb = NULL;
  (*rt)->thread = NULL;
  if (cfg != NULL) {
    /* Copy radio params if specified. */
    (*rt)->radio_dat = *cfg;
  }
  return MSG_OK;
}

/**
 * @brief   Submit a priority radio command to the task manager.
 * @notes   Called from ISR level.
 * @post    A task object is populated and submitted to the radio manager.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   object  radio task object to be submitted.
 * @param[in]   cb      radio manager internal call back (can be NULL).
 *
 * @api
 */
void pktSubmitPriorityRadioTaskI(const radio_unit_t radio,
                         radio_task_object_t *object,
                         const radio_mgr_cb_t cb) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  dyn_objects_fifo_t *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /* Populate the object with information from request. */
  object->mgr_cb = cb;

  /* The user CB is set when the RTO is created. */
  /*
   * Submit the task to the queue.
   * The radio manager thread will process the request.
   */
  chFifoSendObjectAheadI(task_queue, object);
}

/**
 * @brief   Get a radio command task object.
 * @note    The radio manager will lock all objects when closing.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   timeout maximum time to wait for a task to be submitted.
 * @param[in]   rt      pointer to a task object pointer.
 *
 * @return  Status of the operation.
 * @retval  MSG_OK      an object has been fetched.
 * @retval  MSG_TIMEOUT an object could not be obtained within the timeout.
 * @retval  MSG_RESET   the radio manager is closed or closing.
 *
 * @api
 */
msg_t pktGetRadioTaskObject(const radio_unit_t radio,
                            const sysinterval_t timeout,
                            const radio_params_t *rp,
                            radio_task_object_t **rt) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  dyn_objects_fifo_t *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  chSysLock();

  /* FIFO objects are not to be taken. The service is being closed. */
  if (chBSemGetStateI(&handler->mgr_sem)) {
    return MSG_RESET;
  }

  /* Request an RT object. */
  if ((*rt = chFifoTakeObjectTimeoutS(task_queue, timeout)) == NULL) {
    /* No object available. */
    chSysUnlock();
    return MSG_TIMEOUT;
  }

  /* Increment the FIFO ref count. */
  chFactoryDuplicateReference(&task_fifo->element);
  chSysUnlock();

  /* Clear the object then add base data. */
  memset(*rt, 0, sizeof(radio_task_object_t));
  /* Set defaults in RT object. */
  (*rt)->handler = handler;
  (*rt)->result = MSG_OK;
  (*rt)->mgr_cb = NULL;
  (*rt)->thread = NULL;
  if (rp != NULL) {
    /* Copy radio params if specified. */
    (*rt)->radio_dat = *rp;
  }
  return MSG_OK;
}

/**
 * @brief   Submit a radio command to the task manager.
 * @post    A task object is populated and submitted to the radio manager.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   object  radio task object to be submitted.
 * @param[in]   cb      radio manager internal call back (can be NULL).
 *
 * @api
 */
void pktSubmitRadioTask(const radio_unit_t radio,
                         radio_task_object_t *object,
                         const radio_mgr_cb_t cb) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  dyn_objects_fifo_t *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /* Update object information. */
  object->mgr_cb = cb;
  /* Submit the task to the queue.
     The radio manager thread will process the request. */
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
                         const radio_mgr_cb_t cb) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  dyn_objects_fifo_t *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /* Populate the object with information from request. */

  object->mgr_cb = cb;

  /*
   * Submit the task to the queue.
   * The radio manager thread will process the request.
   */
  chFifoSendObjectAhead(task_queue, object);
}

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
 * @retval MSG_RESET    if a resource can not be used due to a semaphore reset.
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
    if ((msg = chBSemWaitTimeout(&handler->radio_sem, timeout)) == MSG_OK) {
      if (timeout == TIME_IMMEDIATE || timeout == TIME_INFINITE)
        remainder = timeout;
      else
        remainder = timeout - chTimeDiffX(now, chVTGetSystemTimeX());
      if ((msg = OV5640_PDCMIlock(remainder)) != MSG_OK)
        /* If PDCMI lock failed then release the radio lock. */
        chBSemSignal(&handler->radio_sem);
    }
    break;
  }

  case RADIO_RX:
    msg = chBSemWaitTimeout(&handler->radio_sem, timeout);
    break;
  } /* End switch. */
  if (msg == MSG_OK)
    handler->lock_mode = mode;
  return msg;
}

/**
 * @brief   Unlock radio.
 * @notes   Returns when radio unit is unlocked.
 * @pre     Receive should be resumed by calling routine if it was active and mode is TX.
 *
 * @param[in] radio    radio unit ID.
 * @api
 */
void pktUnlockRadio(const radio_unit_t radio/*, const radio_mode_t mode*/) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  switch(handler->lock_mode) {
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
  if (str != NULL)
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
  if (band != NULL)
    return conf_sram.freq;

  /* System config frequency not supported by this radio. */
  const radio_config_t *radio_data = pktGetRadioData(radio);

  /*
   * Check if the radio has a valid default set.
   * Could be set to 0 or maybe there is an incorrect configuration.
  */
  if (pktCheckAllowedFrequency(radio, radio_data->def_aprs))
    /* Use default APRS frequency in radio configuration. */
    return radio_data->def_aprs;

  /* Fall back to defined default as last resort. */
  if (pktCheckAllowedFrequency(radio, DEFAULT_OPERATING_FREQ))
    /* Use default APRS frequency in radio configuration. */
    return DEFAULT_OPERATING_FREQ;

  /* None of the options are supported on this radio. */
  return FREQ_INVALID;
}

/**
 * @brief   Get current absolute receive operating frequency.
 *
 * @param[in] radio         Radio unit ID.
 *
 * @return    Receive frequency.
 * @retval    Absolute current receive frequency.
 * @retval    Default operating frequency if receive not active.
 *
 * @notapi
 */
radio_freq_hz_t pktGetAbsoluteReceiveFrequency(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  radio_freq_hz_t op_freq;

  op_freq = pktComputeOperatingFrequency(radio,
                                         handler->radio_rx_config.base_frequency,
                                         handler->radio_rx_config.step_hz,
                                         handler->radio_rx_config.channel,
                                         RADIO_RX);
  return op_freq;
}

/**
 * @brief   Get current receive operating frequency or frequency code.
 *
 * @param[in] radio         Radio unit ID.
 *
 * @return    Actual operating frequency or special code.
 * @retval    Absolute receive frequency or code if receive is active.
 * @retval    Default operating frequency if receive not active.
 *
 * @notapi
 */
static radio_freq_hz_t pktGetReceiveOperatingFrequencyOrCode(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  radio_freq_hz_t op_freq;
  if (pktIsReceiveEnabled(radio)) {
    if (handler->radio_rx_config.base_frequency < FREQ_CODES_END)
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
    if (list->unit == radio) {
      uint8_t x = 0;
      while(list->bands[x] != NULL) {
        if (list->bands[x]->start <= freq
            && freq < list->bands[x]->end)
          return list->bands[x];
        /* Next band. */
        x++;
      } /* End for bands */
    } /* if (!unit == radio) */
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
    if (pktCheckAllowedFrequency(radio_data->unit, op_freq)) {
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
	  if (radio_list[i].unit == radio)
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
 * @brief   Set receive streaming to standby.
 * @pre     The radio must be locked before calling this function.
 * @notes   Will wait a timeout for receive in progress before setting standby.
 * @notes   Called by radio transmit threads and the radio manager.
 * @notes   Receive completion relates with the receiver front end.
 * @notes   The receive stream will be stopped if activity continues past timeout.
 * @notes   Decoding of currently buffered data will continue and may complete.
 * @post    The decoder is not stopped. It waits for a new PWM stream.
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
msg_t pktSetReceiveStreamStandby(const radio_unit_t radio,
                                  const sysinterval_t timeout) {
  chDbgAssert(pktGetRadioLockStatus(radio), "radio not locked");
  if (!pktIsReceiveEnabled(radio))
    return MSG_IDLE;

  packet_svc_t *handler = pktGetServiceObject(radio);
  msg_t msg = MSG_OK;

  /*
   * This function can be called from TX threads.
   * Get the RX link type from the handler (RPKTDx).
   *
   * The radio is locked prior to invoking this function.
   * This is done so the outer has control of the radio.
   */
  switch(handler->rx_link_type) {
  case MOD_AFSK: {
    msg = pktWaitPWMStreamClose(radio, timeout);

    /*
     * The radio must be locked prior to disabling the stream.
     * Actions:
     * 1. Stop transport layer stream data.
     * 2. Write an in-stream stop message to an open stream.
     * 3. The radio device is put into standby state.
     *
     * The AFSK decoder can/will process any currently buffered PWM data.
     * - If the frame is incomplete the decoder will see an in-stream stop message.
     * - In such case the packet is dropped and the decoder resets.
     * - Where an in-stream stop message is after the HDLC frame close it is never seen.
     * - The decoder is able to complete processing of the buffered packet.
     */
    pktDisableRadioStreamProcessing(radio);

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

  if ((base_freq == FREQ_RX_APRS || base_freq == FREQ_SCAN)
                   && mode == RADIO_TX) {
    /* Get current RX frequency (or default) and use that. */
    step = 0;
    chan = 0;
    /* FIXME: Should switch on all special codes for error check. */
    base_freq = pktGetReceiveOperatingFrequencyOrCode(radio);
  }

  /*
   * Check for dynamic frequency determination.
   * Dynamic can return an absolute frequency or a further special code.
   */
  if (base_freq == FREQ_GEOFENCE) {
    /*
     * Get frequency by geofencing.
     * Geofencing can return special code FREQ_APRS_DEFAULT.
     */
    base_freq = getAPRSRegionFrequency();
    step = 0;
    chan = 0;
  }

  if (base_freq == FREQ_INVALID) { // Geofence not resolved
    base_freq = pktGetDefaultOperatingFrequency(radio);
    step = 0;
    chan = 0;
  }

  /* Calculate operating frequency. */
  radio_freq_hz_t op_freq = base_freq + (step * chan);

  if (pktCheckAllowedFrequency(radio, op_freq) != NULL) {
    return op_freq;
  }
  return FREQ_INVALID;
}

/**
 * @brief   Called by transmit threads to advise radio manager.
 * @post    A transmit end task is posted to the radio manager queue.
 *
 * @param[in]   rto     reference to radio task object.
 *
 * @api
 */
void pktRadioSendComplete(radio_task_object_t *const rto) {

  radio_unit_t radio = rto->handler->radio;
  /*
   *  The TX thread has scheduled self terminate using idle terminator.
   *  The RTO will be freed by RM.
   */
  rto->command = PKT_RADIO_TX_DONE;
  /* Submit guaranteed to succeed by design. */
  pktSubmitPriorityRadioTask(radio, rto, NULL);
}

/**
 * Check if a packet is in process of being received by the radio.
 * True if yes, false if no.
 *
 */
bool pktRadioGetInProgress(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);

  if (!pktIsReceiveEnabled(radio)) {
    return false;
  }

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
    return pktIsAFSKReceiveActive(handler);

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
  if (inds == NULL)
    /* Radio has no indicators. */
    return;
  do {
    if (inds->ind != ind)
      /* This is not the indicator specified. */
      continue;
    switch(inds->type) {
    case PKT_IND_GPIO_LINE: {
      if (inds->address.line != PAL_NOLINE)
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
  if (inds == NULL)
    return;
  do {
    if (inds->ind != ind)
      /* This is not the indicator specified. */
      continue;
    switch(inds->type) {
    case PKT_IND_GPIO_LINE: {
      if (inds->address.line != PAL_NOLINE)
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
  if (inds == NULL)
    /* Radio has no indicators. */
    return;
  do {
    if (inds->ind != ind)
      /* This is not the indicator specified. */
      continue;
    switch(inds->type) {
    /* Switch on output type. */
    case PKT_IND_GPIO_LINE: {
      if (inds->address.line == PAL_NOLINE)
        continue;
      if (val == PAL_TOGGLE)
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

  /* Stop ICU. */
  Si446x_detachPWM(radio);
}

#if 0
/**
 * Should configure and start PWM (ICU in case of AFSK)
 */
const ICUConfig *pktLLDradioConfigStream(const radio_unit_t radio,
                                         const radio_mod_t mod,
                                         const palcallback_t cb) {
  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only type at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */

  return Si446x_enablePWMevents(radio, mod, cb);
}
#endif
/**
 * @brief Enable the radio CAA.
 * @post The CCA event is enabled.
 */
void pktLLDradioCCAEnable(const radio_unit_t radio,
                                         const radio_mod_t mod,
                                         const palcallback_t cb) {

  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only type at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */

  Si446x_enablePWMevents(radio, mod, cb);
  /* TODO: Enable Si446x RSSI interrupt. */
}

/**
 * @brief Disable the radio PWM stream.
 * @post The PWM handlers are stopped.
 */
void pktLLDradioCCADisableI(const radio_unit_t radio,
                               const radio_mod_t mod) {

  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only one at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */

  Si446x_disablePWMeventsI(radio, mod);
}

/** @} */
