/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
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
 * @brief   Initializes packet handlers and starts the radio manager.
 * @note    The option to manage multiple radios is not yet implemented.
 * @note    Once initialized the transmit service is available.
 * @note    To activate receive requires an open to be made.
 *
 * @param[in]   radio unit ID.
 *
 *@return   result of operation.
 *@retval   true    service was created.
 *@retval   false   service creation failed or state was not idle.
 *
 * @api
 */
bool pktServiceCreate(radio_unit_t radio) {

  /*
   * Get service object maps radio IDs to service objects
   */
  packet_svc_t *handler = pktGetServiceObject(radio);
  if(handler == NULL)
    return false;

  if(handler->state != PACKET_IDLE)
    return false;
  /*
   * Initialize the packet common event object.
   */
  chEvtObjectInit(pktGetEventSource(handler));

  memset(&handler->radio_rx_config, 0, sizeof(radio_task_object_t));
  memset(&handler->radio_tx_config, 0, sizeof(radio_task_object_t));

  /* Set flags and radio ID. */
  handler->rx_active = false;
  handler->radio_init = false;
  handler->radio = radio;

  /* Set service semaphore to idle state. */
  chBSemObjectInit(&handler->close_sem, false);

  /* Set radio semaphore to free state. */
  chBSemObjectInit(&handler->radio_sem, false);

  /* Send request to create radio manager. */
  if (pktRadioManagerCreate(radio) == NULL)
    return false;
  handler->state = PACKET_READY;
  return true;
}

/**
 * @brief   Releases packet service.
 * @note    The option to manage multiple radios is not yet implemented.
 * @post    The packet service is no longer available for transmit or receive.
 *
 * @param[in] radio unit ID
 *
 *@return   result of operation.
 *@retval   true    service was released.
 *@retval   false   service state is incorrect or invalid radio ID.
 *
 * @api
 */
bool pktServiceRelease(radio_unit_t radio) {

  /*
   * Lookup radio and assign handler (RPKTDx).
   */
  packet_svc_t *handler = pktGetServiceObject(radio);
  if(handler == NULL)
    return false;

  if(handler->state != PACKET_READY)
    return false;
  pktRadioManagerRelease(radio);
  handler->state = PACKET_IDLE;
  return true;
}

/**
 * @brief   Initializes packet handlers and starts the radio manager.
 * @note    The option to manage multiple radios is not yet implemented.
 * @note    Once initialized the transmit service is available.
 * @note    To activate receive requires an open to be made.
 *
 * @param[in]   radio unit ID.
 *
 *@return   result of operation.
 *@retval   true    service was created.
 *@retval   false   service creation failed or state was not idle.
 *
 * @api
 */
bool pktServiceHibernate(radio_unit_t radio) {

  /*
   * Get service object maps radio IDs to service objects
   */
  packet_svc_t *handler = pktGetServiceObject(radio);
  if(handler == NULL)
    return false;

  if(handler->state != PACKET_IDLE)
    return false;
  /*
   * Initialize the packet common event object.
   */
  chEvtObjectInit(pktGetEventSource(handler));

  memset(&handler->radio_rx_config, 0, sizeof(radio_task_object_t));
  memset(&handler->radio_tx_config, 0, sizeof(radio_task_object_t));

  /* Set flags and radio ID. */
  handler->rx_active = false;
  handler->radio_init = false;
  handler->radio = radio;

  /* Set service semaphore to idle state. */
  chBSemObjectInit(&handler->close_sem, false);

  /* Set radio semaphore to free state. */
  chBSemObjectInit(&handler->radio_sem, false);

  /* Send request to create radio manager. */
  if (pktRadioManagerCreate(radio) == NULL)
    return false;
  handler->state = PACKET_READY;
  return true;
}

/**
 * @brief   Releases packet service.
 * @note    The option to manage multiple radios is not yet implemented.
 * @post    The packet service is no longer available for transmit or receive.
 *
 * @param[in] radio unit ID
 *
 *@return   result of operation.
 *@retval   true    service was released.
 *@retval   false   service state is incorrect or invalid radio ID.
 *
 * @api
 */
bool pktServiceWakeup(radio_unit_t radio) {

  /*
   * Lookup radio and assign handler (RPKTDx).
   */
  packet_svc_t *handler = pktGetServiceObject(radio);
  if(handler == NULL)
    return false;

  if(handler->state != PACKET_READY)
    return false;
  pktRadioManagerRelease(radio);
  handler->state = PACKET_IDLE;
  return true;
}

/**
 * @brief   Opens a packet receive service.
 * @post    The packet service is initialized and ready to be started.
 *
 * @param[in] radio     radio unit identifier.
 * @param[in] encoding  radio link level encoding.
 * @param[in] frequency operating frequency (in Hz).
 * @param[in] ch_step   frequency step per channel (in Hz).
 *
 * @return              status of operation.
 * @retval MSG_OK       if the open request was processed.
 * @retval MSG_TIMEOUT  if the open request timed out waiting for resources.
 * @retval MSG_RESET    if state is invalid or bad parameter is submitted.
 *
 * @api
 */
msg_t pktOpenRadioReceive(radio_unit_t radio,
                           encoding_type_t encoding,
                           radio_freq_t frequency,
                           channel_hz_t ch_step) {

  packet_svc_t *handler = pktGetServiceObject(radio);
  if(handler == NULL)
    return MSG_RESET;

  chDbgCheck(handler->state == PACKET_READY);

  if(handler->state != PACKET_READY)
    return MSG_RESET;

  /* Wait for any prior session to complete closing. */
  chBSemWait(&handler->close_sem);

  /* Save radio configuration. */
  handler->radio_rx_config.type = encoding;
  handler->radio_rx_config.base_frequency = frequency;
  handler->radio_rx_config.step_hz = ch_step;

  /* Reset the statistics collection variables. */
  handler->sync_count = 0;
  handler->frame_count = 0;
  handler->valid_count = 0;
  handler->good_count = 0;

  radio_task_object_t rt = handler->radio_rx_config;

  /* Set parameters for radio command. */
  rt.command = PKT_RADIO_RX_OPEN;

  /*
   * Open (init) the radio (via submit radio task).
   */
  msg_t msg = pktSendRadioCommand(radio, &rt);

  if(msg != MSG_OK)
    return msg;

  handler->state = PACKET_OPEN;
  pktAddEventFlags(handler, EVT_PKT_CHANNEL_OPEN);

  return MSG_OK;
}

/**
 * @brief   Starts packet reception.
 * @pre     The packet service must have been opened.
 * @post    The radio is tuned to the specified channel.
 * @post    The packet reception is running if it was stopped.
 *
 * @param[in]   handler pointer to a @p packet handler object.
 * @param[in]   channel radio channel number to select
 * @param[in]   sq      the RSSI setting to be used.
 * @param[in]   cb      callback function called on receipt of packet.
 *
 * @return              Status of the operation.
 * @retval MSG_OK       if the service was started.
 * @retval MSG_RESET    parameter error or service not in correct state.
 * @retval MSG_TIMEOUT  if the service could not be started.
 *
 * @api
 */
msg_t pktStartDataReception(radio_unit_t radio,
                            radio_ch_t channel,
                            radio_squelch_t sq,
                            pkt_buffer_cb_t cb) {

  packet_svc_t *handler = pktGetServiceObject(radio);
  if(handler == NULL)
    return MSG_RESET;

  if(!(handler->state == PACKET_OPEN || handler->state == PACKET_STOP))
    return MSG_RESET;

  handler->usr_callback = cb;

  handler->radio_rx_config.channel = channel;
  handler->radio_rx_config.squelch = sq;

  radio_task_object_t rt = handler->radio_rx_config;

  rt.command = PKT_RADIO_RX_START;

  msg_t msg = pktSendRadioCommand(radio, &rt);
  if(msg != MSG_OK)
    return MSG_TIMEOUT;

  handler->state = PACKET_RUN;
  pktAddEventFlags(handler, EVT_PKT_DECODER_START);
  return MSG_OK;
}

/**
 * @brief   Starts a packet decoder.
 * @pre     The packet channel must have been opened.
 * @post    The packet decoder is running.
 *
 * @param[in]   radio unit ID.
 *
 * @api
 */
void pktStartDecoder(radio_unit_t radio) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  chDbgAssert(handler != NULL, "invalid radio ID");

  event_listener_t el;
  event_source_t *esp;

  switch(handler->radio_rx_config.type) {
    case MOD_AFSK: {

      esp = pktGetEventSource((AFSKDemodDriver *)handler->link_controller);

      pktRegisterEventListener(esp, &el, USR_COMMAND_ACK, DEC_START_EXEC);

      thread_t *the_decoder =
          ((AFSKDemodDriver *)handler->link_controller)->decoder_thd;
      chEvtSignal(the_decoder, DEC_COMMAND_START);
      break;
    } /* End case. */

    case MOD_2FSK: {
      return;
    }

    default:
      return;
  } /* End switch. */

  /* Wait for the decoder to start. */
  eventflags_t evt;
  do {
    /* In reality this is redundant as the only masked event is START. */
    chEvtWaitAny(USR_COMMAND_ACK);

    /* Wait for correct event at source.
     */
    evt = chEvtGetAndClearFlags(&el);
  } while (evt != DEC_START_EXEC);
  pktUnregisterEventListener(esp, &el);
}

/**
 * @brief   Stop reception.
 * @notes   Decoding is stopped.
 * @notes   Any packets out for processing remain in effect.
 * @pre     The packet channel must be running.
 * @post    The packet channel is stopped.
 *
 * @param[in] radio     radio unit ID..
 *
 * @return              Status of the operation.
 * @retval MSG_OK       if the channel was stopped.
 * @retval MSG_RESET    if the channel was not in the correct state.
 * @retval MSG_TIMEOUT  if the channel could not be stopped or is invalid.
 *
 * @api
 */
msg_t pktStopDataReception(radio_unit_t radio) {


  packet_svc_t *handler = pktGetServiceObject(radio);

  if(handler == NULL)
    return MSG_RESET;

  if(handler->state != PACKET_RUN)
    return MSG_RESET;

  /* Stop the radio processing. */

  radio_task_object_t rt = handler->radio_rx_config;

  rt.command = PKT_RADIO_RX_STOP;

  msg_t msg = pktSendRadioCommand(radio, &rt);
  if(msg != MSG_OK)
    return msg;

  handler->state = PACKET_STOP;
  pktAddEventFlags(handler, EVT_PKT_CHANNEL_STOP);
  return MSG_OK;
}

/**
 * @brief   Stops a packet decoder.
 * @pre     The packet channel must be running.
 * @post    The packet decoder is stopped.
 *
 * @param[in]   radio unit ID.
 *
 * @api
 */
void pktStopDecoder(radio_unit_t radio) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  if(handler == NULL)
    chDbgAssert(false, "invalid radio ID");

  event_listener_t el;
  event_source_t *esp;

  switch(handler->radio_rx_config.type) {
    case MOD_AFSK: {
      esp = pktGetEventSource((AFSKDemodDriver *)handler->link_controller);

      pktRegisterEventListener(esp, &el, USR_COMMAND_ACK, DEC_STOP_EXEC);

      thread_t *the_decoder =
          ((AFSKDemodDriver *)handler->link_controller)->decoder_thd;
      chEvtSignal(the_decoder, DEC_COMMAND_STOP);
      break;
    } /* End case. */

    case MOD_2FSK: {
      return;
    }

    default:
      return;
  } /* End switch. */

  /* Wait for the decoder to stop. */
  eventflags_t evt;
  do {
    chEvtWaitAny(USR_COMMAND_ACK);

    /* Wait for correct event at source.
     */
    evt = chEvtGetAndClearFlags(&el);
  } while (evt != DEC_STOP_EXEC);
  pktUnregisterEventListener(esp, &el);
}

/**
 * @brief   Closes a packet receive service.
 * @pre     The packet service must have been stopped.
 * @post    The packet service is closed and returned to ready state.
 * @post    Memory used by the decoder thread is released.
 *
 * @param[in] radio     radio unit ID.
 *
 * @return              Status of the operation.
 * @retval MSG_OK       if the service was closed successfully.
 * @retval MSG_RESET    service not in the correct state or invalid parameter.
 * @retval MSG_TIMEOUT  if the service could not be closed.
 *
 * @api
 */
msg_t pktCloseRadioReceive(radio_unit_t radio) {

  packet_svc_t *handler = pktGetServiceObject(radio);
  if(handler == NULL)
    return MSG_RESET;

  if(!(handler->state == PACKET_STOP || handler->state == PACKET_CLOSE))
    return MSG_RESET;

  handler->state = PACKET_CLOSE;

  /* Set parameters for radio. */;

  radio_task_object_t rt = handler->radio_rx_config;

  rt.command = PKT_RADIO_RX_CLOSE;

  /* Submit command. A timeout can occur waiting for a command queue object. */
  msg_t msg = pktSendRadioCommand(radio, &rt);
  if(msg != MSG_OK)
    return msg;

  pktAddEventFlags(handler, EVT_PKT_CHANNEL_CLOSE);
  handler->state = PACKET_READY;
  return MSG_OK;
}

/**
 * @brief   Stores data in a packet channel buffer.
 * @notes   If the data is an HDLC value it will be escape encoded.
 * @post    The character is stored and the internal buffer index is updated.
 *
 * @param[in] pkt_buffer    pointer to a @p packet buffer object.
 * @param[in] data          the character to be stored
 *
 * @return              Status of the operation.
 * @retval true         The data was stored.
 * @retval false        The data could not be stored (buffer full).
 *
 * @api
 */
bool pktStoreBufferData(pkt_data_object_t *pkt_buffer, ax25char_t data) {
  if((pkt_buffer->packet_size + 1U) > pkt_buffer->buffer_size) {
    /* Buffer full. */
    return false;
  }
  /* Buffer space available. */
  pkt_buffer->buffer[pkt_buffer->packet_size++] = data;
  return true;
}

/**
 * @brief   Dispatch a received buffer object.
 * @notes   The buffer is checked to determine validity and CRC.
 * @post    The buffer status is updated in the packet FIFO.
 * @post    Packet quality statistics are updated.
 * @post    Where no callback is used the buffer is posted to the FIFO mailbox.
 * @post    Where a callback is used a thread is created to execute the callback.
 *
 * @param[in] pkt_buffer    pointer to a @p packet buffer object.
 *
 * @return  Status flags added after packet validity check.
 *
 * @api
 */
eventflags_t pktDispatchReceivedBuffer(pkt_data_object_t *pkt_buffer) {

  chDbgAssert(pkt_buffer != NULL, "no packet buffer");

  packet_svc_t *handler = pkt_buffer->handler;

  chDbgAssert(handler != NULL, "invalid handler");

  eventflags_t flags = EVT_NONE;
  handler->frame_count++;
  if(pktIsBufferValidAX25Frame(pkt_buffer)) {
    handler->valid_count++;
    uint16_t magicCRC =
        calc_crc16(pkt_buffer->buffer, 0,
                   pkt_buffer->packet_size);
    if(magicCRC == CRC_INCLUSIVE_CONSTANT)
        handler->good_count++;
    flags |= (magicCRC == CRC_INCLUSIVE_CONSTANT)
                ? EVT_AX25_FRAME_RDY
                : EVT_AX25_CRC_ERROR;
  } else {
    flags |= EVT_PKT_INVALID_FRAME;
  }

  /* Update status in packet buffer object. */
  pkt_buffer->status |= flags;

  objects_fifo_t *pkt_fifo = chFactoryGetObjectsFIFO(pkt_buffer->pkt_factory);

  chDbgAssert(pkt_fifo != NULL, "no packet FIFO");

  if(pkt_buffer->cb_func == NULL) {

    /* Send the packet buffer to the FIFO queue. */
    chFifoSendObject(pkt_fifo, pkt_buffer);
  } else {
    /* Schedule a callback. */
    thread_t *cb_thd = pktCreateBufferCallback(pkt_buffer);

    chDbgAssert(cb_thd != NULL, "failed to create callback thread");

    if(cb_thd == NULL) {
      /* Failed to create CB thread. Release buffer. Flag event. */
      chFifoReturnObject(pkt_fifo, pkt_buffer);
      flags |= EVT_PKT_FAILED_CB_THD;

    } else {
      /* Increase outstanding callback count. */
      handler->cb_count++;
    }

  }
  return flags;
}

/**
 * @brief   Create a callback processing thread.
 * @notes   Packet callbacks are processed by individual threads.
 * @notes   Thus packet callbacks are non-blocking to the decoder thread.
 * @notes   After callback completes the thread it is scheduled for release.
 * @notes   Release is initiated by posting the packet buffer to the queue.
 *
 * @post    Call back has been executed (for however long it takes).
 * @post    Callback thread release is completed in the terminator thread.
 *
 * @param[in] pkt_data_object_t    pointer to a @p packet buffer object.
 *
 * @return  The callback thread.
 *
 * @api
 */
thread_t *pktCreateBufferCallback(pkt_data_object_t *pkt_buffer) {

  chDbgAssert(pkt_buffer != NULL, "invalid packet buffer");

  /* Create a callback thread name which is the address of the buffer. */
  /* TODO: Create a more meaningful but still unique thread name. */
  chsnprintf(pkt_buffer->cb_thd_name, sizeof(pkt_buffer->cb_thd_name),
             "%x", pkt_buffer);

  /* Start a callback dispatcher thread. */
  thread_t *cb_thd = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(PKT_CALLBACK_WA_SIZE),
              pkt_buffer->cb_thd_name,
              NORMALPRIO - 20,
              pktCallback,
              pkt_buffer);

  return cb_thd;
}

/**
 * @brief   Run a callback processing thread.
 * @notes   Packet callbacks are processed by individual threads.
 * @notes   Thus packet callbacks are non-blocking to the decoder thread.
 * @notes   After callback completes the thread it is scheduled for release.
 * @notes   Release is initiated by posting the packet buffer to the queue.
 *
 * @post    Call back has been executed (for however long it takes).
 * @post    Callback thread release is completed in the terminator thread.
 *
 * @param[in] arg pointer to a @p packet buffer object.
 *
 * @return  status (MSG_OK).
 *
 * @notapi
 */
THD_FUNCTION(pktCallback, arg) {

  chDbgAssert(arg != NULL, "invalid buffer reference");

  pkt_data_object_t *pkt_buffer = arg;

  chDbgAssert(pkt_buffer->cb_func != NULL, "no callback set");

  dyn_objects_fifo_t *pkt_factory = pkt_buffer->pkt_factory;
  chDbgAssert(pkt_factory != NULL, "invalid packet factory reference");

  objects_fifo_t *pkt_fifo = chFactoryGetObjectsFIFO(pkt_factory);
  chDbgAssert(pkt_fifo != NULL, "no packet FIFO");

  /* Save thread pointer for use later in terminator. */
  pkt_buffer->cb_thread = chThdGetSelfX();

  /* Perform the callback. */
  pkt_buffer->cb_func(pkt_buffer);

  /*
   * Upon return buffer is queued for release.
   * Thread is scheduled for destruction in pktReleaseDataBuffer(...).
   * .i.e pktReleaseDataBuffer does not return for callback.
   */
  pktReleaseDataBuffer(pkt_buffer);
}

/**
 * @brief   Process release of completed callbacks.
 * @notes   Release is initiated by posting the packet buffer to the queue.
 * @notes   The queue is used as a completion mechanism in callback mode.
 * @notes   In poll mode the received packet is posted to the consumer
 *
 * @post    Call back thread has been released.
 * @post    Packet buffer object is returned to free pool.
 * @post    Packet object is released (for this instance).
 * @post    If the FIFO is now unused it will be released.
 *
 * @param[in] arg radio unit ID.
 *
 * @return  status (MSG_OK) on exit.
 *
 * @notapi
 */

/* TODO: Deprecate and use radio manager thread for callback release. */
THD_FUNCTION(pktCompletion, arg) {
  packet_svc_t *handler = arg;
#define PKT_COMPLETION_THREAD_TIMER 100 /* 100 mS. */

  chDbgAssert(handler != NULL, "invalid handler reference");

  dyn_objects_fifo_t *pkt_factory = handler->the_packet_fifo;
  objects_fifo_t *pkt_queue = chFactoryGetObjectsFIFO(pkt_factory);
  chDbgAssert(pkt_queue != NULL, "no packet fifo list");

  /* TODO: Implement thread events to control start/stop. */
  while(true) {

    /*
     * Wait for a callback to be outstanding.
     * If no callbacks outstanding check for termination request.
     */
    if(handler->cb_count == 0) {
      if(chThdShouldTerminateX())
        chThdExit(MSG_OK);
      chThdSleep(TIME_MS2I(PKT_COMPLETION_THREAD_TIMER));
      continue;
    }
    /* Wait for a buffer to be released. */
    pkt_data_object_t *pkt_object;

    msg_t fmsg = chFifoReceiveObjectTimeout(pkt_queue,
                         (void *)&pkt_object,
                         TIME_MS2I(PKT_COMPLETION_THREAD_TIMER));
    if(fmsg == MSG_TIMEOUT)
      continue;

    /* Release the callback thread and recover heap. */
    chThdRelease(pkt_object->cb_thread);

    /* Return packet buffer object to free list. */
    chFifoReturnObject(pkt_queue, (pkt_data_object_t *)pkt_object);

    /*
     * Decrease FIFO reference counter (increased by decoder).
     * FIFO will be destroyed if all references now released.
     */
    chFactoryReleaseObjectsFIFO(pkt_factory);

    /* Decrease count of outstanding callbacks. */
    --handler->cb_count;
  }
  chThdExit(MSG_OK);
}

void pktCallbackManagerOpen(radio_unit_t radio) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  chDbgAssert(handler != NULL, "invalid radio ID");

  /* Create the callback handler thread name. */
  chsnprintf(handler->cbend_name, sizeof(handler->cbend_name),
             "%s%02i", PKT_CALLBACK_TERMINATOR_PREFIX, radio);

  /* Start the callback thread terminator. */
  thread_t *cbh = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(PKT_TERMINATOR_WA_SIZE),
              handler->cbend_name,
              NORMALPRIO - 30,
              pktCompletion,
              handler);

  chDbgAssert(cbh != NULL, "failed to create callback terminator thread");
  handler->cb_terminator = cbh;
}

/*
 *
 */
dyn_objects_fifo_t *pktBufferManagerCreate(radio_unit_t radio) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  chDbgAssert(handler != NULL, "invalid radio ID");

  /* Create the packet buffer name for this radio. */
  chsnprintf(handler->pbuff_name, sizeof(handler->pbuff_name),
             "%s%02i", PKT_FRAME_QUEUE_PREFIX, radio);

  /* Check if the packet buffer factory is still in existence.
   * If so we get a pointer to it.
   */
  dyn_objects_fifo_t *dyn_fifo =
      chFactoryFindObjectsFIFO(handler->pbuff_name);

  if(dyn_fifo == NULL) {
    /* Create the dynamic objects FIFO for the packet data queue. */
    dyn_fifo = chFactoryCreateObjectsFIFO(handler->pbuff_name,
        sizeof(pkt_data_object_t),
        NUMBER_PKT_FIFOS, sizeof(msg_t));

    chDbgAssert(dyn_fifo != NULL, "failed to create PKT objects FIFO");

    if(dyn_fifo == NULL) {
      /* TODO: Close decoder on fail. */
      return NULL;
    }
  }

  /* Save the factory FIFO. */
  handler->the_packet_fifo = dyn_fifo;

  /* Initialize packet buffer pointer. */
  handler->active_packet_object = NULL;
  return dyn_fifo;
}


thread_t *pktCallbackManagerCreate(radio_unit_t radio) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  chDbgAssert(handler != NULL, "invalid radio ID");

  /* Create the callback termination thread name. */
  chsnprintf(handler->cbend_name, sizeof(handler->cbend_name),
             "%s%02i", PKT_CALLBACK_TERMINATOR_PREFIX, radio);

  /*
   * Initialize the outstanding callback count.
   */
  handler->cb_count = 0;

  /* Start the callback thread terminator. */
  thread_t *cbh = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(PKT_TERMINATOR_WA_SIZE),
              handler->cbend_name,
              NORMALPRIO - 30,
              pktCompletion,
              handler);

  chDbgAssert(cbh != NULL, "failed to create callback terminator thread");
  handler->cb_terminator = cbh;
  return cbh;
}

void pktBufferManagerRelease(packet_svc_t *handler) {

  /* Release the dynamic objects FIFO for the packet data queue. */
  chFactoryReleaseObjectsFIFO(handler->the_packet_fifo);
  handler->the_packet_fifo = NULL;
}

void pktCallbackManagerRelease(packet_svc_t *handler) {

  /* Tell the callback terminator it should exit. */
  chThdTerminate(handler->cb_terminator);

  /* Wait for it to terminate and release. */
  chThdWait(handler->cb_terminator);
}

/*void pktScheduleThreadRelease(thread_t *thread) {
  (void)thread;
}*/

/** @} */
