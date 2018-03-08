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

packet_svc_t RPKTD1;

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
 *
 *
 * @api
 */
void pktServiceCreate() {

  /* TODO: This should create the top level object for each radio (RPKTDx). */
  packet_svc_t *handler = &RPKTD1;

  /*
   * Initialize the packet common event object.
   */
  chEvtObjectInit(pktGetEventSource(handler));

  memset(&handler->radio_config, 0, sizeof(radio_task_object_t));

  /* Set parameters and send request. */
  handler->radio_config.radio_id = PKT_RADIO_1;

  /* Set service semaphore to idle state. */
  chBSemObjectInit(&handler->close_sem, false);

  pktRadioManagerCreate(handler);
  handler->state = PACKET_IDLE;
}

void pktServiceRelease() {

  /* TODO: This should release top level resources for each radio (RPKTDx). */
  packet_svc_t *handler = &RPKTD1;

  pktRadioManagerRelease(handler);
}

/**
 * @brief   Opens a packet service.
 * @post    A reference to the packet handler is returned.
 * @post    The packet service is initialized and ready to be started.
 *
 * @param[in] radio     radio unit identifier.
 * @param[in] encoding  radio link level encoding.
 * @param[in] frequency operating frequency (in Hz).
 * @param[in] ch_step   frequency step per channel (in Hz).
 *
 * @return              the reference to the packet handler object.
 * @retval MSG_OK       if the open request was processed.
 * @retval MSG_TIMEOUT  if the open request timed out waiting for resources.
 * @retval MSG_RESET    if state is invalid or bad parameter is submitted.
 *
 * @api
 */
msg_t pktOpenRadioService(radio_unit_t radio,
                           encoding_type_t encoding,
                           radio_freq_t frequency,
                           channel_hz_t ch_step,
                           packet_svc_t **ph) {

  /* TODO: implement mapping from radio config to packet handler object.
   * TODO: implement channel step size in Hz in radio driver.
   */
  (void)radio;
  packet_svc_t *handler = &RPKTD1;

  chDbgCheck(handler->state == PACKET_IDLE);

  if(handler->state != PACKET_IDLE)
    return MSG_RESET;

  /* Wait for any prior session to complete closing. */
  chBSemWait(&handler->close_sem);

  /* Save radio configuration. */
  handler->radio_config.type = encoding;
  handler->radio_config.base_frequency = frequency;
  handler->radio_config.step_hz = ch_step;

  /* Reset the statistics collection variables. */
  handler->sync_count = 0;
  handler->frame_count = 0;
  handler->valid_count = 0;
  handler->good_count = 0;

  radio_task_object_t rt = handler->radio_config;

  /* Set parameters for radio command. */
  rt.command = PKT_RADIO_OPEN;

  /*
   * Open (init) the radio (via submit radio task).
   */
  msg_t msg = pktSendRadioCommand(handler, &rt);

  if(msg != MSG_OK)
    return msg;

  handler->state = PACKET_OPEN;
  pktAddEventFlags(handler, EVT_PKT_CHANNEL_OPEN);

  /* Set the pointer. */
  *ph = handler;
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
msg_t pktStartDataReception(packet_svc_t *handler,
                            radio_ch_t channel,
                            radio_squelch_t sq,
                            pkt_buffer_cb_t cb) {

  chDbgAssert(handler != NULL, "invalid handler reference");

  if(!(handler->state == PACKET_OPEN || handler->state == PACKET_STOP))
    return MSG_RESET;

  handler->usr_callback = cb;

  handler->radio_config.channel = channel;
  handler->radio_config.squelch = sq;

  radio_task_object_t rt = handler->radio_config;

  rt.command = PKT_RADIO_RX;

  msg_t msg = pktSendRadioCommand(handler, &rt);
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
 * @param[in]   handler pointer to a @p packet handler object.
 *
 * @api
 */
void pktStartDecoder(packet_svc_t *handler) {

  //chDbgAssert(handler->state == PACKET_RUN, "invalid handler state");

  event_listener_t el;
  event_source_t *esp;

  switch(handler->radio_config.type) {
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
 * @param[in] handler       pointer to a @p packet handler object.
 *
 * @return              Status of the operation.
 * @retval MSG_OK       if the channel was stopped.
 * @retval MSG_RESET    if the channel was not in the correct state.
 * @retval MSG_TIMEOUT  if the channel could not be stopped or is invalid.
 *
 * @api
 */
msg_t pktStopDataReception(packet_svc_t *handler) {
  if(handler->state != PACKET_RUN)
    return MSG_RESET;

  /* Stop the radio processing. */

  radio_task_object_t rt = handler->radio_config;

  rt.command = PKT_RADIO_RX_STOP;

  msg_t msg = pktSendRadioCommand(handler, &rt);
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
 * @param[in]   handler pointer to a @p packet handler object.
 *
 * @api
 */
void pktStopDecoder(packet_svc_t *handler) {

  //chDbgAssert(handler->state == PACKET_STOP, "invalid handler state");

  event_listener_t el;
  event_source_t *esp;

  switch(handler->radio_config.type) {
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
 * @brief   Closes a packet service.
 * @pre     The packet service must have been stopped.
 * @post    The packet service is closed.
 * @post    Memory used by the decoder thread is released.
 *
 * @param[in] handler       pointer to a @p packet handler object.
 *
 * @return              Status of the operation.
 * @retval MSG_OK       if the service was closed successfully.
 * @retval MSG_RESET    service not in the correct state or invalid parameter.
 * @retval MSG_TIMEOUT  if the service could not be closed.
 *
 * @api
 */
msg_t pktCloseRadioService(packet_svc_t *handler) {

  if(handler->state != PACKET_STOP)
    return MSG_RESET;

  handler->state = PACKET_CLOSE;

  /* Set parameters for radio. */;

  radio_task_object_t rt = handler->radio_config;

  rt.command = PKT_RADIO_CLOSE;

  /* Submit command. A timeout can occur waiting for a command queue object. */
  msg_t msg = pktSendRadioCommand(handler, &rt);
  if(msg != MSG_OK)
    return msg;

  handler->state = PACKET_IDLE;
  pktAddEventFlags(handler, EVT_PKT_CHANNEL_CLOSE);
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
    flags |= EVT_AFSK_INVALID_FRAME;
  }

  /* Update status in packet buffer object. */
  pkt_buffer->status |= flags;

  if(pkt_buffer->cb_func == NULL) {
    objects_fifo_t *pkt_fifo = chFactoryGetObjectsFIFO(pkt_buffer->pkt_factory);

    chDbgAssert(pkt_fifo != NULL, "no packet FIFO");

    /* Send the packet buffer to the FIFO queue. */
    chFifoSendObject(pkt_fifo, pkt_buffer);
  } else {
    /* Schedule a callback. */
    thread_t *cb_thd = pktCreateBufferCallback(pkt_buffer);

    chDbgAssert(cb_thd != NULL, "failed to create callback thread");

    /* Increase outstanding callback count. */
    handler->cb_count++;
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
 * @param[in] arg pointer to a @p packet service object.
 *
 * @return  status (MSG_OK) on exit.
 *
 * @notapi
 */
THD_FUNCTION(pktCompletion, arg) {
  packet_svc_t *handler = arg;
#define PKT_COMPLETION_THREAD_TIMER 100 /* 100 mS. */
  chDbgAssert(arg != NULL, "invalid handler reference");

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

void pktCallbackManagerOpen(packet_svc_t *handler) {

  radio_unit_t rid = handler->radio_config.radio_id;

  /* Create the callback handler thread name. */
  chsnprintf(handler->cbend_name, sizeof(handler->cbend_name),
             "%s%02i", PKT_CALLBACK_TERMINATOR_PREFIX, rid);

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

dyn_objects_fifo_t *pktBufferManagerCreate(packet_svc_t *handler) {
  /* The radio associated with this AFSK driver. */
  radio_unit_t rid = handler->radio_config.radio_id;

  /* Create the packet buffer name for this radio. */
  chsnprintf(handler->pbuff_name, sizeof(handler->pbuff_name),
             "%s%02i", PKT_FRAME_QUEUE_PREFIX, rid);

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


void pktCallbackManagerCreate(packet_svc_t *handler) {
  radio_unit_t rid = handler->radio_config.radio_id;

  /* Create the callback termination thread name. */
  chsnprintf(handler->cbend_name, sizeof(handler->cbend_name),
             "%s%02i", PKT_CALLBACK_TERMINATOR_PREFIX, rid);

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

/** @} */
