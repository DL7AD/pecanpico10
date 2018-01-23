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

packet_rx_t RPKTD1;

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
 * @brief   Initializes all packet handlers.
 * @note    The option to manage multiple handlers is not yet implemented.
 *
 *
 * @api
 */
void pktInitReceiveChannels() {
  RPKTD1.state = PACKET_IDLE;
}

/**
 * @brief   Opens a packet channel.
 * @post    A reference to the packet handler is returned.
 * @post    The packet channel is initialized and ready to be started.
 *
 * @param[in] type          the encoding type (currently only AFSK).
 * @param[in] radio_config  pointer to a radio configuration.
 *
 * @return              The reference to the packet handler object.
 * @retval NULL         if resources are not available to open the channel.
 *
 * @api
 */
packet_rx_t *pktOpenReceiveChannel(encoding_type_t type,
                                   radio_config_t *radio_config) {
  /* TODO: Define radio config struct.
   * Function should return a status like MSG_OK, etc.
   */
  packet_rx_t *handler = &RPKTD1;

  chDbgCheck(handler->state == PACKET_IDLE);

  if(handler->state != PACKET_IDLE)
    return NULL;

  /* Set link encoding type. */
  handler->link_type = type;

  /* Create the dynamic objects FIFO for the packet data queue. */
  dyn_objects_fifo_t *dyn_fifo = chFactoryCreateObjectsFIFO("pkt_fifo",
                                        sizeof(pkt_data_fifo_t),
                                        NUMBER_PKT_FIFOS, sizeof(msg_t));

  chDbgAssert(dyn_fifo != NULL, "failed to create PKT objects FIFO");

  if(dyn_fifo == NULL)
    return NULL;

  /* Initialize the counting semaphore for free buffers. */
  chSemObjectInit(&handler->packet_sem, NUMBER_PKT_FIFOS);

  /* Save the factory FIFO. */
  handler->the_packet_fifo = dyn_fifo;

  /* Get the objects FIFO . */
  handler->packet_fifo_pool = chFactoryGetObjectsFIFO(dyn_fifo);

  chDbgAssert(handler->packet_fifo_pool != NULL, "no packet FIFO pool");

  if(handler->packet_fifo_pool == NULL)
    return NULL;

  /* Indicate no active packet buffer. */
  handler->active_packet_object = NULL;

  /* TODO: Should statistics be kept in main (user thread)? */
  handler->packet_count = 0;
  handler->frame_count = 0;
  handler->valid_count = 0;

  switch(type) {
    case DECODE_AFSK: {
      /*
       * Create the AFSK channel.
       * This also handles ICU init and creates the radio PWM FIFO.
       * The PWM FIFO is the communication channel between ICU and decoder.
       * Their may be sequential radio packets queued during decoding.
       */
      AFSKDemodDriver *driver = pktCreateAFSKDecoder(handler,
                                         radio_config->radio_id);

      chDbgCheck(driver != NULL);

      if(driver == NULL) {
        chFactoryReleaseObjectsFIFO(dyn_fifo);
        return NULL;
      }

      handler->link_controller = driver;

      /*
       * TODO: Configure the radio parameters, frequency, etc.
       */

      break;
    } /* End case. */

    case DECODE_FSK: {
      chFactoryReleaseObjectsFIFO(dyn_fifo);
      return NULL;
    }
  } /* End switch. */

  /*
   * Initialize the packet common event object.
   */
  chEvtObjectInit(pktGetEventSource(handler));

  /* Setup offboard red LED. */
  //palSetLineMode(LINE_RED_LED, PAL_MODE_OUTPUT_PUSHPULL);
  //palClearLine(LINE_RED_LED);
  handler->state = PACKET_OPEN;
  return handler;
}

/**
 * @brief   Starts a packet channel.
 * @pre     The packet channel must have been opened.
 * @post    The packet channel is running if it was stopped.
 *
 * @param[in] handler       pointer to a @p packet handler object.
 *
 * @return              Status of the operation.
 * @retval MSG_OK       if the channel was started.
 * @retval MSG_RESET    if the channel was not in the correct state.
 * @retval MSG_TIMEOUT  if the channel could not be started or is invalid.
 *
 * @api
 */
msg_t pktStartDataReception(packet_rx_t *handler) {

  if(!(handler->state == PACKET_OPEN || handler->state == PACKET_STOP))
    return MSG_RESET;
  switch(handler->link_type) {
      case DECODE_AFSK: {
        thread_t *the_decoder =
            ((AFSKDemodDriver *)handler->link_controller)->decoder_thd;
        chEvtSignal(the_decoder, EVT_DECODER_START);
        handler->state = PACKET_RUN;
        return MSG_OK;
      } /* End case. */

      case DECODE_FSK: {
        break;
      }
    } /* End switch. */
  return MSG_TIMEOUT;
}

/**
 * @brief   Stop reception.
 * @notes   Decoding is stopped.
 * @notes   Any packets out for processing remain in effect.
 * @pre     The packet channel must have been started.
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
msg_t pktStopDataReception(packet_rx_t *handler) {
  if(handler->state != PACKET_RUN)
    return MSG_RESET;
  switch(handler->link_type) {
    case DECODE_AFSK: {
      thread_t *the_decoder =
          ((AFSKDemodDriver *)handler->link_controller)->decoder_thd;
      chEvtSignal(the_decoder, EVT_DECODER_STOP);
      handler->state = PACKET_STOP;
      return MSG_OK;
    } /* End case. */

    case DECODE_FSK: {
      break;
    }
  } /* End switch. */
  return MSG_TIMEOUT;
}

/**
 * @brief   Closes a packet channel.
 * @pre     The packet channel must have been stopped.
 * @post    The packet channel is closed.
 * @post    Memory used by the decoder thread is released.
 *
 * @param[in] handler       pointer to a @p packet handler object.
 *
 * @return              Status of the operation.
 * @retval MSG_OK       if the channel was closed.
 * @retval MSG_RESET    if the channel was not in the correct state.
 * @retval MSG_TIMEOUT  if the channel could not be closed.
 *
 * @api
 */
msg_t pktCloseReceiveChannel(packet_rx_t *handler) {

  if(handler->state != PACKET_STOP)
    return MSG_RESET;
  handler->state = PACKET_CLOSE;
  switch(handler->link_type) {
  case DECODE_AFSK: {
    thread_t *decoder =
        ((AFSKDemodDriver *)(handler->link_controller))->decoder_thd;
    chEvtSignal(decoder, EVT_PKT_CHANNEL_CLOSE);
    /* Wait for AFSK thread to terminate. */
    msg_t msg = chThdWait(decoder);

    /* Wait for all buffers to be released. */
    chSemWait(&handler->packet_sem);

    /* Destroy the dynamic objects FIFO for the packet data queue. */
    chFactoryReleaseObjectsFIFO(handler->the_packet_fifo);

    handler->the_packet_fifo = NULL;
    handler->packet_fifo_pool = NULL;

    handler->state = PACKET_IDLE;

    /*
     * TODO: Shut down the radio.
     */

    return msg;
    }

  case DECODE_FSK: {
    break;
    }
  }
  return MSG_TIMEOUT;
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
bool pktStoreBufferData(pkt_data_fifo_t *pkt_buffer, ax25char_t data) {
  /* Check if the data needs to be escaped if it is HDLC control. */
  if(data == HDLC_FLAG || data == HDLC_RESET || data == AX25_ESC) {
    if((pkt_buffer->packet_size + 2U) >= pkt_buffer->buffer_size) {
      /* Buffer full. */
      return false;
    }
    pkt_buffer->buffer[pkt_buffer->packet_size++] = AX25_ESC;
  }
  if((pkt_buffer->packet_size + 1U) >= pkt_buffer->buffer_size) {
    /* Buffer full. */
    return false;
  }
  /* Buffer space available. */
  pkt_buffer->buffer[pkt_buffer->packet_size++] = data;
  return true;
}

/** @} */
