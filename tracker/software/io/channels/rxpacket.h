/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef IO_CHANNELS_RXPACKET_H_
#define IO_CHANNELS_RXPACKET_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/* Main thread events. */
#define EVT_DIAG_OUT_END    EVENT_MASK(0)
#define EVT_PKT_OUT_END     EVENT_MASK(1)

#define PKT_BUFFER_SIZE     AX25_MAX_PACKET_LEN

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/* Packet handler states. */
typedef enum packetHandlerStates {
  PACKET_IDLE = 0,
  PACKET_OPEN,
  PACKET_RUN,
  PACKET_STOP,
  PACKET_CLOSE
} packet_state_t;

/* HDLC frame states. */
typedef enum HDLCFrameStates{
  FRAME_SEARCH,
  FRAME_OPEN,
  FRAME_CLOSE,
  FRAME_RESET
} frame_state_t;

/* Link level encoding type. */
typedef enum streamEncodingTypes{
  DECODE_AFSK,
  DECODE_FSK
} encoding_type_t;


typedef struct packetBuffer {
  struct pool_header        link; /* For safety keep clear - where pool stores its free link. */
  volatile eventflags_t     status;
  size_t                    buffer_size;
  size_t                    packet_size;
  ax25char_t                buffer[PKT_BUFFER_SIZE];
#if USE_AFSK_PHASE_STATISTICS == TRUE
  dsp_phase_t correction;
  dsp_phase_t drift;
#endif
} pkt_data_fifo_t;

typedef struct packetHandlerData {
  packet_state_t            state;
  /**
   * @brief User thread (for events posted to user).
   */

  /**
   * @brief Pointer to link level protocol data.
   */
  void                      *link_controller;

  /**
   * @brief Link level protocol type.
   */
  encoding_type_t           link_type;

  /**
   * @brief AX25 packet guarded FIFO.
   */
  dyn_objects_fifo_t        *the_packet_fifo;

  /**
   * @brief AX25 packet FIFO pool.
   */
  objects_fifo_t            *packet_fifo_pool;

  /**
   * @brief Current active packet fifo object.
   */
  pkt_data_fifo_t           *active_packet_object;

  /**
   * @brief Semaphore for buffer counting.
   */
  semaphore_t               packet_sem;

  /**
   * @brief Event source object.
   */
  event_source_t            event;

  /**
   * @brief Event flags.
   */
  eventflags_t              status;

  /**
   * @brief Packet count.
   */
  uint16_t                  packet_count;

  /**
   * @brief Total frame count.
   */
  uint16_t                  frame_count;

  /**
   * @brief Good (CRC) frame count.
   */
  uint16_t                  valid_count;

  /**
   * @brief Opening HDLC flag sequence found.
   */
} packet_rx_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void pktInitReceiveChannels(void);
  packet_rx_t *pktOpenReceiveChannel(encoding_type_t type,
                                     radio_config_t *radio_config);
  msg_t pktStartDataReception(packet_rx_t *handler);
  msg_t pktStopDataReception(packet_rx_t *handler);
  msg_t pktCloseReceiveChannel(packet_rx_t *handler);
  bool pktStoreBufferData(pkt_data_fifo_t *buffer, ax25char_t data);
#ifdef __cplusplus
}
  #endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name    Macro Functions (packet demod drivers)
 * @{
 */

/**
 * @brief   Returns the I/O condition event source.
 * @details The event source is broadcast when an I/O condition happens.
 *
 * @param[in] ip        pointer to a @p packet demod driver
 *
 * @return              A pointer to an @p EventSource object.
 *
 * @api
 */
#define pktGetEventSource(ip) (&((ip)->event))

/**
 * @brief   Adds status flags to the listeners's flags mask.
 * @details This function is can be called from the thread level (locked) or
 *          I/O ISRs in order to notify I/O conditions such as
 *          data events, errors, signal changes etc.
 *
 * @param[in] ip        pointer to a @p packet handler object
 *
 * @param[in] flags     condition flags to be added to the listener flags mask
 *
 * @iclass
 */
#define pktAddEventFlagsI(ip, flags) {                                       \
  osalEventBroadcastFlagsI(&(ip)->event, flags);                             \
}

/**
 * @brief   Adds status flags to the listeners's flags mask.
 * @details This function is an alias for pktAddsFlagsI for convenience
 *
 * @param[in] ip        pointer to a @p packet handler object
 *
 * @param[in] flags     condition flags to be added to the listener flags mask
 *
 * @sclass
 */
#define pktAddEventFlagsS(ip, flags) pktAddEventFlagsI(ip, flags)

/**
 * @brief   Adds status flags to the listeners's flags mask.
 * @details This function is can be called from the thread level
 *          in order to notify I/O conditions such as
 *          data events, errors, signal changes etc.
 *
 * @param[in] ip        pointer to a @p packet handler object
 *
 * @param[in] flags     condition flags to be added to the listener flags mask
 *
 * @iclass
 */
#define pktAddEventFlags(ip, flags) {                                        \
  osalEventBroadcastFlags(&(ip)->event, flags);                              \
}

/**
 * @brief   Fetches a buffer from the packet buffer free pool.
 * @details This function is called from thread level to obtain a buffer
 *          to write AX25 data into.
 *
 * @param[in]   fifo        pointer to a @p objects FIFO
 *
 * @return      pointer to buffer object.
 * @retval      NULL if no buffer object available.
 *
 * @api
 */
static inline pkt_data_fifo_t *pktTakeDataBuffer(objects_fifo_t *fifo) {
  return (pkt_data_fifo_t *)chFifoTakeObjectTimeout(fifo, TIME_IMMEDIATE);
}

/**
 * @brief   Fetches a buffer from the packet buffer free pool.
 * @details This function is called from locked thread level to an AX25 buffer.
 *
 * @param[in]   fifo        pointer to a @p objects FIFO
 *
 * @return      pointer to packet buffer.
 * @retval      NULL if no buffer available.
 *
 * @sclass
 */
static inline pkt_data_fifo_t *pktTakeDataBufferS(objects_fifo_t *fifo) {
  return (pkt_data_fifo_t *)chFifoTakeObjectI(fifo);
}

/**
 * @brief   Returns a buffer to the packet buffer free pool.
 * @details This function is called from locked thread level to free a buffer.
 *
 * @param[in]   fifo        pointer to a @p objects FIFO.
 * @param[in]   object      pointer to the @p object to return.
 *
 * @api
 */
static inline void pktReturnDataBufferS(objects_fifo_t *fifo,
                                         pkt_data_fifo_t *object) {
  chFifoReturnObjectI(fifo, object);
}

/**
 * @brief   Returns a buffer to the packet buffer free pool.
 * @details This function is called from thread level to free a buffer.
 *
 * @param[in]   handler     pointer to a @p packet handler object.
 * @param[in]   object      pointer to a @p buffer object.
 *
 * @api
 */
static inline void pktReleaseDataBuffer(packet_rx_t *handler,
                                        pkt_data_fifo_t *object) {
  chSysLock();
  pktReturnDataBufferS(handler->packet_fifo_pool, object);

  /* Indicate that the packet consumer has released the buffer. */
  chSemSignalI(&handler->packet_sem);
  chSysUnlock();
}

/**
 * @brief   Resets the buffer index of a packet buffer.
 * @details This function is called from thread level to reset the buffer.
 *
 * @param[in]   object      pointer to the @p buffer to reset.
 *
 * @api
 */
static inline void pktResetDataBuffer(pkt_data_fifo_t *object) {
  object->packet_size = 0;
}

/**
 * @brief   Waits for a buffer to be posted to the FIFO and gets it.
 * @details This function is called from thread level to get a posted buffer.
 *
 * @param[in] handler   pointer to a @p packet handler object.
 * @param[in] timeout   the number of ticks before the operation times out.
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return              A pointer to the received object.
 * @retval MSG_TIMEOUT  if the specified time expired.
 *
 * @api
 */
static inline pkt_data_fifo_t *pktReceiveDataBufferTimeout(packet_rx_t *handler,
                                       sysinterval_t timeout) {

  objects_fifo_t *fifo = handler->packet_fifo_pool;
  pkt_data_fifo_t *object;
  msg_t fifo_msg = chFifoReceiveObjectTimeout(fifo,
                                              (void *)&object, timeout);
  return (fifo_msg == MSG_OK ? object : NULL);
}

/**
 * @brief   Checks if a buffer meets minimum AX25 validity.
 * @note    Validity relates to minimum size only.
 * @note    The frame may have a good or bad CRC.
 * @details This function is called from thread level.
 *
 * @param[in] object    pointer to a @p objects FIFO.
 *
 * @return              Status.
 * @retval MSG_OK       if the buffer meets minimum requirement for AX25 frame.
 * @retval MSG_TIMEOUT  if the minimum requirements are not met.
 *
 * @api
 */
static inline msg_t pktIsBufferValidAX25Frame(pkt_data_fifo_t *object) {
  chDbgAssert(object != NULL, "no pointer to packet object buffer");
  uint16_t frame_size = object->packet_size;
  if((object->status & EVT_AFSK_DECODE_DONE)
    && frame_size >= AX25_MIN_FRAME) {
    return MSG_OK;
  }
  return MSG_RESET;
}

#endif /* IO_CHANNELS_RXPACKET_H_ */

/** @} */
