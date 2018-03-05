/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef PKT_CHANNELS_PKTSERVICE_H_
#define PKT_CHANNELS_PKTSERVICE_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/* Main thread events. */
#define EVT_DIAG_OUT_END        EVENT_MASK(0)
#define EVT_PKT_OUT_END         EVENT_MASK(1)

#define PKT_RX_BUFFER_SIZE      PKT_MAX_RX_PACKET_LEN

#define PKT_FRAME_QUEUE_PREFIX  "pktx_"
#define PKT_CALLBACK_TERMINATOR_PREFIX "cbtx_"

#define PKT_CALLBACK_WA_SIZE    4096
#define PKT_TERMINATOR_WA_SIZE  1024

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
typedef enum HDLCFrameStates {
  FRAME_SEARCH,
  FRAME_OPEN,
  FRAME_DATA,
  FRAME_CLOSE,
  FRAME_RESET
} frame_state_t;

/* Link level encoding type. */
typedef enum {
  DECODE_NOT_SET,
  DECODE_AFSK,
  DECODE_FSK
} encoding_type_t;

#ifdef PKT_IS_TEST_PROJECT
/* Modulation type. */
typedef enum {
    MOD_AFSK,
    MOD_2FSK
} mod_t;

#endif

#include "pktradio.h"

typedef struct packetBuffer pkt_data_object_t;
typedef void (*pkt_buffer_cb_t)(pkt_data_object_t *pkt_buffer);

typedef struct packetBuffer {
  struct pool_header        link; /* For safety keep clear - where pool stores its free link. */
  packet_svc_t              *handler;
  dyn_objects_fifo_t        *pkt_factory;
  thread_t                  *cb_thread;
  char                      cb_thd_name[sizeof(size_t) + 1];
  pkt_buffer_cb_t           cb_func;
  volatile eventflags_t     status;
  size_t                    buffer_size;
  size_t                    packet_size;
  ax25char_t                buffer[PKT_RX_BUFFER_SIZE];
} pkt_data_object_t;


typedef struct packetHandlerData {
  /**
   * @brief State of the packet handler.
   */
  packet_state_t            state;

  /**
   * @brief Radio operating parameters.
   */
  radio_task_object_t       radio_config;

  /**
   * @brief Pointer to link level protocol data.
   */
  void                      *link_controller;

  /**
   * @brief Link level protocol type.
   */
  //encoding_type_t           link_type;

  /**
   * @brief names for the factory FIFOs.
   * @notes packet buffer & radio task and callback
   */
  char                      pbuff_name[CH_CFG_FACTORY_MAX_NAMES_LENGTH];
  char                      rtask_name[CH_CFG_FACTORY_MAX_NAMES_LENGTH];
  char                      cbend_name[CH_CFG_FACTORY_MAX_NAMES_LENGTH];

  /**
   *  @brief Packet system service threads.
   */
  thread_t                  *radio_manager;
  thread_t                  *cb_terminator;

  /**
   * @brief Radio task guarded FIFO.
   */
  dyn_objects_fifo_t        *the_radio_fifo;

  /**
   * @brief AX25 packet guarded FIFO.
   */
  dyn_objects_fifo_t        *the_packet_fifo;

  /**
   * @brief Packet buffer cb_func.
   */
  pkt_buffer_cb_t           usr_callback;

  /**
   * @brief Current active packet fifo object.
   */
  pkt_data_object_t         *active_packet_object;


  /**
   * @brief Counter for active callback threads.
   * TODO: type should be of a generic counter?
   */
  uint8_t                  cb_count;

  /**
   * @brief Event source object.
   */
  binary_semaphore_t       close_sem;

  /**
   * @brief Event source object.
   */
  event_source_t            event;

  /**
   * @brief Packet count.
   */
  uint16_t                  sync_count;

  /**
   * @brief Statistics counters.
   */
  uint16_t                  frame_count;
  uint16_t                  good_count;
  uint16_t                  valid_count;
} packet_svc_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#include "pktradio.h"

#ifdef __cplusplus
extern "C" {
#endif
  void pktServiceCreate(void);
  void pktServiceRelease(void);
  msg_t pktOpenRadioService(radio_unit_t radio,
                                     encoding_type_t encoding,
                                     radio_freq_t frequency,
                                     channel_hz_t ch_step,
                                     packet_svc_t **handler);
  msg_t pktStartDataReception(packet_svc_t *handler,
                              radio_ch_t channel,
                              radio_squelch_t sq,
                              pkt_buffer_cb_t cb);
  void pktStartDecoder(packet_svc_t *handler);
  msg_t pktStopDataReception(packet_svc_t *handler);
  void pktStopDecoder(packet_svc_t *handler);
  msg_t pktCloseRadioService(packet_svc_t *handler);
  bool  pktStoreBufferData(pkt_data_object_t *buffer, ax25char_t data);
  eventflags_t  pktDispatchReceivedBuffer(pkt_data_object_t *pkt_buffer);
  thread_t *pktCreateBufferCallback(pkt_data_object_t *pkt_buffer);
  void pktCallback(void *arg);
  void pktCallbackManagerOpen(packet_svc_t *handler);
  void pktCompletion(void *arg);
  dyn_objects_fifo_t *pktBufferManagerCreate(packet_svc_t *handler);
  void pktCallbackManagerCreate(packet_svc_t *handler);
  void pktBufferManagerRelease(packet_svc_t *handler);
  void pktCallbackManagerRelease(packet_svc_t *handler);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name    Macro Functions (packet system drivers)
 * @{
 */

/**
 * @brief   Returns the I/O condition event source.
 * @details The event source is broadcast when an I/O condition happens.
 *
 * @param[in] ip        pointer to a @p packet system event source
 * @return              A pointer to an @p EventSource object.
 *
 * @api
 */
#define pktGetEventSource(ip) (&((ip)->event))

/**
 * @brief   Adds status flags to the listeners's flags mask.
 * @details This function is called from the thread level (locked) or
 *          I/O ISRs in order to notify I/O conditions such as
 *          data events, errors, signal changes etc.
 *
 * @param[in] ip        pointer to a @p packet system event source
 * @param[in] flags     condition flags to be added to the listener flags mask
 *
 * @iclass
 */
#define pktAddEventFlagsI(ip, flags) {                                       \
    chEvtBroadcastFlagsI(&(ip)->event, flags);                               \
}

/**
 * @brief   Adds status flags to the listeners's flags mask.
 * @details This function is an alias for pktAddsFlagsI for convenience
 *
 * @param[in] ip        pointer to a @p packet system event source
 * @param[in] flags     condition flags to be added to the listener flags mask
 *
 * @sclass
 */
#define pktAddEventFlagsS(ip, flags) pktAddEventFlagsI(ip, flags)

/**
 * @brief   Adds status flags to the listeners's flags mask.
 * @details This function is called from the thread level
 *          in order to notify I/O conditions such as
 *          data events, errors, signal changes etc.
 *
 * @param[in] ip        pointer to a @p packet system event source
 * @param[in] flags     condition flags to be added to the listener flags mask
 *
 * @iclass
 */
#define pktAddEventFlags(ip, flags) {                                        \
    chEvtBroadcastFlags(&(ip)->event, flags);                                \
}


/**
 * @brief   Registers a listener on the specified event source.
 * @details This function can be called from the thread level
 *          to register a listener for I/O conditions such as
 *          data events, errors, signal changes etc.
 *
 * @param[in] ip        pointer to a @p packet system event source
 * @param[in] listener  pointer to event listener object
 * @param[in] events    the events of interest at the event source
 * @param[in] flags     the flags to be added to the listener flags mask
 *
 * @api
 */
#define pktRegisterEventListener(ip, listener, events, flags) {              \
    chEvtRegisterMaskWithFlags(ip, listener, events, flags);       \
}

/**
 * @brief   Unregisters an Event Listener from a packet handler.
 * @note    If the event listener is not registered on the specified packet
 *          handler then the function does nothing.
 *
 * @param[in] ip        pointer to a @p packet system event source
 * @param[in] listener  pointer to the event listener object
 *
 * @api
 */
#define pktUnregisterEventListener(ip, listener) {                            \
  chEvtUnregister(ip, listener);                                             \
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
static inline pkt_data_object_t *pktTakeDataBufferS(packet_svc_t *handler,
                                                    objects_fifo_t *fifo) {
  pkt_data_object_t *pkt_buffer = chFifoTakeObjectI(fifo);
  if(pkt_buffer != NULL) {

    /*
     * Packet buffer available.
     * Save the object pointer.
     */
    handler->active_packet_object = pkt_buffer;

    /* Initialize the buffer fields. */
    pkt_buffer->handler = handler;
    pkt_buffer->status = EVT_STATUS_CLEAR;
    pkt_buffer->packet_size = 0;
    pkt_buffer->buffer_size = PKT_RX_BUFFER_SIZE;
    pkt_buffer->cb_func = handler->usr_callback;

    /* Save the pointer to the packet factory for use when releasing buffer. */
    pkt_buffer->pkt_factory = handler->the_packet_fifo;
  }
  return pkt_buffer;
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
static inline pkt_data_object_t *pktTakeDataBuffer(packet_svc_t *handler,
                                                   objects_fifo_t *fifo) {
  chSysLock();
  pkt_data_object_t *pkt_buffer = pktTakeDataBufferS(handler, fifo);
  chSysUnlock();
  return pkt_buffer;
}


/**
 * @brief   Returns a buffer to the packet buffer free pool.
 * @details This function is called from thread level to free a buffer.
 * @post    The buffer is released back to the free pool.
 * @post    The semaphore for used/free buffer counting is updated.
 * @post    Or...
 * @post    The factory object is released.
 * @post    If the factory reference count reaches zero it will be destroyed.
 * @post    i.e. when the decoder is closed and the last outstanding buffer is released.
 *
 * @param[in]   object      pointer to a @p buffer object.
 *
 * @api
 */
static inline void pktReleaseDataBuffer(pkt_data_object_t *object) {

  dyn_objects_fifo_t *pkt_factory = object->pkt_factory;
  chDbgAssert(pkt_factory != NULL, "no packet factory");

  objects_fifo_t *pkt_fifo = chFactoryGetObjectsFIFO(pkt_factory);
  chDbgAssert(pkt_fifo != NULL, "no packet FIFO");


  /* Is this a callback release? */
  /* TODO: Might be better if this is handled independent of this function? */
  if(object->cb_func != NULL) {
    /*
     * For callback mode send the packet buffer to the FIFO queue.
     * It will be released in the collector thread.
     * The callback thread memory will be recovered.
     * The semaphore will be signalled.
     */
    chSysLock();
    chFifoSendObjectI(pkt_fifo, object);
    chThdExitS(MSG_OK);
    /* We don't get to here. */
  }


  /*
   * Free the object.
   * Decrease the factory reference count.
   * If the service is closed and all buffers freed then the FIFO is destroyed.
   */
  chFifoReturnObject(pkt_fifo, object);
  chFactoryReleaseObjectsFIFO(pkt_factory);
}

/**
 * @brief   Resets the buffer index of a packet buffer.
 * @details This macro resets the buffer count to zero.
 *
 * @param[in]   object      pointer to the @p buffer to reset.
 *
 * @api
 */
static inline void pktResetDataCount(pkt_data_object_t *object) {
  object->packet_size = 0;
}

/**
 * @brief   Waits for a buffer to be posted to the FIFO and gets it.
 * @details This function is called from thread level to get a posted buffer.
 *
 * @param[in] handler   pointer to a @p packet handler object.
 * @param[in] object    pointer to the fetched object reference.
 * @param[in] timeout   the number of ticks before the operation times out.
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return              The operation status.
 * @retval MSG_OK       if an object has been correctly fetched.
 * @retval MSG_TIMEOUT  if the operation has timed out.
 *
 * @api
 */
static inline msg_t pktReceiveDataBufferTimeout(packet_svc_t *handler,
                                                 pkt_data_object_t **object,
                                                 sysinterval_t timeout) {

  objects_fifo_t *pkt_fifo = chFactoryGetObjectsFIFO(handler->the_packet_fifo);

  chDbgAssert(pkt_fifo != NULL, "no packet FIFO");

  msg_t fifo_msg = chFifoReceiveObjectTimeout(pkt_fifo,
                                              (void *)object, timeout);
  return fifo_msg;
}

/**
 * @brief   Checks if a buffer meets minimum AX25 validity.
 * @note    Validity relates to minimum size and AFSK decoding being complete.
 * @note    The frame may have a good or bad CRC.
 * @details This function is called from thread level.
 *
 * @param[in] object    pointer to a @p objects FIFO.
 *
 * @return              The operation status.
 * @retval true         if the buffer meets minimum requirement for AX25 frame.
 * @retval false        if the minimum requirements are not met.
 *
 * @api
 */
static inline bool pktIsBufferValidAX25Frame(pkt_data_object_t *object) {
  chDbgAssert(object != NULL, "no pointer to packet object buffer");
  uint16_t frame_size = object->packet_size;
  return ((object->status & EVT_AFSK_DECODE_DONE)
    && (frame_size >= PKT_MIN_FRAME));
}

#endif /* PKT_CHANNELS_PKTSERVICE_H_ */

/** @} */
