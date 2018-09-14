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

#define PKT_RX_BUFFER_SIZE              PKT_MAX_RX_PACKET_LEN

#define PKT_FRAME_QUEUE_PREFIX          "pktr_"
#define PKT_CALLBACK_TERMINATOR_PREFIX  "cbte_"
#define PKT_CALLBACK_THD_PREFIX         "cb_"

#define PKT_SEND_BUFFER_SEM_NAME        "pbsem"


#define PKT_RX_CALLBACK_WA_SIZE          (1024 * 5)
#define PKT_TERMINATOR_WA_SIZE           (1024 * 1)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/* Packet handler states. */
typedef enum handlerSvcStates {
  PACKET_IDLE = 0,
  PACKET_READY,
  PACKET_CLOSE,
  PACKET_INVALID
} pkt_svc_state_t;

/* Packet handler states. */
typedef enum handlerRxStates {
  PACKET_RX_IDLE = 0,
  PACKET_RX_OPEN,
  PACKET_RX_ENABLED,
  PACKET_RX_CLOSE,
  PACKET_RX_INVALID
} pkt_rx_state_t;

/* HDLC frame states. */
typedef enum HDLCFrameStates {
  FRAME_SEARCH,
  FRAME_OPEN,
  FRAME_DATA,
  FRAME_CLOSE,
  FRAME_RESET
} frame_state_t;

#include "types.h"

/* Link level encoding type. */
//typedef radio_mod_t encoding_type_t;

#include "pktradio.h"

/* Receive packet buffer. */
typedef struct packetBuffer pkt_data_object_t;

/* Receive packet buffer callback. */
typedef void (*pkt_buffer_cb_t)(pkt_data_object_t *pkt_buffer);

typedef struct packetBuffer {
  struct pool_header        link; /* For safety keep clear - where pool stores its free link. */
  packet_svc_t              *handler;
  dyn_objects_fifo_t        *pkt_factory;
  thread_t                  *cb_thread;
  char                      cb_thd_name[PKT_THREAD_NAME_MAX];
  pkt_buffer_cb_t           cb_func;
  volatile eventflags_t     status;
  size_t                    buffer_size;
  size_t                    packet_size;
#if USE_CCM_HEAP_RX_BUFFERS == TRUE
  ax25char_t                *buffer;
#else
  ax25char_t                buffer[PKT_RX_BUFFER_SIZE];
#endif
} pkt_data_object_t;


typedef struct packetHandlerData {
  /**
   * @brief State of the packet handler.
   */
  pkt_svc_state_t            state;

  /**
   * @brief State of the packet handler.
   */
  pkt_rx_state_t            rx_state;

  /**
   * @brief Radio being managed.
   */
  radio_unit_t              radio;

  /**
   * @brief Radio part number.
   */
  radio_part_t              radio_part;

  /**
   * @brief Radio revision level.
   */
  radio_rev_t               radio_rom_rev;

  /**
   * @brief Radio patch ID.
   */
  radio_patch_t             radio_patch;

  /**
   * @brief Radio initialization flag.
   */
  bool                      radio_init;

#if PKT_USE_RADIO_MUTEX != TRUE
  /**
   * @brief Radio locked access semaphore.
   */
  binary_semaphore_t        radio_sem;
#else

  /**
   * @brief Radio locked access semaphore.
   */
  mutex_t                   radio_mtx;
#endif
  /**
   * @brief Radio receiver operating parameters.
   */
  radio_task_object_t       radio_rx_config;

  /**
   * @brief Radio signal strength captured on CCA.
   */
  radio_signal_t            rx_strength;

  /**
   * @brief Radio transmitter operating parameters.
   */
  radio_task_object_t       radio_tx_config;

  /**
   * @brief Counter for active transmit threads.
   */
  uint8_t                   tx_count;

  /**
   * @brief Pointer to link level protocol data.
   */
  void                      *rx_link_control;

  /**
   * @brief Receive decoder type.
   */
  radio_mod_t               rx_link_type;

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
   * @brief AX25 packet guarded FIFO for receive.
   */
  dyn_objects_fifo_t        *the_packet_fifo;

  /**
   * @brief AX25 send buffer throttling semaphore.
   */
  //dyn_semaphore_t           *tx_packet_sem;

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
  uint8_t                   cb_count;

  /**
   * @brief Event source object.
   */
  binary_semaphore_t        close_sem;

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

extern packet_svc_t RPKTD1;

#ifdef __cplusplus
extern "C" {
#endif
  bool pktSystemInit(void);
  bool pktSystemDeinit(void);
  bool pktServiceCreate(const radio_unit_t radio);
  bool pktServiceRelease(const radio_unit_t radio);
  msg_t pktOpenRadioReceive(const radio_unit_t radio,
                                     const radio_mod_t encoding,
                                     const radio_freq_t frequency,
                                     const channel_hz_t ch_step);
  msg_t pktEnableDataReception(const radio_unit_t radio,
                              const radio_ch_t channel,
                              const radio_squelch_t sq,
                              const pkt_buffer_cb_t cb);
  void pktStartDecoder(const radio_unit_t radio);
  msg_t pktDisableDataReception(const radio_unit_t radio);
  void pktStopDecoder(const radio_unit_t radio);
  msg_t pktCloseRadioReceive(const radio_unit_t radio);
  bool  pktStoreReceiveData(pkt_data_object_t *buffer, ax25char_t data);
  eventflags_t  pktDispatchReceivedBuffer(pkt_data_object_t *pkt_buffer);
  thread_t *pktCreateReceiveCallback(pkt_data_object_t *pkt_buffer);
  void pktCallback(void *arg);
  //void pktCallbackManagerOpen(const radio_unit_t radio);
  void pktCompletion(void *arg);
  dyn_objects_fifo_t *pktIncomingBufferPoolCreate(const radio_unit_t radio);
#if PKT_RX_RLS_USE_NO_FIFO != TRUE
  thread_t *pktCallbackManagerCreate(const radio_unit_t radio);
#endif
  void pktCallbackManagerRelease(packet_svc_t *handler);
  void pktIncomingBufferPoolRelease(packet_svc_t *handler);
  dyn_objects_fifo_t *pktCommonBufferPoolCreate(const radio_unit_t radio);
  void pktCommonBufferPoolRelease(const radio_unit_t radio);
  void pktReleaseBufferSemaphore(const radio_unit_t radio);
  msg_t pktGetPacketBuffer(packet_t *pp, sysinterval_t timeout);
  void pktReleasePacketBuffer(packet_t pp);
  dyn_semaphore_t *pktInitBufferControl(void);
  void pktDeinitBufferControl(void);
  packet_svc_t *pktGetServiceObject(radio_unit_t radio);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Macro definitions      .                                                  */
/*===========================================================================*/

#define pktIsReceiveInProgress(radio) pktRadioGetInProgress(radio)

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
    chEvtRegisterMaskWithFlags(ip, listener, events, flags);                 \
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
#define pktUnregisterEventListener(ip, listener) {                           \
  chEvtUnregister(ip, listener);                                             \
}

/**
 * @brief   Fetches a buffer from the packet buffer free pool.
 * @details This function is called from thread level to obtain a buffer
 *          to write AX25 data into.
 *
 * @param[in]   handler     pointer to a @p packet service object
 * @param[in]   fifo        pointer to a @p objects FIFO
 * @paream[in]  timeout     Allowable wait for a free buffer
 *
 * @return      pointer to packet buffer object.
 * @retval      NULL if no buffer object available.
 *
 * @api
 */
static inline pkt_data_object_t *pktTakeDataBuffer(packet_svc_t *handler,
                                                    objects_fifo_t *fifo,
                                                    sysinterval_t timeout) {
  pkt_data_object_t *pkt_buffer = chFifoTakeObjectTimeout(fifo, timeout);
  handler->active_packet_object = pkt_buffer;
  if(pkt_buffer != NULL) {

    /*
     * Packet buffer available.
     */
    //handler->active_packet_object = pkt_buffer;

    /* Initialize the object fields. */
    pkt_buffer->handler = handler;
    pkt_buffer->status = EVT_STATUS_CLEAR;
    pkt_buffer->packet_size = 0;
    pkt_buffer->buffer_size = PKT_RX_BUFFER_SIZE;
    pkt_buffer->cb_func = handler->usr_callback;

    /* Save the pointer to the packet factory for use when releasing object. */
    pkt_buffer->pkt_factory = handler->the_packet_fifo;
#if USE_CCM_HEAP_RX_BUFFERS == TRUE
    extern memory_heap_t *ccm_heap;
    pkt_buffer->buffer = chHeapAlloc(ccm_heap, PKT_RX_BUFFER_SIZE);
    if(pkt_buffer->buffer == NULL) {
      /* No heap available. */
      /* Return packet buffer object to free list. */
      chFifoReturnObject(fifo, (pkt_data_object_t *)pkt_buffer);

      /*
       * Decrease FIFO reference counter (increased by decoder).
       * FIFO will be destroyed when all references are released.
       */
      chFactoryReleaseObjectsFIFO(pkt_buffer->pkt_factory);
      pkt_buffer = NULL;
    }
#endif
  }
  return pkt_buffer;
}

/**
 * @brief   Returns a receive buffer to the packet buffer free pool.
 * @details This function is called from thread level to free a buffer.
 * @post    The buffer is released back to the free pool.
 * @post    The semaphore for used/free buffer counting is updated.
 * @post    Or...
 * @post    The factory object is released.
 * @post    If the factory reference count reaches zero it will be destroyed.
 * @post    i.e. when the decoder is closed with no further outstanding buffers.
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

#if USE_CCM_HEAP_RX_BUFFERS == TRUE
  /* Free the packet buffer in the heap now. */
  chHeapFree(object->buffer);
#endif

  /* Is this a callback release? */
  if(object->cb_func != NULL) {
#if PKT_RX_RLS_USE_NO_FIFO == TRUE
    extern void pktThdTerminateSelf(void);
    /*
     * Free the object.
     * Decrease the factory reference count.
     * If the service is closed and all buffers freed then the FIFO is destroyed.
     * Terminate this thread and have idle thread sweeper clean up memory.
     */
    object->handler->cb_count--;
    chFifoReturnObject(pkt_fifo, object);
    chFactoryReleaseObjectsFIFO(pkt_factory);
    pktThdTerminateSelf();
    /* We don't get to here. */
#endif
    /*
     * For callback mode send the packet buffer to the FIFO queue.
     * It will be released in the collector thread.
     * The callback thread memory will be recovered.
     * The semaphore will be signaled.
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
  return ((object->status & STA_AFSK_DECODE_DONE)
    && (frame_size >= PKT_MIN_FRAME));
}

/**
 * @brief   Gets status of frame.
 * @note    This returns validity (size) and CRC result.
 * @details This function is called from thread level.
 *
 * @param[in] object    pointer to a @p objects FIFO.
 *
 * @return              The operation status.
 * @retval true         if the frame is valid and has good CRC.
 * @retval false        if the frame is invalid or has bad CRC.
 *
 * @api
 */
static inline bool pktGetAX25FrameStatus(pkt_data_object_t *object) {
  chDbgAssert(object != NULL, "no pointer to packet object buffer");
  return (object->status & (STA_PKT_INVALID_FRAME | STA_PKT_CRC_ERROR)) == 0;
}

/**
 * @brief   Gets current state of a packet service..
 *
 * @param[in] radio    radio unit ID.
 *
 * @return                  The service state.
 * @retval PACKET_INVALID   If the radio ID is invalid.
 *
 * @api
 */
static inline pkt_svc_state_t pktGetServiceState(radio_unit_t radio) {
  /*
   * TODO: implement mapping from radio config to packet handler object.
   */
  packet_svc_t *handler = pktGetServiceObject(radio);

  //chDbgAssert(handler != NULL, "invalid radio ID");

  return handler->state;
}

/**
 * @brief   Tests if transmit is available for the radio.
 *
 * @param[in] radio    radio unit ID.
 *
 * @return        Availability.
 * @retval true   If transmit is available.
 * @retval false  If transmit is not available.
 *
 * @api
 */
static inline bool pktIsTransmitOpen(radio_unit_t radio) {
  pkt_svc_state_t state = pktGetServiceState(radio);
  return !(state == PACKET_IDLE || state == PACKET_INVALID);
}

/**
 * @brief   Tests if receive is enabled on the radio.
 *
 * @param[in] radio    radio unit ID.
 *
 * @return        Result.
 * @retval true   If receive is enabled.
 * @retval false  If receive is not enabled.
 *
 * @api
 */
static inline bool pktIsReceiveEnabled(radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  return (handler->rx_state == PACKET_RX_ENABLED);
}

/**
 * @brief   Tests if receive is in progress.
 *
 * @param[in] radio    radio unit ID.
 *
 * @return        Result.
 * @retval true   If receive is in progress.
 * @retval false  If receive is not in progress.
 *
 * @api
 */
/*static inline bool pktIsReceiveInProgress(radio_unit_t radio) {
    return pktRadioGetInProgress(radio);
}*/

/**
 * @brief   Tests if receive is ready to start.
 *
 * @param[in] radio    radio unit ID.
 *
 * @return        Result.
 * @retval true   If receive is ready.
 * @retval false  If receive is not ready.
 *
 * @api
 */
static inline bool pktIsReceiveReady(radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  return (handler->rx_state == PACKET_RX_OPEN);
}

#endif /* PKT_CHANNELS_PKTSERVICE_H_ */

/** @} */
