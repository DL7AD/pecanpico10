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
#define PKT_CALLBACK_THD_PREFIX         "cb_"

#define PKT_SEND_BUFFER_SEM_NAME        "pbsem"

#define PKT_RX_CALLBACK_WA_SIZE          (1024 * 5)
#define PKT_TERMINATOR_WA_SIZE           (1024 * 1)

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

#define PKT_USE_RM_FOR_RX_DISPATCH  FALSE

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/* Packet handler states. */
typedef enum handlerSvcStates {
  PACKET_IDLE = 0,
  PACKET_INIT,
  PACKET_READY,
  PACKET_CLOSE
} pkt_svc_state_t;

/* Packet handler states. */
typedef enum handlerRxStates {
  PACKET_RX_IDLE = 0,
  PACKET_RX_OPEN,
  PACKET_RX_ENABLED,
  PACKET_RX_CLOSE,
  PACKET_RX_INVALID
} pkt_rx_state_t;

#include "types.h"

/* Link level encoding type. */

#include "pktradio.h"

/* Receive packet buffer. */
typedef struct packetBufferObject pkt_data_object_t;

/* Receive packet buffer callback. */
typedef void (*pkt_buffer_cb_t)(pkt_data_object_t *const pkt_object);

typedef struct packetBufferObject {
  struct pool_header        link; /* For safety keep clear - where pool stores its free link. */
  packet_svc_t              *handler;
  ax25char_t                buffer[PKT_RX_BUFFER_SIZE];
  char                      cb_thd_name[PKT_THREAD_NAME_MAX];
  pkt_buffer_cb_t           cb_func;
  volatile statusflags_t    status;
  radio_freq_hz_t           freq;
  radio_signal_t            rssi;
  cnt_t                     seq_num;
  size_t                    buffer_size;
  size_t                    packet_size;

} pkt_data_object_t;


typedef struct packetHandlerData {
  /**
   * @brief State of the packet handler.
   */
  pkt_svc_state_t           state;

  /**
   * @brief State of the packet handler.
   */
  pkt_rx_state_t            rx_state;

  /**
   * @brief Radio being managed.
   */
  radio_unit_t              radio;

  xtal_osc_t                xtal;      /**< XO frequency of main clock.     */

  /**
   * @brief Radio part number.
   */
  radio_part_t              radio_part; // to be put into struct of radio data

  /**
   * @brief Radio revision level.
   */
  radio_rev_t               radio_rom_rev; // to be put into struct of radio data

  /**
   * @brief Radio patch ID.
   */
  radio_patch_t             radio_patch; // to be put into struct radio data

  /**
   * @brief Radio initialization flag.
   */
  bool                      radio_init; /**< Radio has been initialised     */

  binary_semaphore_t        radio_sem;  /**< Radio lock semaphore.          */
  radio_mode_t              lock_mode;  /**< Radio lock mode.               */
  radio_params_t            radio_rx_config;
  radio_params_t            radio_tx_config;

  /**
   * @brief Counter for active transmit threads.
   */
  uint8_t                   txrto_ref_count;

  /**
   * @brief Pointer to link level protocol data.
   */
  void                      *rx_link_control;

  /**
   * @brief Receive decoder type.
   */
  radio_mod_t               rx_link_type;

  /**
   * @brief names for the radio task FIFO
   */
  char                      rtask_name[CH_CFG_FACTORY_MAX_NAMES_LENGTH];

  /**
   *  @brief Packet system service threads.
   */
  thread_t                  *radio_manager;

  /**
   * @brief Radio task guarded FIFO.
   */
  dyn_objects_fifo_t        *the_radio_fifo;

  /**
   * @brief AX25 receive packet objects guarded pool.
   */
  guarded_memory_pool_t     rx_packet_pool;
  pkt_data_object_t         *packet_heap;

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
  uint8_t                   rxcb_ref_count;

  /**
   * @brief The service semaphore. All FIFO requests wait on this.
   */
  binary_semaphore_t        mgr_sem;

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
extern memory_heap_t *ccm_heap;

#ifdef __cplusplus
extern "C" {
#endif
  bool                  pktSystemInit(void);
  bool                  pktSystemDeinit(void);
  bool                  pktServiceCreate(const radio_unit_t radio);
  bool                  pktServiceRelease(const radio_unit_t radio);
/*  msg_t                 pktOpenRadioReceive(const radio_unit_t radio,
                            const radio_mod_t encoding,
                            const radio_freq_hz_t frequency,
                            const radio_chan_hz_t ch_step,
                            const sysinterval_t timeout);*/
  msg_t                 pktEnableDataReception(const radio_unit_t radio,
                               const radio_ch_t channel,
                               const radio_squelch_t sq,
                               const pkt_buffer_cb_t cb,
                               const sysinterval_t to);
  msg_t                 pktOpenReceiveService(const radio_unit_t radio,
                              const radio_mod_t encoding,
                              const radio_freq_hz_t frequency,
                              const radio_chan_hz_t step,
                              const radio_ch_t channel,
                              const radio_squelch_t sq,
                              const pkt_buffer_cb_t cb,
                              const sysinterval_t to);
  //msg_t                 pktStartDecoder(const radio_unit_t radio);
  msg_t                 pktDisableDataReception(const radio_unit_t radio);
  //void                  pktStopDecoder(const radio_unit_t radio);
  //msg_t                 pktCloseRadioReceive(const radio_unit_t radio);
  bool                  pktStoreReceiveData(pkt_data_object_t *const buffer,
                                            const ax25char_t data);
  eventflags_t          pktDispatchReceivedBuffer(pkt_data_object_t *const pkt_buffer);
  thread_t*             pktCreateReceiveCallback(pkt_data_object_t *const pkt_buffer);
  void                  pktCallback(void *arg);
  //void                  pktCompletion(void *arg);
  pkt_data_object_t*    pktIncomingBufferPoolCreate(const radio_unit_t radio);
  void                  pktIncomingBufferPoolRelease(packet_svc_t *const handler);
  dyn_objects_fifo_t*   pktCommonBufferPoolCreate(const radio_unit_t radio);
  void                  pktCommonBufferPoolRelease(const radio_unit_t radio);
  void                  pktReleaseBufferSemaphore(const radio_unit_t radio);
  msg_t                 pktGetCommonPacketBuffer(packet_t *const pp,
                                           const sysinterval_t timeout);
  void                  pktReleaseCommonPacketBuffer(packet_t pp);
  dyn_semaphore_t*      pktInitBufferControl(void);
  void                  pktDeinitBufferControl(void);
  packet_svc_t*         pktGetServiceObject(const radio_unit_t radio);
  pkt_data_object_t*    pktAssignReceivePacketObject(packet_svc_t *const handler,
                                                     const sysinterval_t timeout);
  void                  pktReleaseDataBuffer(pkt_data_object_t *const object);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Macro definitions      .                                                  */
/*===========================================================================*/

#define pktIsReceiveInProgress(radio) pktRadioGetInProgress(radio)

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

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Resets the buffer index of a packet buffer.
 * @details This macro resets the buffer count to zero.
 *
 * @param[in]   object      pointer to the @p buffer to reset.
 *
 * @api
 */
static inline void pktResetDataCount(pkt_data_object_t *const object) {
  object->packet_size = 0;
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
static inline bool pktGetAX25FrameStatus(const pkt_data_object_t *object) {
  chDbgAssert(object != NULL, "no pointer to packet object buffer");
  return (object->status & (STA_PKT_INVALID_FRAME | STA_PKT_CRC_ERROR)) == 0;
}

/**
 * @brief   Gets current state of a packet service..
 *
 * @param[in] radio    radio unit ID.
 *
 * @return                  The service state.
 *
 * @api
 */
static inline pkt_svc_state_t pktGetServiceState(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);

  return handler->state;
}

/**
 * @brief   Tests if service is available for the radio.
 *
 * @param[in] radio    radio unit ID.
 *
 * @return        Availability.
 * @retval true   If service is available.
 * @retval false  If service is not available.
 *
 * @api
 */
static inline bool pktIsServiceAvailable(const radio_unit_t radio) {
  pkt_svc_state_t state = pktGetServiceState(radio);
  return state == PACKET_READY;
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
static inline bool pktIsTransmitAvailable(const radio_unit_t radio) {
  pkt_svc_state_t state = pktGetServiceState(radio);
  return state == PACKET_READY;
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
static inline bool pktIsReceiveEnabled(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  return (handler->state == PACKET_READY
      && handler->rx_state == PACKET_RX_ENABLED);
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
static inline bool pktIsReceiveReady(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  return (handler->rx_state == PACKET_RX_OPEN);
}

#endif /* PKT_CHANNELS_PKTSERVICE_H_ */

/** @} */
