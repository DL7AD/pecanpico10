/*
    Aerospace Decoder - Copyright (C) 2018-2019 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    pktservice.c
 * @brief   Radio service code.
 *
 * @addtogroup managers
 *
 * @{
 */
#include "pktconf.h"
#include "portab.h"
#include "console.h"
#include "threads.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

memory_heap_t *ccm_heap = NULL;

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if PKT_SVC_USE_RADIO1 || defined(__DOXYGEN__)
packet_svc_t RPKTD1;
#endif

#if PKT_SVC_USE_RADIO2 || defined(__DOXYGEN__)
packet_svc_t RPKTD2;
#endif

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static memory_heap_t _ccm_heap;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes the packet system memory.
 * @notes   Allocates a heap in remaining available CCM.
 * @notes   Unless variables are allocated to CCM the heap will be all of CCM.
 * @notes   Buffers (non DMA) are then allocated from the CCM heap.
 *
 *@return   result of operation.
 *@retval   true    system initialized.
 *@retval   false   initialization failed.
 *
 * @notapi
 */
bool pktMemoryInit(void) {
  /*
   * The definition for CCM is in pktconf.h as follows...
   *  #define useCCM  __attribute__((section(".ram4")))
   * How to allocate a variable in CCM...
   *  int example useCCM;
   *
   * The remainder available in CCM is used to create a heap.
   * This can be used for non-DMA access data only in the F413.
   */

   /* Reference the linker created CCM variables to get the available heap area. */
    extern uint8_t __ram4_free__[];
    extern uint8_t __ram4_end__[];

    chDbgAssert(ccm_heap == NULL, "CCM heap already exists");
    /*
     * Create heap in CCM.
     * Once created the CCM heap remains available.
     * TODO: Move CCM heap creation to system level.
     * Include in sysConfigureCoreIO(...) and rename that function. */
    if(ccm_heap == NULL) {
      /* CCM heap not created yet. */
    ccm_heap = &_ccm_heap;
    chHeapObjectInit(ccm_heap, (void *)__ram4_free__,
                     (size_t)(__ram4_end__ - __ram4_free__));
    }

    /*
     * Create common AX25 transmit packet buffer control.
     */
    dyn_semaphore_t *buffers = pktInitBufferControl();
    chDbgAssert(buffers != NULL, "failed to init packet buffers");
    return buffers != NULL;
}

/**
 * @brief   Initializes the packet system.
 * @notes   Allocates a heap in remaining available CCM.
 * @notes   Unless variables are allocated to CCM the heap will be all of CCM.
 * @notes   Buffers (non DMA) are then allocated from the CCM heap.
 *
 *@return   result of operation.
 *@retval   false  system initialized.
 *@retval   true   initialization failed.
 *
 * @notapi
 */
bool pktSystemInit(void) {
  bool result = false;

  /* Create memory spaces. */
  result |= pktMemoryInit();

  /* Setup core IO peripherals. */
  pktConfigureCoreIO();

  /*
   * Setup serial channel for debug.
   * The mutex for trace output is initialized.
   * The UART port/GPIO is setup.
   * The SDU over USB are setup.
   */
  pktConfigureSerialIO();

#if ACTIVATE_CONSOLE
    /* Start console. */
    pktStartConsole(console);
    TRACE_INFO("MAIN > Console startup");
#endif

    /*
     * Create a packet radio service.
     * For now there is just one radio.
     * TODO: Refactor. Only start services when a radio request is made.
     */
    while(!pktServiceCreate(PKT_RADIO_1)) {
      TRACE_ERROR("MAIN > Unable to create packet radio %d services",
                  PKT_RADIO_1);
      result |= true;
      chThdSleep(TIME_S2I(10));
    }

    pktEnableServiceEventTrace(PKT_RADIO_1);
    TRACE_INFO("MAIN > Started packet radio service for radio %d",
               PKT_RADIO_1);

    TRACE_INFO("MAIN > Starting application and ancillary threads");
    // Startup threads
    pktStartSystemServices();  // Startup required modules (tracking manager, watchdog)
    pktStartApplicationServices();       // Startup optional modules (eg. POSITION, LOG, ...)
  return result;
}

/**
 * @brief   Deinit the packet system.
 *
 *@return   result of operation.
 *@retval   true    deinit success.
 *@retval   false   deinit failed.
 *
 * @api
 */
bool pktSystemDeinit(void) {

  /*
   * Remove common packet buffer control.
   */
  chDbgAssert(ccm_heap != NULL, "CCM heap does not exist");
  //chSysLock();

  pktDeinitBufferControl();

  return true;
}

/**
 * @brief   Initializes packet handlers and starts the radio manager.
 * @note    Once initialized the transmit service is available.
 * @note    To activate receive requires a receive open.
 *
 * @param[in]   radio unit ID.
 *
 *@return   result of operation.
 *@retval   true    service was created.
 *@retval   false   service creation failed or state was not idle.
 *
 * @api
 */
bool pktServiceCreate(const radio_unit_t radio) {

  /* Get service object for this radio. */
  packet_svc_t *handler = pktGetServiceObject(radio);
  if(handler == NULL)
    return false;

  chSysLock();
  if(handler->state != PACKET_IDLE) {
    chSysUnlock();
    return false;
  }
  handler->state = PACKET_INIT;
  chSysUnlock();

  /*
   * Initialize the packet common event object.
   */
  chEvtObjectInit(pktGetEventSource(handler));

  /* Reset the radio parameters. */
  memset(&handler->radio_rx_config, 0, sizeof(radio_task_object_t));
  memset(&handler->radio_tx_config, 0, sizeof(radio_task_object_t));

  /* Set flags and radio ID. */
  handler->radio = radio;
  handler->radio_init = false;

  /* Set service semaphore to locked state. */
  chBSemObjectInit(&handler->mgr_sem, true);

  /* Set radio semaphore to locked state. */
  chBSemObjectInit(&handler->radio_sem, true);

  /* Send request to create radio manager. */
  if(pktRadioManagerCreate(radio) == NULL)
    return false;
  handler->state = PACKET_READY;
  handler->rx_state = PACKET_RX_IDLE;
  return true;
}

/**
 * @brief   Releases packet service.
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
bool pktServiceRelease(const radio_unit_t radio) {

  /*
   * Lookup radio, release handler data objects and release RM thread.
   */
  packet_svc_t *handler = pktGetServiceObject(radio);
  if(handler == NULL)
    return false;

  if(handler->state != PACKET_READY)
    return false;

  /* Radio is left in locked state and the service semaphore is left taken. */
  (void)pktRadioManagerRelease(radio);
  handler->state = PACKET_IDLE;
  return true;
}

/**
 * @brief   Request start of packet reception service.
 * @pre     The packet receive service is opened if not already.
 * @notes   This function submits a request to the radio manager.
 * @notes   The request may fail in the radio manager.
 * @post    The radio is tuned to the specified channel.
 * @post    The packet reception is running if it was stopped.
 *
 * @param[in]   radio       radio ID
 * @param[in]   encoding    modulation type
 * @param[in]   frequency   base operating frequency (code or Hz)
 * @param[in]   step        channel step size in Hz
 * @param[in]   channel     radio channel number to select
 * @param[in]   sq          the RSSI setting to be used.
 * @param[in]   cb          callback function called on receipt of packet.
 * @param[in]   to          timeout
 *
 * @return              Status of the service request.
 * @retval MSG_OK       if the service request was queued successfully.
 * @retval MSG_RESET    data lock semaphore was reset.
 * @retval MSG_ERROR    parameter error, service not in correct state.
 * @retval MSG_TIMEOUT  if the service request could not be queued.
 *
 * @api
 */
msg_t pktOpenReceiveService(const radio_unit_t radio,
                            const radio_mod_t encoding,
                            const radio_freq_hz_t frequency,
                            const radio_chan_hz_t step,
                            const radio_ch_t channel,
                            const radio_squelch_t sq,
                            const pkt_buffer_cb_t cb,
                            const sysinterval_t to) {

  packet_svc_t *handler = pktGetServiceObject(radio);
  if(handler == NULL)
    return MSG_ERROR;

  /*
   * Is the packet radio service active?
   * Do we have a callback set? (mandatory)
   */
  if(handler->state != PACKET_READY || cb == NULL)
    return MSG_ERROR;

  /*
   * TODO: Move state checks into radio manager.
   * Use RM internal callbacks to transition between states.
   */

  switch (handler->rx_state) {
  /**
   *
   */
  case PACKET_RX_IDLE: {
    /*
     *  Wait for any prior session to complete closing.
     *  TODO: Check if this is still relevant...
     *   It used to protect handler data integrity while there were FIFO packets out.
     *   But now the pool based packet buffers are independent.
     */
    msg_t msg;
#if 0
    msg = chBSemWait(&handler->mgr_sem);
    if(msg != MSG_OK)
      return msg;
#endif



    /* Set entire radio configuration. */
    handler->radio_rx_config.type = encoding;
    handler->radio_rx_config.base_frequency = frequency;
    handler->radio_rx_config.step_hz = step;
    handler->radio_rx_config.channel = channel;
    handler->radio_rx_config.rssi = sq;

    /* Set the packet callback. */
    handler->usr_callback = cb;

    /* Reset the statistics collection variables. */
    handler->sync_count = 0;
    handler->frame_count = 0;
    handler->valid_count = 0;
    handler->good_count = 0;

    /* Setup the task to open and start receive. */
    msg_t result;
    msg = pktQueueRadioCommand(radio,
                              PKT_RADIO_RX_OPEN,
                              &handler->radio_rx_config,
                              to, &result, NULL);

    if (msg == MSG_OK) {
      if (result == MSG_OK)
        pktAddEventFlags(handler, EVT_PKT_RECEIVE_OPEN | EVT_PKT_RECEIVE_START);
      return result;
    }
    return msg;
  }
  /**
   * The service is already opened for receive.
   * Just start it.
   */
  case PACKET_RX_OPEN: {
    /* Set the packet callback. */
    handler->usr_callback = cb;

    /* Update start parameters in radio configuration. */
    handler->radio_rx_config.channel = channel;
    handler->radio_rx_config.rssi = sq;


    /* TODO: Check other parameters match current values in rx_config.
     * Encoding, frequency, step... actually only encoding matters.
     * In that case close the channel and re-open with new encoding.
     */
    msg_t result;
    msg_t msg = pktQueueRadioCommand(radio,
                                    PKT_RADIO_RX_START,
                                    &handler->radio_rx_config,
                                    to, &result,
                                    NULL);

    if (msg == MSG_OK) {
      if (result == MSG_OK)
        pktAddEventFlags(handler, EVT_PKT_RECEIVE_START);
    }
    return msg;
  }

  /**
   *
   */
  case PACKET_RX_ENABLED:
  case PACKET_RX_CLOSE:
  case PACKET_RX_ERROR:

    break;
  } /* End switch. */
  return MSG_ERROR;
}

/**
 * @brief   Stop reception.
 * @notes   Called from an application level.
 * @notes   Decoding is stopped.
 * @notes   Any packets out for processing remain in effect.
 * @pre     The packet channel must be running.
 * @post    The packet channel is stopped.
 *
 * @param[in] radio     radio unit ID.
 *
 * @return              Status of the operation.
 * @retval MSG_OK       if the reception was stopped.
 * @retval MSG_RESET    if the radio was reset (semaphore reset).
 * @retval MSG_TIMEOUT  if the reception could not be stopped or is invalid.
 * @retval MSG_ERROR    if the service was not in the correct state.
 *
 * @api
 */
msg_t pktDisableDataReception(radio_unit_t radio) {


  packet_svc_t *handler = pktGetServiceObject(radio);

  if (handler == NULL)
    return MSG_ERROR;

  if (!pktIsReceiveEnabled(radio))
    return MSG_ERROR;

  /* Submit command. A timeout can occur waiting for a command queue object. */
  radio_params_t rp = handler->radio_rx_config;
  msg_t result;
  msg_t msg = pktQueueRadioCommand(radio, PKT_RADIO_RX_STOP,
                                  &rp, TIME_INFINITE, &result, NULL);
  if (msg == MSG_OK) {
    if (result == MSG_OK)
      pktAddEventFlags(handler, EVT_PKT_RECEIVE_STOP);
  }
  return msg;
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

  chDbgCheck(handler->state == PACKET_READY);
  if(handler->state != PACKET_READY)
    return MSG_RESET;

  /* Submit command. A timeout can occur waiting for a command queue object. */
  msg_t result;
  msg_t msg = pktQueueRadioCommand(radio, PKT_RADIO_RX_CLOSE,
                                  NULL, TIME_INFINITE, &result, NULL);
  if (msg == MSG_OK) {
    if (result == MSG_OK)
      pktAddEventFlags(handler, EVT_PKT_RECEIVE_CLOSE);
  }
  return msg;
}

/**
 * @brief   Stores receive data in a packet channel buffer.
 * @post    The character is stored and the internal buffer index is updated.
 *
 * @param[in] pkt_object    pointer to a @p packet buffer object.
 * @param[in] data          the character to be stored
 *
 * @return              Status of the operation.
 * @retval true         The data was stored.
 * @retval false        The data could not be stored (buffer full).
 *
 * @api
 */
bool pktStoreReceiveData(pkt_data_object_t *const pkt_object,
                         const ax25char_t data) {
  if((pkt_object->packet_size + 1U) > pkt_object->buffer_size) {
    /* Buffer full. */
    return false;
  }

  /* Buffer space available. */
  pkt_object->buffer[pkt_object->packet_size++] = data;
  return true;
}

/**
 * @brief   Dispatch a received buffer object.
 * @notes   The buffer is checked to determine validity and CRC.
 * @post    The buffer status is updated in the packet FIFO.
 * @post    Packet quality statistics are updated.
 * @post    A callback is used a thread is created to execute the user callback.
 *
 * @param[in] pkt_object    pointer to a @p packet buffer object.
 *
 * @return  Status flags added after packet validity check.
 * TODO: Change return to MSG_OK, MSG_ERROR
 *
 * @api
 */
eventflags_t pktDispatchReceivedBuffer(pkt_data_object_t *const pkt_object) {

  chDbgAssert(pkt_object != NULL, "no packet buffer");

  packet_svc_t *handler = pkt_object->handler;

  chDbgAssert(handler != NULL, "invalid handler");

  eventflags_t flags = EVT_NONE;
  handler->frame_count++;
  if(pktIsBufferValidAX25Frame(pkt_object)) {
    handler->valid_count++;
    uint16_t magicCRC =
        calc_crc16(pkt_object->buffer, 0,
                   pkt_object->packet_size);
    if(magicCRC == CRC_INCLUSIVE_CONSTANT)
      handler->good_count++;
    flags |= (magicCRC == CRC_INCLUSIVE_CONSTANT)
                                ? STA_PKT_FRAME_RDY
                                : STA_PKT_CRC_ERROR;
  } else {
    flags |= STA_PKT_INVALID_FRAME;
  }

  /* Update status in packet buffer object. */
  pkt_object->status |= flags;

#if PKT_USE_RM_FOR_RX_DISPATCH == TRUE
  /* Setup the task to dispatch the receive packet buffer. The RM creates a
     call back thread for the packet to be dispatched via. */
  radio_task_object_t rto = *handler->radio_rx_config;
  rto.radio_dat.pkt = pkt_object;
  msg_t result;
  msg = pktQueueRadioCommand(radio,
                             PKT_RADIO_RX_DISPATCH,
                            &rto,
                            TIME_MS2I(500), &result, NULL);

  if (msg == MSG_OK) {
    msg = result;
  }
  return msg;
#else
  /* Schedule a callback thread. */
  thread_t *cb_thd = pktCreateReceiveCallback(pkt_object);

  if(cb_thd != NULL)
    return flags;

  /*
   * Callback thread create failed.
   * Release the packet buffer and management object.
   * Broadcast event.
   */
  chGuardedPoolFree(&handler->rx_packet_pool, pkt_object);
  pktAddEventFlags(handler, EVT_PKT_FAILED_CB_THD);
#endif
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
thread_t *pktCreateReceiveCallback(pkt_data_object_t *const pkt_object) {

  chDbgAssert(pkt_object != NULL, "invalid packet buffer");

  /* Create a callback thread name which is the address of the buffer. */
  /* TODO: Have thread create name using seq_num. */
  chsnprintf(pkt_object->cb_thd_name, sizeof(pkt_object->cb_thd_name),
             PKT_CALLBACK_THD_PREFIX"%x", pkt_object);

  /* This callback will hold a reference to the service object. */
  pkt_object->handler->rxcb_ref_count++;

  /* Start a callback dispatcher thread. */
  thread_t *cb_thd = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(PKT_RX_CALLBACK_WA_SIZE),
              pkt_object->cb_thd_name,
              LOWPRIO,
              pktCallback,
              pkt_object);


  if(cb_thd == NULL) {
    /*
     * Thread creation failed.
     * Remove reference to the service object.
     */
    pkt_object->handler->rxcb_ref_count--;
  }
  return cb_thd;
}

/**
 * @brief   Run a callback processing thread.
 * @notes   Packet callbacks are processed by individual threads.
 * @notes   Thus packet callbacks are non-blocking to the decoder thread.
 * @notes   After callback completes the thread is scheduled for release.
 *
 * @post    Call back has been executed (for however long it takes).
 * @post    Callback thread release is handled in the terminator (idle) thread.
 *
 * @param[in] arg pointer to a @p packet buffer object.
 *
 * @notapi
 */
THD_FUNCTION(pktCallback, arg) {

  chDbgAssert(arg != NULL, "invalid buffer reference");

  pkt_data_object_t *pkt_object = arg;

  chDbgAssert(pkt_object->cb_func != NULL, "no callback set");

  /* Perform the callback. */
  pkt_object->cb_func(pkt_object);

  /*
   * Upon return the buffer control object is released.
   * The callback thread is scheduled for destruction.
   */

  pktReleaseDataBuffer(pkt_object);
  packet_svc_t *handler = pkt_object->handler;
  chDbgAssert(handler != NULL, "invalid handler object");

  /* The callback no longer holds a reference to the service object. */
  handler->rxcb_ref_count--;
  pktThdTerminateSelf();
}

/*
 * @brief   Create the receive packet management/buffer objects.
 *
 * @returns result of allocation
 * @retval  pointer to heap used for objects
 * @retval  NULL if memory allocation failed
 */
pkt_data_object_t *pktIncomingBufferPoolCreate(radio_unit_t radio) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  /* Initialise guarded pool of receive packet buffer control objects. */
  chGuardedPoolObjectInitAligned(&handler->rx_packet_pool,
                                 sizeof(pkt_data_object_t),
                                 sizeof(size_t));

  pkt_data_object_t *objects = chHeapAllocAligned(
                            USE_CCM_HEAP_RX_BUFFERS ? ccm_heap
                            : NULL,
                            sizeof(pkt_data_object_t) * NUMBER_RX_PKT_BUFFERS,
                            sizeof(msg_t));

  chDbgAssert(objects != NULL, "failed to create space "
                                                "in heap for RX packet pool");
  if(objects != NULL) {

    chGuardedPoolLoadArray(&handler->rx_packet_pool, objects,
                            NUMBER_RX_PKT_BUFFERS);
  }
  return objects;
}

/*
 * Send and APRS share a common pool of packet buffers.
 */
dyn_semaphore_t *pktInitBufferControl() {

  /* Check if the transmit packet buffer semaphore already exists.
   * Calling this twice is an error so assert if enabled.
   * Otherwise get a pointer to it and just return that.
   * If it does not exist create the semaphore and return result.
   */
  dyn_semaphore_t *dyn_sem =
      chFactoryFindSemaphore(PKT_SEND_BUFFER_SEM_NAME);

  if(dyn_sem == NULL) {
    /* Create the semaphore for limiting the packet allocation. */
    dyn_sem = chFactoryCreateSemaphore(PKT_SEND_BUFFER_SEM_NAME,
                                       NUMBER_COMMON_PKT_BUFFERS);

    chDbgAssert(dyn_sem != NULL, "failed to create common packet semaphore");
    if(dyn_sem == NULL)
      return NULL;
    return dyn_sem;
  } else {
    chDbgAssert(false, "common packet semaphore already created");
    return dyn_sem;
  }
}

/*
 * Radio send and APRS packet analysis share a common pool of buffers.
 */
void pktDeinitBufferControl() {

  /* Check if the transmit packet buffer semaphore exists.
   * If so wait for all references to be released.
   * Then release the semaphore.
   */
  dyn_semaphore_t *dyn_sem =
      chFactoryFindSemaphore(PKT_SEND_BUFFER_SEM_NAME);
  chDbgAssert(dyn_sem != NULL, "common packet semaphore does not exist");
  if(dyn_sem == NULL)
    return;
  chSysLock();
  chSemWaitTimeoutS(chFactoryGetSemaphore(dyn_sem), TIME_INFINITE);
  /*
   *  Kick everyone off and set available buffers to zero.
   *  Users need to look for MSG_RESET from wait.
   */
  chSemResetI(&dyn_sem->sem, 0);
  chSchRescheduleS();
  chSysUnlock();
  chFactoryReleaseSemaphore(dyn_sem);
}

/*
 * Send shares a common pool of buffers.
 *
 * @returns status of operation
 * @retval  MSG_OK      if packet buffer assigned
 * @retval  MSG_RESET   if the semaphore has been reset using @p chSemReset().
 * @retval  MSG_TIMEOUT if the semaphore has not been signalled or reset within
 *                        the specified timeout.
 */
msg_t pktGetCommonPacketBuffer(packet_t *pp, const sysinterval_t timeout) {

  /* Check if the packet buffer semaphore already exists.
   * If so we get a pointer to it and get the semaphore.
   */
  dyn_semaphore_t *dyn_sem =
      chFactoryFindSemaphore(PKT_SEND_BUFFER_SEM_NAME);

  chDbgAssert(dyn_sem != NULL, "no send PKT semaphore");

  *pp = NULL;

  if(dyn_sem == NULL)
    return MSG_TIMEOUT;

  /* Wait in queue for permission to allocate a buffer. */
  msg_t msg = chSemWaitTimeout(chFactoryGetSemaphore(dyn_sem), timeout);

  /* Decrease ref count. */
  chFactoryReleaseSemaphore(dyn_sem);

  if(msg != MSG_OK)
    /* This can be MSG_TIMEOUT or MSG_RESET. */
    return msg;

  /* Allocate buffer.
   * If this returns null then all heap is consumed.
   */
  *pp = ax25_new();
  if(pp == NULL)
   return MSG_TIMEOUT;
  return MSG_OK;
}

/*
 * A common pool of AX25 buffers used in TX and APRS.
 */
void pktReleaseCommonPacketBuffer(const packet_t pp) {
  /* Check if the packet buffer semaphore exists.
   * If not this is a system error.
   */
  dyn_semaphore_t *dyn_sem =
      chFactoryFindSemaphore(PKT_SEND_BUFFER_SEM_NAME);

  chDbgAssert(dyn_sem != NULL, "no general packet buffer semaphore");

  /* Free buffer memory. */
  ax25_delete(pp);

  /* Signal buffer is available. */
  chSemSignal(chFactoryGetSemaphore(dyn_sem));

  /* Decrease factory ref count. */
  chFactoryReleaseSemaphore(dyn_sem);
}

/**
 * Release the incoming packet object/buffer pool.
 * This should only be called after all outstanding objects have been freed.
 */
void pktIncomingBufferPoolRelease(packet_svc_t *const handler) {

  /* Release the dynamic objects FIFO for the incoming packet data queue.
     Should only be here when all packets have been released. */
  cnt_t objects = chGuardedPoolGetCounterI(&handler->rx_packet_pool);
  chDbgAssert(objects == 0, "pool has outstanding objects");
  chHeapFree(handler->packet_heap);
}


/**
 * @brief   Gets service object associated with radio.
 *
 * @param[in] radio    radio unit ID.
 *
 * @return        pointer to the service object.
 * @retval NULL   If the radio ID is invalid or no service object assigned.
 *
 * @api
 */
packet_svc_t *pktGetServiceObject(const radio_unit_t radio) {
  /*
   * Get radio configuration object.
   */
  const radio_config_t *data = pktGetRadioData(radio);
  chDbgAssert(data != NULL, "invalid radio ID");
  if(data == NULL)
    return NULL;
  /*
   * Get packet handler object for this radio.
   */
  packet_svc_t *handler = data->pkt;

  chDbgAssert(handler != NULL, "invalid radio packet driver");

  return handler;
}

/**
 * @brief   Gets an sets up a packet management object.
 * @notes   Allocates a management object.
 * @notes   A packet buffer is then obtained and assigned to the object.
 * @details This function is called from thread level to obtain a buffer
 *          to write AX25 data into.
 *
 * @param[in]   handler     pointer to a @p packet service object
 * @param[in]   fifo        pointer to a @p objects FIFO
 * @paream[in]  timeout     Allowable wait for a free data buffer
 *
 * @return      pointer to packet management object.
 * @retval      NULL if no management object or buffer available.
 *
 * @api
 */
pkt_data_object_t* pktAssignReceivePacketObject(packet_svc_t *const handler,
                                                const sysinterval_t timeout) {

  pkt_data_object_t *pkt_object = chGuardedPoolAllocTimeout(
                                            &handler->rx_packet_pool,
                                            timeout);
  if(pkt_object == NULL)
    return NULL;

  /*
   * Packet management object available.
   * Initialize the object fields.
   */
  pkt_object->handler = handler;
  pkt_object->status = EVT_STATUS_CLEAR;
  pkt_object->packet_size = 0;
  pkt_object->buffer_size = PKT_RX_BUFFER_SIZE;
  pkt_object->cb_func = handler->usr_callback;
  return pkt_object;

}

/**
 * @brief   Returns a receive buffer to the packet buffer free pool.
 * @details This function is called from thread level to free a buffer.
 * @post    The packet receive object and buffer are released.
 *
 * @param[in]   object      pointer to a @p receive packet object.
 *
 * @api
 */
void pktReleaseDataBuffer(pkt_data_object_t *const object) {

  packet_svc_t *handler = object->handler;
  chGuardedPoolFree(&handler->rx_packet_pool, object);
}

/**
 * @brief   Release a send packet object memory.
 * @post    The object memory is released.
 *
 * @param[in]   pp     pointer to a @p packet send object
 *
 * @return  next linked packet reference or NULL if none
 *
 * @api
 */
packet_t pktReleaseBufferObject(const packet_t pp) {
  chDbgAssert(pp != NULL, "no packet pointer");
#if USE_CCM_HEAP_FOR_PKT == TRUE
  pktAssertCCMdynamicCheck(pp);
#endif
  packet_t np = pp->nextp;
  pktReleaseCommonPacketBuffer(pp);
  return np;
}
/** @} */
