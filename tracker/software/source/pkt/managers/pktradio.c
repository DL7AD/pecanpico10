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
#ifndef PKT_IS_TEST_PROJECT
#include "radio.h"
#endif
#include "si446x.h"
#include "debug.h"
#include "geofence.h"

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
  /* When waiting for TX tasks to complete. */
#define PKT_RADIO_TX_TASK_RECHECK_WAIT_MS     100

  packet_svc_t *handler = arg;

  dyn_objects_fifo_t *the_radio_fifo = handler->the_radio_fifo;

  chDbgCheck(arg != NULL);

  objects_fifo_t *radio_queue = chFactoryGetObjectsFIFO(the_radio_fifo);

  chDbgAssert(radio_queue != NULL, "no queue in radio manager FIFO");

  const radio_unit_t radio = handler->radio;

  /* Take radio out of shutdown and initialize base registers. */
  bool init = pktLLDradioInit(radio);

  thread_t *initiator = chMsgWait();
  chMsgGet(initiator);
  if(!init) {
    chMsgRelease(initiator, MSG_ERROR);
    chThdExit(MSG_OK);
  }
  chMsgRelease(initiator, MSG_OK);
  /* Run until close request and no outstanding TX tasks. */
  while(true) {
    /* Check for task requests. */
    radio_task_object_t *task_object;
    (void)chFifoReceiveObjectTimeout(radio_queue,
                         (void *)&task_object, TIME_INFINITE);
    /* Something to do. */

    /* Process command. */
    switch(task_object->command) {
    case PKT_RADIO_MGR_CLOSE: {
      /*
       * Radio manager close is sent as a task object.
       * When no TX tasks are outstanding release the FIFO and terminate.
       * The task initiator waits with chThdWait(...).
       */
      if(handler->tx_count == 0) {
        pktLLDradioShutdown(radio);
        chFactoryReleaseObjectsFIFO(handler->the_radio_fifo);
        chThdExit(MSG_OK);
        /* We never arrive here. */
      }
      /*
       * There are still TX sessions running.
       * Wait, repost task, let the FIFO be processed and check again.
       */
      chThdSleep(TIME_MS2I(PKT_RADIO_TX_TASK_RECHECK_WAIT_MS));
      pktSubmitRadioTask(radio, task_object, NULL);
      continue;
    }

    case PKT_RADIO_RX_RSSI: {
      /* TODO: Implement read RSSI radio task. */
      break;
    }

    case PKT_RADIO_RX_OPEN: {

       /* Create the packet management services. */
      if(pktIncomingBufferPoolCreate(radio) == NULL) {
        pktAddEventFlags(handler, (EVT_PKT_BUFFER_MGR_FAIL));
        break;
      }
#if PKT_RX_RLS_USE_NO_FIFO != TRUE
      /* Create callback manager. */
      if(pktCallbackManagerCreate(radio) == NULL) {
        pktAddEventFlags(handler, (EVT_PKT_CBK_MGR_FAIL));
        pktIncomingBufferPoolRelease(handler);
        break;
      }
#else
      /*
       * Initialize the outstanding callback count.
       */
      handler->cb_count = 0;
#endif
      /* Switch on modulation type. */
      switch(task_object->type) {
        case MOD_AFSK: {
          /* TODO: abstract this into the LLD for the radio. */
          /* Create the AFSK decoder (includes PWM, filters, etc.). */
          AFSKDemodDriver *driver = pktCreateAFSKDecoder(handler);
          handler->link_controller = driver;
          /* If AFSK start failed send event but leave managers running. */
          if(driver == NULL) {
            pktAddEventFlags(handler, (EVT_AFSK_START_FAIL));
            break;
          }
          break;
        } /* End case PKT_RADIO_OPEN. */

        case MOD_NONE:
        case MOD_2FSK: {
          break;
        }
        break;
      } /* End switch on modulation type. */

      break;
    } /* End case PKT_RADIO_OPEN. */


    case PKT_RADIO_RX_START: {
      /* The function switches on mod type so no need for switch here. */
      switch(task_object->type) {
      case MOD_AFSK: {
        /* TODO: Add LLD abstraction. */
        pktAcquireRadio(radio, TIME_INFINITE);
        /* Enable receive. */
        if(pktLLDradioEnableReceive(radio, task_object))
          pktLLDradioStartDecoder(radio);

        /* Unlock radio and allow transmit requests. */
        pktReleaseRadio(radio);
        break;
        } /* End case MOD_AFSK. */

      case MOD_NONE:
      case MOD_2FSK: {
        break;
        }
      } /* End switch on task_object->type. */
      break;
    } /* End case PKT_RADIO_RX. */

    case PKT_RADIO_RX_STOP: {
      switch(task_object->type) {
            case MOD_AFSK: {
              /* TODO: Abstract acquire and release in LLD. */
              pktAcquireRadio(radio, TIME_INFINITE);
              pktLLDradioStopDecoder(radio);
              pktReleaseRadio(radio);
              break;
              } /* End case. */

            case MOD_NONE:
            case MOD_2FSK: {
              break;
              }
       } /* End switch. */
      break;
    } /* End case PKT_RADIO_RX_STOP. */

    case PKT_RADIO_TX_SEND: {
      /* Give each send a sequence number. */
      ++handler->radio_tx_config.tx_seq_num;
      /* Pause the decoder. */
      pktLLDradioPauseDecoding(radio);
      if(pktLLDradioSendPacket(task_object)) {
        /*
         * Keep count of active sends.
         * Shutdown or resume receive when all done.
         */
        handler->tx_count++;

        /* Send Successfully enqueued.
         * Unlike receive the task object is held by the TX until complete.
         * This is non blocking as each radio transmit runs in a thread.
         * The radio task object is released through a TX thread release task.
         */
        continue;
      }
      /* Send failed so release send packet object(s) and task object. */
      packet_t pp = task_object->packet_out;
      pktReleaseBufferChain(pp);
      if(pktIsReceivePaused(radio)) {
        if(!pktLLDradioResumeReceive(radio)) {
          TRACE_ERROR("RAD  > Receive on radio %d failed to "
              "resume after transmit", radio);
          break;
        }
        pktLLDradioResumeDecoding(radio);
      }
      break;
    } /* End case PKT_RADIO_TX. */

    case PKT_RADIO_RX_CLOSE: {
      event_listener_t el;
      event_source_t *esp;
      thread_t *decoder = NULL;
      switch(task_object->type) {
      case MOD_AFSK: {
        /* Stop receive. */
        pktLLDradioDisableReceive(radio);
        /* TODO: This should be a function back in pktservice or rxafsk. */
        esp = pktGetEventSource((AFSKDemodDriver *)handler->link_controller);
        pktRegisterEventListener(esp, &el, USR_COMMAND_ACK, DEC_CLOSE_EXEC);
        decoder = ((AFSKDemodDriver *)(handler->link_controller))->decoder_thd;

        /* TODO: Check that decoder will release in WAIT state.
         * Send event to release AFSK resources and terminate thread.
         */
        chEvtSignal(decoder, DEC_COMMAND_CLOSE);

        /* Then release common services and thread heap. */
        break;
        }

      case MOD_NONE:
      case MOD_2FSK: {
        break;
        } /* End case DECODE_FSK. */
      } /* End switch on link_type. */
      if(decoder == NULL)
        /* No decoder processed. */
        break;

      /* Wait for the decoder to stop. */
      eventflags_t evt;
      do {
        chEvtWaitAny(USR_COMMAND_ACK);

        /* Wait for correct event at source.
         */
        evt = chEvtGetAndClearFlags(&el);
      } while (evt != DEC_CLOSE_EXEC);
      pktUnregisterEventListener(esp, &el);

      /*
       *  Release decoder thread heap when it terminates.
       */
      chThdWait(decoder);

      /* Release packet services. */
      pktIncomingBufferPoolRelease(handler);
#if PKT_RX_RLS_USE_NO_FIFO != TRUE
      pktCallbackManagerRelease(handler);
#endif

      /*
       * Signal close completed for this session.
       * Any new open that is queued on the semaphore will be readied.
       */
      chBSemSignal(&handler->close_sem);
      break;
      } /*end case close. */

    case PKT_RADIO_TX_THREAD: {
      /* Get thread exit code and free memory. */
      msg_t send_msg = chThdWait(task_object->thread);

      if(send_msg == MSG_TIMEOUT) {
        TRACE_ERROR("RAD  > Transmit timeout on radio %d", radio);
      }
      if(send_msg == MSG_RESET) {
        TRACE_ERROR("RAD  > Transmit failed to start on radio %d", radio);
      }
      /* If no transmissions pending then enable RX or power down. */
      if(--handler->tx_count == 0) {
        /* Check at handler level is OK. No LLD required. */
        if(pktIsReceivePaused(radio)) {
          if(!pktLLDradioResumeReceive(radio)) {
            TRACE_ERROR("RAD  > Receive on radio %d failed to "
                "resume after transmit", radio);
            break;
          }
          /* TODO: Implement LLD since resume depends on radio and mod type. */
          pktLLDradioResumeDecoding(radio);
        } else {
          /* Enter standby state (low power). */
          TRACE_INFO("RAD  > Radio %d entering standby", radio);
          pktLLDradioStandby(radio);
        }
      } /* Else more TX tasks outstanding so let those complete. */
      break;
    } /* End case PKT_RADIO_TX_THREAD */

    } /* End switch on command. */
    /* Perform radio task callback if specified. */
    if(task_object->callback != NULL)
      /*
       * Perform the callback.
       * The callback should be brief and non-blocking.
       */
      task_object->callback(task_object);
    /* Return radio task object to free list. */
    chFifoReturnObject(radio_queue, (radio_task_object_t *)task_object);
  } /* End while should terminate(). */
  /* Thread has been terminated. */
  chFactoryReleaseObjectsFIFO(handler->the_radio_fifo);
  chThdExit(MSG_OK);
}

/**
 * Create the radio manager thread.
 */
thread_t *pktRadioManagerCreate(const radio_unit_t radio) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  //chDbgAssert(handler != NULL, "invalid radio ID");

  /* Create the radio manager name. */
  chsnprintf(handler->rtask_name, sizeof(handler->rtask_name),
             "%s%02i", PKT_RADIO_TASK_QUEUE_PREFIX, radio);

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

  /* Start the task dispatcher thread. */
  handler->radio_manager = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(PKT_RADIO_MANAGER_WA_SIZE),
              handler->rtask_name,
              NORMALPRIO - 10,
              pktRadioManager,
              handler);

  chDbgAssert(handler->radio_manager != NULL,
              "unable to create radio task thread");

  if(handler->radio_manager == NULL) {
    chFactoryReleaseObjectsFIFO(the_radio_fifo);
    handler->the_packet_fifo = NULL;
    return NULL;
  }
  msg_t init = chMsgSend(handler->radio_manager, MSG_OK);
  if(init == MSG_OK)
    return handler->radio_manager;

  /* Radio init failed so clean up. */

  chFactoryReleaseObjectsFIFO(the_radio_fifo);
  handler->the_packet_fifo = NULL;

  chThdTerminate(handler->radio_manager);

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
void pktRadioManagerRelease(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  /*
   * Get a task object to send to the manager.
   * The radio manager thread will terminate.
   * The FIFO is released in the manager thread before terminating.
   */
  radio_task_object_t *rto = NULL;
  (void)pktGetRadioTaskObject(radio, TIME_INFINITE, &rto);
  rto->command = PKT_RADIO_MGR_CLOSE;
  pktSubmitRadioTask(radio, rto, NULL);
  chThdWait(handler->radio_manager);
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
                            radio_task_object_t **rt) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  //chDbgAssert(handler != NULL, "invalid radio ID");

  dyn_objects_fifo_t  *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  *rt = chFifoTakeObjectI(task_queue);

  if(*rt == NULL) {
    /* No object available. */
    return MSG_TIMEOUT;
  }
  (*rt)->handler = handler;
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
void pktSubmitRadioTaskI(const radio_unit_t radio,
                         radio_task_object_t *object,
                         const radio_task_cb_t cb) {

  packet_svc_t *handler = pktGetServiceObject(radio);
  //chDbgAssert(handler != NULL, "invalid radio ID");

  dyn_objects_fifo_t *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /* Populate the object with information from request. */

  object->handler = handler;
  object->callback = cb;

  /*
   * Submit the task to the queue.
   * The task thread will process the request.
   * The task object is returned to the free list.
   * If a callback is specified it is called before the task object is freed.
   */
  chFifoSendObjectI(task_queue, object);
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
 * @retval  MSG_TIMEOUT an object could not be obtained within the timeout.
 * @retval  MSG_OK      an object has been fetched.
 *
 * @api
 */
msg_t pktGetRadioTaskObject(const radio_unit_t radio,
                            const sysinterval_t timeout,
                            radio_task_object_t **rt) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  //chDbgAssert(handler != NULL, "invalid radio ID");

  dyn_objects_fifo_t *task_fifo =
      chFactoryFindObjectsFIFO(handler->rtask_name);
  chDbgAssert(task_fifo != NULL, "unable to find radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  *rt = chFifoTakeObjectTimeout(task_queue, timeout);

  if(*rt == NULL) {
    /* Timeout waiting for object. */
    /* Release find reference to the FIFO (decrease reference count). */
    chFactoryReleaseObjectsFIFO(task_fifo);
    return MSG_TIMEOUT;
  }
  (*rt)->handler = handler;
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
                         const radio_task_cb_t cb) {

  packet_svc_t *handler = pktGetServiceObject(radio);
  //chDbgAssert(handler != NULL, "invalid radio ID");

  dyn_objects_fifo_t *task_fifo = handler->the_radio_fifo;
  chDbgAssert(task_fifo != NULL, "no radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /* Populate the object with information from request. */

  object->handler = handler;
  object->callback = cb;

  /*
   * Submit the task to the queue.
   * The task thread will process the request.
   * The task object is returned to the free list.
   * If a callback is specified it is called before the task object is freed.
   */
  chFifoSendObject(task_queue, object);
}

/**
 * @brief   Called by transmit threads to schedule release after completing.
 * @post    A thread release task is posted to the radio manager queue.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   thread  thread reference.
 *
 * @api
 */
void pktLLDradioSendComplete(radio_task_object_t *rto,
                              thread_t *thread) {

  packet_svc_t *handler = rto->handler;

  radio_unit_t radio = handler->radio;
  /* The handler and radio ID are set in returned object. */
  rto->command = PKT_RADIO_TX_THREAD;
  rto->thread = thread;
  /* Submit guaranteed to succeed by design. */
  pktSubmitRadioTask(radio, rto, rto->callback);
}

/**
 * @brief   Acquire exclusive access to radio.
 * @notes   returns when radio unit acquired.
 *
 * @param[in] radio     radio unit ID.
 * @param[in] timeout   time to wait for acquisition.
 *
 * @return              A message specifying the result.
 * @retval MSG_OK       if the radio has been successfully acquired.
 * @retval MSG_TIMEOUT  if the radio could not be acquired within specified time.
 * @retval MSG_RESET    if the radio can not be used due to a system abort.
 *
 * @api
 */
msg_t pktAcquireRadio(const radio_unit_t radio,
                      const sysinterval_t timeout) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  return chBSemWaitTimeout(&handler->radio_sem, timeout);
}

/**
 * @brief   Release exclusive access to radio.
 * @notes   returns when radio unit is released.
 *
 * @param[in] radio    radio unit ID.
 *
 * @api
 */
void pktReleaseRadio(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  chBSemSignal(&handler->radio_sem);
}

/*
 * TODO: Refactor this to use an array of strings.
 */
int pktDisplayFrequencyCode(const radio_freq_t code, char *buf, size_t size) {
  char* str = NULL;
  switch(code) {
  case FREQ_RADIO_INVALID:
    str = "No Code";
    break;

  case FREQ_APRS_DYNAMIC:
    str = "APRS Dynamic frequency";
    break;

  case FREQ_APRS_SCAN:
    str = "APRS Scan channel";
    break;

  case FREQ_APRS_RECEIVE:
    str = "APRS Receive frequency";
    break;

  case FREQ_CMDC_RECEIVE:
    str = "CNC Receive frequency";
    break;

  case FREQ_APRS_DEFAULT:
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
 * @brief   Get default operating frequency.
 *
 * @param[in] radio         Radio unit ID.
 *
 * @return      operating frequency
 * @retval      FREQ_RADIO_INVALID if radio ID is invalid
 * @retval      Default frequency otherwise
 *
 * @api
 */
radio_freq_t pktGetDefaultOperatingFrequency(const radio_unit_t radio) {
  /* FIXME: Default frequency in config to be per radio. */
  (void)radio;
  /* FIXME: INVALID relies on 0 in conf if no default set. */
  if(conf_sram.freq != FREQ_RADIO_INVALID)
    return conf_sram.freq;
  else
    return DEFAULT_OPERATING_FREQ;
}

/**
 * @brief   Get current receive operating frequency.
 *
 * @param[in] radio         Radio unit ID.
 *
 * @return      actual operating frequency or special code
 * @retval      0 if requested frequency or radio ID is invalid
 *
 * @notapi
 */
radio_freq_t pktGetReceiveOperatingFrequency(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  radio_freq_t op_freq;
  if(pktIsReceiveActive(radio)) {
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

/*
 * TODO: Rework to use max radio and start and PKT_RADIO_1 in loop.
 */
radio_freq_t pktCheckAllowedFrequency(const radio_unit_t radio,
                                      radio_freq_t freq) {
  /* Check validity. */
  uint8_t radios = pktGetNumRadios();
  const radio_config_t *list = pktGetRadioList();
  for(uint8_t i = 0; i < radios; i++) {
    if(list->unit == radio) {
      for(uint8_t x = 0; x < NUM_BANDS_PER_RADIO; x++) {
        if(list->band[x] == NULL)
          /* Vacant band slot in this radio. */
            continue;
        if(list->band[x]->start <= freq
            && freq < list->band[x]->end)
          return freq;
      }
    } /* End for bands */
  } /* End for radios*/
  return FREQ_RADIO_INVALID;
}

/**
 * Get radio data.
 */
const radio_config_t *pktGetRadioData(radio_unit_t radio) {
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
 * @brief   Compute an operating frequency.
 * @notes   All special frequency codes are resolved to an actual frequency.
 *
 * @param[in] radio         Radio unit ID.
 * @param[in] base_freq     Radio base frequency in Hz.
 * @param[in] step          Radio channel step size in Hz.
 * @param[in] chan          Radio channel number.
 *
 * @return      operating frequency
 * @retval      an absolute operating frequency in Hz.
 * @retval      FREQ_RADIO_INVALID if frequency or radio ID is invalid
 *
 * @api
 */
radio_freq_t pktComputeOperatingFrequency(const radio_unit_t radio,
                                          radio_freq_t base_freq,
                                          channel_hz_t step,
                                          radio_ch_t chan,
                                          const radio_mode_t mode) {

  if((base_freq == FREQ_APRS_RECEIVE || base_freq == FREQ_APRS_SCAN)
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
  if(base_freq == FREQ_APRS_DYNAMIC) {
    /*
     * Get frequency by geofencing.
     * Geofencing can return special code FREQ_APRS_DEFAULT.
     */
    base_freq = getAPRSRegionFrequency();
    step = 0;
    chan = 0;
  }

  /* Check for default. */
  if(base_freq == FREQ_APRS_DEFAULT) {
    base_freq = pktGetDefaultOperatingFrequency(radio);
    step = 0;
    chan = 0;
  }

  /* Calculate operating frequency. */
  radio_freq_t op_freq = base_freq + (step * chan);

  return pktCheckAllowedFrequency(radio, op_freq);
}

/**
 *
 */
bool pktLLDradioInit(const radio_unit_t radio) {
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

void pktLLDradioShutdown(const radio_unit_t radio) {
  /* TODO: Implement hardware mapping. */
  (void)radio;

  /*
   * Put radio in shutdown mode.
   * All registers are lost.
   */
  Si446x_radioShutdown(radio);
}

void pktLLDradioStandby(const radio_unit_t radio) {
  /* TODO: Implement hardware mapping. */
  (void)radio;

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
bool pktLLDradioSendPacket(radio_task_object_t *rto) {
  bool status;
  /* TODO: Implement VMT to functions per radio type. */
  switch(rto->type) {
  case MOD_2FSK:
    status = Si446x_blocSend2FSK(rto);
    break;

  case MOD_AFSK:
    status = Si446x_blocSendAFSK(rto);
    break;

  case MOD_NONE:
    status = false;
  } /* End switch on task_object->type. */
  return status;
}

/**
 * @brief   Enable reception.
 * @notes   This is the API interface to the radio LLD.
 * @notes   Currently just map directly to 446x driver.
 * @notes   In future would implement a lookup and VMT to access radio methods.
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
bool pktLLDradioEnableReceive(const radio_unit_t radio,
                         radio_task_object_t *rto) {
  packet_svc_t *handler = pktGetServiceObject(radio);

  //chDbgAssert(handler != NULL, "invalid radio ID");

  if(handler == NULL)
    return false;

  Si446x_setBandParameters(radio,
                           rto->base_frequency,
                           rto->step_hz);

  Si446x_receiveNoLock(radio,
                       rto->base_frequency,
                       rto->step_hz,
                       rto->channel,
                       rto->squelch,
                       rto->type);
  return true;
}

/**
 *
 */
void pktLLDradioDisableReceive(const radio_unit_t radio) {
  /* TODO: Implement hardware mapping. */
  Si446x_disableReceive(radio);
}


/**
 * @brief   Resume reception paused by transmit task.
 * @notes   This is the API interface to the radio LLD.
 * @notes   Currently just map directly to 446x driver.
 * @notes   In future would implement a lookup and VMT to access radio methods.
 *
 * @param[in] radio radio unit ID.
 *
 * @return  status of the operation
 * @retval  true    operation succeeded.
 * retval   false   operation failed.
 *
 * @notapi
 */
bool pktLLDradioResumeReceive(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);

  //chDbgAssert(handler != NULL, "invalid radio ID");

  radio_freq_t freq = handler->radio_rx_config.base_frequency;
  channel_hz_t step = handler->radio_rx_config.step_hz;
  radio_ch_t chan = handler->radio_rx_config.channel;
  radio_squelch_t rssi = handler->radio_rx_config.squelch;
  mod_t mod = handler->radio_rx_config.type;
  bool result = Si4464_resumeReceive(radio, freq, step, chan, rssi, mod);
  return result;
}

/**
 * @brief   Captures the current signal strength from the radio.
 * @notes   This is the API interface to the radio LLD.
 * @notes   Currently just map directly to 446x driver.
 * @notes   In future would implement a lookup and VMT to access radio methods.
 * @notes   The function should be called directly from the RX front end handler.
 * @notes   Calling from a deferred level will not capture the instantaneous level.
 *
 * @param[in] radio radio unit ID.
 *
 * @notapi
 */
void pktLLDradioCaptureRSSI(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  //chDbgAssert(handler != NULL, "invalid radio ID");
  handler->rx_stength = Si446x_getCurrentRSSI(radio);
}

/**
 *
 */
void pktLLDradioPauseDecoding(const radio_unit_t radio) {
  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only one at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher of radio driver.
   */
  pktPauseDecoding(radio);
}

/**
 *
 */
void pktLLDradioResumeDecoding(const radio_unit_t radio) {
  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only one at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */
  pktResumeDecoding(radio);
}

/**
 *
 */
void pktLLDradioStartDecoder(const radio_unit_t radio) {
  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only one at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */
  pktStartDecoder(radio);
}

/**
 *
 */
void pktLLDradioStopDecoder(const radio_unit_t radio) {
  /*
   * TODO: Implement as VMT inside radio driver (Si446x is only one at present).
   * - Lookup radio type from radio ID.
   * - Then call VMT dispatcher inside radio driver.
   */
  pktStopDecoder(radio);
}

/** @} */
