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

/**
 * @brief   Process radio task requests.
 * @notes   Task objects posted to the queue are processed per radio.
 * @notes   The queue is blocked while the radio driver executes.
 *
 * @param[in] arg pointer to a @p radio task queue for this thread.
 *
 * @return  status (MSG_OK) on exit.
 *
 * @notapi
 */
THD_FUNCTION(pktRadioManager, arg) {
#define PKT_RADIO_TASK_MANAGER_POLL_RATE    100
  dyn_objects_fifo_t *the_radio_fifo = arg;

  chDbgCheck(arg != NULL);

  //bool rx_active = false;

  objects_fifo_t *radio_queue = chFactoryGetObjectsFIFO(the_radio_fifo);

  chDbgAssert(radio_queue != NULL, "no queue in radio manager FIFO");

  while(!chThdShouldTerminateX()) {
    /* Check for task requests. */
    radio_task_object_t *task_object;
    msg_t fifo_msg = chFifoReceiveObjectTimeout(radio_queue,
                         (void *)&task_object,
                         TIME_MS2I(PKT_RADIO_TASK_MANAGER_POLL_RATE));
    if(fifo_msg == MSG_TIMEOUT)
      continue;

    /* Something to do. Handler pointer is in fifo_msg for RX tasks. */

    packet_svc_t *handler = task_object->handler;

    /* Process command. */
    switch(task_object->command) {
    case PKT_RADIO_RX_OPEN: {
      /* Create the packet management services. */
      pktBufferManagerCreate(handler);
      pktCallbackManagerCreate(handler);
      switch(task_object->type) {
        case MOD_AFSK: {
          /* Create the AFSK decoder (includes PWM, filters, etc.). */
          AFSKDemodDriver *driver = pktCreateAFSKDecoder(handler);
          handler->link_controller = driver;
          chDbgCheck(driver != NULL);
          /* TODO: Implement thread events or callback for results. */
          if(driver == NULL) {
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

      /* Initialise the radio. */
      Si446x_conditional_init();
      break;
    } /* End case PKT_RADIO_OPEN. */


    case PKT_RADIO_RX_START: {
      switch(task_object->type) {
      case MOD_AFSK: {
        Si446x_lockRadio(RADIO_RX);
        Si446x_setBandParameters(task_object->base_frequency,
                                task_object->step_hz,
                                RADIO_RX);
        pktStartDecoder(PKT_RADIO_1);

        radio_ch_t chan = task_object->channel;
        radio_squelch_t sq = task_object->squelch;

        Si446x_receiveNoLock(chan, sq, MOD_AFSK);

        /* Allow transmit requests. */
        Si446x_unlockRadio(RADIO_RX);
        //rx_active = true;
        break;
        } /* End case PKT_RADIO_RX. */

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
              pktStopDecoder(PKT_RADIO_1);
              //rx_active = false;
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
      /*
       * TODO: The 446x driver currently pauses decoding itself.
       * Consider moving it to here?
       */
      packet_t pp = task_object->packet_out;
      radio_freq_t freq = task_object->base_frequency;
      channel_hz_t step = task_object->step_hz;
      radio_ch_t chan = task_object->channel;
      uint8_t pwr = task_object->tx_power;
      uint32_t speed = task_object->tx_speed;


      switch(task_object->type) {
      case MOD_2FSK:
        Si446x_send2FSK(pp, chan, pwr, speed);
        break;

      case MOD_AFSK:
        Si446x_lockRadio(RADIO_TX);
        Si446x_setBandParameters(freq, step, RADIO_TX);
        Si446x_sendAFSK(pp, chan, pwr);
        break;

      case MOD_NONE:
        pktReleaseSendObject(pp);
        break;
      } /* End switch on task_object->type. */
      break;
    } /* End case PKT_RADIO_TX. */

    case PKT_RADIO_RX_CLOSE: {
      event_listener_t el;
      event_source_t *esp;
      thread_t *decoder = NULL;
      switch(task_object->type) {
      case MOD_AFSK: {
        Si446x_receiveStop();
        esp = pktGetEventSource((AFSKDemodDriver *)handler->link_controller);
        pktRegisterEventListener(esp, &el, USR_COMMAND_ACK, DEC_CLOSE_EXEC);
        decoder = ((AFSKDemodDriver *)(handler->link_controller))->decoder_thd;

        /* Send event to release AFSK resources and terminate thread. */
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
      pktBufferManagerRelease(handler);
      pktCallbackManagerRelease(handler);
      chBSemSignal(&handler->close_sem);
      break;
      } /*end case close. */
    } /* End switch on command. */
    /* Perform radio task callback if specified. */
    if(task_object->callback != NULL)
      /* Perform the callback. */
      task_object->callback(handler);
    /* Return radio task object to free list. */
    chFifoReturnObject(radio_queue, (radio_task_object_t *)task_object);
  } /* End while should terminate). */
  chThdExit(MSG_OK);
}


thread_t *pktRadioManagerCreate(packet_svc_t *handler) {

  /* The radio associated with this packet handler. */
  radio_unit_t rid = handler->radio_rx_config.radio_id;

  /* Create the radio manager name. */
  chsnprintf(handler->rtask_name, sizeof(handler->rtask_name),
             "%s%02i", PKT_RADIO_TASK_QUEUE_PREFIX, rid);

  dyn_objects_fifo_t *the_radio_fifo =
      chFactoryCreateObjectsFIFO(handler->rtask_name,
      sizeof(radio_task_object_t),
      RADIO_TASK_QUEUE_MAX, sizeof(msg_t));

  chDbgAssert(the_radio_fifo != NULL, "unable to create radio task queue");

  if(the_radio_fifo == NULL)
    return NULL;

  handler->the_radio_fifo = the_radio_fifo;

  dbgPrintf(DBG_INFO, "PKT  > radio manager thread created. FIFO @ 0x%x\r\n",
            the_radio_fifo);

  /* Start the task dispatcher thread. */
  handler->radio_manager = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(PKT_RADIO_MANAGER_WA_SIZE),
              handler->rtask_name,
              NORMALPRIO - 10,
              pktRadioManager,
              the_radio_fifo);

  chDbgAssert(handler->radio_manager != NULL,
              "unable to create radio task thread");

  if(handler->radio_manager == NULL) {
    chFactoryReleaseObjectsFIFO(the_radio_fifo);
    return NULL;
  }
  return handler->radio_manager;
}

/**
 * TODO: This needs review. Is it robust enough?
 */
void pktRadioManagerRelease(packet_svc_t *handler) {
  chThdTerminate(handler->radio_manager);
  chThdWait(handler->radio_manager);
  chFactoryReleaseObjectsFIFO(handler->the_radio_fifo);
}

/**
 * @brief   Get a radio command task object.
 * @post    A task object is returned ready for filling and submission.
 *
 * @param[in]   handler pointer to packet handler object.
 * @param[in]   timeout maximum time to wait for a task to be submitted.
 * @param[in]   rt      pointer to a task object pointer.
 *
 * @return  Status of the operation.
 * @retval  MSG_TIMEOUT an object could not be obtained within the timeout.
 * @retval  MSG_OK      an object has been fetched.
 *
 * @api
 */
msg_t pktGetRadioTaskObject(packet_svc_t *handler,
                            sysinterval_t timeout,
                            radio_task_object_t **rt) {
  dyn_objects_fifo_t *task_fifo =
      chFactoryFindObjectsFIFO(handler->rtask_name);
  chDbgAssert(task_fifo != NULL, "unable to find radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  *rt = chFifoTakeObjectTimeout(task_queue, TIME_MS2I(timeout));

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
 * @post    A task object is created and submitted to the radio manager.
 *
 * @param[in]   handler pointer to packet handler object.
 * @param[in]   object  radio task object to be submitted.
 * @param[in]   cb      function to call with result (can be NULL).
 *
 * @api
 */
void pktSubmitRadioTask(packet_svc_t *handler,
                         radio_task_object_t *object,
                         radio_task_cb_t cb) {

  dyn_objects_fifo_t *task_fifo =
      chFactoryFindObjectsFIFO(handler->rtask_name);
  chDbgAssert(task_fifo != NULL, "unable to find radio task fifo");

  objects_fifo_t *task_queue = chFactoryGetObjectsFIFO(task_fifo);
  chDbgAssert(task_queue != NULL, "no objects fifo list");

  /* Populate the object with information from request. */

  /* TODO: Put command information into queue object.
   * Have to do this so that commands are not overwritten in handler.
   */
  object->handler = handler;
  object->callback = cb;
  // etc.
  /*
   * Submit the task to the queue.
   * The task thread will process the request.
   * The task object is returned to the free list.
   * If a callback is specified it is called after the task object is freed.
   */
  chFifoSendObject(task_queue, object);

  /* Release reference to the FIFO acquired earlier by find. */
  chFactoryReleaseObjectsFIFO(task_fifo);
}

/** @} */
