/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file        rxafsk.c
 * @brief       AFSK channel.
 *
 * @addtogroup  channels
 * @{
 */

#include "pktconf.h"
#include "portab.h"

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/


/* AFSK FIR filters. */
/*float32_t pre_filter_coeff_f32[PRE_FILTER_NUM_TAPS] useCCM;
float32_t mag_filter_coeff_f32[MAG_FILTER_NUM_TAPS] useCCM;*/

/*
 * Data structure for AFSK decoding.
 */
#if PKT_SVC_USE_RADIO1 || defined(__DOXYGEN__)
AFSKDemodDriver AFSKD1;
#endif

#if PKT_SVC_USE_RADIO2 || defined(__DOXYGEN__)
AFSKDemodDriver AFSKD2;
#endif

/*===========================================================================*/
/* Decoder local variables and types.                                        */
/*===========================================================================*/

/*===========================================================================*/
/* Decoder local functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Check the symbol timing.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure
 *
 * @return  status  indicating if symbol decoding should take place.
 * @retval  true    decoding should run before getting next PWM entry.
 * @retval  false   continue immediately with next PWM data entry.
 *
 * @api
 */
static bool pktCheckAFSKSymbolTime(AFSKDemodDriver *myDriver) {
  /*
   * Each decoder filter is setup at init with a sample source.
   * This is set in the filter control structure.
   */

  switch(AFSK_DECODE_TYPE) {
    case AFSK_DSP_QCORR_DECODE: {
      return (get_qcorr_symbol_timing(myDriver));
    }

    case AFSK_DSP_FCORR_DECODE: {
      return (get_fcorr_symbol_timing(myDriver));
    }

    default: {
      chDbgAssert(myDriver != NULL, "Invalid AFSK decoder specified");
    }
  } /* end switch. */
  return false;
}

/**
 * @brief   Update the symbol timing PLL.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure
 *
 * @api
 */
static void pktUpdateAFSKSymbolPLL(AFSKDemodDriver *myDriver) {
  /*
   * Increment PLL timing.
   */

  switch(AFSK_DECODE_TYPE) {
    case AFSK_DSP_QCORR_DECODE: {
      update_qcorr_pll(myDriver);
      break;
    }

    case AFSK_DSP_FCORR_DECODE: {
      update_fcorr_pll(myDriver);
      break;
    }

    default: {
      chDbgAssert(myDriver != NULL, "Invalid AFSK decoder specified");
    }
  } /* end switch. */
  return;
}

/**
 * @brief   Add a sample to the decoder filter input.
 * @notes   The decimated entries are filtered through a BPF.
 *
 * @param[in]   myDriver   pointer to an @p AFSKDemodDriver structure.
 * @param[in]   binary     binary data from the PWM.
 *
 * @api
 */
static void pktAddAFSKFilterSample(AFSKDemodDriver *myDriver, bit_t binary) {
  switch(AFSK_DECODE_TYPE) {
    case AFSK_DSP_QCORR_DECODE: {
      (void)push_qcorr_sample(myDriver, binary);
      break;
    }

    case AFSK_DSP_FCORR_DECODE: {
      (void)push_fcorr_sample(myDriver, binary);
      break;
    }

    default: {
      chDbgAssert(myDriver != NULL, "Invalid AFSK decoder specified");
    }
  } /* End switch. */
}

/**
 * @brief   Process the input sample through the IQ correlation.
 * @notes   There are 4 filters that are run (I & Q for Mark and Space)
 *
 * @param[in]   myDriver   pointer to an @p AFSKDemodDriver structure.
 *
 * @return  filter  status
 * @retval  true    the filter output is valid.
 * @retval  false   the filter output in not yet valid.
 *
 * @api
 */
static bool pktProcessAFSKFilteredSample(AFSKDemodDriver *myDriver) {
  /*
   * Each decoder filter is setup at init with a sample source.
   * This is set in the filter control structure.
   */

  switch(AFSK_DECODE_TYPE) {
    case AFSK_DSP_QCORR_DECODE: {

      /*
       * Next perform the fixed point correlator update.
       * Result is updated MARK and SPACE bins.
       *
       */
      return process_qcorr_output(myDriver);
    }

    case AFSK_DSP_FCORR_DECODE: {
      return process_fcorr_output(myDriver);
    }

    default: {
      chDbgAssert(myDriver != NULL, "Invalid AFSK decoder specified");
    }
  } /* end switch. */
  return false;
}

/**
 * @brief   Decode AFSK symbol into an HDLC bit.
 * @notes   Called at symbol ready time as determined by decoders.
 *
 * @param[in]   myDriver   pointer to an @p AFSKDemodDriver structure.
 *
 * @return  token from HDLC processor.
 *
 * @api
 */
static hdlc_token_t pktDecodeAFSKSymbol(AFSKDemodDriver *myDriver) {
  /*
   * Called when a symbol timeline is ready.
   * Called from normal thread level.
   * TODO: No need to switch on type here really as decode structures are same.
   */

  switch(AFSK_DECODE_TYPE) {

    case AFSK_DSP_QCORR_DECODE: {
      /* Tone analysis is done per sample in QCORR. */
      qcorr_decoder_t *decoder = myDriver->tone_decoder;
      myDriver->rx_hdlc.tone_freq = decoder->current_demod;
      break;
    } /* End case AFSK_DSP_QCORR_DECODE. */

    case AFSK_DSP_FCORR_DECODE: {
      /* Tone analysis is done per sample in FCORR. */
      fcorr_decoder_t *decoder = myDriver->tone_decoder;
      myDriver->rx_hdlc.tone_freq = decoder->current_demod;
      break;
    } /* End case AFSK_DSP_FCORR_DECODE. */

    default:
      chDbgAssert(myDriver != NULL, "Invalid AFSK decoder specified");
  } /* End switch. */

  /* After tone detection generate an HDLC bit. */
  return pktExtractHDLCfromAFSK(&myDriver->rx_hdlc);
} /* End function. */

/**
 * @brief   Processes PWM into a decimated time line for AFSK decoding.
 * @notes   The decimated entries are filtered through a BPF.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure
 *
 * @return  status of operations.
 * @retval  MSG_OK      no error occurred so decimation can continue at next data.
 * @retval  MSG_ERROR   an error occurred and decimation should be aborted.
 *
 * @api
 */
static msg_t pktProcessAFSK(AFSKDemodDriver *myDriver,
                           min_pwmcnt_t current_tone[]) {
  /* Start working on new input data now. */
  packet_svc_t *myHandler = myDriver->packet_handler;
  radio_unit_t radio = myHandler->radio;
  radio_pwm_fifo_t *myFIFO = myDriver->active_demod_stream;

  /* Process the PWM data. */
  for(uint8_t i = 0; i < AFSK_NUM_TONES; i++) {
    myDriver->decimation_accumulator += current_tone[i];
    while(myDriver->decimation_accumulator >= 0) {
      /*
       *  The decoder will process a converted binary sample.
       *  The PWM binary is converted to a +/- sample value.
       *  The sample is passed to pre-filtering (i.e. BPF) as its next input.
       */
      pktAddAFSKFilterSample(myDriver, !(i & 1));

      /*
       * Process the sample at the output side of the pre-filter.
       * The filter chain returns true if the output is now valid.
       */
      if(pktProcessAFSKFilteredSample(myDriver)) {
        /* Filters are ready so decoding can commence. */
        if(pktCheckAFSKSymbolTime(myDriver)) {
          /* A symbol is ready to decode. */
          hdlc_token_t token = pktDecodeAFSKSymbol(myDriver);

          /* Switch on result from HDLC processor. */
          switch(token) {

          case HDLC_TOK_SYNC: {

            /* HDLC has detected an opening bit sync pattern. */
            myHandler->sync_count++;
#if 0
            if(myHandler->active_packet_object == NULL) {
              myDriver->active_demod_stream->status |= STA_AFSK_FRAME_SYNC;
              /* Allocate management object and packet buffer. */
              myHandler->active_packet_object =
                  pktAssignReceivePacketObject(myHandler, TIME_MS2I(100));
              /* If no management object/buffer is available get NULL. */
              if(myHandler->active_packet_object == NULL) {
                pktAddEventFlags(myHandler, EVT_PKT_NO_BUFFER);
                /*
                 * TODO: The STA_PKT_NO_BUFFER status is not checked anywhere.
                 * When RESET runs the STA_AFSK_DECODE_RESET status is set.
                 * This causes PWM to abort the current session.
                 */
                TRACE_DEBUG("AFSK > No packet buffer aborted decode on radio %d", radio);
                myDriver->active_demod_stream->status |= STA_PKT_NO_BUFFER;
                myDriver->decoder_state = DECODER_RESET;
                return MSG_ERROR;
              }
            } else {
              pktResetDataCount(myHandler->active_packet_object);
            }
#else
            /* Reset packet data count if a buffer has been assigned. */
            if(myHandler->active_packet_object != NULL)
              pktResetDataCount(myHandler->active_packet_object);
#endif

            break; /* Continue processing PWM stream. */
          } /* End case HDLC_TOK_SYNC. */

          case HDLC_TOK_FLAG: {

            /*
             * HDLC flag token should only occur after a frame is open.
             * Check if the frame has valid data in which case the flag closes the frame.
             */
            chDbgAssert(myHandler->active_packet_object != NULL, "No packet buffer in frame close");
            if(myHandler->active_packet_object->packet_size < PKT_MIN_FRAME) {
              /*
               *  Frame is too short.
               *  HDLC processor sets search state on detecting a flag.
               */
              pktResetDataCount(myHandler->active_packet_object);
              break; /* Continue processing PWM stream. */
            } /* End frame size check. */

            /* Frame is above minimum payload size. */
  #if PKT_RSSI_CAPTURE == TRUE
            /* Transfer the RSSI reading. */
            myHandler->active_packet_object->rssi = myFIFO->rssi;
  #endif
            myDriver->decoder_state = DECODER_DISPATCH;
            return MSG_OK; /* Packet is ready. */
          }

          case HDLC_TOK_RLL:
          case HDLC_TOK_FEED: {

            /* More HDLC bits needed. */
            break; /* Continue processing PWM stream. */
          } /* End case HDLC_TOK_RLL or HDLC_TOK_FEED. */

          case HDLC_TOK_RESET: {

            if(myHandler->active_packet_object == NULL) {
              /* Frame is not open.  HDLC processor is back in
                 HDLC_FRAME_SEARCH state. */
              pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_DECODE, PAL_LOW);
              break; /* Continue processing PWM stream. */
            }
#if 1
            /* Frame is open so reset data count.
               HDLC processor stays in open state. */
            pktResetDataCount(myHandler->active_packet_object);
            break;
#else
            myFIFO->status |= STA_AFSK_FRAME_RESET;
            if(myHandler->active_packet_object->packet_size < PKT_MIN_FRAME) {
              /* Payload below minimum. Go back to sync search. */
              pktResetDataCount(myHandler->active_packet_object);
              break; /* Continue processing PWM stream. */
            }

            /* Packet is greater than valid data size. */
            pktAddEventFlags(myHandler, EVT_HDLC_RESET_RCVD);
            TRACE_DEBUG("AFSK > HDLC reset aborted decode on radio %d", radio);
            myDriver->decoder_state = DECODER_RESET;
            return MSG_ERROR;
#endif
          } /* End case HDLC_TOK_RESET. */

          case HDLC_TOK_OPEN:
            /* The HDLC processor has received a string of HDLC flags.
             * The decoder PLL has had time to settle.
             * Transition to an open frame now.
             * Note this phase can be re-entered if HDLC RESET is encountered.
             */
            if(myHandler->active_packet_object == NULL) {
                myDriver->active_demod_stream->status |= STA_AFSK_FRAME_OPEN;
                /* Allocate management object and packet buffer. */
                myHandler->active_packet_object =
                    pktAssignReceivePacketObject(myHandler, TIME_MS2I(100));
                /* If no management object/buffer is available get NULL. */
                if(myHandler->active_packet_object == NULL) {
                  pktAddEventFlags(myHandler, EVT_PKT_NO_BUFFER);
                  /*
                   * TODO: The STA_PKT_NO_BUFFER status is not checked anywhere.
                   * When RESET runs the STA_AFSK_DECODE_RESET status is set.
                   * This causes PWM to abort the current session.
                   */
                  TRACE_DEBUG("AFSK > No packet buffer aborted decode on radio %d", radio);
                  myDriver->active_demod_stream->status |= STA_PKT_NO_BUFFER;
                  myDriver->decoder_state = DECODER_RESET;
                  return MSG_ERROR;
                }
              }
            pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_DECODE, PAL_HIGH);
            /* Falls through. */
          case HDLC_TOK_DATA: {
            /*
             * The decoder normally stays in this state during data decoding.
             * A packet management object & buffer has been assigned already.
             * If HDLC is detected then return an HDLC token.
             * Else just process data.
             * RLL encoding is catered for in data processing.
             */

            chDbgAssert(myHandler->active_packet_object != NULL,
                        "No packet buffer in data storage");

            /* Data is ready and available in MyDriver->rx_hdlc.current_byte. */
            if(!pktStoreReceiveData(myHandler->active_packet_object,
                                   getHDLCDataByte(myDriver))) {
              /*
               * AX25 character decoded but buffer is full.
               * Set error state and don't dispatch the AX25 buffer.
               */
              pktAddEventFlags(myHandler, EVT_PKT_BUFFER_FULL);
              TRACE_DEBUG("AFSK > Packet buffer full aborted decode on radio %d", radio);
              myFIFO->status |= STA_PKT_BUFFER_FULL;
              myDriver->decoder_state = DECODER_RESET;
              return MSG_ERROR; /* Terminate processing. */
            }
            myDriver->active_demod_stream->status |= STA_AFSK_FRAME_DATA;
            pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_DECODE, PAL_TOGGLE);
            break; /* Continue processing PWM stream. */
            } /* End case HDLC_TOK_DATA. */

          default:
            /* Unknown HDLC state machine token. */
            chDbgAssert(false, "Unknown HDLC token");
            myFIFO->status |= STA_AFSK_HDLC_ERROR;
            return MSG_ERROR;
          } /* End switch on HDLC decoder token. */
        } /* End if(pktCheckAFSKSymbolTime(myDriver)). */
        /* Update the symbol PLL. */
        pktUpdateAFSKSymbolPLL(myDriver);
      } /* End if(pktProcessAFSKFilteredSample(myDriver)) */
      /* Decrease decimation accumulator after processing. */
      myDriver->decimation_accumulator -= myDriver->decimation_size;
    } /* End while. Accumulator has underflowed. Get more PWM.*/
  } /* End for. */
  return MSG_OK;
}

/**
 * @brief   Reset the AFSK decoder and filter.
 * @notes   Called at startup of decoder and on completion of packet reception.
 * @post    Selected tone decoder and common AFSK data is initialized.
 *
 * @param[in]   myDriver   pointer to an @p AFSKDemodDriver structure.
 *
 * @api
 */
static bool pktResetAFSKDecoder(AFSKDemodDriver *myDriver) {
  /* Reset the HDLC decoder data.*/
#if 0
  myDriver->rx_hdlc.frame_state = HDLC_FLAG_SEARCH;
#if HDLC_SYNC_USE_COUNTER == TRUE
  myDriver->rx_hdlc.flag_count = 0;
#endif
  myDriver->rx_hdlc.tone_freq = TONE_NONE;
  myDriver->rx_hdlc.prior_freq = TONE_NONE;
  myDriver->rx_hdlc.bit_index = 0;
  //myDriver->hdlc_bits = (int32_t)-1;
  myDriver->rx_hdlc.hdlc_bits = (int32_t)-1;
#else
  pktResetHDLCProcessor(&myDriver->rx_hdlc);
#endif

  /* Reset the PWM decimation accumulator. */
  myDriver->decimation_accumulator = 0;

  switch(AFSK_DECODE_TYPE) {

    case AFSK_DSP_QCORR_DECODE: {
      /* Reset QCORR. */
      (void)reset_qcorr_all(myDriver);
      return true;
    }

    case AFSK_DSP_FCORR_DECODE: {
      /* Reset FCORR. */
      (void)reset_fcorr_all(myDriver);
      return true;
    }

    default:
    chDbgAssert(myDriver != NULL, "Invalid AFSK decoder specified");
    return false;
  } /* End switch. */
}

/**
 * @brief   Creates an AFSK channel which decodes PWM data from the radio.
 * @note    The si radio has no AFSK decoding capability.
 * @note    The PWM RX_DATA from the radio is decoded as AFSK by the uC.
 *
 * @post    An ICU and GPIO ports for the radio are attached and initialized.
 * @post    A dynamic object FIFO is created for streaming radio PWM data.
 * @post    A steam object is posted to demodulator where decoding takes place.
 * @post    Multiple PWM sessions may be queued by the radio for demodulation.
 *
 * @param[in]   pktHandler  pointer to a @p PKTDriver structure
 * @param[in]   radio       radio ID.
 *
 * @return  pointer to AFSK driver object.
 * @retval  NULL if initialization failed.
 *
 * @api
 */
AFSKDemodDriver *pktCreateAFSKDecoder(radio_unit_t radio) {
  const radio_config_t *data = pktGetRadioData(radio);
  chDbgAssert(data != NULL, "invalid radio ID");
  if(data == NULL)
    return NULL;

  /* Get afsk object from radio data. */
  AFSKDemodDriver *myDriver = data->afsk;

  /* Set the link from demod driver to the packet driver. */
  myDriver->packet_handler = pktGetServiceObject(radio);

  chDbgAssert(data != NULL, "invalid AFSK driver");

  /* Set the link from demod driver to the packet driver. */
  myDriver->packet_handler = pktGetServiceObject(radio);

  /* Set the caller thread for init handshake. */
  myDriver->caller = chThdGetSelfX();

  /* Create the AFSK decoder thread in system heap. */
  extern memory_heap_t *ccm_heap;
  myDriver->decoder_thd = chThdCreateFromHeap(ccm_heap,
              THD_WORKING_AREA_SIZE(PKT_AFSK_DECODER_WA_SIZE),
              myDriver->decoder_name,
              LOWPRIO,
              pktAFSKDecoder,
              myDriver);

  chDbgAssert(myDriver->decoder_thd != NULL,
              "error in AFSK decoder thread creation");

  if(myDriver->decoder_thd == NULL) {
    return NULL;
  }

  /* TODO: Add init start thread message handshake. */
  thread_t *thd = chMsgWait();
  msg_t msg = chMsgGet(thd);
  chMsgRelease(thd, MSG_OK);
  if(msg == MSG_ERROR)
    return NULL;

  return myDriver;
}

/**
 * @brief   Release AFSK resources.
 * @pre     All PWM buffers and FIFO objects must be free.
 * @post    The ICU is stopped.
 * @post    The dynamic object FIFO for PWM is released.
 * @post    The PWM queue pool heap is released.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure
 *
 * @api
 */
void pktReleaseAFSKDecoder(AFSKDemodDriver *myDriver) {
  chDbgAssert(myDriver != NULL, "no AFSK driver");
  chDbgAssert(myDriver->the_pwm_fifo != NULL, "no CCA FIFO");
  chDbgAssert(myDriver->icudriver != NULL, "no ICU driver");

  radio_unit_t radio = myDriver->packet_handler->radio;

  /* Stop PWM queue. */
  //pktDisableRadioStream(radio);
  //pktLockRadio(radio, RADIO_RX, TIME_INFINITE);
  //pktSetReceiveStreamInactive(radio, TIME_INFINITE);

  /* Detach radio from this AFSK driver. The GPIOs are
    disabled and the accociated ICU is stopped. */
  pktDetachRadio(radio);

  /* Release the PWM stream FIFO. */
  chFactoryReleaseObjectsFIFO(myDriver->the_pwm_fifo);
  myDriver->the_pwm_fifo = NULL;
  myDriver->pwm_fifo_pool = NULL;

  /*
   *  No PWM memory pool objects should be in use now.
   *  So just release the PWM pool heap.
   *  TODO: Add an assert that checks all objects are actually free.
   */
  chHeapFree(myDriver->pwm_queue_heap);
  myDriver->pwm_queue_heap = NULL;

  switch(AFSK_DECODE_TYPE) {
  case AFSK_DSP_QCORR_DECODE:
    release_qcorr_decoder(myDriver);
    break;

  case AFSK_DSP_FCORR_DECODE:
    release_fcorr_decoder(myDriver);
    break;

  default:
    chDbgAssert(myDriver != NULL, "Invalid AFSK decoder specified");
  } /* End switch. */
}

/**
 * @brief   Release PWM buffers in a stream.
 * @notes   There may be trailing PWM beyond the HDLC close flag.
 * @post    The PWM buffers are released back to the pool.
 * @post    The reference to the PWM object chain is removed.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure
 *
 * @api
 */
uint8_t pktReleasePWMbuffers(AFSKDemodDriver *myDriver) {
  chDbgAssert(myDriver != NULL, "no AFSK driver");
  radio_pwm_fifo_t *myFIFO = myDriver->active_demod_stream;
  if(myFIFO == NULL)
    return 0;
  /* Release queue/buffer objects back to the pool. */
  radio_pwm_object_t *object = myFIFO->decode_pwm_queue;
  if(object == NULL)
    return 0;
  radio_pwm_object_t *next;
  do {
    next = qGetLink(&object->queue);
    chPoolFree(&myDriver->pwm_buffer_pool, object);
#if TRACE_PWM_BUFFER_STATS == TRUE
    myDriver->active_demod_stream->rlsd++;
#endif
  } while((object = next) != NULL);
  myFIFO->decode_pwm_queue = NULL;
#if TRACE_PWM_BUFFER_STATS == TRUE
  return myDriver->active_demod_stream->rlsd;
#else
  return 0;
#endif
}

/**
 *
 */
bool pktIsAFSKReceiveActive(packet_svc_t *handler) {
  AFSKDemodDriver *myDemod = handler->rx_link_control;
  chDbgAssert(myDemod != NULL, "no demod driver");
  return myDemod->icustate == PKT_PWM_ACTIVE
    || myDemod->icustate == PKT_PWM_WAITING;
}

/*===========================================================================*/
/* AFSK Decoder thread.                                                      */
/*===========================================================================*/

THD_FUNCTION(pktAFSKDecoder, arg) {

  /*
   * Setup pointers to control structure and resources.
   */
  AFSKDemodDriver *myDriver = arg;
  chDbgAssert(myDriver != NULL, "invalid AFSK driver");
  packet_svc_t *myHandler = myDriver->packet_handler;
  radio_unit_t radio = myHandler->radio;

  /*
   * Initialize the decoder event object.
   */
  chEvtObjectInit(pktGetEventSource(myDriver));


  /* Create a PWM FIFO name for this radio. */
  chsnprintf(myDriver->pwm_fifo_name, sizeof(myDriver->pwm_fifo_name),
             "%s%02i", PKT_PWM_QUEUE_PREFIX, radio);

  /* Create the dynamic objects FIFO for the PWM data queue. */
  myDriver->the_pwm_fifo = chFactoryCreateObjectsFIFO(myDriver->pwm_fifo_name,
                                        sizeof(radio_pwm_fifo_t),
                                        NUMBER_PWM_FIFOS, sizeof(msg_t));

  chDbgAssert(myDriver->the_pwm_fifo != NULL, "failed to create PWM FIFO");

  if(myDriver->the_pwm_fifo == NULL) {
    chMsgSend(myDriver->caller, MSG_ERROR);
    /* We continue and terminate when caller sends release. */
    pktThdTerminateSelf();
  }

  /*
   * Create a memory pool of PWM queue objects in heap.
   * 1. Allocate heap memory
   * 2. Initialise pool manager
   * 3. Load heap with pool buffer objects
   */

  /*
   * Get heap to accommodate buffer objects.
   * The size of the allocation is:
   *  Number of slots for individual PWM entries in each buffer
   *  Multiplied by the number of chained buffers to be allocated
   */
  memory_heap_t *pwm_heap = NULL;
#if USE_CCM_BASED_PWM_HEAP == TRUE
  //extern memory_heap_t *ccm_heap;
  pwm_heap = ccm_heap;
#endif/* USE_CCM_BASED_PWM_HEAP == TRUE */
  myDriver->pwm_queue_heap = chHeapAllocAligned(pwm_heap,
        sizeof(radio_pwm_object_t) * PWM_DATA_BUFFERS, sizeof(msg_t));
  chDbgAssert(myDriver->pwm_queue_heap != NULL, "failed to create space "
                                                "in heap for PWM pool");
  /* Initialize the memory pool to manage buffer objects. */
  chPoolObjectInitAligned(&myDriver->pwm_buffer_pool,
                          sizeof(radio_pwm_object_t),
                          sizeof(msg_t), NULL);

  /* Load the memory pool with buffer objects. */
  chPoolLoadArray(&myDriver->pwm_buffer_pool,
                  myDriver->pwm_queue_heap,
                  PWM_DATA_BUFFERS);

  /* Get the objects FIFO . */
  myDriver->pwm_fifo_pool = chFactoryGetObjectsFIFO(myDriver->the_pwm_fifo);

  /* Indicate no buffer allocated. */
  myDriver->active_radio_stream = NULL;
  myDriver->active_demod_stream = NULL;

  /* Attach and initialize the ICU PWM system. */
  myDriver->icudriver = pktAttachRadio(myHandler->radio);

  /* Set the link from ICU driver to AFSK demod driver. */
  myDriver->icudriver->link = myDriver;

  /* Set state to initial/stopped. */
  myDriver->icustate = PKT_PWM_STOP;

  /* Create the packet buffer name. */
  chsnprintf(myDriver->decoder_name, sizeof(myDriver->decoder_name),
             "%s%02i", PKT_AFSK_THREAD_NAME_PREFIX, radio);

  tprio_t decoder_idle_priority;

  /* No active packet object. */
  myHandler->active_packet_object = NULL;

  /* Activity LED blink rate scaling variable. */
  int16_t led_count = 0;

#define DECODER_WAIT_TIME           100                         /* 100mS. */
#define DECODER_POLL_TIME           10                          /* 10mS. */
#define DECODER_LED_POLL_PULSE      (50/DECODER_POLL_TIME)      /* 50mS. */
#define DECODER_LED_POLL_CYCLE      (30000/DECODER_POLL_TIME)   /* 30S. */

  /* Set thread priority to different level when decoding./ */
#define DECODER_RUN_PRIORITY        NORMALPRIO+10

  /* Set DSP parameters for AFSK. */
  myDriver->decimation_size = ((pwm_accum_t)PWM_ICU_COUNT_FREQUENCY
                                / (pwm_accum_t)AFSK_BAUD_RATE)
                                / (pwm_accum_t)SYMBOL_DECIMATION;

  switch(AFSK_DECODE_TYPE) {
  case AFSK_DSP_QCORR_DECODE:
    init_qcorr_decoder(myDriver);
    break;

  case AFSK_DSP_FCORR_DECODE:
    init_fcorr_decoder(myDriver);
    break;

  default:
    chDbgAssert(myDriver != NULL, "Invalid AFSK decoder specified");
  }

  /* Save the priority that calling thread gave us. */
  decoder_idle_priority = chThdGetPriorityX();

  /* Setup LED for decoder blinker.  */
  pktLLDradioConfigIndicator(radio, PKT_INDICATOR_DECODE);
  pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_DECODE, PAL_HIGH);

  chMsgSend(myDriver->caller, MSG_OK);
  myDriver->decoder_state = DECODER_RESET;
  while(true) {
    switch(myDriver->decoder_state) {

      case DECODER_WAIT: {
        /*
         *  Wait for start or close event.
         */
#if 0
        eventmask_t evt = chEvtWaitAnyTimeout(DEC_COMMAND_START,
                                  TIME_MS2I(DECODER_WAIT_TIME));
        if(evt) {
          /* Reset decoder data ready for decode. */
          myDriver->decoder_state = DECODER_RESET;
          pktAddEventFlags(myDriver, DEC_START_EXEC);
          continue;
        }
        evt = chEvtGetAndClearEvents(DEC_COMMAND_CLOSE);
#else
        eventmask_t evt = chEvtWaitAnyTimeout(DEC_COMMAND_CLOSE,
                                  TIME_MS2I(DECODER_WAIT_TIME));
#endif
        if(evt) {
          pktAddEventFlags(myDriver, DEC_CLOSE_EXEC);
          pktReleaseAFSKDecoder(myDriver);
          myDriver->decoder_state = DECODER_TERMINATED;
          pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_DECODE, PAL_LOW);
          chThdExit(MSG_OK);
          /* Something went wrong if we arrive here. */
          chSysHalt("ThdExit");
        }

        /* Toggle indicator LED in wait state. */
        pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_DECODE, PAL_TOGGLE);
        continue;
      }

      case DECODER_TERMINATED:
        /* Something went wrong if we arrive here. */
        chSysHalt("ThdExit");
        break;

      case DECODER_IDLE: {
        /*
         *  Check for stop event.
         */
        eventmask_t evt = chEvtGetAndClearEvents(DEC_COMMAND_STOP);
        if(evt) {
          pktSetReceiveStreamStandby(radio, TIME_INFINITE);
          myDriver->decoder_state = DECODER_WAIT;
          pktAddEventFlags(myDriver, DEC_STOP_EXEC);
          continue;
        }
        myDriver->decoder_state = DECODER_POLL;
        continue;
      }  /* End case DECODER_IDLE. */

      case DECODER_POLL: {
        radio_pwm_fifo_t *myRadioFIFO;
        msg_t fifo_msg = chFifoReceiveObjectTimeout(myDriver->pwm_fifo_pool,
                             (void *)&myRadioFIFO,
                             TIME_MS2I(DECODER_POLL_TIME));
        if(fifo_msg != MSG_OK) {
          /* Give decoder LED a quick blink if we've been idle for > cycle time. */
          if(led_count < DECODER_LED_POLL_PULSE)
            pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_DECODE, PAL_HIGH);
          if(--led_count < 0) {
            /* Extinguish the indicator. */
            pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_DECODE, PAL_LOW);
            led_count = DECODER_LED_POLL_CYCLE;
          }
          /*
           * No FIFO object posted so loop.
           * Go back through IDLE and check for STOP event.
           */
          myDriver->decoder_state = DECODER_IDLE;
          continue;
        }
        /* Check if PWM queue object released in RESET state. */
        chDbgCheck(myDriver->active_demod_stream == NULL);

        /* Set current PWM stream object. */
        myDriver->active_demod_stream = myRadioFIFO;

        /* Check if prior packet buffer released. */
        chDbgCheck(myHandler->active_packet_object == NULL);

#if AFSK_DEBUG_TYPE == AFSK_PWM_DATA_CAPTURE_DEBUG
          char buf[80];
          int out = chsnprintf(buf, sizeof(buf),
                               "\r\n======= START ===========\r\n");
          pktWrite( (uint8_t *)buf, out);
#endif

        /* Increase thread priority. */
        decoder_idle_priority = chThdSetPriority(DECODER_RUN_PRIORITY);

        /* Capture receive frequency for this packet. */
        myRadioFIFO->freq = pktGetAbsoluteReceiveFrequency(radio);

        //TRACE_DEBUG("AFSK > PWM stream active on radio %d", radio);
        pktAddEventFlags(myDriver, EVT_AFSK_PWM_START);
        /* Enable processing of incoming PWM stream. */
        myDriver->decoder_state = DECODER_ACTIVE;
        continue;
      } /* End case DECODER_SESSION_POLL. */

      case DECODER_ACTIVE: {
        /*
         * We have a packet being processed.
         * Get PWM data from PWM queue.
         */
        radio_pwm_fifo_t *myFIFO = myDriver->active_demod_stream;

        if((myFIFO->status & STA_PWM_QUEUE_ERROR) != 0) {
          /*
           * Error occurred in PWM queue.
           * PWM side has set and broadcast event.
           * Abort this packet.
           */
          myDriver->decoder_state = DECODER_RESET;
          continue;
        }

        /* Get current PWM queue object address. */
        input_queue_t *myQueue = &myFIFO->decode_pwm_queue->queue;
        chDbgAssert(myQueue != NULL, "no queue assigned");
        byte_packed_pwm_t data;
        size_t n = iqReadTimeout(myQueue, data.bytes,
                                 sizeof(packed_pwm_counts_t),
                                 chTimeUS2I(833 * 8 * 20)
                                 /*TIME_MS2I(DECODER_ACTIVE_TIMEOUT)*/);
        /* Timeout calculated as SYMBOL time x 8 x 20. */

        if(n != sizeof(packed_pwm_counts_t)) {
          /*
           * PWM stream wait timeout.
           * No in-band close or abort in stream.
           */
          TRACE_DEBUG("AFSK > PWM stream timeout caused abort on radio %d", radio);
          pktAddEventFlags(myHandler, EVT_PWM_STREAM_TIMEOUT);
          myFIFO->status |= STA_PWM_STREAM_TIMEOUT;
          myDriver->decoder_state = DECODER_RESET;
          continue;
        }
        /*
         * PWM may be 12 or 16 bit in queue.
         *  Normalise to 16 bit PWM and overlayed byte stream. */
        array_min_pwm_counts_t stream;
        pktUnpackPWMData(data, &stream);

#if AFSK_DEBUG_TYPE == AFSK_PWM_DATA_CAPTURE_DEBUG
        char buf[80];
        int out = chsnprintf(buf, sizeof(buf), "%i, %i\r\n",
                  stream.pwm.impulse, stream.pwm.valley);
        pktWrite( (uint8_t *)buf, out);
#endif

        /* Look for "in band" message in PWM data. */
        if(stream.pwm.impulse == PWM_IN_BAND_PREFIX) {
          switch(stream.pwm.valley) {
          case PWM_TERM_PWM_STOP: {
            /*
             *  This may not be found if HDLC closing flag happened already.
             *  If found the PWM stream has been aborted by a stop request.
             *  Can be TX or a service stop as part of closing a service.
             *  PWM stop places an in-band message in an open PWM stream.
             *  The decoder will then (eventually) process the in-band message.
             *  The decode result is invalid due to being killed.
             *  The current AX25 buffer will not be dispatched.
             *  PWM and AX25 buffers and stream object are released in RESET.
             */
            TRACE_DEBUG("AFSK > PWM stream stopped on radio %d", radio);
            myFIFO->status |= STA_AFSK_PWM_STOPPED;
            myDriver->decoder_state = DECODER_RESET;
            pktAddEventFlags(myHandler, EVT_AFSK_PWM_STOP);
            continue; /* Continue in main loop. */
          } /* End case PWM_TERM_PWM_STOP. */

          case PWM_ACK_DECODE_ERROR:
          case PWM_ACK_DECODE_RESET: {
            /*
             * Decoder has set an end or error flag while PWM is still running.
             * The radio side closes the PWM stream.
             * It writes an in-band message in the open stream.
             * However we should never see that in-band.
             * The decoder has already moved out of ACTIVE state and is no longer processing PWM.
             * TODO: Deprecate these cases after confirming they never happen.
             */
            chDbgAssert(false, "unexpected in-band");
            TRACE_DEBUG("AFSK > unexpected in-band on radio %d", radio);
            pktAddEventFlags(myHandler, EVT_PWM_INVALID_INBAND);
            myFIFO->status |= STA_AFSK_UNEXPECTED_INBAND;
            myDriver->decoder_state = DECODER_RESET;

            continue; /* Decoder state switch. */
          } /* End case PWM_ACK_DECODE_END & PWM_ACK_DECODE_ERROR. */


          /*
           *  If PWM reports a zero impulse or valley.
           * The PWM side has already posted a PWM_STREAM_CLOSE event.
           */
          case PWM_TERM_ICU_ZERO:
            pktAddEventFlags(myHandler, EVT_PWM_ICU_ZERO);
            myFIFO->status |= STA_PWM_ICU_ZERO;
            myDriver->decoder_state = DECODER_RESET;
            continue; /* Decoder state switch. */

          /*
           * Occurs if PWM encounters an out of bounds impulse or valley.
           * This will be noise/weak signal related most likely.
           * The PWM side has passed the in-band message in place of the data.
           * If the frame is open and has data then it will be faulty so terminate now.
           * Otherwise discard the in-band and go back for more data.
           */
          case PWM_INFO_ICU_LIMIT:
            if (isHDLCFrameOpen(myDriver)) {
              if (myHandler->active_packet_object->packet_size < PKT_MIN_FRAME)
                continue;
              pktAddEventFlags(myHandler, EVT_PWM_ICU_LIMIT);
              myFIFO->status |= STA_PWM_ICU_LIMIT;
              myDriver->decoder_state = DECODER_RESET;
            }
            continue; /* Decoder state switch. */

            /*
             * If CCA ends and the decoder has not validated any frame.
             */
          case PWM_TERM_STREAM_CLOSE:
            pktAddEventFlags(myHandler, EVT_PWM_CCA_CLOSE);
            myDriver->decoder_state = DECODER_RESET;
            continue; /* Decoder state switch. */

            /*
             * If no PWM data is captured within a timeout.
             * The PWM side has already posted a PWM_NO_DATA event.
             */
          case PWM_TERM_NO_DATA:
            myFIFO->status |= STA_AFSK_PWM_NO_DATA;
            myDriver->decoder_state = DECODER_RESET;
            continue; /* Decoder state switch. */

            /*
             * Timeout of PWM data from radio.
             * The PWM side has already posted a EVT_PWM_RADIO_TIMEOUT event.
             * If the HDLC stream was good this in-band would not be seen.
             */
          case PWM_TERM_PWM_TIMEOUT:
            myFIFO->status |= STA_AFSK_PWM_TIMEOUT;
            myDriver->decoder_state = DECODER_RESET;
            continue; /* Decoder state switch. */

            /*
             * If the ICU timer overflows during PWM capture.
             * TODO: Could be handled like a limit exceeded.
             */
          case PWM_TERM_ICU_OVERFLOW:
            pktAddEventFlags(myHandler, EVT_PWM_ICU_OVERFLOW);
            myFIFO->status |= STA_PWM_ICU_OVERFLOW;
            myDriver->decoder_state = DECODER_RESET;
            continue; /* Decoder state switch. */

            /*
             * This is a debug error.
             * CCA is validated but a PWM is still active.
             * The PWM side has already posted a PWM_FIFO_ORDER event.
             */
          case PWM_TERM_QUEUE_ERR:

            /*
             * If there is no more PWM buffer space available.
             * The PWM side has already posted a PWM_QUEUE_FULL event.
             */
          case PWM_TERM_QUEUE_FULL: {
            /* Transit to RESET state where all buffers/objects are released. */
            //myFIFO->status |= STA_PWM_BUFFER_FULL;
            myDriver->decoder_state = DECODER_RESET;
            continue; /* Decoder state switch. */
          } /* End case PWM_TERM_QUEUE_FULL and fall through cases above it. */

          case PWM_INFO_QUEUE_SWAP: {
            /* Radio made a queue swap (filled the buffer). */
            /* Get reference to next queue/buffer object. */
            radio_pwm_object_t *nextObject = qGetLink(&myFIFO->decode_pwm_queue->queue);
            chDbgAssert(nextObject != NULL, "No linked queue in swap");
            if(nextObject != NULL) {
              /*
               *  Release the now empty prior buffer object back to the pool.
               *  Switch to the next queue/buffer object for decoding.
               */
              chPoolFree(&myDriver->pwm_buffer_pool, myFIFO->decode_pwm_queue);
#if TRACE_PWM_BUFFER_STATS == TRUE
              myFIFO->rlsd++;
              if(myDriver->frame_state == FRAME_SEARCH)
                myFIFO->sync++;
#endif
              myFIFO->decode_pwm_queue = nextObject;
            } else {
              /*
               *  This is an error condition.
               *  An in-band swap message should always have a linked object.
               *  Reset state releases AX25, PWM and stream FIFO objects.
               */
              pktAddEventFlags(myHandler, EVT_PWM_INVALID_SWAP);
              myFIFO->status |= STA_AFSK_INVALID_SWAP;
              myDriver->decoder_state = DECODER_RESET;
            }
            continue; /* Decoder state switch. */
          } /* End case PWM_INFO_QUEUE_SWAP. */

          default: {
            /* Unknown in-band message from PWM. */
            pktAddEventFlags(myHandler, EVT_PWM_INVALID_INBAND);
            myFIFO->status |= STA_AFSK_INVALID_INBAND;
            myDriver->decoder_state = DECODER_RESET;
            continue;  /* Enclosing state switch. */
            } /* End case default. */
          } /* End switch on in-band. */
          //continue; /* Decoder state switch. */
        } /* End if in-band. */

        /*
         * Not in-band so process the AFSK into an HDLC bit and AX25 data.
         * Processing checks HDLC, stores data and changes decoder state as required.
         */
        (void)pktProcessAFSK(myDriver, stream.array);
        continue;
      } /* End case DECODER_ACTIVE. */

      case DECODER_DISPATCH: {
        chDbgAssert(myHandler->active_packet_object != NULL, "No packet buffer in dispatch");
        /*
         * Indicate AFSK decode done.
         * PWM handler will terminate capture session if still active.
         */
        myDriver->active_demod_stream->status |= STA_AFSK_DECODE_DONE;

        /* Transfer receive information into packet buffer object. */
        myHandler->active_packet_object->status =
            myDriver->active_demod_stream->status;
        myHandler->active_packet_object->seq_num =
            myDriver->active_demod_stream->seq_num;
        myHandler->active_packet_object->freq =
            myDriver->active_demod_stream->freq;

        /* Dispatch the packet buffer object. */
        (void)pktDispatchReceivedBuffer(myHandler->active_packet_object);

        /* Packet object has been handed over. Remove our reference. */
        myHandler->active_packet_object = NULL;

        /* PWM buffers released and decoder reset executed in RESET state. */
        myDriver->decoder_state = DECODER_RESET;
        continue;
      } /* End case DECODER_DISPATCH. */

      /*
       * RESET readies the decoder for the next session.
       * It frees any held buffers/objects.
       * The DSP system is reset and then decoder transitions to IDLE.
       * The decode indicator is extinguished.
       */
      case DECODER_RESET: {
#if AFSK_DEBUG_TYPE == AFSK_PACKET_RESET_STATUS
        radio_pwm_fifo_t *demod_object = myDriver->active_demod_stream;
        char buf[120];
        if(demod_object != NULL) {
          int out = chsnprintf(buf, sizeof(buf),
                               "AFSK Reset demod status: %x\r\n",
                               demod_object->status);
          pktWrite( (uint8_t *)buf, out);
        } else {
          int out = chsnprintf(buf, sizeof(buf),
                               "AFSK Reset has no demod object\r\n");
          pktWrite( (uint8_t *)buf, out);
        }
#endif
        /* There won't be a packet buffer object if the decoder is just being reset. */
        if(myHandler->active_packet_object != NULL) {
#if AFSK_DEBUG_TYPE == AFSK_PWM_DATA_CAPTURE_DEBUG
          char buf[80];
          int out = chsnprintf(buf, sizeof(buf),
                               "\r\n======= STOP ===========\r\n");
          pktWrite( (uint8_t *)buf, out);
#endif
#if AFSK_DEBUG_TYPE == AFSK_AX25_RAW_PACKET_DUMP                             \
          || AFSK_DEBUG_TYPE == AFSK_PWM_DATA_CAPTURE_DEBUG
          pktDumpAX25Frame(myHandler->active_packet_object->buffer,
                           myHandler->active_packet_object->packet_size,
                           AX25_DUMP_RAW);
#endif
          pktReleaseDataBuffer(myHandler->active_packet_object);

          /* Forget the receive packet buffer management object. */
          myHandler->active_packet_object = NULL;
        }

        /*
         * Return stream FIFO to pool if there is one active.
         */
        radio_pwm_fifo_t *myFIFO = myDriver->active_demod_stream;
        /* There won't be a demod object if the decoder is just being reset. */
        if(myFIFO != NULL) {
          TRACE_DEBUG("AFSK > Decode %d end on radio %d, sync type %d, flag count %d, status 0x%08x and HDLC token %s",
                      myDriver->active_demod_stream->seq_num,
                      radio, myDriver->rx_hdlc.lead_type,
                      myDriver->rx_hdlc.flag_count,
                      myDriver->active_demod_stream->status,
                      pktGetHDLCTokenName(myDriver->rx_hdlc.last_token));
          /*
           * Signal the PWM queue that decoding is finished.
           * The PWM front end can stream switch if it is still associated
           * with this stream.
           */
          myDriver->active_demod_stream->status |= STA_AFSK_DECODE_RESET;

          /* Wait for stream control object to be released by the radio. */
          (void)chBSemWait(&myFIFO->sem);

          /*
           * Check the STA_PWM_STREAM_SWITCH status. If this is set
           * then the front end is still steaming PWM.
           * This occurs when CCA does not drop or a packet with a long HDLC tail.
           * In this case simply reset the HDLC processor.
           * The stream FIFO, PWM queue and DSP status remain as is.
           * State is set to ACTIVE and decoding continues with the HDLC bits
           * and the PWM stream already buffered.
           */
          if ((myDriver->active_demod_stream->status & STA_PWM_STREAM_SWITCH) != 0) {
            /* Only reset the HDLC processor and status for stream switch if
               there was no closed packet. Filters and decoders continue as is. */
            if ((myDriver->active_demod_stream->status & STA_AFSK_DECODE_DONE) == 0)
              /* This is a failed decode or a simple reset (no closed packet).
                 Reset HDLC processor so we re-start bit sync immediately. */
              pktResetHDLCProcessor(&myDriver->rx_hdlc);

            myDriver->active_demod_stream->status = 0;

            /* Set decoder back to active. */
            myDriver->decoder_state = DECODER_ACTIVE;
            continue;
          }

          /* Release PWM queue/buffer objects back to the pool. */
          radio_pwm_fifo_t *myFIFO = myDriver->active_demod_stream;
          uint8_t n = pktReleasePWMbuffers(myDriver);
#if TRACE_PWM_BUFFER_STATS == TRUE
          uint8_t u = myFIFO->in_use;
          TRACE_DEBUG("AFSK > PWM buffer use:"
              " pool %d, cycled %d, search %d, released %d, queue max %d",
              PWM_DATA_BUFFERS, u, myFIFO->sync, n, myFIFO->peak);
#else
          (void)n;
#endif

          /* Decode session is now closed. Release the FIFO stream object. */
          chFifoReturnObject(myDriver->pwm_fifo_pool, myFIFO);

          /* Forget the demod side reference to the object. */
          myDriver->active_demod_stream = NULL;
        }

        /* Reset the correlation decoder and its filters. */
        (void)pktResetAFSKDecoder(myDriver);

        /* Turn off decoder indicator and reset wink time interval. */
        pktLLDradioUpdateIndicator(radio, PKT_INDICATOR_DECODE, PAL_LOW);
        led_count = 0;

        (void)chThdSetPriority(decoder_idle_priority);

        /* Set decoder back to idle. */
        myDriver->decoder_state = DECODER_IDLE;
        continue;
      } /* End case DECODER_RESET. */
    } /* End switch on decoder state. */
  } /* End thread while(true). */
}

/** @} */
