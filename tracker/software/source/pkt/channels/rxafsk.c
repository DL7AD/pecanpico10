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

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/* TODO: Remove or recalculate Matlab/Octave filter coefficients. */

#if MAG_FILTER_GEN_COEFF == TRUE

float32_t mag_filter_coeff_f32[MAG_FILTER_NUM_TAPS] useCCM;

#else
/*
 * Magnitude (LPF) coefficients.
 * Fs=28800, f1 = 1400, number of taps = 29
 * Matlab/Octave parameters:
 * hc = fir1(28, 1400/(Fs/2), 'low');
 */

float32_t mag_filter_coeff_f32[MAG_FILTER_NUM_TAPS] = {
   -0.0016890122f, -0.0016647590f, -0.0016305187f, -0.0010016907f,
   0.0009925971f, 0.0051420766f, 0.0120550411f, 0.0219694930f,
   0.0346207799f, 0.0492010182f, 0.0644272330f, 0.0787120655f,
   0.0904082200f, 0.0980807163f, 0.1007534795f, 0.0980807163f,
   0.0904082200f, 0.0787120655f, 0.0644272330f, 0.0492010182f,
   0.0346207799f, 0.0219694930f, 0.0120550411f, 0.0051420766f,
   0.0009925971f, -0.0010016907f, -0.0016305187f, -0.0016647590f,
   -0.0016890122f
};
#endif /* PREQ_FILTER_GEN_COEFF == TRUE */


#if PRE_FILTER_GEN_COEFF == TRUE

float32_t pre_filter_coeff_f32[PRE_FILTER_NUM_TAPS] useCCM;

#else
/*
 * Pre-filter (BPF) coefficients.
 * Fs=28800, f1 = 925, f2 = 2475, number of taps = 311
 * Matlab/Octave parameters:
 * hc = fir1(310, [925, 2475]/(Fs/2), 'pass');
 */

float32_t pre_filter_coeff_f32[PRE_FILTER_NUM_TAPS] = {
  0.0002630141f, 0.0002628323f, 0.0002174784f, 0.0001439800f,
  0.0000662689f, 0.0000087403f, -0.0000104737f, 0.0000152369f,
  0.0000786174f, 0.0001598901f, 0.0002316217f, 0.0002660169f,
  0.0002428125f, 0.0001556541f, 0.0000150847f, -0.0001529389f,
  -0.0003129410f, -0.0004289345f, -0.0004742560f, -0.0004395107f,
  -0.0003362076f, -0.0001946865f, -0.0000564585f, 0.0000372882f,
  0.0000579911f, -0.0000007155f, -0.0001192216f, -0.0002555907f,
  -0.0003562318f, -0.0003710739f, -0.0002691589f, -0.0000500718f,
  0.0002525572f, 0.0005772283f, 0.0008494657f, 0.0010017974f,
  0.0009933667f, 0.0008233662f, 0.0005338281f, 0.0002000953f,
  -0.0000891793f, -0.0002573906f, -0.0002635755f, -0.0001170745f,
  0.0001218157f, 0.0003565040f, 0.0004809200f, 0.0004114493f,
  0.0001148717f, -0.0003763382f, -0.0009669091f, -0.0015204528f,
  -0.0018949887f, -0.0019829034f, -0.0017439270f, -0.0012209388f,
  -0.0005327554f, 0.0001555675f, 0.0006777111f, 0.0009141328f,
  0.0008292156f, 0.0004858414f, 0.0000315873f, -0.0003412629f,
  -0.0004518110f, -0.0001872172f, 0.0004567840f, 0.0013646274f,
  0.0023242755f, 0.0030807181f, 0.0034059854f, 0.0031663042f,
  0.0023666159f, 0.0011580344f, -0.0001962396f, -0.0013891652f,
  -0.0021544494f, -0.0023437972f, -0.0019738661f, -0.0012274564f,
  -0.0004059321f, 0.0001560330f, 0.0001905957f, -0.0004167487f,
  -0.0015805414f, -0.0030247966f, -0.0043492747f, -0.0051368262f,
  -0.0050731091f, -0.0040446519f, -0.0021854927f, 0.0001442042f,
  0.0024453937f, 0.0042118692f, 0.0050709497f, 0.0048929927f,
  0.0038363447f, 0.0023111613f, 0.0008681171f, 0.0000401684f,
  0.0001807741f, 0.0013455353f, 0.0032543614f, 0.0053501112f,
  0.0069426437f, 0.0074018164f, 0.0063466090f, 0.0037754712f,
  0.0000966124f, -0.0039567264f, -0.0075087956f, -0.0097752826f,
  -0.0102847504f, -0.0090233718f, -0.0064569060f, -0.0034162907f,
  -0.0008715304f, 0.0003478556f, -0.0001914293f, -0.0023698932f,
  -0.0055096823f, -0.0085320077f, -0.0102437760f, -0.0096804313f,
  -0.0064109478f, -0.0007150685f, 0.0064272605f, 0.0135454918f,
  0.0190429897f, 0.0216254883f, 0.0206778373f, 0.0164791731f,
  0.0101834165f, 0.0035516810f, -0.0015090359f, -0.0034886155f,
  -0.0017541991f, 0.0031705084f, 0.0096227405f, 0.0151387937f,
  0.0170529057f, 0.0132228528f, 0.0027036543f, -0.0138107229f,
  -0.0339131039f, -0.0538295639f, -0.0691388898f, -0.0757219326f,
  -0.0707320414f, -0.0533595170f, -0.0251988255f, 0.0098893389f,
  0.0464165977f, 0.0782860621f, 0.0999812724f, 0.1076699117f,
  0.0999812724f, 0.0782860621f, 0.0464165977f, 0.0098893389f,
  -0.0251988255f, -0.0533595170f, -0.0707320414f, -0.0757219326f,
  -0.0691388898f, -0.0538295639f, -0.0339131039f, -0.0138107229f,
  0.0027036543f, 0.0132228528f, 0.0170529057f, 0.0151387937f,
  0.0096227405f, 0.0031705084f, -0.0017541991f, -0.0034886155f,
  -0.0015090359f, 0.0035516810f, 0.0101834165f, 0.0164791731f,
  0.0206778373f, 0.0216254883f, 0.0190429897f, 0.0135454918f,
  0.0064272605f, -0.0007150685f, -0.0064109478f, -0.0096804313f,
  -0.0102437760f, -0.0085320077f, -0.0055096823f, -0.0023698932f,
  -0.0001914293f, 0.0003478556f, -0.0008715304f, -0.0034162907f,
  -0.0064569060f, -0.0090233718f, -0.0102847504f, -0.0097752826f,
  -0.0075087956f, -0.0039567264f, 0.0000966124f, 0.0037754712f,
  0.0063466090f, 0.0074018164f, 0.0069426437f, 0.0053501112f,
  0.0032543614f, 0.0013455353f, 0.0001807741f, 0.0000401684f,
  0.0008681171f, 0.0023111613f, 0.0038363447f, 0.0048929927f,
  0.0050709497f, 0.0042118692f, 0.0024453937f, 0.0001442042f,
  -0.0021854927f, -0.0040446519f, -0.0050731091f, -0.0051368262f,
  -0.0043492747f, -0.0030247966f, -0.0015805414f, -0.0004167487f,
  0.0001905957f, 0.0001560330f, -0.0004059321f, -0.0012274564f,
  -0.0019738661f, -0.0023437972f, -0.0021544494f, -0.0013891652f,
  -0.0001962396f, 0.0011580344f, 0.0023666159f, 0.0031663042f,
  0.0034059854f, 0.0030807181f, 0.0023242755f, 0.0013646274f,
  0.0004567840f, -0.0001872172f, -0.0004518110f, -0.0003412629f,
  0.0000315873f, 0.0004858414f, 0.0008292156f, 0.0009141328f,
  0.0006777111f, 0.0001555675f, -0.0005327554f, -0.0012209388f,
  -0.0017439270f, -0.0019829034f, -0.0018949887f, -0.0015204528f,
  -0.0009669091f, -0.0003763382f, 0.0001148717f, 0.0004114493f,
  0.0004809200f, 0.0003565040f, 0.0001218157f, -0.0001170745f,
  -0.0002635755f, -0.0002573906f, -0.0000891793f, 0.0002000953f,
  0.0005338281f, 0.0008233662f, 0.0009933667f, 0.0010017974f,
  0.0008494657f, 0.0005772283f, 0.0002525572f, -0.0000500718f,
  -0.0002691589f, -0.0003710739f, -0.0003562318f, -0.0002555907f,
  -0.0001192216f, -0.0000007155f, 0.0000579911f, 0.0000372882f,
  -0.0000564585f, -0.0001946865f, -0.0003362076f, -0.0004395107f,
  -0.0004742560f, -0.0004289345f, -0.0003129410f, -0.0001529389f,
  0.0000150847f, 0.0001556541f, 0.0002428125f, 0.0002660169f,
  0.0002316217f, 0.0001598901f, 0.0000786174f, 0.0000152369f,
  -0.0000104737f, 0.0000087403f, 0.0000662689f, 0.0001439800f,
  0.0002174784f, 0.0002628323f, 0.0002630141f
};
#endif

/*
 * Data structure for AFSK decoding.
 */
AFSKDemodDriver AFSKD1;

/*===========================================================================*/
/* Decoder local variables and types.                                        */
/*===========================================================================*/

/*===========================================================================*/
/* Decoder local functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Enables PWM stream from radio.
 * @post    The ICU is configured and started.
 * @post    The ports and timers for CCA input are configured.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure
 *
 * @api
 */
/* TODO: Move this into rxpwm.c ?? */
void pktEnablePWM(AFSKDemodDriver *myDriver) {

  chDbgAssert(myDriver->icudriver != NULL, "no ICU driver");

  /* Set callback for squelch events. */
  palSetLineCallback(LINE_CCA, (palcallback_t)pktRadioCCAInput,
                     myDriver->icudriver);

  pktICUStart(myDriver->icudriver);

  /* Enabling events on both edges of CCA.*/
  palEnableLineEvent(LINE_CCA, PAL_EVENT_MODE_BOTH_EDGES);

  myDriver->icustate = PKT_PWM_READY;
}

/**
 * @brief   Disables PWM stream from radio.
 * @post    The PWM channel is closed.
 * @post    All PWM related timers are stopped.
 * @post    The port for CCA input is disabled.
 * @post    The ICU remains ready to be restarted.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure
 * @api
 */
/* TODO: Move this into rxpwm.c ?? */
void pktDisablePWM(AFSKDemodDriver *myDriver) {

  chDbgAssert(myDriver->icudriver != NULL, "no ICU driver");

  chSysLock();

  /* Close the PWM stream. */
  pktClosePWMChannelI(myDriver->icudriver, 0, PWM_TERM_DECODE_STOP);

  /* Stop ICU capture. */
  icuStopCaptureI(myDriver->icudriver);

  /* Stop any timeouts in ICU handling. */
  pktStopAllICUTimersI(myDriver->icudriver);

  /* Disable CCA port event. */
  palDisableLineEventI(LINE_CCA);

  myDriver->icustate = PKT_PWM_STOP;

  /* Reschedule is required to avoid a "priority order violation". */
  chSchRescheduleS();
  chSysUnlock();
}

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
bool pktCheckAFSKSymbolTime(AFSKDemodDriver *myDriver) {
  /*
   * Each decoder filter is setup at init with a sample source.
   * This is set in the filter control structure.
   */

  switch(AFSK_DECODE_TYPE) {
    case AFSK_DSP_QCORR_DECODE: {

      /*
       * Check if symbol decode should be run now.
       */
      return (get_qcorr_symbol_timing(myDriver));
    }

    case AFSK_DSP_FCORR_DECODE: {
      break;
    }

    default: {
      break;
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
void pktUpdateAFSKSymbolPLL(AFSKDemodDriver *myDriver) {
  /*
   * Increment PLL timing.
   */

  switch(AFSK_DECODE_TYPE) {
    case AFSK_DSP_QCORR_DECODE: {

      /*
       *
       */

      update_qcorr_pll(myDriver);
      break;
    }

    case AFSK_DSP_FCORR_DECODE: {

    }

    default: {
      break;
    }
  } /* end switch. */
  return;
}

/**
 * @brief   Processes PWM into a decimated time line for AFSK decoding.
 * @notes   The decimated entries are filtered through a BPF.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure
 *
 * @return  status of operations.
 * @retval  true    no error occurred so decimation can continue at next data.
 * @retval  false   an error occurred and decimation should be aborted.
 *
 * @api
 */
bool pktProcessAFSK(AFSKDemodDriver *myDriver, min_pwmcnt_t current_tone[]) {
  /* Start working on new input data now. */
  uint8_t i = 0;
  for(i = 0; i < (sizeof(min_pwm_counts_t) / sizeof(min_pwmcnt_t)); i++) {
    myDriver->decimation_accumulator += current_tone[i];
    while(myDriver->decimation_accumulator >= 0) {

      /*
       *  The decoder will process a converted binary sample.
       *  The PWM binary is converted to a q31 +/- sample value.
       *  The sample is passed to pre-filtering (i.e. BPF) as its next input.
       */
      (void)pktAddAFSKFilterSample(myDriver, !(i & 1));

      /*
       * Process the sample at the output side of the pre-filter.
       * The filter returns true if its output is now valid.
       */
      if(pktProcessAFSKFilteredSample(myDriver)) {
        /* Filters are ready so decoding can commence. */
        if(pktCheckAFSKSymbolTime(myDriver)) {
          /* A symbol is ready to decode. */
          if(!pktDecodeAFSKSymbol(myDriver))
            /* Unable to store character - buffer full. */
            return false;
        }
        pktUpdateAFSKSymbolPLL(myDriver);
      }
      myDriver->decimation_accumulator -= myDriver->decimation_size;
    } /* End while. Accumulator has underflowed. */
  } /* End for. */
  return true;
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
void pktAddAFSKFilterSample(AFSKDemodDriver *myDriver, bit_t binary) {
  switch(AFSK_DECODE_TYPE) {
    case AFSK_DSP_QCORR_DECODE: {
      (void)push_qcorr_sample(myDriver, binary);
      break;
    }

    case AFSK_DSP_FCORR_DECODE: {
      //(void)push_fcorr_sample(myDriver, binary);
      break;
    }

    default: {
      break;
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
bool pktProcessAFSKFilteredSample(AFSKDemodDriver *myDriver) {
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

    }

    default: {
      break;
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
 * @return  status of operation
 * @retval  true - success
 * @retval  false - an error occurred in processing (buffer full)
 *
 * @api
 */
bool pktDecodeAFSKSymbol(AFSKDemodDriver *myDriver) {
  /*
   * Called when a symbol timeline is ready.
   * Called from normal thread level.
   */

  switch(AFSK_DECODE_TYPE) {

    case AFSK_DSP_QCORR_DECODE: {
      /* Tone analysis is done per sample in QCORR. */
      qcorr_decoder_t *decoder = myDriver->tone_decoder;
      myDriver->tone_freq = decoder->current_demod;
      break;
    } /* End case AFSK_DSP_QCORR_DECODE. */

    case AFSK_DSP_FCORR_DECODE: {
      /* Tone analysis is done per sample in FCORR. */
      break;
    } /* End case AFSK_DSP_FCORR_DECODE. */

    case AFSK_NULL_DECODE: {
      /*
       * Do nothing (used when in debug capture mode).
       */
      return true;
    } /* End case AFSK_NULL_DECODE. */
  } /* End switch. */

  /* After tone detection generate an HDLC bit. */
  return pktExtractHDLCfromAFSK(myDriver);
} /* End function. */

/**
 * @brief   Reset the AFSK decoder and filter.
 * @notes   Called at completion of packet reception.
 * @post    Selected tone decoder and common AFSK data is initialized.
 *
 * @param[in]   myDriver   pointer to an @p AFSKDemodDriver structure.
 *
 * @api
 */
void pktResetAFSKDecoder(AFSKDemodDriver *myDriver) {
  /*
   * Called when a decode stream has completed.
   * Called from normal thread level.
   */

  /* Reset the decoder data.*/
  myDriver->frame_state = FRAME_SEARCH;
  myDriver->prior_freq = TONE_NONE;
  myDriver->bit_index = 0;
  myDriver->decimation_accumulator = 0;

  /* Set the hdlc bits to all ones. */
  myDriver->hdlc_bits = (int32_t)-1;

  switch(AFSK_DECODE_TYPE) {

    case AFSK_DSP_QCORR_DECODE: {
      /* Reset QCORR. */
      (void)reset_qcorr_all(myDriver);
      break;
    }

    case AFSK_DSP_FCORR_DECODE: {
      /* TODO: Reset FCORR. */
      break;
    }

    case AFSK_NULL_DECODE: {
      /*
       * Do nothing (used when in debug capture mode).
       */
      break;
    } /* End case AFSK_NULL_DECODE. */
  } /* End switch. */
}

/**
 * @brief   Creates an AFSK channel which decodes PWM data from the radio.
 * @note    The si radio has no AFSK decoding capability.
 * @note    The PWM RX_DATA from the radio is decoded as AFSK by the uC.
 *
 * @post    An ICU and GPIO ports for the Radio are attached and initialized.
 * @post    A dynamic object FIFO is created for buffering radio PWM data.
 * @post    Buffers are posted to demodulator where decoding takes place.
 * @post    Multiple PWM sessions may be queued by the Radio for demodulation.
 *
 * @param[in]   pktHandler  pointer to a @p PKTDriver structure
 * @param[in]   radio       radio ID.
 *
 * @return  pointer to AFSK driver object.
 * @retval  NULL if initialization failed.
 *
 * @api
 */
AFSKDemodDriver *pktCreateAFSKDecoder(packet_svc_t *pktHandler) {

  chDbgAssert(pktHandler != NULL, "no packet handler");

  AFSKDemodDriver *myDriver = &AFSKD1;

  /*
   * Initialize the decoder event object.
   */
  chEvtObjectInit(pktGetEventSource(myDriver));

  /* Set the link from demod driver to the packet driver. */
  myDriver->packet_handler = pktHandler;

  /* The radio associated with this AFSK driver. */
  radio_unit_t rid = myDriver->packet_handler->radio;

  /* Create a PWM FIFO name for this radio. */
  chsnprintf(myDriver->pwm_fifo_name, sizeof(myDriver->pwm_fifo_name),
             "%s%02i", PKT_PWM_QUEUE_PREFIX, rid);

  /* Create the dynamic objects FIFO for the PWM data queue. */
  myDriver->the_pwm_fifo = chFactoryCreateObjectsFIFO(myDriver->pwm_fifo_name,
                                        sizeof(radio_pwm_fifo_t),
                                        NUMBER_PWM_FIFOS, sizeof(msg_t));

  chDbgAssert(myDriver->the_pwm_fifo != NULL, "failed to create PWM FIFO");

  if(myDriver->the_pwm_fifo == NULL) {
    return NULL;
  }

#if USE_HEAP_PWM_BUFFER == TRUE
  /*
   * Create a memory pool of PWM queue objects in heap.
   * 1. Allocate heap memory
   * 2. Initialise pool manager
   * 3. Load heap with pool objects
   */

  /*
   * Get heap to accommodate buffer objects.
   * The size of the allocation is:
   *  Number of slots for individual PWM entries in each buffer
   *  Multiplied by the number of chainable buffers to be allocated
   */
#if USE_CCM_BASED_HEAP == TRUE
  extern memory_heap_t *ccm_heap;
  myDriver->pwm_queue_heap = chHeapAlloc(ccm_heap,
        sizeof(radio_pwm_object_t) * PWM_DATA_BUFFERS);
#else
  myDriver->pwm_queue_heap = chHeapAlloc(NULL,
        sizeof(radio_pwm_object_t) * PWM_DATA_BUFFERS);
#endif
  chDbgAssert(myDriver->pwm_queue_heap != NULL, "failed to create PWM heap");
  /* Initialize the memory pool to manage buffer objects. */
  chPoolObjectInitAligned(&myDriver->pwm_buffer_pool,
                          sizeof(radio_pwm_object_t),
                          sizeof(msg_t), NULL);
  /* Load the memory pool with buffer objects. */
  chPoolLoadArray(&myDriver->pwm_buffer_pool,
                  myDriver->pwm_queue_heap,
                  PWM_DATA_BUFFERS);
#endif

  /* Get the objects FIFO . */
  myDriver->pwm_fifo_pool = chFactoryGetObjectsFIFO(myDriver->the_pwm_fifo);

  /* Indicate no buffer allocated. */
  myDriver->active_radio_object = NULL;
  myDriver->active_demod_object = NULL;

  /* Attach and initialize the ICU PWM system. */
  myDriver->icudriver = pktAttachICU(pktHandler->radio);

  /* Set the link from ICU driver to AFSK demod driver. */
  myDriver->icudriver->link = myDriver;
  myDriver->icustate = PKT_PWM_READY;

  /* Create the packet buffer name. */
  chsnprintf(myDriver->decoder_name, sizeof(myDriver->decoder_name),
             "%s%02i", PKT_AFSK_THREAD_NAME_PREFIX, rid);

  /* Create the AFSK decoder thread. */
  extern memory_heap_t *ccm_heap;
  myDriver->decoder_thd = chThdCreateFromHeap(ccm_heap,
              THD_WORKING_AREA_SIZE(PKT_AFSK_DECODER_WA_SIZE),
              myDriver->decoder_name,
              NORMALPRIO - 10,
              pktAFSKDecoder,
              myDriver);

  chDbgAssert(myDriver->decoder_thd != NULL,
              "error in decoder thread creation");

  if(myDriver->decoder_thd == NULL) {
    chFactoryReleaseObjectsFIFO(myDriver->the_pwm_fifo);
    return NULL;
  }

  /* Set DSP parameters. */
  myDriver->decimation_size = ((pwm_accum_t)ICU_COUNT_FREQUENCY
                                / (pwm_accum_t)AFSK_BAUD_RATE)
                                / (pwm_accum_t)SYMBOL_DECIMATION;

  /* Generate the BPF and LPF filter coordinates. */
#if PRE_FILTER_GEN_COEFF == TRUE

  gen_fir_bpf((float32_t)PRE_FILTER_LOW / (float32_t)FILTER_SAMPLE_RATE,
              (float32_t)PRE_FILTER_HIGH / (float32_t)FILTER_SAMPLE_RATE,
              pre_filter_coeff_f32,
              PRE_FILTER_NUM_TAPS,
              TD_WINDOW_NONE);
#endif

#if MAG_FILTER_GEN_COEFF == TRUE

  gen_fir_lpf((float32_t)MAG_FILTER_HIGH / (float32_t)FILTER_SAMPLE_RATE,
              mag_filter_coeff_f32,
              MAG_FILTER_NUM_TAPS,
              TD_WINDOW_NONE);
#endif

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

  /* Stop PWM queue. */
  pktDisablePWM(myDriver);

  /* Detach ICU from this AFSK driver. */
  pktDetachICU(myDriver->icudriver);

  /* Release the PWM FIFO. */
  chFactoryReleaseObjectsFIFO(myDriver->the_pwm_fifo);
  myDriver->the_pwm_fifo = NULL;
  myDriver->pwm_fifo_pool = NULL;

#if USE_HEAP_PWM_BUFFER == TRUE
  /*
   *  No memory pool objects should be in use.
   *  So just release the PWM pool heap.
   */
  chHeapFree(myDriver->pwm_queue_heap);
  myDriver->pwm_queue_heap = NULL;
#endif
}

#if USE_HEAP_PWM_BUFFER == TRUE
/**
 * @brief   Release PWM buffers.
 * @post    The PWM buffers are released back to the pool.
 * @post    The reference to the PWM object chain is removed.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure
 *
 * @api
 */
uint8_t pktReleasePWMbuffers(AFSKDemodDriver *myDriver) {
  radio_pwm_fifo_t *myFIFO = myDriver->active_demod_object;
  /* Release queue/buffer objects back to the pool. */
  radio_pwm_object_t *object = myFIFO->decode_pwm_queue;
  if(object == NULL)
    return 0;
  radio_pwm_object_t *next;
  do {
    next = qGetLink(&object->queue);
    chPoolFree(&myDriver->pwm_buffer_pool, object);
    myDriver->active_demod_object->rlsd++;
  } while((object = next) != NULL);
  myFIFO->decode_pwm_queue = NULL;
  return myDriver->active_demod_object->rlsd;
}
#endif

/*===========================================================================*/
/* AFSK Decoder thread.                                                      */
/*===========================================================================*/

THD_FUNCTION(pktAFSKDecoder, arg) {

  /*
   * Setup pointers to control structure and resources.
   */
  AFSKDemodDriver *myDriver = arg;
  packet_svc_t *myHandler = myDriver->packet_handler;

  /* No active packet object. */
  myHandler->active_packet_object = NULL;

  /* Activity LED blink rate scaling variable. */
  int16_t led_count = 0;

#define DECODER_WAIT_TIME           100    /* 100mS. */
#define DECODER_POLL_TIME           10    /* 10mS. */
#define DECODER_LED_POLL_PULSE      (50/DECODER_POLL_TIME) /* 50mS. */
#define DECODER_LED_POLL_CYCLE      (30000/DECODER_POLL_TIME)    /* 30S. */

  /* Set thread priority to different level when decoding./ */
#define DECODER_RUN_PRIORITY        NORMALPRIO+10

#if AFSK_DECODE_TYPE == AFSK_DSP_QCORR_DECODE
  init_qcorr_decoder(myDriver);
#endif

  /* Save the priority that calling thread gave us. */
  tprio_t decoder_idle_priority = chThdGetPriorityX();

  /* Setup LED for decoder blinker. */
  pktSetGPIOlineMode(LINE_DECODER_LED, PAL_MODE_OUTPUT_PUSHPULL);

  pktWriteGPIOline(LINE_DECODER_LED, PAL_HIGH);

   /* Acknowledge open then wait for start or close of decoder. */
  pktAddEventFlags(myDriver, DEC_OPEN_EXEC);
  myDriver->decoder_state = DECODER_WAIT;
  while(true) {
    switch(myDriver->decoder_state) {

      case DECODER_WAIT: {
        /*
         *  Wait for start or close event.
         */
        eventmask_t evt = chEvtWaitAnyTimeout(DEC_COMMAND_START,
                                  TIME_MS2I(DECODER_WAIT_TIME));
        if(evt) {
          pktEnablePWM(myDriver);
          myDriver->decoder_state = DECODER_RESET;
          pktAddEventFlags(myDriver, DEC_START_EXEC);
          break;
        }
        evt = chEvtGetAndClearEvents(DEC_COMMAND_CLOSE);
        if(evt) {
          pktAddEventFlags(myDriver, DEC_CLOSE_EXEC);
          pktReleaseAFSKDecoder(myDriver);
          myDriver->decoder_state = DECODER_TERMINATED;
          pktWriteGPIOline(LINE_DECODER_LED, PAL_LOW);
          chThdExit(MSG_OK);
          /* Something went wrong if we arrive here. */
          chSysHalt("ThdExit");
        }
        /* Toggle decoder LED in wait state. */
        pktWriteGPIOline(LINE_DECODER_LED, PAL_TOGGLE);
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
          pktDisablePWM(myDriver);
          myDriver->decoder_state = DECODER_WAIT;
          pktAddEventFlags(myDriver, DEC_STOP_EXEC);
          break;
        }
        myDriver->decoder_state = DECODER_POLL;
        break;
      }  /* End case DECODER_IDLE. */

      case DECODER_POLL: {
        radio_pwm_fifo_t *myRadioFIFO;
        msg_t fifo_msg = chFifoReceiveObjectTimeout(myDriver->pwm_fifo_pool,
                             (void *)&myRadioFIFO,
                             TIME_MS2I(DECODER_POLL_TIME));
        if(fifo_msg != MSG_OK) {
          /* Give decoder LED a quick blink if we've been idle for > cycle time. */
          if(led_count < DECODER_LED_POLL_PULSE)
            pktWriteGPIOline(LINE_DECODER_LED, PAL_HIGH);
          if(--led_count < 0) {
            pktWriteGPIOline(LINE_DECODER_LED, PAL_LOW);
            led_count = DECODER_LED_POLL_CYCLE;
          }
          /*
           * No FIFO object posted so loop.
           * Go back through IDLE and check for STOP event.
           */
          myDriver->decoder_state = DECODER_IDLE;
          break;
        }
        /* Check if PWM queue object released in RESET state. */
        chDbgCheck(myDriver->active_demod_object == NULL);

        /* Set current PWM queue object. */
        myDriver->active_demod_object = myRadioFIFO;

        /* Check if prior packet buffer released. */
        chDbgCheck(myHandler->active_packet_object == NULL);

        /* Get a packet buffer. */
        dyn_objects_fifo_t *pkt_fifo =
            chFactoryFindObjectsFIFO(myHandler->pbuff_name);
        chDbgAssert(pkt_fifo != NULL, "unable to find packet fifo");

        /* The factory reference count is increased. */
        objects_fifo_t *pkt_buffer_pool = chFactoryGetObjectsFIFO(pkt_fifo);
        chDbgAssert(pkt_buffer_pool != NULL, "no packet fifo list");

        /* Get a buffer and have it initialized ready for use. */
        pkt_data_object_t *myPktBuffer = pktTakeDataBuffer(myHandler,
                                                            pkt_buffer_pool,
                                                            TIME_MS2I(100));

        /* If no buffer is available the handler pointer is also set to NULL. */
        if(myPktBuffer == NULL) {
          /* Decrease ref count on AX25 FIFO. */
          chFactoryReleaseObjectsFIFO(pkt_fifo);
          pktAddEventFlags(myHandler, EVT_AX25_NO_BUFFER);
          myDriver->active_demod_object->status |=
              EVT_AX25_NO_BUFFER;
          myDriver->decoder_state = DECODER_RESET;
          break;
        }

#if AFSK_DEBUG_TYPE == AFSK_PWM_DATA_CAPTURE_DEBUG
          char buf[80];
          int out = chsnprintf(buf, sizeof(buf),
                               "\r\n======= START ===========\r\n");
          chnWrite(pkt_out, (uint8_t *)buf, out);
#endif
        /* Increase thread priority. */
        (void)chThdSetPriority(DECODER_RUN_PRIORITY);
        /* Turn on the decoder LED. */
        pktWriteGPIOline(LINE_DECODER_LED, PAL_HIGH);
        /* Enable processing of incoming PWM stream. */
        myDriver->decoder_state = DECODER_ACTIVE;
        break;
      } /* End case DECODER_SESSION_POLL. */

      case DECODER_ACTIVE: {
        /*
         * We have a packet being processed.
         * Get PWM data from PWM queue.
         */
        radio_pwm_fifo_t *myFIFO = myDriver->active_demod_object;

#if USE_HEAP_PWM_BUFFER == TRUE
        /* Get current PWM queue object address. */
        input_queue_t *myQueue = &myFIFO->decode_pwm_queue->queue;
#else
        /* Use the common PWM -> decoder queue. */
        input_queue_t *myQueue = &myFIFO->radio_pwm_queue;
#endif
        chDbgAssert(myQueue != NULL, "no queue assigned");

        byte_packed_pwm_t data;
        size_t n = iqReadTimeout(myQueue, data.bytes,
                                 sizeof(packed_pwm_counts_t),
                                 chTimeUS2I(833 * 8 * 20)
                                 /*TIME_MS2I(DECODER_ACTIVE_TIMEOUT)*/);
        /* Timeout calculated as SYMBOL time x 8 x 20. */

        if(n != sizeof(packed_pwm_counts_t)) {
          /* PWM stream wait timeout. */
          pktAddEventFlags(myHandler, EVT_PWM_STREAM_TIMEOUT);
          myDriver->active_demod_object->status |= EVT_PWM_STREAM_TIMEOUT;
          myDriver->decoder_state = DECODER_RESET;
          break;
        }
        array_min_pwm_counts_t radio;
        pktUnpackPWMData(data, &radio);

#if AFSK_DEBUG_TYPE == AFSK_PWM_DATA_CAPTURE_DEBUG
        char buf[80];
        int out = chsnprintf(buf, sizeof(buf), "%i, %i\r\n",
                  radio.pwm.impulse, radio.pwm.valley);
        chnWrite(pkt_out, (uint8_t *)buf, out);
#endif

        /* Look for "in band" message in radio data. */
        if(radio.pwm.impulse == PWM_IN_BAND_PREFIX) {
          switch(radio.pwm.valley) {
          case PWM_TERM_DECODE_STOP: {
            /*
             *  The receive controller has issued a decoder stop.
             *  This happens when a TX request is submitted or an service stop is requested.
             *  The PWM stop places an in-band message in the PWM stream if open.
             *  The decoder will then (eventually) process the in-band message.
             *  The decode result will be invalid due to being stopped.
             *  The current AX25 buffer will not be dispatched.
             *  PWM and AX25 buffers and the stream manager object will be released in RESET.
             */
            myDriver->decoder_state = DECODER_RESET;
            continue; /* Continue in main loop. */
          } /* End case. */

          case PWM_ACK_DECODE_ERROR:
          case PWM_ACK_DECODE_END: {
            /*
             * Decoder has set an end or error flag while PWM is still running.
             * The radio side closes the PWM stream.
             * It writes an in-band message in the open stream.
             * Although we should never see that in-band.
             * The decoder has already moved out of ACTIVE state and is no longer processing PWM.
             * TODO: Deprecate these cases after checking they never happen.
             */
            pktAddEventFlags(myHandler, EVT_PWM_INVALID_INBAND);
            myDriver->decoder_state = DECODER_RESET;

            continue; /* Decoder state switch. */
          } /* End case. */

            /* If CCA ends and the decoder has not validated any frame.
             * The PWM side has already posted a PWM_STREAM_CLOSE event.
             */
          case PWM_TERM_CCA_CLOSE:

            /* If no PWM data is captured within a timeout.
             * The PWM side has already posted a PWM_NO_DATA event.
             */
          case PWM_TERM_NO_DATA:

            /* If the ICU timer overflows during PWM capture.
             * The PWM side has already posted a ICU_OVERFLOW event.
             */
          case PWM_TERM_ICU_OVERFLOW:

            /* This is a debug error.
             * CCA is validated but a PWM is still active.
             * The PWM side has already posted a PWM_FIFO_REMNANT event.
             */
          case PWM_TERM_QUEUE_ERR:

            /* If there is no more PWM buffer space available.
             * The PWM side has already posted a PWM_QUEUE_FULL event.
             */
          case PWM_TERM_QUEUE_FULL: {
            /* Transit to RESET state where all buffers/objects are released. */
            myDriver->decoder_state = DECODER_RESET;
            continue; /* Decoder state switch. */
          } /* End case. */

#if USE_HEAP_PWM_BUFFER == TRUE
          case PWM_INFO_QUEUE_SWAP: {
            /* Radio made a queue swap (filled the buffer). */
            /* Get reference to next queue/buffer object. */
            radio_pwm_object_t *nextObject = qGetLink(&myFIFO->decode_pwm_queue->queue);
            if(nextObject != NULL) {
              /*
               *  Release the now empty prior buffer object back to the pool.
               *  Switch to the next queue/buffer object for decoding.
               */
              chPoolFree(&myDriver->pwm_buffer_pool, myFIFO->decode_pwm_queue);
              myFIFO->rlsd++;
              myFIFO->decode_pwm_queue = nextObject;
              //myQueue = &nextObject->queue;
            } else {
              /*
               *  This is an error condition.
               *  An in-band swap message should always have a linked object.
               *  Error state will release the AX25 and PWM objects.
               *  Then reset state will release the FIFO object.
               */

              /* TODO: Need an EVT code freed up to add INVALID_SWAP. */
              myDriver->decoder_state = DECODER_RESET;
            }
            continue; /* Decoder state switch. */
          } /* End case. */
#endif

          default: {
            /* Unknown in-band message from PWM. */
            pktAddEventFlags(myHandler, EVT_PWM_INVALID_INBAND);
            myDriver->decoder_state = DECODER_RESET;
            continue;  /* Enclosing state switch. */
            } /* End case default. */
          } /* End switch on in-band. */
          continue; /* Decoder state switch. */
        } /* End if in-band. */

        /*
         * If not in-band the process the AFSK into an HDLC bit and AX25 data.
         */
        if(!pktProcessAFSK(myDriver, radio.array)) {
          /* AX25 character decoded but buffer is full.
           * Status set and event sent by HDLC processor.
           * Set error state and don't dispatch the AX25 buffer.
           * TODO: Move status and event broadcast to here?
           */
          myDriver->decoder_state = DECODER_RESET;
          break; /* From this case. */
        }

        /* Check for change of frame state. */
        switch(myDriver->frame_state) {
        case FRAME_SEARCH:
          pktWriteGPIOline(LINE_DECODER_LED, PAL_TOGGLE);
          continue;

        case FRAME_OPEN:
        case FRAME_DATA:
          pktWriteGPIOline(LINE_DECODER_LED, PAL_HIGH);
          continue;

        /* HDLC reset after frame open and minimum valid data received. */
        case FRAME_RESET:
          myDriver->decoder_state = DECODER_RESET;
          continue;

        case FRAME_CLOSE: {
          myDriver->decoder_state = DECODER_DISPATCH;
          continue; /* From this case. */
          }
        } /* End switch. */
        break; /* Keep GCC 7 happy. */
      } /* End case DECODER_ACTIVE. */

      /*
       * RESET readies the decoder for the next session.
       * It frees any held buffers/objects.
       * The DSP system is reset and then transition to IDLE.
       */
      case DECODER_RESET: {
        if(myHandler->active_packet_object != NULL) {
#if USE_CCM_HEAP_RX_BUFFERS == TRUE
          /* Free the packet buffer in the heap now. */
          chHeapFree(myHandler->active_packet_object->buffer);
#endif
          /* Release the AX25 receive packet buffer management object. */
          objects_fifo_t *pkt_fifo =
              chFactoryGetObjectsFIFO(myHandler->active_packet_object->pkt_factory);

          chDbgAssert(pkt_fifo != NULL, "no packet FIFO");

          chFifoReturnObject(pkt_fifo, myHandler->active_packet_object);

          /* Forget the AX25 buffer management object. */
          myHandler->active_packet_object = NULL;
        }

        /*
         * Return stream FIFO to pool if there is one active.
         */
        radio_pwm_fifo_t *myFIFO = myDriver->active_demod_object;
        if(myFIFO != NULL) {
          /*
           * Lock the PWM queue to stop any further radio data being written.
           */
          myDriver->active_demod_object->status |= EVT_AFSK_DECODE_RESET;
          /*
           * Wait for FIFO stream control object to be free from the radio.
           * Normally this semaphore will not suspend as decoding is slow.
           * If can be a forced release by semaphore reset.
           * TODO: This may happen if the watchdog system forces reset.
           * TBD.
           */
          (void)chBSemWait(&myFIFO->sem);

#if USE_HEAP_PWM_BUFFER == TRUE
          /* Release PWM queue/buffer objects back to the pool. */
          radio_pwm_fifo_t *myFIFO = myDriver->active_demod_object;
          uint8_t u = myFIFO->in_use;
          uint8_t n = pktReleasePWMbuffers(myDriver);
#if TRACE_PWM_BUFFER_STATS == TRUE
          TRACE_DEBUG("AFSK > PWM buffer use:"
              " allocated %d, consumed %d, released %d, peak lag %d",
              PWM_DATA_BUFFERS, u, n, myFIFO->peak);
#else
          (void)u;
          (void)n;
#endif
#endif
          /* Decode session is now closed. Release the FIFO stream object. */
          chFifoReturnObject(myDriver->pwm_fifo_pool, myFIFO);

          /* Forget the demod side reference to the object. */
          myDriver->active_demod_object = NULL;
        }

        /* Reset the correlation decoder and its filters. */
        pktResetAFSKDecoder(myDriver);

        /* Turn off decoder and reset time interval. */
        pktWriteGPIOline(LINE_DECODER_LED, PAL_LOW);
        led_count = 0;

        (void)chThdSetPriority(decoder_idle_priority);

        /* Set decoder back to idle. */
        myDriver->decoder_state = DECODER_IDLE;
        break;
      } /* End case DECODER_RESET. */

      case DECODER_DISPATCH: {
        if(myHandler->active_packet_object != NULL) {

          /* TODO: PWM chain is also released in RESET so this can be removed. */
#if USE_HEAP_PWM_BUFFER == TRUE
          /* Release PWM queue/buffer objects back to the pool. */
          radio_pwm_fifo_t *myFIFO = myDriver->active_demod_object;
          uint8_t u = myFIFO->in_use;
          uint8_t n = pktReleasePWMbuffers(myDriver);
#if TRACE_PWM_BUFFER_STATS == TRUE
          TRACE_DEBUG("AFSK > PWM buffer use:"
            " allocated %d, consumed %d, released %d, peak lag %d",
            PWM_DATA_BUFFERS, u, n, myFIFO->peak);
#else
          (void)u;
          (void)n;
#endif
#endif
          /*
           * Indicate AFSK decode done.
           * If PWM is still being captured for this stream capture will cease.
           */
          eventflags_t evtf = EVT_AFSK_DECODE_DONE;
          myDriver->active_demod_object->status |= evtf;

          /* Copy latest status into packet buffer object. */
          myHandler->active_packet_object->status =
              myDriver->active_demod_object->status;

          /* Dispatch the packet buffer object and get AX25 events. */
          evtf |= pktDispatchReceivedBuffer(myHandler->active_packet_object);

          /* Forget the packet object. */
          myHandler->active_packet_object = NULL;

          /* Send events then update demod object status. */
          pktAddEventFlags(myHandler, evtf);
          myDriver->active_demod_object->status |= evtf;
        } /* Active packet object != NULL. */
        myDriver->decoder_state = DECODER_RESET;
        break;
      } /* End case DECODER_DISPATCH. */
    } /* End switch on decoder state. */
  } /* End thread while(true). */
}

/** @} */
