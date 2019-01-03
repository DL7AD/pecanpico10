/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    corr_q31.c
 * @brief   CORR_Q31 decoder implementation.
 *
 * @addtogroup DSP
 * @{
 */


#include "pktconf.h"


#if AFSK_DECODE_TYPE == AFSK_DSP_QCORR_DECODE

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Resets the AFSK filter states.
 * @post    Filter state is reset.
 * @post    Filter variables are reset.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure.
 *
 * @api
 */
void reset_qcorr_all(AFSKDemodDriver *driver) {
  qcorr_decoder_t *decoder = driver->tone_decoder;

  chDbgAssert(decoder != NULL, "no tone decoder");

  /* Reset the pre-filter state data. */
  reset_qfir_embedded_filter(&decoder->pre_filter.core.instance);

  decoder->prefilter_out = 0;

  uint8_t i;
  for(i = 0; i < decoder->number_bins; i++) {
#if USE_QCORR_MAG_LPF == TRUE
    /* Reset the magnitude filter. */
    reset_qfir_embedded_filter(&decoder->filter_bins[i].mag_filter.core.instance);

#endif
    /* Reset the correlation filters. */
    qcorr_tone_t *myBin = &decoder->filter_bins[i];
    reset_qfir_embedded_filter(&myBin->cos_filter.core.instance);
    reset_qfir_embedded_filter(&myBin->sin_filter.core.instance);
    decoder->filter_bins[i].raw_mag = 0;
    decoder->filter_bins[i].mag = 0;
  } /* End for (number_bins). */
  decoder->current_n = 0;
  decoder->filter_valid = 0;

  decoder->prior_demod = TONE_NONE;
  decoder->current_demod = TONE_NONE;

  decoder->symbol_pll = 0;

}

/**
 * @brief   Called at each new sample to pre-process sample.
 * @post    New sample applied to pre-filter input.
 * @post    Latest output from pre-filter computed.
 * @note    Latest output value is saved in filter object.
 *
 * @param[in] myDriver  pointer to driver structure.
 * @param[in] sample    input binary value.
 *
 * @retrun  Latest sample output from filter.
 * @retval  Re-scaled q31 value from filter.
 *
 * @api
 */
q31_t push_qcorr_sample(AFSKDemodDriver *myDriver, bit_t sample) {
  qcorr_decoder_t *decoder = myDriver->tone_decoder;
  qfir_emb_filter_t *myFilter = &decoder->pre_filter.core;

  apply_qfir_embedded_filter(myFilter, &decoder->sample_level[sample],
                    &decoder->prefilter_out);
#if AFSK_DEBUG_TYPE == AFSK_QCORR_FIR_DEBUG
    char buf[80];
    int out = chsnprintf(buf, sizeof(buf), "%X\r\n", scaledOut);
    pktWrite( (uint8_t *)buf, out);
#endif

  /*
   * Return most recent filter output.
   */
  return decoder->prefilter_out;
}

/**
 * @brief   Called at each new sample to process correlation.
 * @notes   The correlation filters are run for each tone and IQ phase.
 * @notes   The magnitude of each tone is calculated.
 * @notes   The comparative strength of symbol tones is evaluated and updated.
 * @notes   If the symbol is complete then HDLC decoding is enabled.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure.
 *
 * @return      Status for symbol
 * @retval      false if the decoder output is not valid.
 * @retval      true if the decoder output is valid.
 *
 */
bool process_qcorr_output(AFSKDemodDriver *myDriver) {
  qcorr_decoder_t *decoder = myDriver->tone_decoder;

  /*
   * The decoder structure contains the filtered and scaled sample.
  */

  uint8_t i;

  for(i = 0; i < decoder->number_bins; i++) {
    qcorr_tone_t *myBin = &decoder->filter_bins[i];

    /*
     * Run correlation for bin.
     */
    apply_qfir_embedded_filter(&myBin->cos_filter.core,
                               &decoder->prefilter_out, &myBin->cos_out);
    apply_qfir_embedded_filter(&myBin->sin_filter.core,
                               &decoder->prefilter_out, &myBin->sin_out);

#if AFSK_DEBUG_TYPE == AFSK_QCORR_DEC_CS_DEBUG
    char buf[200];
    if(i == 0) {
    int out =chsnprintf(buf, sizeof(buf), "%i, %i, ",
      myBin->cos_out, myBin->sin_out);
    } else {
      int out = chsnprintf(buf, sizeof(buf), "%i, %i\r\n",
        myBin->cos_out, myBin->sin_out);
    }
    pktWrite( (uint8_t *)buf, out);
#endif
  }

  /*
   * Wait for initial data to be valid from pre-filter + correlators.
   */
  if(++decoder->filter_valid <
      PRE_FILTER_NUM_TAPS + DECODE_FILTER_LENGTH)
    return false;

  /*
   *  Samples have propagated through pre-filter and correlators.
   *  Compute magnitude of bins from now on.
   */
  calc_qcorr_magnitude(myDriver);

#if USE_QCORR_MAG_LPF == TRUE
  /* Filter magnitude of correlator outputs. */
  filter_qcorr_magnitude(myDriver);
  /* Further delay result by mag filter size. */
  if(decoder->filter_valid < (PRE_FILTER_NUM_TAPS + DECODE_FILTER_LENGTH
                                    + MAG_FILTER_NUM_TAPS))
    return false;
#endif

#if AFSK_DEBUG_TYPE == AFSK_QCORR_DATA_DEBUG
  char buf[200];
  for(i = 0; i < decoder->number_bins; i++) {
    int out = chsnprintf(buf, sizeof(buf),
      "BIN %i mag %i N %i N%% %i\r\n",
      i, decoder->filter_bins[i].mag,
      decoder->current_n,
      decoder->current_n % decoder->decode_length);
    pktWrite( (uint8_t *)buf, out);
  }
#endif

  /* Do magnitude comparison on tone bins and save results. */
  evaluate_qcorr_tone(myDriver);

  return true;
}

/**
 * @brief       Checks the symbol timing.
 *
 * @param[in]   myDriver    pointer to AFSKDemodDriver structure.
 *
 * @return      Status for symbol timing.
 * @retval      false if the symbol is not complete.
 * @retval      true if the symbol is ready for HDLC detection.
 *
 * @api
 */
bool get_qcorr_symbol_timing(AFSKDemodDriver *myDriver) {
  qcorr_decoder_t *decoder = myDriver->tone_decoder;

  /* Update symbol PLL. */
  decoder->prior_pll = decoder->symbol_pll;

  /* PLL increment is max of uint32_t / decimation rate. */
#define PLL_INCREMENT (UINT32_MAX / SYMBOL_DECIMATION)
  decoder->symbol_pll = (int32_t)((uint32_t)(decoder->symbol_pll) + PLL_INCREMENT);
  /*
   * Check if the symbol period was reached and return status.
   * The symbol period is reached when the PLL counter wraps around.
   */
  return ((decoder->symbol_pll < 0) && (decoder->prior_pll > 0));
}

/**
 * @brief Advances the symbol PLL timing.
 * @notes The rate of advance is determined by the HDLC frame state.
 * @notes If a frame start has not been detected a faster search rate is used.
 * @notes This aids in finding the HDLC sync point as soon as possible.
 * @notes After HDLC frame start has been found the PLL search rate is reduced.
 *
 * @param[in] myDriver    pointer to AFSKDemodDriver structure.
 *
 * @api
 */
void update_qcorr_pll(AFSKDemodDriver *myDriver) {
  qcorr_decoder_t *decoder = myDriver->tone_decoder;
  /*
   * Now test if a tone transition has taken place.
   */
  if(decoder->current_demod != decoder->prior_demod) {

    /* Update tone state. */
    decoder->prior_demod = decoder->current_demod;

    /* Adjust symbol PLL. */
    if (isHDLCSynchronising(myDriver)) {
      decoder->symbol_pll = (int32_t)((float32_t)decoder->symbol_pll
          * QCORR_PLL_SEARCH_RATE);
    } else {
      decoder->symbol_pll = (int32_t)((float32_t)decoder->symbol_pll
          * QCORR_PLL_LOCKED_RATE);
    }
  }
}

/**
 * @brief Calculate magnitudes.
 *
 * @param[in] myDriver    pointer to AFSKDemodDriver structure.
 *
 * @api
 */
void calc_qcorr_magnitude(AFSKDemodDriver *myDriver) {
  qcorr_decoder_t *decoder = myDriver->tone_decoder;

  uint8_t i;

  /* Compute magnitude of each bin. */
  for(i = 0; i < decoder->number_bins; i++) {
    qcorr_tone_t *myBin = &decoder->filter_bins[i];
#ifdef QCORR_MAG_USE_FLOAT
    float32_t cos, sin, mag2;
    q31_t mag;
    (void)arm_q31_to_float(&myBin->cos_out, &cos, 1);
    (void)arm_q31_to_float(&myBin->sin_out, &sin, 1);
    mag2 = (cos * cos + sin * sin);
    //mag2 = cos2 + sin2;
    (void)arm_float_to_q31(&mag2, &mag, 1);
    arm_status status = arm_sqrt_q31(mag, &mag);
    if(status == ARM_MATH_SUCCESS) {
      /* Update raw bin magnitude. */
      decoder->filter_bins[i].raw_mag = mag;
    } else { /* arm_sqrt_q31 failed. */
#if AFSK_ERROR_TYPE == AFSK_SQRT_ERROR
      char buf[200];
      int out = chsnprintf(buf, sizeof(buf),                                 \
        "MAG SQRT failed bin %i, cosQ %X, sinQ %X, cos %f, sin %f, mag2 %f, mag %X, index %i\r\n",                                              \
        i, myBin->cos_out, myBin->sin_out, cos, sin, mag2, raw_mag, decoder->current_n);
      pktWrite( (uint8_t *)buf, out);
#endif /* AFSK_ERROR_TYPE == AFSK_SQRT_ERROR */

#else
      q31_t mag2, mag, cos, sin;
      (void)arm_mult_q31(&myBin->cos_out, &myBin->cos_out, &cos, 1);
      (void)arm_mult_q31(&myBin->sin_out, &myBin->sin_out, &sin, 1);
      (void)arm_add_q31(&cos, &sin, &mag2, 1);
      arm_status status = arm_sqrt_q31(mag2, &mag);
      if(status == ARM_MATH_SUCCESS) {
        /* Update raw bin magnitude. */
        decoder->filter_bins[i].raw_mag = mag;
      } else { /* arm_sqrt_q31 failed. */
  #if AFSK_ERROR_TYPE == AFSK_SQRT_ERROR
        char buf[200];
        int out = chsnprintf(buf, sizeof(buf),
          "MAG SQRT failed bin %i, cosQ %X, sinQ %X, cos %X, sin %X,"
          "mag2 %X, mag %X, index %i\r\n",
          i, myBin->cos_out, myBin->sin_out, cos, sin, mag2,
          decoder->filter_bins[i].raw_mag, decoder->current_n);
        pktWrite( (uint8_t *)buf, out);
  #endif /* AFSK_ERROR_TYPE == AFSK_SQRT_ERROR */
#endif /* QCORR_MAG_USE_FLOAT */
    }
#if AFSK_DEBUG_TYPE == AFSK_QCORR_DEC_MAG_DEBUG
    char buf[200];
    int out;
    if(i == 0) {
      out = chsnprintf(buf, sizeof(buf), "%i, ", raw_mag);
    } else {
      out = chsnprintf(buf, sizeof(buf), "%i\r\n", raw_mag);
    }
    pktWrite( (uint8_t *)buf, out);
#endif
  }
}

/**
 * @brief Apply LPF to magnitude of each filter bin.
 *
 * @param[in] myDriver    pointer to AFSKDemodDriver structure.
 *
 * @api
 */
void filter_qcorr_magnitude(AFSKDemodDriver *myDriver) {
  qcorr_decoder_t *decoder = myDriver->tone_decoder;
  uint8_t i;
  for(i = 0; i < decoder->number_bins; i++) {

    /*
     * Filter the magnitude and compute next output sample.
     */
    qfir_emb_filter_t *myFilter = &decoder->filter_bins[i].mag_filter.core;

    apply_qfir_embedded_filter(myFilter, &decoder->filter_bins[i].raw_mag,
                      &decoder->filter_bins[i].filtered_mag);
#if AFSK_DEBUG_TYPE == AFSK_QCORR_DEC_MFIL_DEBUG
    char buf[200];
    int out;
    if(i == 0) {
      out = chsnprintf(buf, sizeof(buf), "%i, %i, ",
                       decoder->filter_bins[i].raw_mag,
                       decoder->filter_bins[i].filtered_mag);
    } else {
      out = chsnprintf(buf, sizeof(buf), "%i, %i\r\n",
                       decoder->filter_bins[i].raw_mag,
                       decoder->filter_bins[i].filtered_mag);
    }
    pktWrite( (uint8_t *)buf, out);
#endif
  }
}

/**
 * @brief Called to evaluate the tone strengths in the filters.
 * @notes Hysteresis is applied such that an unclear result is no change.
 * @notes This can/will happen as the tone transitions from one to the other.
 * @post  The tone memory will be set to the current strongest at this sample.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure.
 *
 */
void evaluate_qcorr_tone(AFSKDemodDriver *myDriver) {
  qcorr_decoder_t *myDecoder = (qcorr_decoder_t *)myDriver->tone_decoder;
  q31_t mark, space;
  q31_t delta;

  /*
   * Check if the prior tone detection differs to the current tone.
   */

#if USE_QCORR_MAG_LPF == TRUE
  mark = myDecoder->filter_bins[AFSK_MARK_INDEX].filtered_mag;
  space = myDecoder->filter_bins[AFSK_SPACE_INDEX].filtered_mag;
#else
  mark = myDecoder->filter_bins[AFSK_MARK_INDEX].raw_mag;
  space = myDecoder->filter_bins[AFSK_SPACE_INDEX].raw_mag;
#endif
  delta = mark - space;
  if(delta > myDecoder->hysteresis) {
    /* Mark symbol dominant. */
    myDecoder->current_demod = TONE_MARK;
  } else if (delta < -myDecoder->hysteresis) {
    /* Space symbol dominant. */
    myDecoder->current_demod = TONE_SPACE;
  }
  /* Else don't change current_demod so it remains as prior. */
#if AFSK_DEBUG_TYPE == AFSK_QCORR_DEC_MS_DEBUG
    char buf[200];
    int out = chsnprintf(buf, sizeof(buf), "%i, %i\r\n",
      mark, space);
    pktWrite( (uint8_t *)buf, out);
#endif
}

/**
 * @brief Setup the correlation pre-filter.
 *
 * @param[in]   decoder   pointer to a @p qcorr_decoder_t structure.
 *
 *@api
 */
static void setup_qcorr_prefilter(qcorr_decoder_t *decoder) {

  /* Generate the pre-filter coordinates. */

  float32_t pre_filter_coeff_f32[PRE_FILTER_NUM_TAPS];

  gen_fir_bpf((float32_t)PRE_FILTER_LOW / (float32_t)FILTER_SAMPLE_RATE,
              (float32_t)PRE_FILTER_HIGH / (float32_t)FILTER_SAMPLE_RATE,
              pre_filter_coeff_f32,
              PRE_FILTER_NUM_TAPS,
              TD_WINDOW_NONE);

  create_qfir_embedded_filter((qfir_emb_filter_t *)&decoder->pre_filter,
                              &decoder->pre_filter.core.instance,
                              PRE_FILTER_NUM_TAPS,
                              decoder->pre_filter.coeffs,
                              decoder->pre_filter.state,
                              pre_filter_coeff_f32);

#if REPORT_QCORR_COEFFS == TRUE
  /*
   * Report the coefficient totals in f32 and q31
   */
  float32_t coeff_total_f32 = 0;
  q31_t coeff_total_q31 = 0;

  uint16_t i;

  for(i = 0; i < PRE_FILTER_NUM_TAPS; i++) {
    coeff_total_f32 += pre_filter_coeff_f32[i];
    coeff_total_q31 += pre_filter_coeff_q31[i];
  }
  char buf[80];
  int out = chsnprintf(buf, sizeof(buf),
    "PRE FILTER COEFF %f %x\r\n", coeff_total_f32, coeff_total_q31);
  pktWrite( (uint8_t *)buf, out);
#endif
}

/**
 * @brief Setup the correlation IQ filters.
 *
 * @param[in]   decoder   pointer to a @p qcorr_decoder_t structure.
 *
 *@api
 */
void setup_qcorr_IQfilters(qcorr_decoder_t *decoder) {

  /* Set COS and SIN filters for Mark and Space. */

  /* Temporary float coeff arrays. */
  float32_t cos_table[decoder->decode_length];
  float32_t sin_table[decoder->decode_length];

  /* Calculate the IQ filter coefficients for Mark. */
  float32_t norm_freq = (float32_t)decoder->filter_bins[AFSK_MARK_INDEX].freq
      / (float32_t)decoder->sample_rate;

  gen_fir_iqf(cos_table, sin_table, decoder->decode_length,
              norm_freq, QCORR_IQ_WINDOW);

  /*
   * Create the Mark correlation filters.
   */
  qfir_emb_filter_t *filter;

  filter = &decoder->qcorr_bins[AFSK_MARK_INDEX].cos_filter.core;
  create_qfir_embedded_filter(filter,
                              &filter->instance,
                              decoder->decode_length,
                              decoder->qcorr_bins[AFSK_MARK_INDEX].cos_filter.coeffs,
                              decoder->qcorr_bins[AFSK_MARK_INDEX].cos_filter.state,
                              cos_table
                              );

  filter = &decoder->qcorr_bins[AFSK_MARK_INDEX].sin_filter.core;
  create_qfir_embedded_filter(filter,
                              &filter->instance,
                              decoder->decode_length,
                              decoder->qcorr_bins[AFSK_MARK_INDEX].sin_filter.coeffs,
                              decoder->qcorr_bins[AFSK_MARK_INDEX].sin_filter.state,
                              sin_table
                              );

  /* Calculate the IQ filter coefficients for Space. */
  norm_freq = (float32_t)decoder->filter_bins[AFSK_SPACE_INDEX].freq
      / (float32_t)decoder->sample_rate;

  gen_fir_iqf(cos_table, sin_table, decoder->decode_length,
              norm_freq, QCORR_IQ_WINDOW);

  /*
   * Create the Space correlation filters.
   */
  filter = &decoder->qcorr_bins[AFSK_SPACE_INDEX].cos_filter.core;
  create_qfir_embedded_filter(filter,
                              &filter->instance,
                              decoder->decode_length,
                              decoder->qcorr_bins[AFSK_SPACE_INDEX].cos_filter.coeffs,
                              decoder->qcorr_bins[AFSK_SPACE_INDEX].cos_filter.state,
                              cos_table
                              );

  filter = &decoder->qcorr_bins[AFSK_SPACE_INDEX].sin_filter.core;
  create_qfir_embedded_filter(filter,
                              &filter->instance,
                              decoder->decode_length,
                              decoder->qcorr_bins[AFSK_SPACE_INDEX].sin_filter.coeffs,
                              decoder->qcorr_bins[AFSK_SPACE_INDEX].sin_filter.state,
                              sin_table
                              );
}

#if USE_QCORR_MAG_LPF == TRUE
/**
 * @brief Setup the magnitude filter.
 *
 * @param[in]   decoder   pointer to a @p qcorr_decoder_t structure.
 *
 *@api
 */
static void setup_qcorr_magfilter(qcorr_decoder_t *decoder) {

  float32_t mag_filter_coeff_f32[MAG_FILTER_NUM_TAPS];

  gen_fir_lpf((float32_t)MAG_FILTER_HIGH / (float32_t)FILTER_SAMPLE_RATE,
              mag_filter_coeff_f32,
              MAG_FILTER_NUM_TAPS,
              TD_WINDOW_NONE);

   /*
    * Initialise the magnitude filters.
    */

  qfir_emb_filter_t *filter;

  filter = &decoder->qcorr_bins[AFSK_MARK_INDEX].mag_filter.core;
  create_qfir_embedded_filter(filter,
                              &filter->instance,
                              MAG_FILTER_NUM_TAPS,
                              decoder->qcorr_bins[AFSK_MARK_INDEX].mag_filter.coeffs,
                              decoder->qcorr_bins[AFSK_MARK_INDEX].mag_filter.state,
                              mag_filter_coeff_f32);

  filter = &decoder->qcorr_bins[AFSK_SPACE_INDEX].mag_filter.core;
  create_qfir_embedded_filter(filter,
                              &filter->instance,
                              MAG_FILTER_NUM_TAPS,
                              decoder->qcorr_bins[AFSK_SPACE_INDEX].mag_filter.coeffs,
                              decoder->qcorr_bins[AFSK_SPACE_INDEX].mag_filter.state,
                              mag_filter_coeff_f32);

#if REPORT_QCORR_COEFFS == TRUE
  /*
  * Report the coefficient totals in f32 and q31
  */
  float32_t bin_coeff_total_f32 = 0;
  q31_t bin_coeff_total_q31 = 0;

  uint16_t i;
  for(i = 0; i < MAG_FILTER_NUM_TAPS; i++) {
    bin_coeff_total_f32 += mag_filter_coeff_f32[i];
    bin_coeff_total_q31 += mag_filter_coeff_q31[i];
  }

  char buf[80];
  int out = chsnprintf(buf, sizeof(buf),
  "MAG FILTER COEFF %f %x\r\n", bin_coeff_total_f32, bin_coeff_total_q31);
  pktWrite( (uint8_t *)buf, out);
#endif
}
#endif

/**
 * @brief   Called once to initialise the QCORR parameters.
 *
 * @param[in] myDriver  pointer to AFSKDemodDriver data structure.
 *
 *@api
 */
void init_qcorr_decoder(AFSKDemodDriver *myDriver) {
  /* Assign the correlator control record. */

  extern memory_heap_t *ccm_heap;
  qcorr_decoder_t *decoder = chHeapAlloc(ccm_heap, sizeof(qcorr_decoder_t));

  chDbgAssert(decoder != NULL, "failed to create QCORR record in CCM heap");

  /* Calculate the sample rate for the system.
   * TODO: Centralize sample rate definition/calculation. */
  decoder->sample_rate = FILTER_SAMPLE_RATE;

  /* low level initialization. */
  decoder->decode_length = DECODE_FILTER_LENGTH;
  decoder->number_bins = QCORR_FILTER_BINS;
  decoder->filter_bins = decoder->qcorr_bins;
  myDriver->tone_decoder = decoder;

  /* Calculate magnitude comparison hysteresis value. */
  float32_t hysteresis = QCORR_HYSTERESIS;
  arm_float_to_q31(&hysteresis, &decoder->hysteresis, 1);

  /* Create and attach the fixed point pre-filter. */
  setup_qcorr_prefilter(decoder);

  /*
   * Set the conversion level from binary to -h to +h filter input value.
   * For convenience an array of 2 x q31 values hold + and - values.
   * The value is scaled as required in the Q31 FIR filter function.
   * Scaling avoids fixed point arithmetic wrap around in Q31 FIR.
   *
   */

  /* First set the value for a PWM 1. */
  float32_t input = QCORR_SAMPLE_LEVEL;

  /* Convert float sample value to q31. */
  arm_float_to_q31(&input, &decoder->sample_level[1], 1);

  /* Then set the value for a PWM 0. */
  decoder->sample_level[0] = -decoder->sample_level[1];

  /* Setup the decoder tone IQ filters. */
  /* Set tone frequencies. */
  decoder->filter_bins[AFSK_MARK_INDEX].freq = AFSK_MARK_FREQUENCY;
  decoder->filter_bins[AFSK_SPACE_INDEX].freq = AFSK_SPACE_FREQUENCY;
  setup_qcorr_IQfilters(decoder);

#if USE_QCORR_MAG_LPF == TRUE
  /* Setup the IQ magnitude LPFs. */
  setup_qcorr_magfilter(decoder);
#endif
}

/**
 * @brief   Called release the QCORR decoder resources.
 *
 * @param[in] myDriver  pointer to AFSKDemodDriver data structure.
 *
 *@api
 */
void release_qcorr_decoder(AFSKDemodDriver *myDriver) {
  /*
   * Release the QCORR object in heap.
   * All filters are contained in this object.
   */
  chHeapFree(myDriver->tone_decoder);
}

#endif /* AFSK_DSP_QCORR_DECODE */

/** @} */
