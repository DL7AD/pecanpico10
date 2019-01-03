/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    corr_f32.c
 * @brief   CORR_F32 decoder implementation.
 *
 * @addtogroup DSP
 * @{
 */


#include "pktconf.h"


#if AFSK_DECODE_TYPE == AFSK_DSP_FCORR_DECODE

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
void reset_fcorr_all(AFSKDemodDriver *driver) {
  fcorr_decoder_t *decoder = driver->tone_decoder;

  chDbgAssert(decoder != NULL, "no tone decoder");

  /* Reset the pre-filter state data. */
  reset_ffir_embedded_filter(&decoder->pre_filter.core.instance);

  decoder->prefilter_out = 0;

  uint8_t i;
  for(i = 0; i < decoder->number_bins; i++) {
#if USE_FCORR_MAG_LPF == TRUE
    /* Reset the magnitude filter. */
    reset_ffir_embedded_filter(&decoder->filter_bins[i].mag_filter.core.instance);

#endif
    /* Reset the correlation filters. */
    fcorr_tone_t *myBin = &decoder->filter_bins[i];
    reset_ffir_embedded_filter(&myBin->cos_filter.core.instance);
    reset_ffir_embedded_filter(&myBin->sin_filter.core.instance);
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
 * @retval  f32 value from filter.
 *
 * @api
 */
float32_t push_fcorr_sample(AFSKDemodDriver *myDriver, bit_t sample) {
  fcorr_decoder_t *decoder = myDriver->tone_decoder;
  ffir_emb_filter_t *myFilter = &decoder->pre_filter.core;

  apply_ffir_embedded_filter(myFilter, &decoder->sample_level[sample],
                    &decoder->prefilter_out);

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
bool process_fcorr_output(AFSKDemodDriver *myDriver) {
  fcorr_decoder_t *decoder = myDriver->tone_decoder;

  /*
   * The decoder structure contains the filtered and scaled sample.
  */

  uint8_t i;

  for(i = 0; i < decoder->number_bins; i++) {
    fcorr_tone_t *myBin = &decoder->filter_bins[i];

    /*
     * Run correlation for bin.
     */
    apply_ffir_embedded_filter(&myBin->cos_filter.core,
                               &decoder->prefilter_out, &myBin->cos_out);
    apply_ffir_embedded_filter(&myBin->sin_filter.core,
                               &decoder->prefilter_out, &myBin->sin_out);
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
  calc_fcorr_magnitude(myDriver);

#if USE_FCORR_MAG_LPF == TRUE
  /* Filter magnitude of correlator outputs. */
  filter_fcorr_magnitude(myDriver);
  /* Further delay result by mag filter size. */
  if(decoder->filter_valid < (PRE_FILTER_NUM_TAPS + DECODE_FILTER_LENGTH
                                    + MAG_FILTER_NUM_TAPS))
    return false;
#endif

  /* Do magnitude comparison on tone bins and save results. */
  evaluate_fcorr_tone(myDriver);

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
bool get_fcorr_symbol_timing(AFSKDemodDriver *myDriver) {
  fcorr_decoder_t *decoder = myDriver->tone_decoder;

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
 * @notea After HDLC frame start has been found the PLL search rate is reduced.
 *
 * @param[in] myDriver    pointer to AFSKDemodDriver structure.
 *
 * @api
 */
void update_fcorr_pll(AFSKDemodDriver *myDriver) {
  fcorr_decoder_t *decoder = myDriver->tone_decoder;
  /*
   * Now test if a tone transition has taken place.
   */
  if(decoder->current_demod != decoder->prior_demod) {

    /* Update tone state. */
    decoder->prior_demod = decoder->current_demod;

    /* Update PLL. */
    if (isHDLCSynchronising(myDriver)) {
      decoder->symbol_pll = (int32_t)((float32_t)decoder->symbol_pll
          * FCORR_PLL_SEARCH_RATE);
    } else {
      decoder->symbol_pll = (int32_t)((float32_t)decoder->symbol_pll
          * FCORR_PLL_LOCKED_RATE);
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
void calc_fcorr_magnitude(AFSKDemodDriver *myDriver) {
  fcorr_decoder_t *decoder = myDriver->tone_decoder;

  uint8_t i;

  /* Compute magnitude of each bin. */
  for(i = 0; i < decoder->number_bins; i++) {
    fcorr_tone_t *myBin = &decoder->filter_bins[i];
#ifdef FCORR_MAG_USE_FLOAT
    float32_t cos, sin, mag2;
    cos = *myBin->cos;
    sin = *myBin->sin;
    mag2 = (cos * cos + sin * sin);

    arm_status status = arm_sqrt_f32(mag, &mag);
    if(status == ARM_MATH_SUCCESS) {
      /* Update raw bin magnitude. */
      decoder->filter_bins[i].raw_mag = mag;
    } else { /* arm_sqrt_f32 failed. */
#if AFSK_ERROR_TYPE == AFSK_SQRT_ERROR
      char buf[200];
      int out = chsnprintf(buf, sizeof(buf),                                 \
        "MAG SQRT failed bin %i, cosF %X, sinF %X, cos %f, sin %f, mag2 %f, mag %X, index %i\r\n",                                              \
        i, myBin->cos_out, myBin->sin_out, cos, sin, mag2, raw_mag, decoder->current_n);
      pktWrite( (uint8_t *)buf, out);
#endif /* AFSK_ERROR_TYPE == AFSK_SQRT_ERROR */

#else
      float32_t mag2, mag, cos, sin;
      (void)arm_mult_f32(&myBin->cos_out, &myBin->cos_out, &cos, 1);
      (void)arm_mult_f32(&myBin->sin_out, &myBin->sin_out, &sin, 1);
      (void)arm_add_f32(&cos, &sin, &mag2, 1);
      arm_status status = arm_sqrt_f32(mag2, &mag);
      if(status == ARM_MATH_SUCCESS) {
        /* Update raw bin magnitude. */
        decoder->filter_bins[i].raw_mag = mag;
      } else { /* arm_sqrt_f32 failed. */
  #if AFSK_ERROR_TYPE == AFSK_SQRT_ERROR
        char buf[200];
        int out = chsnprintf(buf, sizeof(buf),
          "MAG SQRT failed bin %i, cosF %X, sinF %X, cos %X, sin %X,"
          "mag2 %X, mag %X, index %i\r\n",
          i, myBin->cos_out, myBin->sin_out, cos, sin, mag2,
          decoder->filter_bins[i].raw_mag, decoder->current_n);
        pktWrite( (uint8_t *)buf, out);
  #endif /* AFSK_ERROR_TYPE == AFSK_SQRT_ERROR */
#endif /* FCORR_MAG_USE_FLOAT */
    }
  }
}

/**
 * @brief Apply LPF to magnitude of each filter bin.
 *
 * @param[in] myDriver    pointer to AFSKDemodDriver structure.
 *
 * @api
 */
void filter_fcorr_magnitude(AFSKDemodDriver *myDriver) {
  fcorr_decoder_t *decoder = myDriver->tone_decoder;
  uint8_t i;
  for(i = 0; i < decoder->number_bins; i++) {

    /*
     * Filter the magnitude and compute next output sample.
     */
    ffir_emb_filter_t *myFilter = &decoder->filter_bins[i].mag_filter.core;

    apply_ffir_embedded_filter(myFilter, &decoder->filter_bins[i].raw_mag,
                      &decoder->filter_bins[i].filtered_mag);
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
void evaluate_fcorr_tone(AFSKDemodDriver *myDriver) {
  fcorr_decoder_t *myDecoder = (fcorr_decoder_t *)myDriver->tone_decoder;
  float32_t mark, space;
  float32_t delta;

  /*
   * Check if the prior tone detection differs to the current tone.
   */

#if USE_FCORR_MAG_LPF == TRUE
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
}

/**
 * @brief Setup the correlation pre-filter.
 *
 * @param[in]   decoder   pointer to a @p fcorr_decoder_t structure.
 *
 *@api
 */
static void setup_fcorr_prefilter(fcorr_decoder_t *decoder) {

  /* Generate the pre-filter coordinates. */

  float32_t pre_filter_coeff_f32[PRE_FILTER_NUM_TAPS];

  gen_fir_bpf((float32_t)PRE_FILTER_LOW / (float32_t)FILTER_SAMPLE_RATE,
              (float32_t)PRE_FILTER_HIGH / (float32_t)FILTER_SAMPLE_RATE,
              pre_filter_coeff_f32,
              PRE_FILTER_NUM_TAPS,
              TD_WINDOW_NONE);

  create_ffir_embedded_filter((ffir_emb_filter_t *)&decoder->pre_filter,
                              &decoder->pre_filter.core.instance,
                              PRE_FILTER_NUM_TAPS,
                              decoder->pre_filter.coeffs,
                              decoder->pre_filter.state,
                              pre_filter_coeff_f32);
}

/**
 * @brief Setup the correlation IQ filters.
 *
 * @param[in]   decoder   pointer to a @p fcorr_decoder_t structure.
 *
 *@api
 */
void setup_fcorr_IQfilters(fcorr_decoder_t *decoder) {

  /* Set COS and SIN filters for Mark and Space. */

  /* Temporary float coeff arrays. */
  float32_t cos_table[decoder->decode_length];
  float32_t sin_table[decoder->decode_length];

  /* Calculate the IQ filter coefficients for Mark. */
  float32_t norm_freq = (float32_t)decoder->filter_bins[AFSK_MARK_INDEX].freq
      / (float32_t)decoder->sample_rate;

  gen_fir_iqf(cos_table, sin_table, decoder->decode_length,
              norm_freq, FCORR_IQ_WINDOW);

  /*
   * Create the Mark correlation filters.
   */
  ffir_emb_filter_t *filter;

  filter = &decoder->fcorr_bins[AFSK_MARK_INDEX].cos_filter.core;
  create_ffir_embedded_filter(filter,
                              &filter->instance,
                              decoder->decode_length,
                              decoder->fcorr_bins[AFSK_MARK_INDEX].cos_filter.coeffs,
                              decoder->fcorr_bins[AFSK_MARK_INDEX].cos_filter.state,
                              cos_table
                              );

  filter = &decoder->fcorr_bins[AFSK_MARK_INDEX].sin_filter.core;
  create_ffir_embedded_filter(filter,
                              &filter->instance,
                              decoder->decode_length,
                              decoder->fcorr_bins[AFSK_MARK_INDEX].sin_filter.coeffs,
                              decoder->fcorr_bins[AFSK_MARK_INDEX].sin_filter.state,
                              sin_table
                              );

  /* Calculate the IQ filter coefficients for Space. */
  norm_freq = (float32_t)decoder->filter_bins[AFSK_SPACE_INDEX].freq
      / (float32_t)decoder->sample_rate;

  gen_fir_iqf(cos_table, sin_table, decoder->decode_length,
              norm_freq, FCORR_IQ_WINDOW);

  /*
   * Create the Space correlation filters.
   */
  filter = &decoder->fcorr_bins[AFSK_SPACE_INDEX].cos_filter.core;
  create_ffir_embedded_filter(filter,
                              &filter->instance,
                              decoder->decode_length,
                              decoder->fcorr_bins[AFSK_SPACE_INDEX].cos_filter.coeffs,
                              decoder->fcorr_bins[AFSK_SPACE_INDEX].cos_filter.state,
                              cos_table
                              );

  filter = &decoder->fcorr_bins[AFSK_SPACE_INDEX].sin_filter.core;
  create_ffir_embedded_filter(filter,
                              &filter->instance,
                              decoder->decode_length,
                              decoder->fcorr_bins[AFSK_SPACE_INDEX].sin_filter.coeffs,
                              decoder->fcorr_bins[AFSK_SPACE_INDEX].sin_filter.state,
                              sin_table
                              );
}

#if USE_FCORR_MAG_LPF == TRUE
/**
 * @brief Setup the magnitude filter.
 *
 * @param[in]   decoder   pointer to a @p fcorr_decoder_t structure.
 *
 *@api
 */
static void setup_fcorr_magfilter(fcorr_decoder_t *decoder) {

  float32_t mag_filter_coeff_f32[MAG_FILTER_NUM_TAPS];

  gen_fir_lpf((float32_t)MAG_FILTER_HIGH / (float32_t)FILTER_SAMPLE_RATE,
              mag_filter_coeff_f32,
              MAG_FILTER_NUM_TAPS,
              TD_WINDOW_NONE);

   /*
    * Initialise the magnitude filters.
    */

  ffir_emb_filter_t *filter;

  filter = &decoder->fcorr_bins[AFSK_MARK_INDEX].mag_filter.core;
  create_ffir_embedded_filter(filter,
                              &filter->instance,
                              MAG_FILTER_NUM_TAPS,
                              decoder->fcorr_bins[AFSK_MARK_INDEX].mag_filter.coeffs,
                              decoder->fcorr_bins[AFSK_MARK_INDEX].mag_filter.state,
                              mag_filter_coeff_f32);

  filter = &decoder->fcorr_bins[AFSK_SPACE_INDEX].mag_filter.core;
  create_ffir_embedded_filter(filter,
                              &filter->instance,
                              MAG_FILTER_NUM_TAPS,
                              decoder->fcorr_bins[AFSK_SPACE_INDEX].mag_filter.coeffs,
                              decoder->fcorr_bins[AFSK_SPACE_INDEX].mag_filter.state,
                              mag_filter_coeff_f32);
}
#endif

/**
 * @brief   Called once to initialise the FCORR parameters.
 *
 * @param[in] myDriver  pointer to AFSKDemodDriver data structure.
 *
 *@api
 */
void init_fcorr_decoder(AFSKDemodDriver *myDriver) {
  /* Assign the correlator control record. */

  extern memory_heap_t *ccm_heap;
  fcorr_decoder_t *decoder = chHeapAlloc(ccm_heap, sizeof(fcorr_decoder_t));

  chDbgAssert(decoder != NULL, "failed to create FCORR record in CCM heap");

  /* Calculate the sample rate for the system.
   * TODO: Centralize sample rate definition/calculation. */
  decoder->sample_rate = FILTER_SAMPLE_RATE;

  /* low level initialization. */
  decoder->decode_length = DECODE_FILTER_LENGTH;
  decoder->number_bins = FCORR_FILTER_BINS;
  decoder->filter_bins = decoder->fcorr_bins;
  myDriver->tone_decoder = decoder;

  /* Set magnitude comparison hysteresis value. */
  decoder->hysteresis = FCORR_HYSTERESIS;

  /* Create and attach the floating point pre-filter. */
  setup_fcorr_prefilter(decoder);

  /*
   * Set the conversion level from binary to -h to +h representation.
   * For convenience an array of 2 x f32 values hold + and - values.
   */

  /* First set the value for a PWM 1. */
  decoder->sample_level[1] = FCORR_SAMPLE_LEVEL;
  /* Then set the value for a PWM 0. */
  decoder->sample_level[0] = -FCORR_SAMPLE_LEVEL;

  /* Setup the decoder tone IQ filters. */
  /* Set tone frequencies. */
  decoder->filter_bins[AFSK_MARK_INDEX].freq = AFSK_MARK_FREQUENCY;
  decoder->filter_bins[AFSK_SPACE_INDEX].freq = AFSK_SPACE_FREQUENCY;
  setup_fcorr_IQfilters(decoder);

#if USE_FCORR_MAG_LPF == TRUE
  /* Setup the IQ magnitude LPFs. */
  setup_fcorr_magfilter(decoder);
#endif
}

/**
 * @brief   Called release the FCORR decoder resources.
 *
 * @param[in] myDriver  pointer to AFSKDemodDriver data structure.
 *
 *@api
 */
void release_fcorr_decoder(AFSKDemodDriver *myDriver) {
  /*
   * Release the FCORR object in heap.
   * All filters are contained in this object.
   */
  chHeapFree(myDriver->tone_decoder);
}

#endif /* AFSK_DSP_FCORR_DECODE */

/** @} */
