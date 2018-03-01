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

/* Allocate the decoder main structure and the tone bins. */
qcorr_decoder_t QCORR1;
qcorr_tone_t qcorr_bins[QCORR_FILTER_BINS];

/**
 * @brief   AFSK_PWM_QFILTER pre-filter identifier.
 * @note    Allocate a pre-filter FIR record.
 */

qfir_filter_t AFSK_PWM_QFILTER;

/*
 * Allocate data for prefilter FIR.
 */
static arm_fir_instance_q31 pre_filter_instance_q31;
static q31_t pre_filter_state_q31[PRE_FILTER_BLOCK_SIZE
                                  + PRE_FILTER_NUM_TAPS - 1];
static q31_t pre_filter_coeff_q31[PRE_FILTER_NUM_TAPS];

#if USE_QCORR_MAG_LPF == TRUE

/* Allocate the FIR filter structures. */
qfir_filter_t QFILT_M_MAG;
qfir_filter_t QFILT_S_MAG;

/*
* Allocate data for mag FIR filter.
*/
static q31_t mag_filter_coeff_q31[MAG_FILTER_NUM_TAPS];

static arm_fir_instance_q31 m_mag_filter_instance_q31;
static q31_t m_mag_filter_state_q31[MAG_FILTER_BLOCK_SIZE
                                + MAG_FILTER_NUM_TAPS - 1];

static arm_fir_instance_q31 s_mag_filter_instance_q31;
static q31_t s_mag_filter_state_q31[MAG_FILTER_BLOCK_SIZE
                                + MAG_FILTER_NUM_TAPS - 1];

#endif /* USE_QCORR_MAG_LPF == TRUE */

/* Mark and Space correlation filter instances. */
qfir_filter_t QFILT_M_COS;
qfir_filter_t QFILT_M_SIN;

qfir_filter_t QFILT_S_COS;
qfir_filter_t QFILT_S_SIN;

/*
* Allocate data for Mark and Space correlation filters.
*/

/* q31 filter coefficient arrays. */
static q31_t m_cos_filter_coeff_q31[DECODE_FILTER_LENGTH];
static q31_t m_sin_filter_coeff_q31[DECODE_FILTER_LENGTH];
static q31_t s_cos_filter_coeff_q31[DECODE_FILTER_LENGTH];
static q31_t s_sin_filter_coeff_q31[DECODE_FILTER_LENGTH];

/* q31 fir instance records. */
static arm_fir_instance_q31 m_cos_filter_instance_q31;
static arm_fir_instance_q31 m_sin_filter_instance_q31;
static arm_fir_instance_q31 s_cos_filter_instance_q31;
static arm_fir_instance_q31 s_sin_filter_instance_q31;

#define QCORR_FILTER_BLOCK_SIZE 1U

/* q31 filter state arrays. */
static q31_t m_cos_filter_state_q31[QCORR_FILTER_BLOCK_SIZE
                                + DECODE_FILTER_LENGTH - 1];
static q31_t m_sin_filter_state_q31[QCORR_FILTER_BLOCK_SIZE
                                + DECODE_FILTER_LENGTH - 1];
static q31_t s_cos_filter_state_q31[QCORR_FILTER_BLOCK_SIZE
                                + DECODE_FILTER_LENGTH - 1];
static q31_t s_sin_filter_state_q31[QCORR_FILTER_BLOCK_SIZE
                                + DECODE_FILTER_LENGTH - 1];



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
 * @brief   Resets the correlator state.
 * @post    Filter state is reset.
 * @post    Filter variables are reset.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure.
 *
 * @api
 */
void reset_qcorr_all(AFSKDemodDriver *driver) {
  qcorr_decoder_t *decoder = driver->tone_decoder;
  qfir_filter_t *input_filter = decoder->input_filter;
  if(input_filter != NULL)
    (void)reset_qfir_filter(input_filter);
  decoder->preFilterOut = 0;

  uint8_t i;
  for(i = 0; i < decoder->number_bins; i++) {

    /* Reset the magnitude filter. */
    qfir_filter_t *mag_filter = decoder->filter_bins[i].mag_filter;
    if(mag_filter != NULL)
    (void)reset_qfir_filter(mag_filter);

    /* Reset the correlation filters. */
    qcorr_tone_t *myBin = &decoder->filter_bins[i];
    qfir_filter_t *myCosFilter = myBin->tone_filter[QCORR_COS_INDEX];
    qfir_filter_t *mySinFilter = myBin->tone_filter[QCORR_SIN_INDEX];
    if(myCosFilter != NULL)
     (void)reset_qfir_filter(myCosFilter);
    if(mySinFilter != NULL)
     (void)reset_qfir_filter(mySinFilter);

    decoder->filter_bins[i].raw_mag = 0;
    decoder->filter_bins[i].mag = 0;
  } /* End for (number_bins). */
  decoder->current_n = 0;
  decoder->filter_valid = 0;

  decoder->prior_demod = TONE_NONE;
  decoder->current_demod = TONE_NONE;

  decoder->phase_correction = 0;

#if  USE_QCORR_FRACTIONAL_PLL == TRUE
  decoder->symbol_pll = 0/*(int32_t)-1*/;
#endif
  decoder->search_rate = 0;
  /*
   * Reset phase drift analysis.
   */
  decoder->phase_delta = 0;
  decoder->pll_locked_integrator = 0;

  /*
   * Clear the comb filter.
   */
  for(i = QCORR_PLL_COMB_SIZE - 1; i > 0; i--) {
    decoder->pll_comb_filter[i] = 0;
  }

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
  qfir_filter_t *myFilter = decoder->input_filter;

  apply_qfir_filter(myFilter, &decoder->sample_level[sample],
                    &decoder->preFilterOut);
#if AFSK_DEBUG_TYPE == AFSK_QCORR_FIR_DEBUG
    char buf[80];
    int out = chsnprintf(buf, sizeof(buf), "%X\r\n", scaledOut);
    chnWrite(pkt_out, (uint8_t *)buf, out);
#endif

  /*
   * Return most recent filter output.
   */
  return decoder->preFilterOut;
}

/**
 * @brief   Called at each new sample to process correlation.
 * @notes   The correlation filters are run for each tone and phase.
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
    qfir_filter_t *myCosFilter = myBin->tone_filter[QCORR_COS_INDEX];
    qfir_filter_t *mySinFilter = myBin->tone_filter[QCORR_SIN_INDEX];

    /*
     * Run correlation for bin.
     */
    apply_qfir_filter(myCosFilter, &decoder->preFilterOut, &myBin->cos_out);

    apply_qfir_filter(mySinFilter, &decoder->preFilterOut, &myBin->sin_out);


#if AFSK_DEBUG_TYPE == AFSK_QCORR_DEC_CS_DEBUG
    char buf[200];
    if(i == 0) {
    int out =chsnprintf(buf, sizeof(buf), "%i, %i, ",
      myBin->cos_out, myBin->sin_out);
    } else {
      int out = chsnprintf(buf, sizeof(buf), "%i, %i\r\n",
        myBin->cos_out, myBin->sin_out);
    }
    chnWrite(pkt_out, (uint8_t *)buf, out);
#endif
  }

  /* Wait for initial data to appear from pre-filter. */
  if(++decoder->filter_valid <
      PRE_FILTER_NUM_TAPS + DECODE_FILTER_LENGTH)
    return false;

  /* Compute magnitude of bins. */
  calc_qcorr_magnitude(myDriver);

  /* Filter magnitude. */
#if USE_QCORR_MAG_LPF == TRUE
  filter_qcorr_magnitude(myDriver);
  /* Delay filter ready by mag filter size + pre-filter size. */
  if(decoder->filter_valid <
      (decoder->input_filter->filter_instance->numTaps
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
    chnWrite(pkt_out, (uint8_t *)buf, out);
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

#if USE_QCORR_FRACTIONAL_PLL == TRUE
  decoder->prior_pll = decoder->symbol_pll;
#define PLL_INCREMENT (UINT_MAX / SYMBOL_DECIMATION)
  decoder->symbol_pll = (int32_t)((uint32_t)(decoder->symbol_pll) + PLL_INCREMENT);
  /* Check the symbol period was reached and return status. */
  return ((decoder->symbol_pll < 0) && (decoder->prior_pll > 0));
#else
  /*
   * Calculate current phase point in symbol.
   */
  uint8_t symbol_phase = (decoder->current_n + decoder->phase_correction)
      % SYMBOL_DECIMATION/*myDriver->decimation_slices*/;

  return (symbol_phase == 0);
#endif
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
void update_qcorr_pll(AFSKDemodDriver *myDriver) {
  qcorr_decoder_t *decoder = myDriver->tone_decoder;
  /*
   * Now test if a tone transition has taken place.
   */
  if(decoder->current_demod != decoder->prior_demod) {

    /* Update tone state. */
    decoder->prior_demod = decoder->current_demod;
#if USE_QCORR_FRACTIONAL_PLL == TRUE
    if(myDriver->frame_state == FRAME_SEARCH) {
      decoder->symbol_pll = (int32_t)((float32_t)decoder->symbol_pll
          * QCORR_PLL_SEARCH_RATE);
    } else {
      decoder->symbol_pll = (int32_t)((float32_t)decoder->symbol_pll
          * QCORR_PLL_LOCKED_RATE);
    }
  }
#else

    /*
     * Calculate the phase delta from the center of the filter.
     * A positive delta means the tone transition is late in the window.
     * The phase correction will be increased when delta is added.
     * A negative delta means the tone transition is early in the window.
     * The phase correction will be decreased when delta is added.
     *
     * After CIC filtering the delta is applied to phase correction at symbol end.
     */
      decoder->phase_delta = (decoder->current_n % decoder->decode_length)
          - SYMBOL_DECIMATION;

  }
  /* Filter current sample count has already been updated. */
  ++decoder->current_n;
#endif
}

void placeholder_qcorr_pll(AFSKDemodDriver *myDriver) {
  qcorr_decoder_t *decoder = myDriver->tone_decoder;
  /*
   * Calculate current phase point in symbol.
   */
  uint8_t symbol_phase = (decoder->current_n + decoder->phase_correction)
      % SYMBOL_DECIMATION/*myDriver->decimation_slices*/;
  /*
   * If a symbol end has been reached then process.
   */
  if(symbol_phase == 0) {
    /*
     * A moving average filter is used to calculate phase delta.
     *
     * The MA filter is implemented as a CIC running sum filter.
     * Values taken from the accumulator must be scaled by the filter size (taps).
     * This normalizes the gain inherent in the accumulator.
     *
     * Get the phase delta from N symbol periods ago.
     * Apply comb filter: Yn = Zn - Z(n - N).
     *
     * Note phase delta is captured at tone transition.
     * In the case there is no transition in the symbol period a zero is added.
     */
    dsp_phase_t history_comb_out = decoder->phase_delta -
        decoder->pll_comb_filter[QCORR_PLL_COMB_SIZE - 1];

    /*
     * Compute new integrator value Z.
     * Z is a sum of phase errors * comb size.
     * Update Z0 and Z-1 saved values.
     */
    decoder->pll_locked_integrator += history_comb_out;

    /*
     * Push the comb filter history down.
     */
    uint8_t i;
    for(i = QCORR_PLL_COMB_SIZE - 1; i > 0; i--) {
      decoder->pll_comb_filter[i] = decoder->pll_comb_filter[i - 1];
    }

    /*
     * Add the most recent symbol phase delta to the comb filter.
     * There may be no tone transition prior to next symbol end.
     * So clear the phase error after using the current value.
     * Thus a zero will be fed to the comb filter at next symbol time.
     */
    decoder->pll_comb_filter[0] = decoder->phase_delta;
    decoder->phase_delta = 0;

    if(myDriver->frame_state == FRAME_SEARCH) {
      /*
       * Adjust sample search offset timing within symbol.
       * TODO: Is this really helping versus using si radio clock re-timing?
       */
/*      ++decoder->search_rate;
      if((decoder->search_rate > 32) && (decoder->search_rate % 8 == 0)) {
          decoder->phase_correction =
          (decoder->phase_correction + QCORR_PHASE_SEARCH)
          % SYMBOL_DECIMATION;
      }*/
    }
    //return true;
    return;
  }
  if(symbol_phase == (SYMBOL_DECIMATION / 2)) {
    /*
     * Update the phase correction at the center of the symbol time.
     * This ensures that a symbol end is not re-detected or missed.
     */

    if(myDriver->frame_state == FRAME_OPEN) {
      /* The CIC value has to scaled by the number of taps in the filter. */
      //decoder->phase_correction +=
          //(decoder->pll_locked_integrator / QCORR_PLL_COMB_SIZE);
    }
  }
  return;
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
      chnWrite(pkt_out, (uint8_t *)buf, out);
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
  #if AFSK_ERROR_TYPE == AFSK_QSQRT_ERROR
        char buf[200];
        int out = chsnprintf(buf, sizeof(buf),
          "MAG SQRT failed bin %i, cosQ %X, sinQ %X, cos %X, sin %X,"
          "mag2 %X, mag %X, index %i\r\n",
          i, myBin->cos_out, myBin->sin_out, cos, sin, mag2,
          decoder->filter_bins[i].raw_mag, decoder->current_n);
        chnWrite(pkt_out, (uint8_t *)buf, out);
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
    chnWrite(pkt_out, (uint8_t *)buf, out);
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

    apply_qfir_filter(decoder->filter_bins[i].mag_filter,
                      &decoder->filter_bins[i].raw_mag,
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
    chnWrite(pkt_out, (uint8_t *)buf, out);
#endif
  }
}

/**
 * @brief Called evaluate the tone strengths in the filters.
 *
 * @param[in]   myDriver   pointer to a @p AFSKDemodDriver structure.
 *
 */
void evaluate_qcorr_tone(AFSKDemodDriver *myDriver) {
  qcorr_decoder_t *myDecoder = (qcorr_decoder_t *)myDriver->tone_decoder;
  q31_t mark, space;
  q31_t delta;

  /*
   * Check if the prior symbol tone is different to the current symbol tone.
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
    chnWrite(pkt_out, (uint8_t *)buf, out);
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

  decoder->input_filter = &AFSK_PWM_QFILTER;
  /*
   * Initialise the pre-filter.
   */
  create_qfir_filter(decoder->input_filter,
    &pre_filter_instance_q31,
    PRE_FILTER_NUM_TAPS,
    pre_filter_coeff_q31,
    pre_filter_state_q31,
    PRE_FILTER_BLOCK_SIZE,
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
  chnWrite(pkt_out, (uint8_t *)buf, out);
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
  /* Set tone frequencies. */
  decoder->filter_bins[AFSK_MARK_INDEX].freq = AFSK_MARK_FREQUENCY;
  decoder->filter_bins[AFSK_SPACE_INDEX].freq = AFSK_SPACE_FREQUENCY;

  /* Set COS and SIN filters for Mark and Space. */

  decoder->filter_bins[AFSK_MARK_INDEX].tone_filter[QCORR_COS_INDEX]
                                                    = &QFILT_M_COS;
  decoder->filter_bins[AFSK_MARK_INDEX].tone_filter[QCORR_SIN_INDEX]
                                                    = &QFILT_M_SIN;


  decoder->filter_bins[AFSK_SPACE_INDEX].tone_filter[QCORR_COS_INDEX]
                                                     = &QFILT_S_COS;
  decoder->filter_bins[AFSK_SPACE_INDEX].tone_filter[QCORR_SIN_INDEX]
                                                     = &QFILT_S_SIN;

  /* Temporary float coeff arrays. */
  float32_t cos_table[decoder->decode_length];
  float32_t sin_table[decoder->decode_length];

  /* Calculate the IQ filter coefficients for Mark. */
  float32_t norm_freq = (float32_t)AFSK_MARK_FREQUENCY
      / (float32_t)decoder->sample_rate;

  gen_fir_iqf(cos_table, sin_table, decoder->decode_length,
              norm_freq, QCORR_IQ_WINDOW);
  /*
   * Create the Mark correlation filters.
   */
  create_qfir_filter(&QFILT_M_COS,
    &m_cos_filter_instance_q31,
    DECODE_FILTER_LENGTH,
    m_cos_filter_coeff_q31,
    m_cos_filter_state_q31,
    QCORR_FILTER_BLOCK_SIZE,
    cos_table);

  create_qfir_filter(&QFILT_M_SIN,
    &m_sin_filter_instance_q31,
    DECODE_FILTER_LENGTH,
    m_sin_filter_coeff_q31,
    m_sin_filter_state_q31,
    QCORR_FILTER_BLOCK_SIZE,
    sin_table);

  /* Calculate the IQ filter coefficients for Space. */
  norm_freq = (float32_t)AFSK_SPACE_FREQUENCY
      / (float32_t)decoder->sample_rate;

  gen_fir_iqf(cos_table, sin_table, decoder->decode_length,
              norm_freq, QCORR_IQ_WINDOW);

  /*
   * Create the Space correlation filters.
   */
  create_qfir_filter(&QFILT_S_COS,
     &s_cos_filter_instance_q31,
     DECODE_FILTER_LENGTH,
     s_cos_filter_coeff_q31,
     s_cos_filter_state_q31,
     QCORR_FILTER_BLOCK_SIZE,
     cos_table);

  create_qfir_filter(&QFILT_S_SIN,
     &s_sin_filter_instance_q31,
     DECODE_FILTER_LENGTH,
     s_sin_filter_coeff_q31,
     s_sin_filter_state_q31,
     QCORR_FILTER_BLOCK_SIZE,
     sin_table);
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
  (void)decoder;

   /*
    * Initialise the magnitude filters.
    */
  create_qfir_filter(&QFILT_M_MAG,
    &m_mag_filter_instance_q31,
    MAG_FILTER_NUM_TAPS,
    mag_filter_coeff_q31,
    m_mag_filter_state_q31,
    MAG_FILTER_BLOCK_SIZE,
    mag_filter_coeff_f32);

  create_qfir_filter(&QFILT_S_MAG,
    &s_mag_filter_instance_q31,
    MAG_FILTER_NUM_TAPS,
    mag_filter_coeff_q31,
    s_mag_filter_state_q31,
    MAG_FILTER_BLOCK_SIZE,
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
  chnWrite(pkt_out, (uint8_t *)buf, out);
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
  qcorr_decoder_t *decoder = &QCORR1;

  /* Calculate the sample rate for the system.
   * TODO: Centralize sample rate definition/calculation. */
  decoder->sample_rate = FILTER_SAMPLE_RATE;

  /* low level initialization. */
  decoder->decode_length = DECODE_FILTER_LENGTH;
  decoder->number_bins = QCORR_FILTER_BINS;
  decoder->filter_bins = qcorr_bins;
  myDriver->tone_decoder = decoder;

  /* Calculate hysteresis value. */
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

  /* First set the value for PWM 1. */
  float32_t input = QCORR_SAMPLE_LEVEL;

  /* Convert float sample value to q31. */
  arm_float_to_q31(&input, &decoder->sample_level[1], 1);

  /* Then set the value for PWM 0. */
  decoder->sample_level[0] = -decoder->sample_level[1];

  /* Setup the decoder tone IQ filters. */
  setup_qcorr_IQfilters(decoder);

#if USE_QCORR_MAG_LPF == TRUE
  /* Setup the IQ magnitude LPFs. */
  decoder->filter_bins[AFSK_MARK_INDEX].mag_filter = &QFILT_M_MAG;
  decoder->filter_bins[AFSK_SPACE_INDEX].mag_filter = &QFILT_S_MAG;
  setup_qcorr_magfilter(decoder);
#endif
}

#endif /* AFSK_DSP_QCORR_DECODE */

/** @} */
