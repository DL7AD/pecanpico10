/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    corr_q31.h
 * @brief   Correlator using fixed point Q31.
 *
 * @addtogroup DSP
 * @{
 */

#ifndef IO_DECODERS_QCORR_H_
#define IO_DECODERS_QCORR_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define QCORR_FILTER_BINS           AFSK_NUM_TONES /* Set by AFSK header. */

#define QCORR_SAMPLE_LEVEL          0.9f
#define QCORR_HYSTERESIS            0.01f

#define QCORR_PHASE_SEARCH          1
#define QCORR_PLL_COMB_SIZE         64

#if (QCORR_PLL_COMB_SIZE & (QCORR_PLL_COMB_SIZE - 1)) != 0
#error "QCORR_PLL_COMB_SIZE is not a power of two"
#endif

#define USE_QCORR_FRACTIONAL_PLL    TRUE
#define QCORR_PLL_SEARCH_RATE       0.5f
#define QCORR_PLL_LOCKED_RATE       0.75f

#define QCORR_IQ_WINDOW             TD_WINDOW_CHEBYSCHEV

/* Used for indexing of IQ filter sections. */
#define QCORR_COS_INDEX             0U
#define QCORR_SIN_INDEX             1U

#define REPORT_QCORR_COEFFS         FALSE

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Correlation diagnostics statistics.
 *
 */
typedef struct qStats {
  dsp_phase_t   correction;
  dsp_phase_t   drift_max;
  dsp_phase_t   drift_min;
} qcorr_stats_t;

/**
 * @brief   Correlation decoder bin (tone) structure.
 *
 * @note    Each bin is defined and maintains its specific parameters.
 */
typedef struct qTone {
  uint16_t          freq;
  qfir_filter_t     *tone_filter[AFSK_NUM_TONES];
  qfir_filter_t     *mag_filter;
  q31_t             raw_mag;
  q31_t             filtered_mag;
  q31_t             mag;
  q31_t             cos_out;
  q31_t             sin_out;
} qcorr_tone_t;

/**
 * @brief   Correlation decoder control structure.
 *
 */
typedef struct qCorrFilter {
  AFSKDemodDriver   *demod_driver;
  qfir_filter_t     *input_filter;
  uint16_t          decode_length;
  uint32_t          current_n;
  uint32_t          sample_rate;
  q31_t             preFilterOut;
  uint32_t          filter_valid;
  uint8_t           number_bins;
  qcorr_tone_t      *filter_bins;
  q31_t             sample_level[2];
  q31_t             hysteresis;
  tone_t            prior_demod;
  tone_t            current_demod;
  dsp_phase_t       phase_correction;
  dsp_phase_t       phase_delta;
  dsp_phase_t       search_rate;
  dsp_phase_t       pll_comb_filter[QCORR_PLL_COMB_SIZE];
  dsp_phase_t       pll_locked_integrator;
#if  USE_QCORR_FRACTIONAL_PLL == TRUE
  int32_t           symbol_pll;
  int32_t           prior_pll;
#endif
#if USE_AFSK_PHASE_STATISTICS == TRUE
  qcorr_stats_t     statistics;
#endif
} qcorr_decoder_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  q31_t push_qcorr_sample(AFSKDemodDriver *myDriver, bit_t sample);
  bool process_qcorr_output(AFSKDemodDriver *myDriver);
  void calc_qcorr_magnitude(AFSKDemodDriver *myDriver);
  void filter_qcorr_magnitude(AFSKDemodDriver *myDriver);
  void reset_qcorr_all(AFSKDemodDriver *myDriver);
  void evaluate_qcorr_tone(AFSKDemodDriver *myDriver);
  bool get_qcorr_symbol_timing(AFSKDemodDriver *myDriver);
  void update_qcorr_pll(AFSKDemodDriver *myDriver);
  void init_qcorr_decoder(AFSKDemodDriver *myDriver);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#if USE_AFSK_PHASE_STATISTICS
static inline qcorr_stats_t *get_qcorr_statistics(AFSKDemodDriver *myDriver) {
  qcorr_decoder_t *decoder = (qcorr_decoder_t *)myDriver->tone_decoder;
  qcorr_stats_t *statistics = &decoder->statistics;
  return statistics;
}
#endif

#endif /* IO_DECODERS_QCORR_H_ */

/** @} */
