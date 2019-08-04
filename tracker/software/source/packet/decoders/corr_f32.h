/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    corr_f32.h
 * @brief   Correlator using floating point F32.
 *
 * @addtogroup DSP
 * @{
 */

#ifndef IO_DECODERS_FCORR_H_
#define IO_DECODERS_FCORR_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define FCORR_FILTER_BINS           AFSK_NUM_TONES /* Set by AFSK header. */

#define FCORR_SAMPLE_LEVEL          0.9f
#define FCORR_HYSTERESIS            0.01f

/* Symbol PLL parameters. */
#define FCORR_PLL_SEARCH_RATE       0.5f
#define FCORR_PLL_LOCKED_RATE       0.75f

#define FCORR_IQ_WINDOW             TD_WINDOW_CHEBYSCHEV

/* Used for indexing of IQ filter sections. */
#define FCORR_COS_INDEX             0U
#define FCORR_SIN_INDEX             1U
#define FCORR_IQ_COUNT              2U

//#define FCORR_FILTER_BLOCK_SIZE     1U

#define REPORT_FCORR_COEFFS         FALSE

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
 * @brief   Correlation decoder bin (tone) structure.
 *
 * @note    Each bin defined maintains its specific parameters.
 */
typedef struct fTone {
  uint16_t          freq;
  PKT_EMBED_FFIR(DECODE_FILTER_LENGTH, cos_filter)
  PKT_EMBED_FFIR(DECODE_FILTER_LENGTH, sin_filter)
  PKT_EMBED_FFIR(MAG_FILTER_NUM_TAPS, mag_filter)
  float32_t         raw_mag;
  float32_t         filtered_mag;
  float32_t         mag;
  float32_t         cos_out;
  float32_t         sin_out;
} fcorr_tone_t;

/**
 * @brief   Correlation decoder control structure.
 *
 */
typedef struct fCorrFilter {
  PKT_EMBED_FFIR(PRE_FILTER_NUM_TAPS, pre_filter)
  uint16_t          decode_length;
  uint32_t          current_n;
  uint32_t          sample_rate;
  float32_t         prefilter_out;
  uint32_t          filter_valid;
  uint8_t           number_bins;
  fcorr_tone_t      *filter_bins;
  fcorr_tone_t      fcorr_bins[FCORR_FILTER_BINS];
  float32_t         sample_level[2];
  float32_t         hysteresis;
  tone_t            prior_demod;
  tone_t            current_demod;
  int32_t           symbol_pll;
  int32_t           prior_pll;
} fcorr_decoder_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  float32_t push_fcorr_sample(AFSKDemodDriver *myDriver, bit_t sample);
  bool      process_fcorr_output(AFSKDemodDriver *myDriver);
  void      calc_fcorr_magnitude(AFSKDemodDriver *myDriver);
  void      filter_fcorr_magnitude(AFSKDemodDriver *myDriver);
  void      reset_fcorr_all(AFSKDemodDriver *myDriver);
  void      evaluate_fcorr_tone(AFSKDemodDriver *myDriver);
  bool      get_fcorr_symbol_timing(AFSKDemodDriver *myDriver);
  void      update_fcorr_pll(AFSKDemodDriver *myDriver);
  void      init_fcorr_decoder(AFSKDemodDriver *myDriver);
  void      release_fcorr_decoder(AFSKDemodDriver *myDriver);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/


#endif /* IO_DECODERS_FCORR_H_ */

/** @} */
