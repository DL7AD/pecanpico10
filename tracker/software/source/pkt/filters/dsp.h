/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    dsp.h
 * @brief   DSP data types.
 *
 * @addtogroup DSP
 * @{
 */

#ifndef PKT_FILTERS_DSP_H_
#define PKT_FILTERS_DSP_H_

#include <limits.h>

/**
 * @brief   Type of Complex data.
 *
 * @note    Type is defined to be CMSIS-DSP compatible.
 */
typedef float32_t complex_f32_t[2];
typedef q31_t     complex_q31_t[2];


/**
 * @brief   Limits for fixed point Q31 data.
 *
 * @note    Used to set limit values.
 */
#define Q31_MAX INT_MAX
#define Q31_MIN INT_MIN

/* DSP time domain windowing definitions. */
typedef enum td_window_e {
  TD_WINDOW_NONE,
  TD_WINDOW_TRUNCATED,
  TD_WINDOW_FLATTOP,
  TD_WINDOW_COSINE,
  TD_WINDOW_SINE,
  TD_WINDOW_HAMMING,
  TD_WINDOW_EXACT_BLACKMAN,
  TD_WINDOW_NUTTALL,
  TD_WINDOW_BLACKMAN_NUTTALL,
  TD_WINDOW_BLACKMAN_HARRIS,
  TD_WINDOW_HANNING,
  TD_WINDOW_CHEBYSCHEV
} td_window_t;

/* DSP frequency domain windowing definitions. */
typedef enum fd_window_e {
  FD_WINDOW_NONE,
  FD_WINDOW_HAMMING,
  FD_WINDOW_HANNING
} fd_window_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

  #ifdef __cplusplus
  extern "C" {
  #endif
    float32_t dsp_window(td_window_t type, size_t size, size_t j);
    void gen_fir_iqf(float32_t *pCos, float32_t *pSin,
                                      uint16_t length,
                                      float32_t norm_freq,
                                      td_window_t w_type);
    void gen_fir_lpf (float32_t fc, float32_t *coeff,
                      size_t numTaps, td_window_t window);
    void gen_fir_bpf (float32_t f1, float32_t f2, float32_t *coeff,
                      size_t numTaps, td_window_t window);
  #ifdef __cplusplus
  }
  #endif

#endif /* PKT_FILTERS_DSP_H_ */

/** @} */
