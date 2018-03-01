/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    firfilter_q31.h
 * @brief   Fixed point FIR filter structures and macros.
 * @details This module implements generic FIR filter control.
 *
 * @addtogroup DSP
 * @{
 */

#ifndef IO_FILTERS_FIR_Q31_H_
#define IO_FILTERS_FIR_Q31_H_

/**
 * @brief   FIR filter control structure.
 *
 * @note    This is a generic FIR filter.
 * @note    The type is determined by coefficients set by decoder.
 */
typedef struct QFIRFilter {
  arm_fir_instance_q31  *filter_instance;
  uint16_t              block_size;
  uint8_t               scale;
} qfir_filter_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

  #ifdef __cplusplus
  extern "C" {
  #endif
    void create_qfir_filter(
      qfir_filter_t *filter,
      arm_fir_instance_q31 *instance,
      uint16_t numTaps,
      q31_t * pCoeffs,
      q31_t * pState,
      uint32_t blockSize,
      float32_t * pf32Coeffs);
    void reset_qfir_filter(qfir_filter_t *filter);
    void apply_qfir_filter(qfir_filter_t *filter, q31_t *input, q31_t *output);
    void compute_qfir_coefficents(qfir_filter_t *filter);
    void transpose_qfir_coefficients(arm_fir_instance_q31 *instance);
  #ifdef __cplusplus
  }
  #endif

#endif /* IO_FILTERS_FIR_Q31_H_ */

/** @} */
