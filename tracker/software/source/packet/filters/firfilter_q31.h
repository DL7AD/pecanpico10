/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    firfilter_q31.h
 * @brief   Fixed point Q31 FIR filter structures and macros.
 * @details This module implements generic FIR filter control.
 *
 * @addtogroup DSP
 * @{
 */

#ifndef IO_FILTERS_FIR_Q31_H_
#define IO_FILTERS_FIR_Q31_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/* All QFIR processing is one sample at a time. */

#define PKT_QFIR_BLOCK_SIZE 1U

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
 * @brief   Q31 FIR filter control structure for general filters.
 *
 * @note    This is a generic Q31 FIR filter.
 * @note    The type is determined by coefficients set by decoder.
 * @note    It allows for block size to be set per filter (unused feature)
 */
typedef struct QFIRFilter {
  arm_fir_instance_q31  *filter_instance;
  uint16_t              block_size;
  uint8_t               scale;
} qfir_filter_t;

/**
 * @brief   Q31 FIR filter control structure for embedded filters.
 *
 * @note    This is a structure for use in embedded Q31 FIR filter.
 * @note    The type is determined by coefficients set by decoder.
 * @note    Filters of this type use a common fixed block size.
 */
typedef struct QFIREmbeddedFilter {
  arm_fir_instance_q31  instance;
  uint8_t               scale;
} qfir_emb_filter_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Allocate a complete QFIR filter data structure.
 * @notes   The instance, state and coefficient data are held in one structure.
 * @notes   Used for fully embedding a filter inside another object/struct.
 * @notes   Note the requirement for use of an anonymous struct in the macro.
 * @notes   This allows the macro to be re-used (else why make a macro...)
 *
 * @param[in]   taps    number of taps in the filter
 * @param[in]   name    name of the filter
 */
#define PKT_EMBED_QFIR(taps, name)                                           \
  struct {                                                                   \
    qfir_emb_filter_t       core;                                            \
    uint8_t                 scale;                                           \
    q31_t                   coeffs[taps];                                    \
    q31_t                   state[PKT_QFIR_BLOCK_SIZE + taps - 1];           \
} name;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

  #ifdef __cplusplus
  extern "C" {
  #endif
    void create_qfir_filter(qfir_filter_t *filter,
                            arm_fir_instance_q31 *instance,
                            uint16_t numTaps,
                            q31_t * pCoeffs,
                            q31_t * pState,
                            uint32_t blockSize,
                            float32_t *const pf32Coeffs);
    void reset_qfir_filter(qfir_filter_t *filter);
    void reset_qfir_embedded_filter(arm_fir_instance_q31 *instance);
    void apply_qfir_filter(qfir_filter_t *filter, q31_t *input, q31_t *output);
    void create_qfir_embedded_filter(qfir_emb_filter_t *filter,
                                     arm_fir_instance_q31 *instance,
                                     uint16_t numTaps,
                                     q31_t *pCoeffs,
                                     q31_t *pState,
                                     float32_t *const pf32Coeffs);
    void apply_qfir_embedded_filter(qfir_emb_filter_t *filter,
                                    q31_t *input, q31_t *output);
    void compute_qfir_coefficents(qfir_filter_t *filter);
    void transpose_qfir_coefficients(arm_fir_instance_q31 *instance);
  #ifdef __cplusplus
  }
  #endif

#endif /* IO_FILTERS_FIR_Q31_H_ */

/** @} */
