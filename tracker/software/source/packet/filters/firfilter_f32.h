/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    firfilter_f32.h
 * @brief   Floating point F32 FIR filter structures and macros.
 * @details This module implements generic FIR filter control.
 *
 * @addtogroup DSP
 * @{
 */

#ifndef IO_FILTERS_FIR_F32_H_
#define IO_FILTERS_FIR_F32_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/* All FFIR processing is one sample at a time. */

#define PKT_FFIR_BLOCK_SIZE 1U

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
 * @brief   F32 FIR filter control structure for general filters.
 *
 * @note    This is a generic F32 FIR filter.
 * @note    The type is determined by coefficients set by decoder.
 * @note    It allows for block size to be set per filter (unused feature)
 */
typedef struct FFIRFilter {
  arm_fir_instance_f32  *filter_instance;
  uint16_t              block_size;
} ffir_filter_t;

/**
 * @brief   F32 FIR filter control structure for embedded filters.
 *
 * @note    This is a structure for use in embedded F32 FIR filter.
 * @note    The type is determined by coefficients set by decoder.
 * @note    Filters of this type use a common fixed block size.
 */
typedef struct FFIREmbeddedFilter {
  arm_fir_instance_f32  instance;
} ffir_emb_filter_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Allocate a complete FFIR filter data structure.
 * @notes   The instance, state and coefficient data are held in one structure.
 * @notes   Used for fully embedding a filter inside another object/struct.
 * @notes   Note the requirement for use of an anonymous struct in the macro.
 * @notes   This allows the macro to be re-used.
 *
 * @param[in]   taps    number of taps in the filter
 * @param[in]   name    name of the filter
 */
#define PKT_EMBED_FFIR(taps, name)                                           \
  struct {                                                                   \
    ffir_emb_filter_t       core;                                            \
    float32_t               coeffs[taps];                                    \
    float32_t               state[PKT_FFIR_BLOCK_SIZE + taps - 1];           \
} name;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

  #ifdef __cplusplus
  extern "C" {
  #endif
    void create_ffir_filter(ffir_filter_t *filter,
                            arm_fir_instance_f32 *instance,
                            uint16_t numTaps,
                            float32_t * pCoeffs,
                            float32_t * pState,
                            uint32_t blockSize,
                            float32_t *const pf32Coeffs);
    void reset_ffir_filter(ffir_filter_t *filter);
    void reset_ffir_embedded_filter(arm_fir_instance_f32 *instance);
    void apply_ffir_filter(ffir_filter_t *filter, float32_t *input,
                           float32_t *output);
    void create_ffir_embedded_filter(ffir_emb_filter_t *filter,
                                     arm_fir_instance_f32 *instance,
                                     uint16_t numTaps,
                                     float32_t *pCoeffs,
                                     float32_t *pState,
                                     float32_t *const pf32Coeffs);
    void apply_ffir_embedded_filter(ffir_emb_filter_t *filter,
                                    float32_t *input, float32_t *output);
    void compute_ffir_coefficents(ffir_filter_t *filter);
    void transpose_ffir_coefficients(arm_fir_instance_f32 *instance);
  #ifdef __cplusplus
  }
  #endif

#endif /* IO_FILTERS_FIR_F32_H_ */

/** @} */
