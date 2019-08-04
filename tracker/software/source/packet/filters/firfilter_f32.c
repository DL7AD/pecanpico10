/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/


/**
 * @file    firfilter_f32.c
 * @brief   F32 FIR filter implementation.
 *
 * @addtogroup DSP
 * @{
 */


#include "pktconf.h"

/*===========================================================================*/
/* Filter exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Filter local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Filter exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Creates a F32 floating point FIR filter.
 *
 * @param[in] filter        pointer to a @p ffir_filter_t structure
 * @param[in] instance      pointer to a @p arm_fir_instance_f32 structure
 * @param[in] numTaps       the number of taps in the filter
 * @param[in] pCoeffs       pointer to array of f32 filter coefficients
 * @param[in] pState        pointer to f32 state values used by filter
 * @param[in] instance      pointer to a @p arm_fir_instance_f32 structure
 * @param[in] blockSize     the number of samples processed at a
 *                          time in the filter
 * @param[in] pf32Coeffs    pointer to array of float32 filter coefficients
 *                          If NULL f32 coefficients to be otherwise filled
 *
 * @api
 */
void create_ffir_filter(
  ffir_filter_t *filter,
  arm_fir_instance_f32 *instance,
  uint16_t numTaps,
  float32_t *pCoeffs,
  float32_t *pState,
  uint32_t blockSize,
  float32_t *const pf32Coeffs) {

  /* Save instance. */
  filter->filter_instance = instance;

  /* Assign filter taps */
  instance->numTaps = numTaps;

  /* Assign coefficient pointer */
  instance->pCoeffs = pCoeffs;

  /* Assign state pointer */
  instance->pState = pState;

  /* Save blocksize. */
  filter->block_size = blockSize;

  /* Copy and transpose float32 coefficients if supplied. */
  if(pf32Coeffs != NULL) {
    arm_copy_f32(pf32Coeffs, pCoeffs, numTaps);
    transpose_ffir_coefficients(instance);
  }

  /* Clear state buffer and state array size is (blockSize + numTaps - 1) */
  reset_ffir_filter(filter);
}


/**
 * @brief   Resets the filter internal state data.
 *
 * @param[in] filter        pointer to filter data structure.
 */
void reset_ffir_filter(ffir_filter_t *filter) {
  uint16_t pState_size = filter->filter_instance->numTaps
      + filter->block_size - 1;
  arm_fill_f32(0, filter->filter_instance->pState, pState_size);
}

/**
 * @brief   Pushes new input sample(s) through the filter and fetches output(s).
 * @note    Sample scaling is not required in F32 filters.
 *
 * @param[in] filter    pointer to a @p ffir_filter_t structure
 * @param[in] input     pointer to input sample(s) buffer
 * @param[in] output    pointer to output sample(s) buffer
 *
 * @api
 */
void apply_ffir_filter(ffir_filter_t *filter, float32_t *input,
                       float32_t *output) {

  /*
   * Apply the input(s) to the filter and compute the output result(s).
   */
  arm_fir_f32(filter->filter_instance, input,
              output, filter->block_size);
}

/**
 * @brief   Creates a F32 floating point FIR filter.
 *
 * @param[in] filter        pointer to a @p FFIREmbeddedFilter structure
 * @param[in] instance      pointer to a @p arm_fir_instance_f32 structure
 * @param[in] numTaps       the number of taps in the filter
 * @param[in] pCoeffs       pointer to array of f32 filter coefficients
 * @param[in] pState        pointer to f32 state values used by filter
 * @param[in] pf32Coeffs    pointer to array of float32 filter coefficients
 *                          If NULL f32 coefficients to be otherwise filled
 *
 * @api
 */
void create_ffir_embedded_filter(ffir_emb_filter_t *filter,
                                 arm_fir_instance_f32 *instance,
                                 uint16_t numTaps,
                                 float32_t *pCoeffs,
                                 float32_t *pState,
                                 float32_t *const pf32Coeffs) {
  (void)filter;

  /* Assign filter taps */
  instance->numTaps = numTaps;

  /* Assign coefficient pointer */
  instance->pCoeffs = pCoeffs;

  /* Assign state pointer */
  instance->pState = pState;

  /* Copy and transpose float32 coefficients if supplied. */
  if(pf32Coeffs != NULL) {
    arm_copy_f32(pf32Coeffs, pCoeffs, numTaps);
    transpose_ffir_coefficients(instance);
  }

  /* Clear state buffer and state array. */
  reset_ffir_embedded_filter(instance);
}

/**
 * @brief   Resets the filter internal state data.
 *
 * @param[in] filter        pointer to filter object.
 */
void reset_ffir_embedded_filter(arm_fir_instance_f32 *instance) {
  arm_fill_f32(0.0f, instance->pState,
               PKT_FFIR_BLOCK_SIZE + instance->numTaps - 1);
}

/**
 * @brief   Pushes new input sample(s) through the filter and fetches output(s).
 * @note    The new sample(s) are not scaled down before being pushed.
 * @note    Sample scaling is not required in F32 filters.
 *
 * @param[in] filter    pointer to a @p ffir_filter_t structure
 * @param[in] input     pointer to input sample(s) buffer
 * @param[in] output    pointer to output sample(s) buffer
 *
 * @api
 */
void apply_ffir_embedded_filter(ffir_emb_filter_t *filter,
                                float32_t *input, float32_t *output) {

  /*
   * Apply the input(s) to the filter and compute the output result(s).
   */
  arm_fir_f32(&filter->instance, input,
              output, PKT_FFIR_BLOCK_SIZE);
}

/**
 * @brief   Reverse order of coefficients for F32 FIR.
 * @note    CMSIS DSP filters use coefficients in reverse order.
 * @param[in] coeff     pointer to a @p coefficient array.
 * @param[in] numTaps   number of taps in the filter.
 *
 * @api
 */
void transpose_ffir_coefficients(arm_fir_instance_f32 *instance) {

  chDbgAssert(instance != NULL, "invalid filter instance pointer");

  chDbgCheck(instance->numTaps > 2U);

  float32_t *coeff = instance->pCoeffs;
  uint16_t tapIndex = instance->numTaps - 1;

  uint16_t n;
  for(n = 0; n < (tapIndex / 2); n++) {
    /* Swap coefficient orders. */
    float32_t coeff_f32 = coeff[n];
    coeff[n] = coeff[tapIndex - n];
    coeff[tapIndex - n] = coeff_f32;
  }
}

/** @} */
