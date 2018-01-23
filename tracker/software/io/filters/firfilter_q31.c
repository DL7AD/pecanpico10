/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/


/**
 * @file    firfilter_q31.c
 * @brief   Q31 FIR filter implementation.
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
 * @brief   Creates a Q31 fixed point FIR filter.
 *
 * @param[in] filter        pointer to a @p qfir_filter_t structure
 * @param[in] instance      pointer to a @p arm_fir_instance_q31 structure
 * @param[in] numTaps       the number of taps in the filter
 * @param[in] pCoeffs       pointer to array of q31 filter coefficients
 * @param[in] pState        pointer to q31 state values used by filter
 * @param[in] instance      pointer to a @p arm_fir_instance_q31 structure
 * @param[in] blockSize     the number of samples processed at a
 *                          time in the filter
 * @param[in] pf32Coeffs    pointer to array of float32 filter coefficients
 *                          If NULL q31 coefficients to be otherwise filled
 *
 * @api
 */
void create_qfir_filter(
  qfir_filter_t *filter,
  arm_fir_instance_q31 *instance,
  uint16_t numTaps,
  q31_t *pCoeffs,
  q31_t *pState,
  uint32_t blockSize,
  float32_t *pf32Coeffs) {

  /* Save instance. */
  filter->filter_instance = instance;

  /* Assign filter taps */
  instance->numTaps = numTaps;

  /* Assign coefficient pointer */
  instance->pCoeffs = pCoeffs;

  /* Assign state pointer */
  instance->pState = pState;

  /* Setup scaling to be used to avoid q31 FIR wrap in intermediate calcs. */
  filter->scale = log2(numTaps);

  /* Save blocksize. */
  filter->block_size = blockSize;

  /* Convert float32 coefficients if supplied. */
  if(pf32Coeffs != NULL) {
    /*
     * Convert the float coefficients into Q31 format.
     */
    arm_float_to_q31(pf32Coeffs, pCoeffs, numTaps);
    transpose_qfir_coefficients(instance);
  }

  /* Clear state buffer and state array size is (blockSize + numTaps - 1) */
  reset_qfir_filter(filter);
}

/**
 * @brief   Resets the filter internal state data.
 *
 * @param[in] filter        pointer to filter data structure.
 */
void reset_qfir_filter(qfir_filter_t *filter) {
  uint16_t pState_size = filter->filter_instance->numTaps
      + filter->block_size - 1;
  memset(filter->filter_instance->pState, 0, pState_size * sizeof(q31_t));
}

/**
 * @brief   Pushes new input sample(s) through the filter and fetches output(s).
 * @note    The new sample(s) are copied and scaled down before being pushed.
 * @note    Scaling prevents fixed point wrap around in filter calculations.
 * @note    Data exiting the filter is scaled back up.
 *
 * @param[in] filter    pointer to a @p qfir_filter_t structure
 * @param[in] input     pointer to input sample(s) buffer
 * @param[in] output    pointer to output sample(s) buffer
 *
 * @api
 */
void apply_qfir_filter(qfir_filter_t *filter, q31_t *input, q31_t *output) {
  /* For temporary copy of input data. */
  q31_t input_copy[filter->block_size];

  /* Scale the input(s) down. */
  arm_scale_q31(input, Q31_MAX, -filter->scale, input_copy,
                filter->block_size);

  /*
   * Apply the scaled input(s) to the filter and compute the output result(s).
   */
  arm_fir_q31(filter->filter_instance, input_copy,
              output, filter->block_size);

  /* Scale the output(s) up. */
  arm_scale_q31(output, Q31_MAX, filter->scale, output, filter->block_size);
}

/**
 * @brief   Reverse order of coefficients for Q31 FIR.
 * @note    CMSIS DSP filters use coefficients in reverse order.
 * @param[in] coeff     pointer to a @p coefficient array.
 * @param[in] numTaps   number of taps in the filter.
 *
 * @api
 */
void transpose_qfir_coefficients(arm_fir_instance_q31 *instance) {

  chDbgAssert(instance != NULL, "invalid filter instance pointer");

  chDbgCheck(instance->numTaps > 2U);

  q31_t *coeff = instance->pCoeffs;
  uint16_t tapIndex = instance->numTaps - 1;

  uint16_t n;
  for(n = 0; n < (tapIndex / 2); n++) {
    /* Swap coefficient orders. */
    q31_t coeff_q31 = coeff[n];
    coeff[n] = coeff[tapIndex - n];
    coeff[tapIndex - n] = coeff_q31;
  }
}

/** @} */
