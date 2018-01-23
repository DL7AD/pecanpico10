/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    dsp.c
 * @brief   Common DSP functions.
 * @notes   requires CMSIS-DSP fast math functions.
 *
 * @addtogroup DSP
 * @{
 */

#include "pktconf.h"

/*
 * Support functions for windowing.
 */

static inline float32_t window_beta(size_t n, float32_t alpha) {
  return cosh(acosh(pow(10,alpha))/(n-1));
}

static inline double window_T(double n, double x) {
  if(fabs(x) <= 1) {
    return cos(n * acos(x));
  } else {
    return cosh(n * acosh(x));
  }
}

/**
 * @brief Calculate a window coefficient for a specified type
 *
 * @param[in] type of window to be applied
 * @param[in] size of the window
 * @param[in] index within the window
 * @return  coefficient for window shape at index j
 */
float32_t dsp_window(td_window_t type, size_t size, size_t j) {
  float32_t center;
  float32_t w;

  center = 0.5 * (size - 1);

  switch (type) {

    case TD_WINDOW_COSINE:
      w = arm_cos_f32((j - center) / size * M_PI);
      //w = cos(j * M_PI / (size - 1));
      break;

    case TD_WINDOW_SINE:
      w = arm_sin_f32((j - center) / size * M_PI);
      //w = sin(j * M_PI / (size - 1));
      break;

    case TD_WINDOW_HAMMING:
      w = 0.53836 - 0.46164 * arm_cos_f32((j * 2 * M_PI) / (size - 1));
      break;

    case TD_WINDOW_EXACT_BLACKMAN:
      w =  0.42659 - 0.49656 * arm_cos_f32((j * 2 * M_PI) / (size - 1))
           + 0.076849 * arm_cos_f32((j * 4 * M_PI) / (size - 1));
      break;

    case TD_WINDOW_NUTTALL:
      w =  0.355768 - 0.487396 * arm_cos_f32((j * 2 * M_PI) / (size - 1))
           + 0.144232 * arm_cos_f32((j * 4 * M_PI) / (size - 1))
           + 0.012604 * arm_cos_f32((j * 6 * M_PI) / (size - 1));
      break;

    case TD_WINDOW_BLACKMAN_NUTTALL:
      w =  0.3635819 - 0.4891775 * arm_cos_f32((j * 2 * M_PI) / (size - 1))
           + 0.1365995 * arm_cos_f32((j * 4 * M_PI) / (size - 1))
           + 0.0106411 * arm_cos_f32((j * 6 * M_PI) / (size - 1));
      break;
    case TD_WINDOW_BLACKMAN_HARRIS:
      w =  0.35875 - 0.48829 * arm_cos_f32((j * 2 * M_PI) / (size - 1))
           + 0.14128 * arm_cos_f32((j * 4 * M_PI) / (size - 1))
           + 0.01168 * arm_cos_f32((j * 6 * M_PI) / (size - 1));
      break;

    case TD_WINDOW_FLATTOP:
      w =  1.0 - 1.93  * arm_cos_f32((j * 2 * M_PI) / (size - 1))
          + 1.29  * arm_cos_f32((j * 4 * M_PI) / (size - 1))
          - 0.388 * arm_cos_f32((j * 6 * M_PI) / (size - 1))
          + 0.028 * arm_cos_f32((j * 8 * M_PI) / (size - 1));
      break;

    case TD_WINDOW_HANNING:
      w = 0.5 * (1 - arm_cos_f32((j * 2 * M_PI) / (size - 1)));
      break;

    case TD_WINDOW_CHEBYSCHEV: {
      /* Dolph-Chebyshev window (a=6., all sidelobes < -120dB) */
      float32_t a = 6.;
      size_t k;
      size_t M = size / 2;
      size_t N = M * 2;
      double b = window_beta(N, a);
      double sum = 0;
      for(k = 0;k < M;k++) {
        sum += (k & 1 ? -1 : 1) * window_T(N, b * cos(M_PI * k/N)) * cos(2 * j * k * M_PI/N);
      }
      sum /= window_T(N, b);
      sum -= .5;
      w = sum;
      break;
    }

    case TD_WINDOW_NONE:
    case TD_WINDOW_TRUNCATED:
    default:
      w = 1.0;
      break;
  }
  return w;
}

/**
 * @brief Calculate coefficients for a correlation filter
 * @post  The coefficient arrays are populated
 *
 * @param[in] pCos      pointer I (cos) to coefficient result array
 * @param[in] pSin      pointer Q (sin) to coefficient result array
 * @param[in] length    number of taps in filter
 * @param[in] window    window type to apply to coefficients
 *
 */
void gen_fir_iqf(float32_t *pCos, float32_t *pSin,
                                  uint16_t length,
                                  float32_t norm_freq,
                                  td_window_t w_type) {

  chDbgCheck(pCos != NULL && pSin != NULL);
  chDbgCheck(length > 0);

  float32_t gain_s = 0, gain_c = 0;
  uint16_t n;
  for (n = 0; n < length; n++) {
    float32_t center = 0.5f * (length - 1);
    /* angle = (n_center_offset / sample_rate) * frequency * 2PI. */
    float32_t angle = (float32_t)(n - center) * norm_freq
        * 2.0f * (float32_t)M_PI;

    /* Apply windowing to correlation coefficients. */

    float32_t w_coeff = dsp_window(w_type, length, n);

    pCos[n] = cosf(angle) * w_coeff;
    pSin[n] = sinf(angle) * w_coeff;

    gain_c += pCos[n] * cosf(angle);
    gain_s += pSin[n] * sinf(angle);
  }

  /* Normalize to unity gain */

  for (n = 0; n < length; n++) {
    pCos[n] = pCos[n] / gain_c;
    pSin[n] = pSin[n] / gain_s;
  }
}


/**
 * @brief Calculate coefficients for a LPF
 * @post  The coefficient array is populated
 *
 * @param[in] fc        cutoff frequency of filter
 * @param[in] coeff     pointer to coefficient result array
 * @param[in] numTaps   number of taps in filter
 * @param[in] window    window type to apply to coefficients
 *
 */
void gen_fir_lpf (float32_t fc, float32_t *coeff,
                  size_t numTaps, td_window_t window) {
  size_t j;
  float32_t gain;

  for (j = 0; j < numTaps; j++) {
    float32_t sinc;
    float32_t shape;
    float32_t center = 0.5 * (numTaps - 1);

    if (j - center == 0) {
      sinc = 2 * fc;
    } else {
      sinc = sin(2 * M_PI * fc * (j - center)) / (M_PI * (j - center));
    }
    shape = dsp_window (window, numTaps, j);
    coeff[j] = sinc * shape;
  }

  /*
   * Normalize for unity gain at DC.
   * Lowpass gain = sum k=0:k=N-1 {h[k]}
   */
  for (j = 0, gain = 0; j < numTaps; j++) {
    gain += coeff[j];
  }
  for (j = 0; j < numTaps; j++) {
    coeff[j] /= gain;
  }
}

/**
 * @brief Calculate coefficients for a BPF
 * @post  The coefficient array is populated
 *
 * @param[in] f1        lower frequency of filter bandpass
 * @param[in] f2        upper frequency of filter bandpass
 * @param[in] coeff     pointer to coefficient result array
 * @param[in] numTaps   number of taps in filter
 * @param[in] window    window type to apply to coefficients
 *
 */
void gen_fir_bpf (float32_t f1, float32_t f2, float32_t *coeff,
                  size_t numTaps, td_window_t window) {
  float32_t omega;
  float32_t gain;
  float32_t center = 0.5 * (numTaps - 1);
  size_t j;
  for (j = 0; j < numTaps; j++) {
    float32_t sinc;
    float32_t shape;

    if (j - center == 0) {
      sinc = 2 * (f2 - f1);
    }
    else {
      sinc = sin(2 * M_PI * f2 * (j - center)) / (M_PI * (j-center))
             - sin(2 * M_PI * f1 * (j - center)) / (M_PI * (j-center));
    }

    shape = dsp_window (window, numTaps, j);
    coeff[j] = sinc * shape;
  }

  /*
   * Normalize for unity gain in middle of passband.
   * Passband gain = sum k=0:k=N-1 {h[k]^e-jwk}
   */
  omega = 2 * M_PI * ((f1 + f2) / 2);
  for (j = 0, gain = 0; j < numTaps; j++) {
    float32_t angle = j * omega;
    /* Using Euler's identity for calculation. */
    gain += coeff[j] * (cos(angle) - sin(angle));
  }
  for (j = 0; j < numTaps; j++) {
    coeff[j] = coeff[j] / gain;
  }
}

/** @} */
