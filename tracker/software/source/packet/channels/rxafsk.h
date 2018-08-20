/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file        rxafsk.h
 * @brief       AFSK decoding definitions.
 *
 * @addtogroup decoders
 * @{
 */

#ifndef CHANNELS_RXAFSK_H_
#define CHANNELS_RXAFSK_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/
/*
 * AFSK decoding definitions.
 */
#define AFSK_BAUD_RATE              1200U

#define AFSK_NUM_TONES              2U

#define AFSK_MARK_INDEX             0U
#define AFSK_MARK_FREQUENCY         1200U
#define AFSK_SPACE_INDEX            1U
#define AFSK_SPACE_FREQUENCY        2200U

/* Thread working area size. */
#define PKT_AFSK_DECODER_WA_SIZE    (1024 * 5)

/* AFSK decoder type selection. */
#define AFSK_NULL_DECODE            0
#define AFSK_DSP_QCORR_DECODE       1
#define AFSK_DSP_FCORR_DECODE       2 /* Currently unimplemented. */

#define AFSK_DECODE_TYPE            AFSK_DSP_QCORR_DECODE

/* Debug output type selection. */
#define AFSK_NO_DEBUG               0
#define AFSK_QCORR_FIR_DEBUG        1
#define AFSK_QCORR_DEC_MAG_DEBUG    2
#define AFSK_QCORR_DATA_DEBUG       3
#define AFSK_QCORR_DEC_MS_DEBUG     4
#define AFSK_QCORR_DEC_CS_DEBUG     5
#define AFSK_QCORR_DEC_MFIL_DEBUG   6
#define AFSK_PWM_DATA_CAPTURE_DEBUG 7
#define AFSK_AX25_RAW_PACKET_DUMP   8
#define AFSK_PACKET_RESET_STATUS    9

#define AFSK_DEBUG_TYPE             AFSK_NO_DEBUG

/* Error output type selection. */
#define AFSK_NO_ERROR               0
#define AFSK_QSQRT_ERROR            1

#define AFSK_ERROR_TYPE             AFSK_NO_ERROR

//#define PRE_FILTER_GEN_COEFF        TRUE
#define PRE_FILTER_LOW              925
#define PRE_FILTER_HIGH             2475

//#define MAG_FILTER_GEN_COEFF        TRUE
#define MAG_FILTER_HIGH             1400

#define PRE_FILTER_NUM_TAPS         55U
#define PRE_FILTER_BLOCK_SIZE       1U
#if PRE_FILTER_BLOCK_SIZE != 1
#error "Filter block size must be 1"
#endif

#define USE_QCORR_MAG_LPF           TRUE

#define MAG_FILTER_NUM_TAPS         15U
#define MAG_FILTER_BLOCK_SIZE       1U
#if MAG_FILTER_BLOCK_SIZE != 1
#error "Filter block size must be 1"
#endif



#if AFSK_DECODE_TYPE == AFSK_DSP_QCORR_DECODE
/* BPF followed by fixed point IQ correlation decoder.
 * Changing decimation changes the filter sample rate.
 * Coefficients created dynamically are calculated at run-time.
 * Coefficients for fixed arrays can be generated externally in Matlab/Octave.
 *
 * Pre-filter (BPF) coefficients.
 * Fs=sample_rate, f1 = low_corner, f2 = high_corner, number of taps = N
 * Matlab/Octave parameters:
 * hc = fir1(N-1, [low_corner, high_corner]/(Fs/2), 'pass');
 *
 * Magnitude (LPF) coefficients.
 * Fs=sample_rate, f1 = high_corner, number of taps = N
 * Matlab/Octave parameters:
 * hc = fir1(N-1, high_corner/(Fs/2), 'low');
 *
 */

#define SYMBOL_DECIMATION           (12U)
/* Sample rate in Hz. */
#define FILTER_SAMPLE_RATE          (SYMBOL_DECIMATION * AFSK_BAUD_RATE)
#define DECODE_FILTER_LENGTH        (2U * SYMBOL_DECIMATION)
#endif

/* Named services. */
#define PKT_PWM_QUEUE_PREFIX        "pwmx_"
#define PKT_PWM_MBOX_PREFIX         "pwmd_"
#define PKT_AFSK_THREAD_NAME_PREFIX "rxafsk_"

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
 * @brief   This enumeration describes a tone field within the decoder structure.
 */
typedef enum {
  TONE_NONE,
  TONE_MARK,
  TONE_SPACE
} tone_t;

/**
 * @brief   AFSK demod state machine states.
 */
typedef enum {
  DECODER_WAIT = 0,
  DECODER_IDLE,
  DECODER_POLL,
  DECODER_ACTIVE,
  DECODER_DISPATCH,
  DECODER_RESET,
  DECODER_TERMINATED
} afskdemodstate_t;

typedef float32_t   pwm_accum_t;
typedef int16_t     dsp_phase_t;

#include "rxpwm.h"
#include "pktservice.h"
/**
 * @brief   Structure representing an AFSK demod driver.
 */
typedef struct AFSK_data {

  char                      decoder_name[PKT_THREAD_NAME_MAX];

  /**
   * @brief pointer to the packet handler.
   */
  packet_svc_t              *packet_handler;

  /**
   * @brief Event source object.
   */
  event_source_t            event;

  /**
   * @brief Decoder thread (for events posted to decoder).
   */
  thread_t                  *decoder_thd;

  /**
   * @brief Frame byte being built.
   */
  ax25char_t                current_byte;

  /**
   * @brief HDLC bit count.
   */
  uint8_t                   bit_index;

  /**
   * @brief AFSK decoder states. TODO: non volatile?
   */
  afskdemodstate_t          decoder_state;

  /**
   * @brief Demod decimation timeline accumulator.
   */
  pwm_accum_t               decimation_accumulator;

  /**
   * @brief Decimation amount per slice.
   */
  pwm_accum_t               decimation_size;

  /**
   * @brief ICU driver being used.
   */
  ICUDriver                 *icudriver;

  /**
   * @brief ICU driver state.
   */
  rx_icu_state_t            icustate;

  /**
   * @brief PWM queue heap memory.
   */
  void                      *pwm_queue_heap;

  /**
   * @brief PWM buffer object pool manager.
   */
  memory_pool_t             pwm_buffer_pool;

  /**
   * @brief PWM FIFO manager name.
   */
  char                      pwm_fifo_name[CH_CFG_FACTORY_MAX_NAMES_LENGTH];

  /**
   * @brief ICU guarded FIFO.
   */
  dyn_objects_fifo_t        *the_pwm_fifo;

  /**
   * @brief PWM channel FIFO pool.
   */
  objects_fifo_t            *pwm_fifo_pool;

  /**
   * @brief Current radio PWM fifo object.
   */
  radio_pwm_fifo_t          *active_radio_object;

  /**
   * @brief Current demod PWM fifo object.
   */
  radio_pwm_fifo_t          *active_demod_object;

  /**
   * @brief current symbol frequency.
   */
  tone_t                    tone_freq;

  /**
   * @brief Prior symbol frequency.
   */
  tone_t                    prior_freq;

  /**
   * @brief     Pointer to a decoder data structure.
   * @details   This may be Q31 or F32 type.
   */
  void                      *tone_decoder;

  /**
   * @brief Symbol incoming bit stream.
   */
  /* TODO: Should typdef this? */
  uint32_t                  hdlc_bits;

  /**
   * @brief Opening HDLC flag sequence found.
   */
  frame_state_t             frame_state;
} AFSKDemodDriver;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

static inline void pktResyncAFSKDecoder(AFSKDemodDriver *myDriver) {
  packet_svc_t *myHandler = myDriver->packet_handler;
  myDriver->frame_state = FRAME_OPEN;
  myHandler->active_packet_object->packet_size = 0;
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern float32_t pre_filter_coeff_f32[];
extern float32_t mag_filter_coeff_f32[];
extern struct AFSK_data AFSKD1;
extern struct qCorrFilter QCORR1;
//extern struct QFIRFilter QMAGM1;
//extern struct QFIRFilter QMAGS1;

#ifdef __cplusplus
extern "C" {
#endif
  AFSKDemodDriver *pktCreateAFSKDecoder(packet_svc_t *pktDriver);
  void pktReleaseAFSKDecoder(AFSKDemodDriver *myDriver);
  void pktAFSKDecoder(void *arg);
#ifdef __cplusplus
}
#endif

/*
 * Test of flow graph and protocol diagrams.
 * TODO: Remove or implement.
 */

/**
\dot
digraph G {
main -> parse -> execute;
main -> init;
main -> cleanup;
execute -> make_string;
execute -> printf
init -> make_string;
main -> printf;
execute -> compare;
}
\enddot
*/

/**
\msc
arcgradient = 8;
a [label="Client"],b [label="Server"];
a-xb [label="get accel"];
a=>b [label="get accel"];
a<=b [label="ack"];
a<=b [label="accel data"];
\endmsc
*/

#endif /* CHANNELS_RXAFSK_H_ */

/** @} */
