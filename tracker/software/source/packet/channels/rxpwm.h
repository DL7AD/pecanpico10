/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    rxpwm.h
 * @brief   ICU user macros and structures.
 *
 * @addtogroup channels
 * @{
 */

#ifndef PKT_CHANNELS_RXPWM_H_
#define PKT_CHANNELS_RXPWM_H_

#include "pktconf.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define PKT_RSSI_CAPTURE            TRUE
#define PKT_USE_CCA_LEADING_ONLY    FALSE
#define PKT_USE_CCA_DEGLITCH        FALSE
#define PKT_CATCH_ICU_OVERFLOW      FALSE
#define PKT_TRAILING_SYMBOL_TIMEOUT 20

/* Limit of ICU and PWM count for packed format. */

#if USE_12_BIT_PWM == TRUE
#define PWM_MAX_COUNT   0xFFFU
#define MAX_PWM_BITS    12
#else
#define PWM_MAX_COUNT   0xFFFFU
#define MAX_PWM_BITS    16
#endif

/* PWM stream in-band prefix. */
#define PWM_IN_BAND_PREFIX      0

/* PWM stream terminate in-band reason codes. */
#define PWM_TERM_STREAM_CLOSE   0
#define PWM_TERM_QUEUE_FULL     1
#define PWM_TERM_ICU_OVERFLOW   2
#define PWM_TERM_QUEUE_ERR      3
#define PWM_ACK_DECODE_RESET    4
#define PWM_TERM_PWM_STOP       5
#define PWM_TERM_NO_DATA        6
#define PWM_TERM_ICU_ZERO       7
#define PWM_INFO_QUEUE_SWAP     8
#define PWM_ACK_DECODE_ERROR    9
#define PWM_TERM_QUEUE_ERROR    10
#define PWM_TERM_PWM_TIMEOUT    11
#define PWM_INFO_ICU_LIMIT      12

/* If all PWM buffers are consumed assume jamming and wait this timeout. */
#define PWM_JAMMING_TIMEOUT     TIME_S2I(10)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/* Reason code for closing a PWM stream. */
typedef uint8_t pwm_code_t;

typedef enum ICUStates {
  PKT_PWM_INIT = 0,
  PKT_PWM_READY,
  PKT_PWM_WAITING,
  PKT_PWM_ACTIVE,
  PKT_PWM_STOP
} rx_icu_state_t;

/*
 *  Types for ICU and PWM data are sized to 16 bits only.
 *  Thus PWM can use any ICU capable timer.
 */
typedef uint16_t            min_icucnt_t;
typedef uint16_t            min_pwmcnt_t;

typedef uint8_t             packed_pwm_data_t;
#if USE_12_BIT_PWM == TRUE
typedef packed_pwm_data_t   packed_pwmcnt_t;
typedef packed_pwm_data_t   packed_pwmxtn_t;

/* Structure containing packed PWM results. */
typedef struct {
  packed_pwmcnt_t           impulse;
  packed_pwmcnt_t           valley;
  packed_pwmxtn_t           xtn;
} __attribute__((packed)) packed_pwm_counts_t;

#else
typedef min_pwmcnt_t        packed_pwmcnt_t;

/* Structure containing packed PWM results. */
typedef struct {
  packed_pwmcnt_t           impulse;
  packed_pwmcnt_t           valley;
} packed_pwm_counts_t;
#endif

/* Union of packed PWM results and byte array representation. */
typedef union {
  packed_pwm_counts_t       pwm;
  packed_pwm_data_t         bytes[sizeof(packed_pwm_counts_t)];
} byte_packed_pwm_t;

/*
 * Structure holding PWM entries after unpacking.
 * Limited to 16 bit maximum counts.
 */
typedef struct {
  min_pwmcnt_t              impulse;
  min_pwmcnt_t              valley;
} __attribute__((packed)) min_pwm_counts_t;

/*
 * Union of PWM results and byte array representation.
 * Used for unpacking 12 or 16 bit PWM.
 * Accessible as 16 bit PWM representation and overlay byte stream.
 */
typedef union {
  min_pwm_counts_t          pwm;
  min_pwmcnt_t              array[sizeof(min_pwm_counts_t)
                                  / sizeof(min_pwmcnt_t)];
} array_min_pwm_counts_t;

/* Union of packed PWM data buffer and byte array representation. */
typedef union {
  byte_packed_pwm_t         pwm_buffer[PWM_DATA_SLOTS];
  packed_pwm_data_t         pwm_bytes[sizeof(byte_packed_pwm_t)
                                 * PWM_DATA_SLOTS];
} radio_pwm_buffer_t;

#if USE_HEAP_PWM_BUFFER == TRUE
/* Forward declare struct. */
typedef struct PWMobject radio_pwm_object_t;

typedef struct PWMobject {
  radio_pwm_buffer_t        buffer;
  /* In linked mode the reference to the next PWM queue is saved here.
   * The decoder will continue to process linked PWM queues until completion.
   */
  input_queue_t             queue;
} radio_pwm_object_t;
#endif

/*
 * PWM FIFO object. Path between ICU and decoder during an AFSK decode.
 */
typedef struct {
  /* For safety keep clear - where FIFO pool stores its free link. */
  struct pool_header        link;
#if USE_HEAP_PWM_BUFFER == TRUE
  /*
   * There are two PWM object pointers in a PWM stream record.
   * One for the radio (producer) side.
   * And one for the decoder (consumer) side.
   */
  radio_pwm_object_t        *radio_pwm_queue;
  radio_pwm_object_t        *decode_pwm_queue;
#if TRACE_PWM_BUFFER_STATS == TRUE
  uint8_t                   in_use;
  uint8_t                   sync;
  uint8_t                   rlsd;
  uint8_t                   peak;
#endif
#else
  /* Allocate a PWM buffer in the queue object. */
  radio_pwm_buffer_t        packed_buffer;

  /*
   * This is the current radio queue object.
   * In single queue mode PWM is written to a single queue only.
   * The queue has a single large buffer and used for the entire PWM session.
   *
   * In linked buffer mode PWM can chain multiple smaller input buffers.
   * After getting a new PWM buffer object the queue is re-initialized.
   * The queue fill with further PWM then continues.
   * As PWM buffers are consumed by the decoder they are recycled back to the pool.
   * The radio PWM can then re-use those buffers which in theory reduces memory utilisation.
   */
  input_queue_t             radio_pwm_queue;
#endif
  /*
   * The semaphore controls the release of the PWM buffer and FIFO resources.
   * In non-linked mode the buffer is enclosed within the FIFO object.
   * In linked mode the last PWM buffer is protected along with the FIFO.
   * The semaphore prevents any release during trailing PWM buffering.
   * Trailing PWM is not used but the object(s) are still in use by the radio.
   */
  binary_semaphore_t        sem;
  volatile eventflags_t     status;
  radio_signal_t            rssi;
  cnt_t                     seq_num;
  radio_freq_hz_t           freq;
} radio_pwm_fifo_t;

/*===========================================================================*/
/* Module macro definitions.                                                 */
/*===========================================================================*/


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  ICUDriver *pktAttachRadio(const radio_unit_t radio_id);
  void      pktEnableRadioStream(const radio_unit_t radio);
  void      pktDisableRadioStream(const radio_unit_t radio);
  void      pktDetachRadio(const radio_unit_t radio_id);
  void      pktRadioICUWidth(ICUDriver *myICU);
  void      pktRadioICUPeriod(ICUDriver *myICU);
  void      pktRadioICUOverflow(ICUDriver *myICU);
  void      pktRadioCCAInput(ICUDriver *myICU);
  void      pktStopAllICUtimersI(ICUDriver *myICU);
  void      pktSleepICUI(ICUDriver *myICU);
  void      pktClosePWMStreamI(ICUDriver *myICU,
                               statusflags_t sta,
                               eventflags_t evt,
                               pwm_code_t reason);
  void      pktSwitchPWMStreamI(ICUDriver *myICU,
                          statusflags_t sta,
                          eventflags_t evt);
  void      pktPWMInactivityTimeout(ICUDriver *myICU);
  msg_t     pktWritePWMQueueI(input_queue_t *queue, byte_packed_pwm_t pack);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Convert ICU data to PWM data and pack into minimized buffer.
 * @note    This function deals with ICU data packed into 12 bits or 16 bits.
 *
 * @param[in] icup      pointer to ICU driver.
 * @param[in] dest      pointer to the object for PWM data.
 *
 * @api
 */
static inline void pktConvertICUtoPWM(ICUDriver *icup,
                                      byte_packed_pwm_t *dest) {
  icucnt_t impulse = icuGetWidthX(icup);
  icucnt_t valley = icuGetPeriodX(icup) - impulse;

  if ((impulse | valley) > PWM_MAX_COUNT) {
    /*
     *  ICU count has exceeded capacity of PWM encoding.
     *  Substitute in-band message.
     *  Decoder will determine what to do based on state.
     */
    impulse = PWM_IN_BAND_PREFIX;
    valley = PWM_INFO_ICU_LIMIT;
/*    pktWritePWMinBandMessageI(input_queue_t *queue,
                                           pwm_code_t reason) {*/
  }

#if USE_12_BIT_PWM == TRUE
  dest->pwm.impulse = (packed_pwmcnt_t)impulse & 0xFFU;
  dest->pwm.valley = (packed_pwmcnt_t)valley & 0xFFU;
  /*
   * Pack extension bits 8-11 of impulse and valley into a byte.
   * Impulse goes into low nibble and valley into high nibble.
   */
  valley >>= 4;
  impulse >>= 8;
  dest->pwm.xtn = ((packed_pwmxtn_t)(impulse) & 0x000FU);
  dest->pwm.xtn |= ((packed_pwmxtn_t)(valley) & 0x00F0U);
#else
  dest->pwm.impulse = (packed_pwmcnt_t)impulse & 0xFFFFU;
  dest->pwm.valley = (packed_pwmcnt_t)valley & 0xFFFFU;
#endif
}

/**
 * @brief   Unpack PWM data into PWM structure.
 * @note    This function deals with ICU data up to 12 bits.
 *
 * @param[in] src       Buffer containing packed impulse and valley data.
 * @param[in] dest      pointer to the destination object for PWM data.
 *
 * @api
 */
static inline void pktUnpackPWMData(byte_packed_pwm_t src,
                                    array_min_pwm_counts_t *dest) {
#if USE_12_BIT_PWM == TRUE
  min_icucnt_t duration = src.pwm.impulse;
  duration |= ((min_icucnt_t)(src.pwm.xtn & 0x0FU) << 8);
  dest->pwm.impulse = duration;
  duration = src.pwm.valley;
  duration |= ((min_icucnt_t)(src.pwm.xtn & 0xF0U) << 4);
  dest->pwm.valley = duration;
#else
  dest->pwm.impulse = src.pwm.impulse;
  dest->pwm.valley = src.pwm.valley;
#endif
}

/**
 * @brief   Write in-band message to PWM queue.
 * @note    This function deals with ICU data packed into 12 bits or 16 bits.
 *
 * @param[in] queue     pointer to an input queue object.
 * @param[in] reason    in-band reason code for closing the queue
 *
 * @return              The operation status.
 * @retval MSG_OK       The PWM entry has been queued.
 * @retval MSG_RESET    One slot remains which is reserved for an in-band signal.
 * @retval MSG_TIMEOUT  The queue is full.
 * @retval MSG_ERROR    The queue chunk size has become incorrect.
 *
 * @iclass
 */
static inline msg_t pktWritePWMinBandMessageI(input_queue_t *queue,
                                       pwm_code_t reason) {
#if USE_12_BIT_PWM == TRUE
      byte_packed_pwm_t pack = {{PWM_IN_BAND_PREFIX, reason, 0}};
#else
      byte_packed_pwm_t pack = {{PWM_IN_BAND_PREFIX, reason}};
#endif
      return pktWritePWMQueueI(queue, pack);
}

#endif /* PKT_CHANNELS_RXPWM_H_ */

/** @} */
