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

#ifndef IO_CHANNELS_RXPWM_H_
#define IO_CHANNELS_RXPWM_H_

#include "pktconf.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/


/* Definitions for ICU FIFO implemented using chfactory. */
#define NUMBER_PWM_FIFOS 3
#define PWM_BUFFER_SLOTS 4000

/**
 *  ICU related definitions.
 */

/* ICU counter frequency (2.88MHz). */
/*
 * TODO: This should be calculated using SYSTEM CLOCK.
 * ICU has to run at an integer divide from SYSTEM CLOCK.
 */
//#define ICU_COUNT_FREQUENCY     3000000U
#define ICU_COUNT_FREQUENCY     2880000U

/* Limit of ICU and PWM count for packed format. */
#define ICU_MAX_COUNT   0xFFFFFF
#define MAX_PWM_BITS    12

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum ICUStates {
  PWM_ICU_IDLE,
  PWM_ICU_ACTIVE,
  PWM_ICU_STOP
} rx_icu_state_t;

/* Types for ICU and PWM data. */
typedef uint16_t            min_icucnt_t;
typedef uint16_t            min_pwmcnt_t;

typedef uint8_t             packed_pwm_data_t;
typedef packed_pwm_data_t   packed_pwmcnt_t;
typedef packed_pwm_data_t   packed_pwmxtn_t;

/* Structure containing packed PWM results. */
typedef struct {
  packed_pwmcnt_t           impulse;
  packed_pwmcnt_t           valley;
  packed_pwmxtn_t           xtn;
} packed_pwm_counts_t;

/* Union of packed PWM results and byte array representation. */
typedef union {
  packed_pwm_counts_t       pwm;
  packed_pwm_data_t         bytes[sizeof(packed_pwm_counts_t)];
} byte_packed_pwm_t;

/* Structure holding PWM entries created from ICU results. */
typedef struct {
  min_pwmcnt_t              impulse;
  min_pwmcnt_t              valley;
} min_pwm_counts_t;

/* Union of PWM results and byte array representation. */
typedef union {
  min_pwm_counts_t          pwm;
  min_pwmcnt_t              array[sizeof(min_pwm_counts_t) / sizeof(min_pwmcnt_t)];
} array_min_pwm_counts_t;

/* Union of packed PWM data buffer and byte array representation. */
typedef union {
  byte_packed_pwm_t         pwm_buffer[PWM_BUFFER_SLOTS];
  packed_pwm_data_t         pwm_bytes[sizeof(byte_packed_pwm_t) * PWM_BUFFER_SLOTS];
} radio_pwm_buffer_t;

/* PWM FIFO object with embedded queue shared between ICU and decoder. */
typedef struct {
  struct pool_header        link; /* For safety keep clear - where pool stores its free link. */
  radio_pwm_buffer_t        packed_buffer;
  input_queue_t             radio_pwm_queue;
  semaphore_t               sem;
  volatile eventflags_t     status;
} radio_cca_fifo_t;

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Convert ICU data to PWM data and pack into minimized buffer.
 * @note    This function deals with ICU data up to 12 bits.
 *
 * @param[in] icup      pointer to ICU driver.
 * @param[in] dest      pointer to the object for PWM data.
 *
 * @api
 */
static inline void pktConvertICUtoPWM(ICUDriver *icup, byte_packed_pwm_t *dest) {
  icucnt_t impulse = icuGetWidthX(icup);
  icucnt_t valley = icuGetPeriodX(icup) - impulse;
  dest->pwm.impulse = impulse & 0xFFU;
  dest->pwm.valley = valley & 0xFFU;
  /*
   * Pack extension bits 8-11 of impulse and valley into a byte.
   * Impulse goes into low nibble and valley into high nibble.
   */
  valley >>= 4;
  impulse >>= 8;
  dest->pwm.xtn = ((icucnt_t)(impulse) & 0x0FU);
  dest->pwm.xtn += ((icucnt_t)(valley) & 0xF0U);
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
static inline void pktUnpackPWMData(byte_packed_pwm_t src, array_min_pwm_counts_t *dest) {
  min_icucnt_t duration = src.pwm.impulse;
  duration += ((min_icucnt_t)(src.pwm.xtn & 0x0FU) << 8);
  dest->pwm.impulse = duration;
  duration = src.pwm.valley;
  duration += ((min_icucnt_t)(src.pwm.xtn & 0xF0U) << 4);
  dest->pwm.valley = duration;
}

/**
 * @brief   Write PWM data into input queue.
 * @note    This function deals with PWM data packed in sequential bytes.
 *
 * @param[in] queue     pointer to an input queue object.
 * @param[in] pack      PWM packed data object.
 *
 * @return              The operation status.
 * @retval MSG_OK       The PWM data has been queued.
 * @retval MSG_TIMEOUT  The queue is already full.
 * @retval MSG_RESET    Queue space would be exhausted so an OVF
 *                      flag is written in place of PWM data
 *                      unless the data is an EOT flag.
 * @api
 */
static inline msg_t pktWritePWMQueue(input_queue_t *queue,
                                     byte_packed_pwm_t pack) {
  size_t qsz = sizeof(pack.bytes);
  /* Check for required space. */
  if(iqGetEmptyI(queue) < qsz) {
    return MSG_TIMEOUT;
  }
  msg_t ret_val = MSG_OK;
  if(iqGetEmptyI(queue) == qsz) {
    /* TODO: Define in band data flags 0 & 1. */
    if((pack.pwm.impulse + pack.pwm.valley + pack.pwm.xtn) != 0) {
      byte_packed_pwm_t eob = {{0, 1, 0}}; /* OVF flag. */
      pack = eob;
      ret_val = MSG_RESET;
    }
  }
  uint8_t b;
  for(b = 0; b < sizeof(pack.bytes); b++) {
    msg_t result = iqPutI(queue, pack.bytes[b]);
    if(result != MSG_OK)
      return result;
  }
  return ret_val;
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  ICUDriver *pktAttachICU(radio_unit_t radio_id);
  void pktDetachICU(ICUDriver *myICU);
  void pktICUStart(ICUDriver *myICU);
  void pktRadioICUPeriod(ICUDriver *myICU);
  void PktRadioICUOverflow(ICUDriver *myICU);
  void pktRadioCCAInput(ICUDriver *myICU);
  void pktStopAllICUTimersS(ICUDriver *myICU);
  void pktSleepICUI(ICUDriver *myICU);
  msg_t pktQueuePWMDataI(ICUDriver *myICU);
  void pktClosePWMChannelI(ICUDriver *myICU, eventflags_t evt);
#ifdef __cplusplus
}
#endif

#endif /* IO_CHANNELS_RXPWM_H_ */

/** @} */
