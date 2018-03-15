/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    pktradio.h
 * @brief   Generic radio definitions.
 *
 * @addtogroup managers
 * @{
 */

#ifndef PKT_MANAGERS_PKTRADIO_H_
#define PKT_MANAGERS_PKTRADIO_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/* Thread working area size. */
#define PKT_RADIO_MANAGER_WA_SIZE       1024

#define PKT_RADIO_TASK_QUEUE_PREFIX     "radx_"

#define BAND_START_2M                   144000000
#define AU_APRS_2M_FREQUENCY            145175000

#define PKT_RADIO_CHANNEL_STEPPING_NONE  0

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Radio manager control commands.
 * @details Radio task requests execute these commands.
 */
typedef enum radioCommand {
  PKT_RADIO_RX_OPEN,
  PKT_RADIO_RX_START,
  PKT_RADIO_RX_STOP,
  PKT_RADIO_TX_SEND,
  PKT_RADIO_RX_CLOSE
} radio_command_t;

/**
 * @brief   Definition of radio unit ID.
 * @details Defines the radio unit used in configuring and enabling a radio.
 *
 */
typedef enum radioUnit {
  PKT_RADIO_1
} radio_unit_t;

/* Radio frequency in Hz. */
typedef uint32_t radio_freq_t;

/* Channel step in Hz. */
typedef uint16_t channel_hz_t;

/* Channel selector for radio frequency. */
typedef uint8_t radio_ch_t;

/* Radio squelch setting. */
typedef uint8_t radio_squelch_t;

/**
 * Forward declare structure types.
 */
typedef struct radioTask radio_task_object_t;
typedef struct packetHandlerData packet_svc_t;

/**
 * @brief           Radio task notification callback type.
 *
 * @param[in] rcob  pointer to a @p radio task object
 */
typedef void (*radio_task_cb_t)(packet_svc_t *handler);

/**
 * @brief   Radio request object.
 */
/*typedef struct radioRequest {
  radio_unit_t          radio;
  radio_command_t       command;
  encoding_type_t       type;
  radio_freq_t          base_frequency;
  radio_ch_t            channel;
  radio_task_cb_t       callback;
} radio_request_t;*/

#include "ax25_pad.h"
/**
 * @brief       Radio task object.
 * @details     object submitted by FIFO to manager for radio task requests.
 */
struct radioTask {
  /* For safety keep clear - where pool stores its free link. */
  struct pool_header        link;
  radio_unit_t              radio_id;
  radio_command_t           command;
  encoding_type_t           type;
  radio_freq_t              base_frequency;
  channel_hz_t              step_hz;
  radio_ch_t                channel;
  radio_squelch_t           squelch;
  radio_task_cb_t           callback;
  packet_svc_t              *handler;
  packet_t                  packet_out;
  uint8_t                   tx_power;
  uint32_t                  tx_speed;
};

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  thread_t *pktRadioManagerCreate(packet_svc_t *handler);
  void pktRadioManagerRelease(packet_svc_t *handler);
  void pktRadioManager(void *arg);
  msg_t pktGetRadioTaskObject(packet_svc_t *handler,
                              sysinterval_t timeout,
                              radio_task_object_t **rt);
  void pktSubmitRadioTask(packet_svc_t *handler,
                          radio_task_object_t *object,
                          radio_task_cb_t cb);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Alias for convenience of pktStopDecoder.
 *
 * @param[in] ip        pointer to a @p packet handler object
 *
 * @api
 */
#define pktPauseDecoder(handler) pktStopDecoder(handler)

/**
 * @brief   Alias for convenience of pktStartDecoder.
 *
 * @param[in] ip        pointer to a @p packet handler object
 *
 * @api
 */
#define pktResumeDecoder(handler) pktStartDecoder(handler)

#endif /* PKT_MANAGERS_PKTRADIO_H_ */

/** @} */
