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
#define PKT_RADIO_MANAGER_WA_SIZE       (1 * 1024)

#define PKT_RADIO_TASK_QUEUE_PREFIX     "radm_"

/* The number of radio task objects the FIFO has. */
#define RADIO_TASK_QUEUE_MAX            10

/* Use the idle thread sweeper to recover terminated threads. */
//#define PKT_RADIO_MANAGER_TASK_KILL     TRUE

/* Set TRUE to use mutex instead of bsem. */
#define PKT_USE_RADIO_MUTEX             FALSE

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum indicator {
  PKT_INDICATOR_NONE = 0,
  PKT_INDICATOR_DECODE,
  PKT_INDICATOR_SQUELCH,
  PKT_INDICATOR_FIFO,
  PKT_INDICATOR_NO_BUFF,
  PKT_INDICATOR_OVERFLOW,
  PKT_INDICATOR_ERROR,
  PKT_INDICATOR_PWM_ERROR
} indicator_t;

typedef enum indicatorType {
  PKT_IND_GPIO_LINE,
  PKT_IND_EXT_I2C,
  PKT_IND_EXT_SPI
} indicator_type_t;

typedef uint32_t indicator_msg_t;
typedef uint8_t indicator_pos_t;

typedef struct indicatorIO {
  indicator_t       ind;
  indicator_type_t  type;
  indicator_pos_t   pos;    /*<< Position of indicator in output device. */
  union addr {
    ioline_t    line;       /*<< GPIO for direct output or SPI select. */
    i2caddr_t   addr;
  } address;
  union driver {
    I2CDriver   i2c;
    SPIDriver   spi;
    iomode_t    mode;
  } driver;
} indicator_io_t;

/**
 * @brief   Radio manager control commands.
 * @details Radio task requests execute these commands.
 */
typedef enum radioCommand {
  PKT_RADIO_RX_OPEN,
  PKT_RADIO_RX_START,
  PKT_RADIO_RX_STOP,
  PKT_RADIO_TX_SEND,
  PKT_RADIO_RX_CLOSE,
  PKT_RADIO_TX_DONE,
  PKT_RADIO_MGR_CLOSE,
  PKT_RADIO_RX_RSSI
} radio_command_t;

/**
 * Forward declare structure types.
 */
typedef struct radioTask radio_task_object_t;
typedef struct packetHandlerData packet_svc_t;
typedef struct radioConfig radio_config_t;

/**
 * @brief   Radio task notification callback type.
 *
 * @param[in] task_object  pointer to a @p radio task object
 */
typedef bool (*radio_task_cb_t)(radio_task_object_t *task_object);

#include "ax25_pad.h"
/**
 *
 */
typedef struct radioSettings {
  radio_mod_t               type;
  radio_freq_t              base_frequency;
  channel_hz_t              step_hz;
  radio_ch_t                channel;
  radio_squelch_t           squelch;
} radio_settings_t;

/**
 *
 */
typedef struct modParams {
  radio_mod_t               type;
  link_speed_t              tx_speed;
  deviation_hz_t            tx_dev;
} mod_params_t;

/**
 * @brief       Radio task object.
 * @details     queue object submitted via FIFO or radio task requests.
 */
struct radioTask {
  /* For safety keep clear - where pool stores its free link. */
  struct pool_header        link;
  radio_command_t           command;
  radio_mod_t               type;
  radio_freq_t              base_frequency;
  channel_hz_t              step_hz;
  radio_ch_t                channel;
  radio_squelch_t           squelch;
  radio_task_cb_t           callback;
  msg_t                     result;
  thread_t                  *thread;
  packet_svc_t              *handler;
  packet_t                  packet_out;
  radio_pwr_t               tx_power;
  uint8_t                   tx_seq_num;
};

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Alias of pktStopDecoder for convenience.
 *
 * @param[in] radio radio unit ID
 *
 * @api
 */
#define pktPauseDecoding(radio) pktStopDecoder(radio)

/**
 * @brief   Alias for convenience of pktStartDecoder.
 *
 * @param[in] radio radio unit ID
 *
 * @api
 */
#define pktResumeDecoding(radio) pktStartDecoder(radio)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  thread_t  		*pktRadioManagerCreate(const radio_unit_t radio);
  msg_t      		pktRadioManagerRelease(const radio_unit_t radio);
  msg_t             pktStartRadioReceive(const radio_unit_t radio,
                            radio_task_object_t *rto, sysinterval_t timeout);
  msg_t             pktStopRadioReceive(const radio_unit_t radio,
                           radio_task_object_t *rto);
  void      		pktRadioManager(void *arg);
  msg_t     		pktGetRadioTaskObject(const radio_unit_t radio,
                              const sysinterval_t timeout,
                              radio_task_object_t **rt);
  void      		pktSubmitRadioTask(const radio_unit_t radio,
                          radio_task_object_t *object,
                          radio_task_cb_t cb);
  void              pktSubmitPriorityRadioTask(const radio_unit_t radio,
                           radio_task_object_t *object,
                           const radio_task_cb_t cb);
  void      		pktScheduleThreadRelease(const radio_unit_t radio,
                                thread_t *thread);
  msg_t     		pktLockRadio(const radio_unit_t radio,
            		                const radio_mode_t mode,
            		                const sysinterval_t timeout);
  void      		pktUnlockRadio(const radio_unit_t radio,
                                   const radio_mode_t mode);
  const radio_config_t *pktGetRadioList(void);
  uint8_t           pktGetNumRadios(void);
  radio_band_t 		*pktCheckAllowedFrequency(const radio_unit_t radio,
                                        const radio_freq_t freq);
  radio_freq_t 		pktComputeOperatingFrequency(const radio_unit_t radio,
                                            radio_freq_t base_freq,
                                            channel_hz_t step,
                                            radio_ch_t chan,
                                            const radio_mode_t mode);
  radio_unit_t      pktSelectRadioForFrequency(const radio_freq_t freq,
                                          const channel_hz_t step,
                                          const radio_ch_t chan,
                                          const radio_mode_t mode);
  msg_t             pktSetReceiveInactive(const radio_unit_t radio,
                                          sysinterval_t timeout);
  void      		pktRadioStartDecoder(const radio_unit_t radio);
  void      		pktRadioStopDecoder(const radio_unit_t radio);
  void      		pktRadioSendComplete(radio_task_object_t *rto,
                                thread_t *thread);
  ICUDriver         *pktLLDradioAttachStream(const radio_unit_t radio);
  void              pktLLDradioDetachStream(const radio_unit_t radio);
  const ICUConfig   *pktLLDradioStreamEnable(const radio_unit_t radio,
                                         palcallback_t cb);
  void              pktLLDradioStreamDisableI(const radio_unit_t radio);
  bool              pktRadioGetInProgress(const radio_unit_t radio);
  void      		pktStartDecoder(const radio_unit_t radio);
  void      		pktStopDecoder(const radio_unit_t radio);
  int       	 	pktDisplayFrequencyCode(radio_freq_t code, char *buf,
            	 	                        size_t size);
  const radio_config_t	*pktGetRadioData(radio_unit_t radio);
  bool              pktLookupModParameters(const radio_unit_t radio,
                                           mod_params_t *mp);
  uint8_t           pktLLDradioReadCCAlineI(const radio_unit_t radio);
  void              pktLLDradioConfigIndicator(const radio_unit_t radio,
                                               const indicator_t ind);
  void              pktLLDradioDeconfigIndicator(const radio_unit_t radio,
                                                 const indicator_t ind);
  void              pktLLDradioUpdateIndicator(const radio_unit_t radio,
                                               const indicator_t ind,
                                               const indicator_msg_t val);
  bool              pktLLDradioOscUpdate(const radio_unit_t radio,
                                         xtal_osc_t freq);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* PKT_MANAGERS_PKTRADIO_H_ */

/** @} */
