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
#define PKT_RADIO_MANAGER_WA_SIZE       (2 * 1024)

#define PKT_RADIO_TASK_QUEUE_PREFIX     "radm_"

/* The number of radio task objects the FIFO has. */
#define RADIO_TASK_QUEUE_MAX            10

/* New RTM implementation supports outer and inner level callbacks in the RTO. */
#define PKT_RTO_HAS_INNER_CB             TRUE

/* Temporary switch while testing TX thread self terminate (versus RTM termination). */
#define PKT_TRANSMIT_TASK_SELF_TERMINATE TRUE

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
  PKT_RADIO_MGR_CLOSE,
  PKT_RADIO_TCXO_UPDATE,
  PKT_RADIO_RX_RSSI,
  /* Internal or function API accessed RM tasks only. */
  PKT_RADIO_TX_DONE,
  PKT_RADIO_RX_DECODE
} radio_command_t;

#define PKT_RADIO_TASK_MAX  PKT_RADIO_RX_RSSI
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
#if PKT_RTO_HAS_INNER_CB == TRUE
typedef void (*radio_task_cb_t)(radio_task_object_t *task_object);
typedef bool (*radio_mgr_cb_t)(radio_task_object_t *task_object);
#else
typedef bool (*radio_task_cb_t)(radio_task_object_t *task_object);
#endif


#include "ax25_pad.h"

/**
 *
 */
typedef struct modParams {
  radio_mod_t               type;
  link_speed_t              tx_speed;
  radio_dev_hz_t            tx_dev;
} mod_params_t;

typedef struct radioParams {
  radio_mod_t               type;
  radio_freq_hz_t           base_frequency;
  radio_chan_hz_t           step_hz;
  radio_ch_t                channel;
  radio_signal_t            rssi;
  radio_pwr_t               tx_power;
  sysinterval_t             tto;
  packet_t                  packet_out;
  cnt_t                     seq_num;
} radio_params_t;

/**
 * @brief       Radio task object.
 * @details     queue object submitted via FIFO or radio task requests.
 *
 */
struct radioTask {
  /* For safety keep clear - where pool stores its free link. */
  struct pool_header        link;
  radio_command_t           command;
  packet_svc_t              *handler;
#if PKT_RTO_HAS_INNER_CB != TRUE
  radio_mod_t               type;
  radio_freq_hz_t           base_frequency;
  radio_chan_hz_t           step_hz;
  radio_ch_t                channel;
  radio_squelch_t           squelch;
  packet_t                  packet_out;
  radio_pwr_t               tx_power;
  uint8_t                   rt_seq;
#else
  radio_params_t            radio_dat;
  radio_mgr_cb_t            mgr_cb;
#endif
  radio_task_cb_t           user_cb;
  msg_t                     result;
  radio_signal_t            rssi;
#if PKT_TRANSMIT_TASK_SELF_TERMINATE != TRUE
  thread_t                  *thread;
#endif
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
  void      		pktRadioManager(void *arg);
  msg_t     		pktGetRadioTaskObject(const radio_unit_t radio,
                              const sysinterval_t timeout,
#if PKT_RTO_HAS_INNER_CB == TRUE
                              const radio_params_t *cfg,
#endif
                              radio_task_object_t **rt);
  msg_t             pktGetRadioTaskObjectI(const radio_unit_t radio,
#if PKT_RTO_HAS_INNER_CB == TRUE
                            const radio_params_t *cfg,
#endif
                              radio_task_object_t **rt);
  void      		pktSubmitRadioTask(const radio_unit_t radio,
                          radio_task_object_t *object,
#if PKT_RTO_HAS_INNER_CB == TRUE
                           const radio_mgr_cb_t cb);
#else
                           const radio_task_cb_t cb);
#endif
  void              pktSubmitPriorityRadioTask(const radio_unit_t radio,
                           radio_task_object_t *object,
#if PKT_RTO_HAS_INNER_CB == TRUE
                           const radio_mgr_cb_t cb);
#else
                           const radio_task_cb_t cb);
#endif
  void              pktSubmitPriorityRadioTaskI(const radio_unit_t radio,
                           radio_task_object_t *object,
#if PKT_RTO_HAS_INNER_CB == TRUE
                           const radio_mgr_cb_t cb);
#else
                           const radio_task_cb_t cb);
#endif

  void      		pktScheduleThreadRelease(const radio_unit_t radio,
                                thread_t *thread);
  msg_t     		pktLockRadio(const radio_unit_t radio,
            		                const radio_mode_t mode,
            		                const sysinterval_t timeout);
  void      		pktUnlockRadio(const radio_unit_t radio,
                                   const radio_mode_t mode);
  void              pktResetRadioLock(const radio_unit_t radio,
                                      const bool taken);
  const radio_config_t *pktGetRadioList(void);
  uint8_t           pktGetNumRadios(void);
  radio_band_t 		*pktCheckAllowedFrequency(const radio_unit_t radio,
                                        const radio_freq_hz_t freq);
  radio_freq_hz_t   pktComputeOperatingFrequency(const radio_unit_t radio,
                                            radio_freq_hz_t base_freq,
                                            radio_chan_hz_t step,
                                            radio_ch_t chan,
                                            const radio_mode_t mode);
  radio_unit_t      pktSelectRadioForFrequency(const radio_freq_hz_t freq,
                                          const radio_chan_hz_t step,
                                          const radio_ch_t chan,
                                          const radio_mode_t mode);
  msg_t             pktSetReceiveStreamInactive(const radio_unit_t radio,
                                          const radio_task_object_t *rto,
                                          const sysinterval_t timeout);
#if PKT_TRANSMIT_TASK_SELF_TERMINATE == TRUE
  void      		pktRadioSendComplete(radio_task_object_t *const rto);
#else
  void              pktRadioSendComplete(radio_task_object_t *const rto,
                                          thread_t *const thread);
#endif
  ICUDriver         *pktLLDradioAttachStream(const radio_unit_t radio);
  void              pktLLDradioDetachStream(const radio_unit_t radio);
  const ICUConfig   *pktLLDradioStreamEnable(const radio_unit_t radio,
                                             const radio_mod_t mod,
                                             const palcallback_t cb);
  void              pktLLDradioStreamDisableI(const radio_unit_t radio,
                                              const radio_mod_t mod);
  bool              pktRadioGetInProgress(const radio_unit_t radio);
  int       	 	pktDisplayFrequencyCode(radio_freq_hz_t code, char *buf,
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
