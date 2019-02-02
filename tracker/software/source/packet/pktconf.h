/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    pktconf.h
 * @brief   Configuration file template.
 * @details Include this file in each source code module.
 *
 * @addtogroup pktconfig
 * @details Decoder related settings and configuration.
 * @note    Refer to halconf.h, mcuconf.h and halconf.h for specific settings.
 * @{
 */

#ifndef _PKTCONF_H_
#define _PKTCONF_H_

/*===========================================================================*/
/* ChibiOS required common and system includes.                              */
/*===========================================================================*/

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include <stdio.h>
#include <string.h>
#include "shell.h"
#include <stdlib.h>
#include <math.h>

/*
 * For F103 ARM_MATH_CM3 set -DARM_MATH_CM3 in the makefile CDefines section.
 * For F413 ARM_MATH_CM4 set -DARM_MATH_CM4 in the makefile CDefines section.
 */
#include "arm_math.h"

/*===========================================================================*/
/* Aerospace decoder system function includes.                               */
/*===========================================================================*/

#include "bit_array.h"

/* Extend standard OS result codes. */
#define MSG_ERROR           (msg_t)-3   /**< @brief Error condition.  */
#define MSG_IDLE            (msg_t)-4   /**< @brief Idle condition. */

/* General event definitions. */
#define EVT_NONE                    0
#define EVT_PRIORITY_BASE           0

/* Event diagnostics events. */
#define PKT_DIAGNOSTIC_EVENT_CODE    0
#define AFSK_DIAGNOSTIC_EVENT_CODE   1
/*
 * Decoder global system event masks.
 * The packet channel object holds the global events.
 * Events are broadcast to any listeners.
 */
#define EVT_RADIO_CCA_DROP      EVENT_MASK(EVT_PRIORITY_BASE +  0)
#define EVT_PKT_BUFFER_FULL     EVENT_MASK(EVT_PRIORITY_BASE +  1)
#define EVT_PWM_ACTIVE          EVENT_MASK(EVT_PRIORITY_BASE +  2)
#define EVT_PKT_NO_BUFFER       EVENT_MASK(EVT_PRIORITY_BASE +  3)

#define EVT_PWM_CCA_CLOSE       EVENT_MASK(EVT_PRIORITY_BASE +  4)
#define EVT_AFSK_START_FAIL     EVENT_MASK(EVT_PRIORITY_BASE +  5)
#define EVT_PWM_ICU_ZERO        EVENT_MASK(EVT_PRIORITY_BASE +  6)
#define EVT_PWM_INVALID_SWAP    EVENT_MASK(EVT_PRIORITY_BASE +  7)

/* TODO: Move events for the PWM & radio events to AFSK event object */
#define EVT_PWM_NO_DATA         EVENT_MASK(EVT_PRIORITY_BASE +  8)
#define EVT_PWM_INVALID_INBAND  EVENT_MASK(EVT_PRIORITY_BASE +  9)
#define EVT_PWM_FIFO_EMPTY      EVENT_MASK(EVT_PRIORITY_BASE + 10)
#define EVT_PWM_QUEUE_FULL      EVENT_MASK(EVT_PRIORITY_BASE + 11)

#define EVT_PWM_JAMMING_RESET   EVENT_MASK(EVT_PRIORITY_BASE + 12)
#define EVT_PWM_STREAM_TIMEOUT  EVENT_MASK(EVT_PRIORITY_BASE + 13)
#define EVT_PWM_QUEUE_ERROR     EVENT_MASK(EVT_PRIORITY_BASE + 14)
#define EVT_PWM_BUFFER_FAIL     EVENT_MASK(EVT_PRIORITY_BASE + 15)

#define EVT_RAD_STREAM_OPEN     EVENT_MASK(EVT_PRIORITY_BASE + 16)
#define PWM_FIFO_ORDER          EVENT_MASK(EVT_PRIORITY_BASE + 17)
#define EVT_RAD_STREAM_CLOSE    EVENT_MASK(EVT_PRIORITY_BASE + 18)
#define EVT_AFSK_PWM_STOP       EVENT_MASK(EVT_PRIORITY_BASE + 19)

#define EVT_PKT_FAILED_CB_THD   EVENT_MASK(EVT_PRIORITY_BASE + 20)
#define EVT_PWM_ICU_LIMIT       EVENT_MASK(EVT_PRIORITY_BASE + 21)
#define EVT_PKT_RECEIVE_START   EVENT_MASK(EVT_PRIORITY_BASE + 22)
#define EVT_PKT_CBK_MGR_FAIL    EVENT_MASK(EVT_PRIORITY_BASE + 23)

#define EVT_PKT_RECEIVE_STOP    EVENT_MASK(EVT_PRIORITY_BASE + 24)
#define EVT_PKT_RECEIVE_CLOSE   EVENT_MASK(EVT_PRIORITY_BASE + 25)
#define EVT_PKT_RECEIVE_OPEN    EVENT_MASK(EVT_PRIORITY_BASE + 26)
#define EVT_PWM_RADIO_TIMEOUT   EVENT_MASK(EVT_PRIORITY_BASE + 27)

#define EVT_RADIO_CCA_SPIKE     EVENT_MASK(EVT_PRIORITY_BASE + 28)
#define EVT_PWM_ICU_OVERFLOW    EVENT_MASK(EVT_PRIORITY_BASE + 29)
//#define EVT_HDLC_RESET_RCVD     EVENT_MASK(EVT_PRIORITY_BASE + 30)
#define EVT_RAD_STREAM_SWITCH   EVENT_MASK(EVT_PRIORITY_BASE + 30)
#define EVT_HDLC_OPENING_FLAG   EVENT_MASK(EVT_PRIORITY_BASE + 31)

/* Decoder thread event masks (sent from initiator to a decoder). */
#define DEC_COMMAND_START       EVENT_MASK(EVT_PRIORITY_BASE + 0)
#define DEC_COMMAND_STOP        EVENT_MASK(EVT_PRIORITY_BASE + 1)
#define DEC_COMMAND_CLOSE       EVENT_MASK(EVT_PRIORITY_BASE + 2)

/* Common decoder command event masks (from decoder to initiator). */
#define DEC_OPEN_EXEC           EVENT_MASK(EVT_PRIORITY_BASE + 16)
#define DEC_START_EXEC          EVENT_MASK(EVT_PRIORITY_BASE + 17)
#define DEC_STOP_EXEC           EVENT_MASK(EVT_PRIORITY_BASE + 18)
#define DEC_CLOSE_EXEC          EVENT_MASK(EVT_PRIORITY_BASE + 19)
#define USR_COMMAND_ACK         EVENT_MASK(EVT_PRIORITY_BASE + 20)

/* AFSK functional event masks (error or other application events). */
#define EVT_AFSK_PWM_START      EVENT_MASK(EVT_PRIORITY_BASE + 21)
#define EVT_PWM_2               EVENT_MASK(EVT_PRIORITY_BASE + 22)
#define EVT_PWM_3               EVENT_MASK(EVT_PRIORITY_BASE + 23)
#define EVT_PWM_4               EVENT_MASK(EVT_PRIORITY_BASE + 24)
#define EVT_PWM_5               EVENT_MASK(EVT_PRIORITY_BASE + 25)

/* Console thread event masks. */
#define CONSOLE_CHANNEL_EVT     EVENT_MASK(EVT_PRIORITY_BASE + 0)
#define CONSOLE_SHELL_EVT       EVENT_MASK(EVT_PRIORITY_BASE + 1)

/* Global thread event masks (guaranteed to be unique). */
#define GTE_RECEIVE_INACTIVE    EVENT_MASK(EVT_PRIORITY_BASE + 25)

#define EVT_STATUS_CLEAR        EVT_NONE

/**
 * PWM stream status bits.
 */
typedef uint32_t            statusmask_t;    /**< Mask of status identifiers. */

/**
 * @brief   Returns an event mask from an event identifier.
 */
#define STATUS_MASK(sid) ((statusmask_t)1 << (statusmask_t)(sid))

#define STA_NONE                    0

/* TODO: Classify status by PKT, AFSK and 2FSK types. */
#define STA_PKT_FRAME_RDY           STATUS_MASK(0) // These should go in RPKTDx
#define STA_PKT_CRC_ERROR           STATUS_MASK(1)
#define STA_PKT_INVALID_FRAME       STATUS_MASK(2)
#define STA_AFSK_DECODE_RESET       STATUS_MASK(3)

#define STA_AFSK_DECODE_DONE        STATUS_MASK(4)
#define STA_PWM_STREAM_CLOSED       STATUS_MASK(5)
#define STA_AFSK_FRAME_RESET        STATUS_MASK(6)
#define STA_PKT_BUFFER_FULL         STATUS_MASK(7)

#define STA_AFSK_INVALID_INBAND     STATUS_MASK(8)
#define STA_AFSK_INVALID_SWAP       STATUS_MASK(9)
#define STA_PWM_STREAM_TIMEOUT      STATUS_MASK(10)
#define STA_PKT_NO_BUFFER           STATUS_MASK(11)

#define STA_PWM_QUEUE_ERROR         STATUS_MASK(12)
#define STA_PWM_BUFFER_FULL         STATUS_MASK(13)
#define STA_PWM_ICU_OVERFLOW        STATUS_MASK(14)
#define STA_PWM_ICU_ZERO            STATUS_MASK(15)

#define STA_AFSK_PWM_STOPPED        STATUS_MASK(16)
#define STA_AFSK_PWM_NO_DATA        STATUS_MASK(17)
#define STA_AFSK_UNEXPECTED_INBAND  STATUS_MASK(18)
#define STA_AFSK_PWM_TIMEOUT        STATUS_MASK(19)

#define STA_AFSK_FRAME_OPEN         STATUS_MASK(20)
#define STA_AFSK_FRAME_DATA         STATUS_MASK(21)
#define STA_PWM_STREAM_DISABLE      STATUS_MASK(22)
#define STA_PWM_RADIO_STOP          STATUS_MASK(23)

#define STA_PWM_NO_RADIO_DATA       STATUS_MASK(24)
#define STA_CCA_RADIO_DROP          STATUS_MASK(25)
#define STA_PWM_STREAM_SWITCH       STATUS_MASK(26)
#define STA_AFSK_HDLC_ERROR         STATUS_MASK(27)

#define STA_PWM_STREAM_STOP         STATUS_MASK(28)
#define STA_CCA_RADIO_SPIKE         STATUS_MASK(29)
#define STA_CCA_RADIO_CONTINUE      STATUS_MASK(30)
#define STA_PWM_ICU_LIMIT           STATUS_MASK(31)


/**
 * Use this attribute to put variables in CCM.
 */
#define useCCM  __attribute__((section(".ram4")))

#define PKT_THREAD_NAME_MAX     20

/* Extra GPIO value used in local GPIO set/clear/toggle functions. */
#define PAL_TOGGLE              2U
#define PAL_INVALID             -1

/*===========================================================================*/
/* Aerospace decoder subsystem includes.                                     */
/*===========================================================================*/

#include "pkttypes.h"
#include "portab.h"
#include "rxax25.h"
#include "pktservice.h"
#include "pktradio.h"
#include "diagstrm.h"
#include "dsp.h"
#include "crc_calc.h"
#include "rxpwm.h"
#include "firfilter_q31.h"
#include "firfilter_f32.h"
#include "rxafsk.h"
#include "corr_q31.h"
#include "corr_f32.h"
#include "rxhdlc.h"
#include "txhdlc.h"
#include "ihex_out.h"
#include "ax25_dump.h"
#include "si446x.h"
#include "pktevt.h"
#include "debug.h"
#include "threads.h"

/*===========================================================================*/
/* Driver definitions.                                                       */
/*===========================================================================*/

#if !defined(PKT_SVC_USE_RADIO1)
#define PKT_SVC_USE_RADIO1 FALSE
#endif

#if !defined(PKT_SVC_USE_RADIO2)
#define PKT_SVC_USE_RADIO2 FALSE
#endif

#if 0
 /*===========================================================================*/
 /* Driver macros.                                                            */
 /*===========================================================================*/

 /**
  * @brief   Wakes up the waiting thread for a radio task.
  *
  * @param[in] rto      pointer to the @p radioTask object
  *
  * @notapi
  */
 #define _rcmd_wakeup_cb(rto) do {                                           \
   chSysLock();                                                              \
   chThdResumeS(&(rto)->thread, rto->result);                                \
   chSysUnlock();                                                            \
 } while(0)
#endif
/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Generalized GPIO handling for optional IO.
 * @notes   These functions are primarily for LED line control.
 *
 * @api
 */
static inline void pktSetGPIOlineMode(ioline_t line, iomode_t mode) {
  if(line != PAL_NOLINE)
    palSetLineMode(line, mode);
}

static inline void pktUnsetGPIOlineMode(ioline_t line) {
  if(line != PAL_NOLINE)
    palSetLineMode(line, PAL_MODE_UNCONNECTED);
}

static inline void pktWriteGPIOline(ioline_t line, uint8_t state) {
  if(line != PAL_NOLINE) {
    if(state != PAL_TOGGLE)
      palWriteLine(line, state);
    else
      palToggleLine(line);
  }
}

static inline int8_t pktReadGPIOline(ioline_t line) {
  if(line != PAL_NOLINE)
    return palReadLine(line);
  else
    return PAL_INVALID;
}


/**
 * @brief   Sends a command request to the radio manager queue.
 * @notes   The task descriptor is copied into a task object which is posted.
 * @post    The command object posted to the radio manager queue.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   task    pointer to the task descriptor.
 *                      this is usually a persistent descriptor in the handler.
 * @param[in]   cmd     the radio task to be queued
 * @param[in]   cfg     pointer to radio configuration
 * @param[in]   timeout the timeout in system ticks, the special values are
 *                      handled as follow:
 *                      - @a TIME_INFINITE the thread enters an infinite sleep
 *                        state.
 *                      - @a TIME_IMMEDIATE the thread is not enqueued and
 *                        the function returns @p MSG_TIMEOUT as if a timeout
 *                        occurred.
 * @param[in]   result  pointer to variable for task result (can be NULL).
 * @param[in]   cb      function to call with result (can be NULL).
 *
 * @return      status of operation
 * @retval      MSG_OK      if command queued successfully
 * @retval      MSG_TIMEOUT if a task object could not be obtained
 * @retval      MSG_RESET   the radio manager is closing.
 *
 * @api
 */
static inline msg_t pktQueueRadioCommand(const radio_unit_t radio,
                                        const radio_command_t cmd,
                                        const radio_params_t *cfg,
                                        const sysinterval_t timeout,
                                        msg_t *const result,
                                        const radio_task_cb_t cb) {


  if (cmd > PKT_RADIO_TASK_MAX) {
    chDbgAssert(false, "invalid radio command");
    return MSG_ERROR;
  }

  radio_task_object_t *rt = NULL;

  msg_t msg = pktGetRadioTaskObject(radio, timeout, cfg, &rt);
  if (msg != MSG_OK)
    return msg;
  rt->user_cb = cb;
  rt->command = cmd;
  pktSubmitRadioTask(radio, rt, NULL);

  /* Result check required so suspend. */
  if (result != NULL) {
    chSysLock();
    *result = chThdSuspendS(&rt->thread);
    chSysUnlock();
  }
  return MSG_OK;
}

/**
 * @brief   Sends a priority command request to the radio manager queue.
 * @notes   The task descriptor is copied into a task object which is posted.
 * @post    The command object posted to the radio manager queue.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   task    pointer to the task descriptor.
 *                      this is usually a persistent descriptor in the handler.
 * @param[in]   cmd     the radio task to be queued
 * @param[in]   cfg     pointer to radio configuration
 * @param[in]   cb      function to call with result (can be NULL).
 *
 * @return      status of operation
 * @retval      MSG_OK      Command has been queued.
 * @retval      MSG_TIMEOUT No task object available.
 *
 * @api
 */
static inline msg_t pktQueuePriorityRadioCommandI(const radio_unit_t radio,
                                        const radio_command_t cmd,
                                        const radio_params_t *cfg,
                                        const radio_task_cb_t cb) {


  if (cmd > PKT_RADIO_TASK_MAX) {
    chDbgAssert(false, "invalid radio command");
    return MSG_ERROR;
  }

  radio_task_object_t *rt = NULL;
  msg_t msg = pktGetRadioTaskObjectI(radio, cfg, &rt);
  if (msg == MSG_TIMEOUT)
    return MSG_TIMEOUT;
  rt->user_cb = cb;
  rt->command = cmd;
  pktSubmitPriorityRadioTaskI(radio, rt, NULL);

  return msg;
}

/**
 * @brief   Sends a priority command request to the radio manager queue.
 * @notes   The task descriptor is copied into a task object which is posted.
 * @post    The command object posted to the radio manager queue.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   task    pointer to the task descriptor.
 *                      this is usually a persistent descriptor in the handler.
 * @param[in]   cmd     the radio task to be queued
 * @param[in]   cfg     pointer to radio configuration
 * @param[in]   timeout the timeout in system ticks, the special values are
 *                      handled as follow:
 *                      - @a TIME_INFINITE the thread enters an infinite sleep
 *                        state.
 *                      - @a TIME_IMMEDIATE the thread is not enqueued and
 *                        the function returns @p MSG_TIMEOUT as if a timeout
 *                        occurred.
 * @param[in]   result  pointer to variable for task result (can be NULL).
 * @param[in]   cb      function to call with result (can be NULL).
 *
 * @return      status of operation
 * @retval      MSG_OK      Command has been queued.
 * @retval      MSG_TIMEOUT No task object available.
 * @retval      MSG_RESET   the radio manager is closing.
 *
 * @api
 */
static inline msg_t pktQueuePriorityRadioCommand(const radio_unit_t radio,
                                        const radio_command_t cmd,
                                        const radio_params_t *cfg,
                                        const sysinterval_t timeout,
                                        msg_t *const result,
                                        const radio_task_cb_t cb) {


  if (cmd > PKT_RADIO_TASK_MAX) {
    chDbgAssert(false, "invalid radio command");
    return MSG_ERROR;
  }
  radio_task_object_t *rt = NULL;
  msg_t msg = pktGetRadioTaskObject(radio, timeout, cfg, &rt);
  if(msg != MSG_OK)
    return msg;
  rt->user_cb = cb;
  rt->command = cmd;
  pktSubmitPriorityRadioTask(radio, rt, NULL);

  /* Result check required so suspend. */
  if (result != NULL) {
    chSysLock();
    *result = chThdSuspendS(&rt->thread);
    chSysUnlock();
  }
  return MSG_OK;
}

/**
 * @brief   Release a send packet object memory.
 * @post    The object memory is released.
 *
 * @param[in]   pp     pointer to a @p packet send object
 *
 * @return  next linked packet reference or NULL if none
 *
 * @api
 */
static inline packet_t pktReleaseBufferObject(packet_t pp) {
  chDbgAssert(pp != NULL, "no packet pointer");
#if USE_CCM_HEAP_FOR_PKT == TRUE
  pktAssertCCMdynamicCheck(pp);
#endif
  packet_t np = pp->nextp;
  pktReleaseCommonPacketBuffer(pp);
  return np;
}

/**
 * @brief   Release memory from one or more send object(s).
 * @notes   a linked list will have all members released.
 * @post    The object memory is released.
 *
 * @param[in]   pp     pointer to a @p packet send object
 *
 * @api
 */
static inline void pktReleaseBufferChain(packet_t pp) {
  chDbgAssert(pp != NULL, "no packet pointer");
  /* Release all packets in linked list. */
  while((pp = pktReleaseBufferObject(pp)) != NULL)
    ;
}

#endif /* _PKTCONF_H_ */

/** @} */
