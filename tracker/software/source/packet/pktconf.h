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

/* General event definitions. */
#define EVT_NONE                0
#define EVT_PRIORITY_BASE       0

/*
 * Decoder global system event masks.
 * The packet channel object holds the global events.
 * Events are broadcast to any listeners.
 */
//#define STA_AX25_FRAME_RDY      EVENT_MASK(EVT_PRIORITY_BASE +  0)
#define EVT_PKT_BUFFER_FULL    EVENT_MASK(EVT_PRIORITY_BASE +  1)
//#define STA_AX25_CRC_ERROR      EVENT_MASK(EVT_PRIORITY_BASE +  2)
#define EVT_PKT_NO_BUFFER      EVENT_MASK(EVT_PRIORITY_BASE +  3)

//#define EVT_AFSK_TERMINATED     EVENT_MASK(EVT_PRIORITY_BASE +  4)
#define EVT_AFSK_START_FAIL     EVENT_MASK(EVT_PRIORITY_BASE +  5)
//#define STA_AFSK_DECODE_RESET   EVENT_MASK(EVT_PRIORITY_BASE +  6)
#define EVT_PWM_INVALID_SWAP    EVENT_MASK(EVT_PRIORITY_BASE +  7)

/* TODO: Create an AKSK event field in decoder for the PWM & radio events? */
#define EVT_PWM_NO_DATA         EVENT_MASK(EVT_PRIORITY_BASE +  8)
#define EVT_PWM_INVALID_INBAND  EVENT_MASK(EVT_PRIORITY_BASE +  9)
#define EVT_PWM_FIFO_EMPTY      EVENT_MASK(EVT_PRIORITY_BASE + 10)
#define EVT_PWM_QUEUE_FULL      EVENT_MASK(EVT_PRIORITY_BASE + 11)

//#define STA_PWM_STREAM_CLOSED   EVENT_MASK(EVT_PRIORITY_BASE + 12)
#define EVT_PWM_STREAM_TIMEOUT  EVENT_MASK(EVT_PRIORITY_BASE + 13)
#define EVT_PWM_QUEUE_OVERRUN   EVENT_MASK(EVT_PRIORITY_BASE + 14)
#define EVT_PWM_BUFFER_FAIL     EVENT_MASK(EVT_PRIORITY_BASE + 15)

#define EVT_PWM_STREAM_OPEN     EVENT_MASK(EVT_PRIORITY_BASE + 16)
#define EVT_PWM_FIFO_REMNANT    EVENT_MASK(EVT_PRIORITY_BASE + 17)
//#define EVT_PWM_STREAM_CLOSE    EVENT_MASK(EVT_PRIORITY_BASE + 18)
//#define STA_PKT_INVALID_FRAME   EVENT_MASK(EVT_PRIORITY_BASE + 19)

#define EVT_PKT_FAILED_CB_THD   EVENT_MASK(EVT_PRIORITY_BASE + 20)
#define EVT_PKT_BUFFER_MGR_FAIL EVENT_MASK(EVT_PRIORITY_BASE + 21)
#define EVT_PKT_DECODER_START   EVENT_MASK(EVT_PRIORITY_BASE + 22)
#define EVT_PKT_CBK_MGR_FAIL    EVENT_MASK(EVT_PRIORITY_BASE + 23)

#define EVT_PKT_CHANNEL_STOP    EVENT_MASK(EVT_PRIORITY_BASE + 24)
#define EVT_PKT_CHANNEL_CLOSE   EVENT_MASK(EVT_PRIORITY_BASE + 25)
#define EVT_PKT_CHANNEL_OPEN    EVENT_MASK(EVT_PRIORITY_BASE + 26)
#define EVT_RADIO_CCA_GLITCH    EVENT_MASK(EVT_PRIORITY_BASE + 27)

#define EVT_RADIO_CCA_SPIKE     EVENT_MASK(EVT_PRIORITY_BASE + 28)
#define EVT_ICU_SLEEP_TIMEOUT   EVENT_MASK(EVT_PRIORITY_BASE + 29)
//#define EVT_ICU_OVERFLOW        EVENT_MASK(EVT_PRIORITY_BASE + 30)
#define EVT_HDLC_RESET_RCVD     EVENT_MASK(EVT_PRIORITY_BASE + 31)


/* Decoder thread event masks (sent from initiator to decoder). */
#define DEC_COMMAND_START       EVENT_MASK(EVT_PRIORITY_BASE + 0)
#define DEC_COMMAND_STOP        EVENT_MASK(EVT_PRIORITY_BASE + 1)
#define DEC_COMMAND_CLOSE       EVENT_MASK(EVT_PRIORITY_BASE + 2)
#define DEC_DIAG_OUT_END        EVENT_MASK(EVT_PRIORITY_BASE + 3)

/* Console thread event masks. */
#define CONSOLE_CHANNEL_EVT     EVENT_MASK(EVT_PRIORITY_BASE + 0)

/* Response thread event masks (from decoder to initiator). */
#define DEC_OPEN_EXEC           EVENT_MASK(EVT_PRIORITY_BASE + 15)
#define DEC_START_EXEC          EVENT_MASK(EVT_PRIORITY_BASE + 16)
#define DEC_STOP_EXEC           EVENT_MASK(EVT_PRIORITY_BASE + 17)
#define DEC_CLOSE_EXEC          EVENT_MASK(EVT_PRIORITY_BASE + 18)
#define USR_COMMAND_ACK         EVENT_MASK(EVT_PRIORITY_BASE + 19)

#define EVT_STATUS_CLEAR        EVT_NONE

/**
 * PWM stream status bits.
 */
typedef uint32_t            statusmask_t;    /**< Mask of status identifiers. */

/**
 * @brief   Returns an event mask from an event identifier.
 */
#define STATUS_MASK(sid) ((statusmask_t)1 << (statusmask_t)(sid))

/* TODO: Classify status by PKT, AFSK and 2FSK types. */
#define STA_PKT_FRAME_RDY           STATUS_MASK(0)
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

/**
 * Use this attribute to put variables in CCM.
 */
#define useCCM  __attribute__((section(".ram4")))

#ifdef PKT_IS_TEST_PROJECT
/* Define macro replacements for TRACE. */
#define TRACE_DEBUG(format, args...) dbgPrintf(DBG_DEBUG, format, ##args)
#define TRACE_INFO(format, args...) dbgPrintf(DBG_INFO, format, ##args)
#define TRACE_WARN(format, args...) dbgPrintf(DBG_WARN, format, ##args)
#define TRACE_ERROR(format, args...) dbgPrintf(DBG_ERROR, format, ##args)
#endif

#define PKT_THREAD_NAME_MAX     12

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
#include "dbguart.h"
#include "dsp.h"
#include "crc_calc.h"
#include "rxpwm.h"
#include "firfilter_q31.h"
#include "rxafsk.h"
#include "corr_q31.h"
#include "rxhdlc.h"
#include "txhdlc.h"
#include "ihex_out.h"
#include "ax25_dump.h"
#include "si446x.h"
#include "pktevt.h"
#include "debug.h"

/*===========================================================================*/
/* Driver definitions.                                                       */
/*===========================================================================*/

#if !defined(PKT_SVC_USE_RADIO1)
#define PKT_SVC_USE_RADIO1 FALSE
#endif

#if !defined(PKT_SVC_USE_RADIO2)
#define PKT_SVC_USE_RADIO2 FALSE
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

//extern packet_svc_t RPKTD1;

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
 * @brief   Sends a command request to the radio manager.
 * @notes   The task descriptor is copied into a task object which is posted.
 * @post    The command object posted to the radio manager queue.
 *
 * @param[in]   radio   radio unit ID.
 * @param[in]   task    pointer to the task descriptor.
 *                      this is usually a persistent descriptor in the handler.
 * @param[in]   cb      function to call with result (can be NULL).
 *
 * @api
 */
static inline msg_t pktSendRadioCommand(radio_unit_t radio,
                                        radio_task_object_t *task,
                                        radio_task_cb_t cb) {
  radio_task_object_t *rt = NULL;
  msg_t msg = pktGetRadioTaskObject(radio, TIME_INFINITE, &rt);
  if(msg != MSG_OK)
    return MSG_TIMEOUT;
  *rt = *task;
  pktSubmitRadioTask(radio, rt, cb);
  return msg;
}

/**
 * @brief   Release a send packet object memory.
 * @post    The object memory is released.
 *
 * @param[in]   pp     pointer to a @p packet send object
 *
 * @api
 */
static inline void pktReleaseBufferObject(packet_t pp) {
  chDbgAssert(pp != NULL, "no packet pointer");
  pktReleasePacketBuffer(pp);
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
  do {
    packet_t np = pp->nextp;
    pktReleasePacketBuffer(pp);
    pp = np;
  } while(pp != NULL);
}

#endif /* _PKTCONF_H_ */

/** @} */
