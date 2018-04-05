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
 * Decoder global system events.
 * The packet channel object holds the global events.
 * Events are broadcast to any listeners.
 */
#define EVT_AX25_FRAME_RDY      EVENT_MASK(EVT_PRIORITY_BASE + 0)
#define EVT_RADIO_CCA_GLITCH    EVENT_MASK(EVT_PRIORITY_BASE + 1)
#define EVT_RADIO_CCA_CLOSE     EVENT_MASK(EVT_PRIORITY_BASE + 2)
#define EVT_DECODER_ERROR       EVENT_MASK(EVT_PRIORITY_BASE + 3)

#define EVT_AFSK_TERMINATED     EVENT_MASK(EVT_PRIORITY_BASE + 4)
#define EVT_PWM_UNKNOWN_INBAND  EVENT_MASK(EVT_PRIORITY_BASE + 5)
#define EVT_ICU_OVERFLOW        EVENT_MASK(EVT_PRIORITY_BASE + 6)
#define EVT_PKT_FAILED_CB_THD   EVENT_MASK(EVT_PRIORITY_BASE + 7)

#define EVT_PWM_NO_DATA         EVENT_MASK(EVT_PRIORITY_BASE + 8)
#define EVT_AFSK_START_FAIL     EVENT_MASK(EVT_PRIORITY_BASE + 9)
#define EVT_RADIO_CCA_OPEN      EVENT_MASK(EVT_PRIORITY_BASE + 10)
#define EVT_PWM_QUEUE_FULL      EVENT_MASK(EVT_PRIORITY_BASE + 11)

#define EVT_PWM_FIFO_EMPTY      EVENT_MASK(EVT_PRIORITY_BASE + 12)
#define EVT_PWM_STREAM_TIMEOUT  EVENT_MASK(EVT_PRIORITY_BASE + 13)
#define EVT_PWM_QUEUE_LOCK      EVENT_MASK(EVT_PRIORITY_BASE + 14)
#define EVT_PKT_DECODER_START   EVENT_MASK(EVT_PRIORITY_BASE + 15)

#define EVT_PKT_CHANNEL_STOP    EVENT_MASK(EVT_PRIORITY_BASE + 16)
#define EVT_RADIO_CCA_FIFO_ERR  EVENT_MASK(EVT_PRIORITY_BASE + 17)
#define EVT_AX25_BUFFER_FULL    EVENT_MASK(EVT_PRIORITY_BASE + 18)
#define EVT_PKT_INVALID_FRAME   EVENT_MASK(EVT_PRIORITY_BASE + 19)

#define EVT_AX25_CRC_ERROR      EVENT_MASK(EVT_PRIORITY_BASE + 20)
#define EVT_HDLC_RESET_RCVD     EVENT_MASK(EVT_PRIORITY_BASE + 21)
#define EVT_AX25_NO_BUFFER      EVENT_MASK(EVT_PRIORITY_BASE + 22)
#define EVT_ICU_SLEEP_TIMEOUT   EVENT_MASK(EVT_PRIORITY_BASE + 23)

#define EVT_PWM_STREAM_CLOSED   EVENT_MASK(EVT_PRIORITY_BASE + 24)
#define EVT_PKT_CHANNEL_CLOSE   EVENT_MASK(EVT_PRIORITY_BASE + 25)
#define EVT_PKT_CHANNEL_OPEN    EVENT_MASK(EVT_PRIORITY_BASE + 26)
#define EVT_AFSK_DECODE_DONE    EVENT_MASK(EVT_PRIORITY_BASE + 27)

#define EVT_RADIO_CCA_SPIKE     EVENT_MASK(EVT_PRIORITY_BASE + 28)
#define EVT_PKT_BUFFER_MGR_FAIL EVENT_MASK(EVT_PRIORITY_BASE + 29)
#define EVT_PWM_BUFFER_FAIL     EVENT_MASK(EVT_PRIORITY_BASE + 30)
#define EVT_PKT_CBK_MGR_FAIL    EVENT_MASK(EVT_PRIORITY_BASE + 31)


/* Decoder thread events (sent from initiator to decoder). */

#define DEC_COMMAND_START       EVENT_MASK(EVT_PRIORITY_BASE + 0)
#define DEC_COMMAND_STOP        EVENT_MASK(EVT_PRIORITY_BASE + 1)
#define DEC_COMMAND_CLOSE       EVENT_MASK(EVT_PRIORITY_BASE + 2)
#define DEC_DIAG_OUT_END        EVENT_MASK(EVT_PRIORITY_BASE + 3)
#define DEC_SUSPEND_EXIT        EVENT_MASK(EVT_PRIORITY_BASE + 4)


/* Reserved system thread events (in user threads level). */
#define USB_SHELL_EVT           EVENT_MASK(EVT_PRIORITY_BASE + 0)

/* Response thread events (from decoder to initiator). */
#define DEC_OPEN_EXEC           EVENT_MASK(EVT_PRIORITY_BASE + 15)
#define DEC_START_EXEC          EVENT_MASK(EVT_PRIORITY_BASE + 16)
#define DEC_STOP_EXEC           EVENT_MASK(EVT_PRIORITY_BASE + 17)
#define DEC_CLOSE_EXEC          EVENT_MASK(EVT_PRIORITY_BASE + 18)
#define USR_COMMAND_ACK         EVENT_MASK(EVT_PRIORITY_BASE + 19)

/* Diagnostic events. */
#define EVT_DIAG_OUT_END        EVENT_MASK(EVT_PRIORITY_BASE + 20)
#define EVT_PKT_OUT_END         EVENT_MASK(EVT_PRIORITY_BASE + 21)

#define EVT_STATUS_CLEAR        EVT_NONE

/*
 * Diagnostic output definitions.
 * TODO: Deprecate.
 *
 */

#define NO_SUSPEND              1
#define RELEASE_ON_OUTPUT       2

#define SUSPEND_HANDLING        NO_SUSPEND

#ifdef PKT_IS_TEST_PROJECT
/* Define macro replacements for TRACE. */
#define TRACE_DEBUG(format, args...) dbgPrintf(DBG_DEBUG, format, ##args)
#define TRACE_INFO(format, args...) dbgPrintf(DBG_INFO, format, ##args)
#define TRACE_WARN(format, args...) dbgPrintf(DBG_WARN, format, ##args)
#define TRACE_ERROR(format, args...) dbgPrintf(DBG_ERROR, format, ##args)
#endif

/* Extra GPIO value used in local GPIO set/clear/toggle functions. */
#define PAL_TOGGLE              2U

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

#ifndef PKT_IS_TEST_PROJECT
#include "debug.h"
#endif
extern packet_svc_t RPKTD1;

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
* @brief   Define GPIO port where the NIRQ from the radio is connected.
* @notes   The NIRQ line is set in the radio to output the CCA condition.
*
* @api
*/
static inline void pktSetLineModeCCA(void) {
  palSetLineMode(LINE_CCA, PAL_MODE_INPUT_PULLUP);
}

/**
 * @brief   For driving an indicator LED for decoder status.
 * @notes   These functions control the LED on a GPIO line if defined.
 *
 * @api
 */
static inline void pktSetLineModeDecoderLED(void) {
#if defined(LINE_DECODER_LED)
  palSetLineMode(LINE_DECODER_LED, PAL_MODE_OUTPUT_PUSHPULL);
#endif
}

static inline void pktUnsetLineModeDecoderLED(void) {
#if defined(LINE_DECODER_LED)
  palSetLineMode(LINE_DECODER_LED, PAL_MODE_UNCONNECTED);
#endif
}

static inline void pktWriteDecoderLED(uint8_t state) {
#if defined(LINE_DECODER_LED)
  if(state != PAL_TOGGLE)
    palWriteLine(LINE_DECODER_LED, state);
  else
    palToggleLine(LINE_DECODER_LED);
#else
  (void)state;
#endif
}

/**
 * @brief   For driving an indicator LED for PWM CCA asserted.
 * @notes   These functions control the LED on a GPIO line if defined.
 *
 * @api
 */
static inline void pktSetLineModeSquelchLED(void) {
#if defined(LINE_SQUELCH_LED)
  palSetLineMode(LINE_SQUELCH_LED, PAL_MODE_OUTPUT_PUSHPULL);
#endif
}

static inline void pktWriteSquelchLED(uint8_t state) {
#if defined(LINE_SQUELCH_LED)
  if(state != PAL_TOGGLE)
    palWriteLine(LINE_SQUELCH_LED, state);
  else
    palToggleLine(LINE_SQUELCH_LED);
#else
  (void)state;
#endif
}

static inline void pktUnsetLineModeSquelchLED(void) {
#if defined(LINE_SQUELCH_LED)
  palSetLineMode(LINE_SQUELCH_LED, PAL_MODE_UNCONNECTED);
#endif
}

/**
 * @brief   For driving an indicator LED for PWM queue space exhausted.
 * @notes   These functions control the LED on a GPIO line if defined.
 *
 * @api
 */
static inline void pktSetLineModeOverflowLED(void) {
#if defined(LINE_OVERFLOW_LED)
  palSetLineMode(LINE_OVERFLOW_LED, PAL_MODE_OUTPUT_PUSHPULL);
#endif
}

static inline void pktWriteOverflowLED(uint8_t state) {
#if defined(LINE_OVERFLOW_LED)
  if(state != PAL_TOGGLE)
    palWriteLine(LINE_OVERFLOW_LED, state);
  else
    palToggleLine(LINE_OVERFLOW_LED);
#else
  (void)state;
#endif
}

static inline void pktUnsetLineModeOverflowLED(void) {
#if defined(LINE_OVERFLOW_LED)
  palSetLineMode(LINE_OVERFLOW_LED, PAL_MODE_UNCONNECTED);
#endif
}

/**
 * @brief   For driving an indicator LED for PWM buffers exhausted.
 * @notes   These functions control the LED on a GPIO line if defined.
 *
 * @api
 */
static inline void pktSetLineModeNoFIFOLED(void) {
#if defined(LINE_NO_FIFO_LED)
  palSetLineMode(LINE_NO_FIFO_LED, PAL_MODE_OUTPUT_PUSHPULL);
#endif
}

static inline void pktWriteNoFIFOLED(uint8_t state) {
#if defined(LINE_NO_FIFO_LED)
  if(state != PAL_TOGGLE)
    palWriteLine(LINE_NO_FIFO_LED, state);
  else
    palToggleLine(LINE_NO_FIFO_LED);
#else
  (void)state;
#endif
}

static inline void pktUnsetLineModeNoFIFOLED(void) {
#if defined(LINE_NO_FIFO_LED)
  palSetLineMode(LINE_NO_FIFO_LED, PAL_MODE_UNCONNECTED);
#endif
}

/**
 * @brief   For diagnostics only.
 * @notes   These functions control the mirroring of radio PWM data to a GPIO.
 *
 * @notapi
 */
static inline void pktSetLineModePWMMirror(void) {
#if defined(LINE_PWM_MIRROR)
  palSetLineMode(LINE_PWM_MIRROR, PAL_MODE_OUTPUT_PUSHPULL);
#endif
}

static inline void pktUnsetLineModePWMMirror(void) {
#if defined(LINE_PWM_MIRROR)
  palSetLineMode(LINE_PWM_MIRROR, PAL_MODE_UNCONNECTED);
#endif
}

static inline void pktWritePWMMirror(uint8_t state) {
#if defined(LINE_PWM_MIRROR)
  if(state != PAL_TOGGLE)
    palWriteLine(LINE_PWM_MIRROR, state);
  else
    palToggleLine(LINE_PWM_MIRROR);
#else
  (void)state;
#endif
}

/**
 * @brief   Sends a command request to a radio.
 * @post    The command object posted to the radio manager queue.
 *
 * @param[in]   radio    radio unit ID.
 * @param[in]   task     pointer to a task object.
 *
 * @api
 */
static inline msg_t pktSendRadioCommand(radio_unit_t radio,
                                        radio_task_object_t *task) {
#if USE_SPI_ATTACHED_RADIO == TRUE
  radio_task_object_t *rt = NULL;
  msg_t msg = pktGetRadioTaskObject(radio, TIME_MS2I(3000), &rt);
  if(msg != MSG_OK)
    return MSG_TIMEOUT;
  *rt = *task;
  pktSubmitRadioTask(radio, rt, NULL);
  return msg;
#else
  (void)task;
  (void)handler;
  return MSG_OK;
#endif
}

/**
 * @brief   Release a send packet object memory.
 * @post    The object memory is released.
 *
 * @param[in]   pp     pointer to a @p packet send object
 *
 * @api
 */
static inline void pktReleaseSendObject(packet_t pp) {
#if USE_SPI_ATTACHED_RADIO == TRUE
#if USE_NEW_PKT_TX_ALLOC == TRUE

      pktReleaseOutgoingBuffer(pp);
#else
      ax25_delete (pp);
#endif
#else
  (void)pp;
#endif
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
static inline void pktReleaseSendQueue(packet_t pp) {
#if USE_SPI_ATTACHED_RADIO == TRUE
#if USE_NEW_PKT_TX_ALLOC == TRUE
  /* Release all packets in linked list. */
  do {
    packet_t np = pp->nextp;
    pktReleaseOutgoingBuffer(pp);
    pp = np;
  } while(pp != NULL);
#else
      ax25_delete (pp);
#endif
#else
  (void)pp;
#endif
}

#endif /* _PKTCONF_H_ */

/** @} */
