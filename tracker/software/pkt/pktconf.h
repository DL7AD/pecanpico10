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
//#define EVT_SUSPEND_EXIT        EVENT_MASK(EVT_PRIORITY_BASE + 7)

#define EVT_PWM_NO_DATA         EVENT_MASK(EVT_PRIORITY_BASE + 8)
#define EVT_PWM_FIFO_SENT       EVENT_MASK(EVT_PRIORITY_BASE + 9)
#define EVT_RADIO_CCA_OPEN      EVENT_MASK(EVT_PRIORITY_BASE + 10)
#define EVT_PWM_QUEUE_FULL      EVENT_MASK(EVT_PRIORITY_BASE + 11)

#define EVT_PWM_FIFO_EMPTY      EVENT_MASK(EVT_PRIORITY_BASE + 12)
#define EVT_PWM_STREAM_TIMEOUT  EVENT_MASK(EVT_PRIORITY_BASE + 13)
#define EVT_PWM_QUEUE_LOCK      EVENT_MASK(EVT_PRIORITY_BASE + 14)
#define EVT_PKT_DECODER_START   EVENT_MASK(EVT_PRIORITY_BASE + 15)

#define EVT_PKT_CHANNEL_STOP    EVENT_MASK(EVT_PRIORITY_BASE + 16)
#define EVT_RADIO_CCA_FIFO_ERR  EVENT_MASK(EVT_PRIORITY_BASE + 17)
#define EVT_AX25_BUFFER_FULL    EVENT_MASK(EVT_PRIORITY_BASE + 18)
#define EVT_AFSK_INVALID_FRAME  EVENT_MASK(EVT_PRIORITY_BASE + 19)

#define EVT_AX25_CRC_ERROR      EVENT_MASK(EVT_PRIORITY_BASE + 20)
#define EVT_HDLC_RESET_RCVD     EVENT_MASK(EVT_PRIORITY_BASE + 21)
#define EVT_AX25_NO_BUFFER      EVENT_MASK(EVT_PRIORITY_BASE + 22)
#define EVT_ICU_SLEEP_TIMEOUT   EVENT_MASK(EVT_PRIORITY_BASE + 23)

#define EVT_PWM_STREAM_CLOSED   EVENT_MASK(EVT_PRIORITY_BASE + 24)
#define EVT_PKT_CHANNEL_CLOSE   EVENT_MASK(EVT_PRIORITY_BASE + 25)
#define EVT_PKT_CHANNEL_OPEN    EVENT_MASK(EVT_PRIORITY_BASE + 26)
#define EVT_AFSK_DECODE_DONE    EVENT_MASK(EVT_PRIORITY_BASE + 27)

#define EVT_RADIO_CCA_SPIKE     EVENT_MASK(EVT_PRIORITY_BASE + 28)
//#define EVT_SERIAL_PKT_OUT_END  EVENT_MASK(EVT_PRIORITY_BASE + 29)
#define EVT_ICU_OUT_OF_RANGE    EVENT_MASK(EVT_PRIORITY_BASE + 30)

/* Initiator thread events (from decoder to initiator). */
#define DEC_OPEN_EXEC           EVENT_MASK(EVT_PRIORITY_BASE + 0)
#define DEC_START_EXEC          EVENT_MASK(EVT_PRIORITY_BASE + 1)
#define DEC_STOP_EXEC           EVENT_MASK(EVT_PRIORITY_BASE + 2)
#define DEC_CLOSE_EXEC          EVENT_MASK(EVT_PRIORITY_BASE + 2)

/* Decoder thread events (from initiator to decoder). */

#define DEC_COMMAND_START       EVENT_MASK(EVT_PRIORITY_BASE + 0)
#define DEC_COMMAND_STOP        EVENT_MASK(EVT_PRIORITY_BASE + 1)
#define DEC_COMMAND_CLOSE       EVENT_MASK(EVT_PRIORITY_BASE + 2)
#define DEC_DIAG_OUT_END        EVENT_MASK(EVT_PRIORITY_BASE + 3)
#define DEC_SUSPEND_EXIT        EVENT_MASK(EVT_PRIORITY_BASE + 4)

/* Reserved system thread events (in user threads). */
#define USR_COMMAND_ACK         EVENT_MASK(EVT_PRIORITY_BASE + 0)

#define EVT_STATUS_CLEAR        EVT_NONE

/*
 * Diagnostic output definitions.
 *
 */

#define NO_SUSPEND              1
#define RELEASE_ON_OUTPUT       2

#define SUSPEND_HANDLING        NO_SUSPEND

/* Extra GPIO value. */
#define PAL_TOGGLE              2U

/*===========================================================================*/
/* Aerospace decoder subsystem includes.                                     */
/*===========================================================================*/

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
#include "ihex_out.h"
#include "ax25_dump.h"
#include "si446x.h"

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

static inline void pktSetLineModeCCA(void) {
  palSetLineMode(LINE_CCA, PAL_MODE_INPUT_PULLUP);
}

/* Decoder blinker LED. */
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

/* Squelch (CCA) indicator. */
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

/* Overflow in FIFO queue space. */
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

/* LED for FIFO out at PWM/ICU side. */
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

/* PWM mirroring to a GPIO for diagnostics. */
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

/* Radio configuration for SPI connected radio. */
/*static inline msg_t pktOpenRadio(packet_svc_t *handler) {
#if USE_SPI_ATTACHED_RADIO == TRUE
  msg_t msg = pktSubmitRadioTask(handler, TIME_S2I(10), NULL);
  return msg;
#else
  (void)handler;
  return MSG_OK;
#endif
}*/

/*static inline msg_t pktStartRadio(packet_svc_t *handler) {
#if USE_SPI_ATTACHED_RADIO == TRUE
  msg_t msg = pktSubmitRadioTask(handler, TIME_MS2I(100), NULL);
  return msg;
#else
  (void)handler;
  return MSG_OK;
#endif
}*/

/*static inline void pktStopRadio(packet_svc_t *handler) {
#if USE_SPI_ATTACHED_RADIO == TRUE
  (void)handler;
#else
  (void)handler;
#endif
}*/

/*static inline msg_t pktCloseRadio(packet_svc_t *handler) {
#if USE_SPI_ATTACHED_RADIO == TRUE
  (void)handler;
  //Si446x_shutdown();
#else
  (void)handler;
#endif
  return MSG_OK;
}*/


static inline msg_t pktSendRadioCommand(packet_svc_t *handler,
                                        radio_task_object_t *task) {
#if USE_SPI_ATTACHED_RADIO == TRUE
  radio_task_object_t *rt = NULL;
  msg_t msg = pktGetRadioTaskObject(handler, TIME_MS2I(3000), &rt);
  if(msg != MSG_OK)
    return MSG_TIMEOUT;
  *rt = *task;
  pktSubmitRadioTask(handler, rt, NULL);
  return msg;
#else
  (void)task;
  (void)handler;
  return MSG_OK;
#endif
}

#endif /* _PKTCONF_H_ */

/** @} */
