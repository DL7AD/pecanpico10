/*
    Aerospace Decoder - Copyright (C) 2018-2019 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    serialmux.c
 * @brief   Control for multiplexed serial port.
 *
 * @addtogroup drivers
 * @{
 */
#include "hal.h"
#include "pktconf.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

BSEMAPHORE_DECL(serial_mux, false);
event_source_t *esp;
event_listener_t el;

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 *
 */
msg_t pktAcquireMuxedSerial(const SerialDriver *serial,
                            const sysinterval_t timeout) {
#if defined(SERIAL_MUX_DEVICE)
  if (serial != &SERIAL_MUX_DEVICE) {
    return MSG_OK;
  }
  return chBSemWaitTimeout(&serial_mux, timeout);
#else
  return MSG_OK;
#endif
}

/**
 *
 */
void pktReleaseMuxedSerial(const SerialDriver *serial) {
#if defined(SERIAL_MUX_DEVICE)
  if (serial == &SERIAL_MUX_DEVICE) {
    sdStop(&SERIAL_MUX_DEVICE);
    chBSemSignal(&serial_mux);
  }
#endif
}


/**
 *
 */
msg_t pktReleaseMuxedSerialEOT(const SerialDriver *serial, const uint8_t *out,
                              const size_t len, const sysinterval_t timeout) {
#if defined(SERIAL_MUX_DEVICE)
  msg_t msg = MSG_OK;
  if (serial == &SERIAL_MUX_DEVICE) {
    if (out != NULL) {
      esp = chnGetEventSource((BaseAsynchronousChannel *)serial);
      /* Register for CHN_TRANSMISSION_END */
      chEvtRegisterMaskWithFlags(esp, &el, TRACE_SERIAL_EOT_EVT,
                                         CHN_TRANSMISSION_END);
      chEvtGetAndClearFlags(&el);
      /* Write the data to the serial channel. */
      sdWrite(&SERIAL_MUX_DEVICE, out, len);

      /* Wait for CHN_TRANSMISSION_END */
      if (chEvtWaitAnyTimeout(ALL_EVENTS, timeout) == 0) {
        msg = MSG_TIMEOUT;
      }
      pktUnregisterEventListener(esp, &el);
    }
    sdStop(&SERIAL_MUX_DEVICE);
    chBSemSignal(&serial_mux);
  }
  return msg;
#endif
  return MSG_OK;
}

/** @} */
