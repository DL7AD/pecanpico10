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
#include "serialmux.h"

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
 * @brief Connect to multiplexed serial channel
 * @note  Only supports UART based serial channels
 *
 */
msg_t pktConnectMuxedSerial(const SerialDriver *serial,
		                    const SerialConfig *cfg,
							const smux_gpio_t *gpio,
                            const sysinterval_t timeout) {
#if defined(SERIAL_MUX_DEVICE)
  if (serial != &SERIAL_MUX_DEVICE) {
    return MSG_ERROR;
  }
  /* Acquire the multiplexed serial channel. */
  msg_t msg = chBSemWaitTimeout(&serial_mux, timeout);
  if (msg != MSG_OK) {
	  return msg;
  }
  /* Configure the GPIO to map to the UART. */
  if (gpio->rx.line != PAL_NOLINE) {
	  palSetLineMode(gpio->rx.line, gpio->rx.mode);
  }
  if (gpio->tx.line != PAL_NOLINE) {
	  palSetLineMode(gpio->tx.line, gpio->tx.mode);
  }
  sdStart(&SERIAL_MUX_DEVICE, cfg);
  return MSG_OK;
#endif
  (void)serial;
  (void)cfg;
  (void)gpio;
  (void)timeout;
  return MSG_ERROR;
}

/**
 * @brief Disconnect multiplexed serial channel
 * @note  Only supports UART based serial channels
 *
 */
msg_t pktDisconnectMuxedSerial(const SerialDriver *serial,
							   const smux_gpio_t *gpio,
							   const sysinterval_t timeout) {
#if defined(SERIAL_MUX_DEVICE)
  if (serial != &SERIAL_MUX_DEVICE) {
    return MSG_ERROR;
  }
  sdStop(&SERIAL_MUX_DEVICE);
  /* Remove the GPIO map to the UART. */
  if (gpio->rx.line != PAL_NOLINE) {
	  palSetLineMode(gpio->rx.line, PAL_MODE_INPUT);
  }
  if (gpio->tx.line != PAL_NOLINE) {
	  palSetLine(gpio->tx.line);
	  palSetLineMode(gpio->tx.line, PAL_MODE_OUTPUT_PUSHPULL);
  }
  /* Release port lock. */
  chBSemSignal(&serial_mux);
  return MSG_OK;
#endif
  (void)serial;
  (void)gpio;
  (void)timeout;
  return MSG_ERROR;
}

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
  (void)serial;
  (void)timeout;
  return MSG_ERROR;
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
  (void)serial;
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
  (void)serial;
  (void)out;
  (void)len;
  (void)timeout;
  return MSG_ERROR;
}

/** @} */
