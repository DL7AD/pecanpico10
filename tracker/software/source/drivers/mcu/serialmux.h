/*
    Aerospace Decoder - Copyright (C) 2018-2019 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    serialmux.h
 * @brief   Control for multiplexed serial port.
 *
 * @addtogroup drivers
 * @{
 */

#ifndef SERIALMUX_H
#define SERIALMUX_H

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

#include "pkttypes.h"

typedef struct serialMuxGPIO {
	ioline_cfg_t rx;
	ioline_cfg_t tx;
} smux_gpio_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  msg_t pktConnectMuxedSerial(const SerialDriver *serial,
		                      const SerialConfig *cfg,
							  const smux_gpio_t *gpio,
                              const sysinterval_t timeout);
  msg_t pktDisconnectMuxedSerial(const SerialDriver *serial,
  							     const smux_gpio_t *gpio,
  							     const sysinterval_t timeout);
  msg_t pktAcquireMuxedSerial(const SerialDriver *serial,
		                      const sysinterval_t timeout);
  void  pktReleaseMuxedSerial(const SerialDriver *serial);
  msg_t pktReleaseMuxedSerialEOT(const SerialDriver *serial,
		                         const uint8_t *out,
                                 const size_t len,
								 const sysinterval_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* SERIALMUX_H */

/** @} */
