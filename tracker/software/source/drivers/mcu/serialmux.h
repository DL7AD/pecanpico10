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
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  msg_t pktAcquireMuxedSerial(SerialDriver *serial, sysinterval_t timeout);
  void  pktReleaseMuxedSerial(SerialDriver *serial);
  msg_t pktReleaseMuxedSerialEOT(SerialDriver *serial, uint8_t *out,
                                size_t len, sysinterval_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* SERIALMUX_H */

/** @} */
