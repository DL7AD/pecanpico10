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
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Multiplexed serial driver enable switch.
 * @details If set to @p TRUE support for multiplexed serial is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SERIAL_USE_MUX) || defined(__DOXYGEN__)
//#define STM32_SERIAL_USE_MUX             TRUE
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

#include "pkttypes.h"

/* GPIO configuration. */
typedef struct {
  struct {
    ioline_cfg_t rx;
    ioline_cfg_t tx;
  } open;
  struct {
    ioline_cfg_t rx;
    ioline_cfg_t tx;
  } close;
  SerialConfig    sdcfg;
} SerialMuxConfig;

/* Driver structure. */
typedef struct {
  binary_semaphore_t    msem;
  SerialDriver          *sd;
  const SerialMuxConfig *mcfg;
  event_source_t        *esp;
  event_listener_t      el;
} SerialMuxDriver;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  msg_t pktOpenMuxedSerial(SerialMuxDriver *const msd,
                           const SerialMuxConfig *mcfg,
                           const sysinterval_t timeout);
  msg_t pktCloseMuxedSerial(SerialMuxDriver *const msd,
                            const uint8_t *out,
                            const size_t len,
                            const sysinterval_t timeout);
  void smd_lld_init(void);
#ifdef __cplusplus
}
#endif

#endif /* SERIALMUX_H */

/** @} */
