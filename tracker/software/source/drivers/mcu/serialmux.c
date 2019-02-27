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
#include "serialmux.h"

#if (HAL_USE_SERIAL && STM32_SERIAL_USE_MUX) || defined(__DOXYGEN__)

#include "pktconf.h"


/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART1 serial driver identifier.*/
#if STM32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
SerialMuxDriver SM1;
#endif

/** @brief USART2 serial driver identifier.*/
#if STM32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
SerialMuxDriver SM2;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
SerialMuxDriver SM3;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
SerialMuxDriver SM4;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
SerialMuxDriver SM5;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
SerialMuxDriver SM6;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
SerialMuxDriver SM7;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
SerialMuxDriver SM9;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_HAS_LPUART1 || defined(__DOXYGEN__)
#if STM32_SERIAL_USE_LPUART1
SerialMuxDriver LPSM1;
#endif
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial multiplexer driver initialization.
 *
 * @notapi
 */
void smd_lld_init(void) {

#if STM32_SERIAL_USE_USART1
  chBSemObjectInit(&SM1.msem, false);
  SM1.sd = &SD1;
  SM1.esp = NULL;
#endif

#if STM32_SERIAL_USE_USART2
  chBSemObjectInit(&SM2.msem, false);
  SM2.sd = &SD2;
  SM2.esp = NULL;
#endif

#if STM32_SERIAL_USE_USART3
  chBSemObjectInit(&SM3.msem, false);
  SM3.sd = &SD3;
  SM3.esp = NULL;
#endif

#if STM32_SERIAL_USE_UART4
  chBSemObjectInit(&SM4.msem, false);
  SM4.sd = &SD4;
  SM4.esp = NULL;
#endif

#if STM32_SERIAL_USE_UART5
  chBSemObjectInit(&SM5.msem, false);
  SM5.sd = &SD5;
  SM5.esp = NULL;
#endif

#if STM32_SERIAL_USE_USART6
  chBSemObjectInit(&SM6.msem, false);
  SM6.sd = &SD6;
  SM6.esp = NULL;
#endif

#if STM32_SERIAL_USE_UART7
  chBSemObjectInit(&SM7.msem, false);
  SM7.sd = &SD7;
  SM7.esp = NULL;
#endif

#if STM32_SERIAL_USE_UART8
  chBSemObjectInit(&SM8.msem, false);
  SM8.sd = &SD8;
  SM8.esp = NULL;
#endif

#if STM32_HAS_LPUART1
#if STM32_SERIAL_USE_LPUART1
  chBSemObjectInit(&LPSM1.msem, false);
  LPSM1.sd = &LPSD1;
  LPSM1.esp = NULL;
#endif
#endif
}

/**
 * @brief Connect to multiplexed serial channel
 * @note  Only supports UART/USART based serial channels
 *
 */
msg_t pktOpenMuxedSerial(SerialMuxDriver *const smd,
                         const SerialConfig *sdcfg,
                         const SerialMuxConfig *mcfg,
                         const sysinterval_t timeout) {
  /* Acquire the multiplexed serial channel. */
  msg_t msg = chBSemWaitTimeout(&smd->msem, timeout);
  if (msg != MSG_OK) {
    return msg;
  }
  /* Configure the GPIO to map to the UART. */
  if (mcfg->open.rx.line != PAL_NOLINE) {
    palSetLineMode(mcfg->open.rx.line, mcfg->open.rx.mode);
  }
  if (mcfg->open.tx.line != PAL_NOLINE) {
    palSetLineMode(mcfg->open.tx.line, mcfg->open.tx.mode);
  }
  sdStart(smd->sd, sdcfg);
  return MSG_OK;
}

/**
 * @brief Disconnect multiplexed serial channel
 * @note  Only supports UART based serial channels
 *
 * @param[in] smd       pointer to serial multiplexer driver.
 * @param[in] timeout   the number of ticks before the operation timeouts, the
 *                      special values are handled as follow:
 *                      - @a TIME_INFINITE is allowed but interpreted as a
 *                        normal time specification.
 *                      - @a TIME_IMMEDIATE this value does not wait for the
 *                        serial transmission to end.
 */
msg_t pktCloseMuxedSerial(SerialMuxDriver *const smd,
                          const SerialMuxConfig *mcfg,
                          const uint8_t *out,
                          const size_t len,
                          const sysinterval_t timeout) {
  msg_t msg = MSG_OK;
  if (out != NULL) {
    smd->esp = chnGetEventSource((BaseAsynchronousChannel *)smd->sd);
    /* Register for CHN_TRANSMISSION_END. To avoid any conflict with other
       event codes we register the listener with no event. */

    chEvtRegisterMaskWithFlags(smd->esp, &smd->el, 0, CHN_TRANSMISSION_END);

    /* Write the data to the serial channel. */
    sdWrite(smd->sd, out, len);

    if (timeout != TIME_IMMEDIATE) {
      /* Wait for CHN_TRANSMISSION_END */
      virtual_timer_t vtp;
      chVTSet(&vtp, timeout, NULL, NULL);
      while (chVTIsArmed(&vtp)
          && (chEvtGetAndClearFlags(&smd->el) | CHN_TRANSMISSION_END) == 0);
      if (!chVTIsArmed(&vtp)) {
        msg = MSG_TIMEOUT;
      }
      pktUnregisterEventListener(smd->esp, &smd->el);
    }
  }

  /* Stop serial driver. */
  sdStop(smd->sd);

  /* Remove the GPIO map to the UART. */
  if (mcfg->close.rx.line != PAL_NOLINE) {
    if (mcfg->close.rx.mode == PAL_MODE_RESET) {
      palSetLineMode(mcfg->close.rx.line, PAL_MODE_INPUT);
    } else {
      palSetLineMode(mcfg->close.rx.line, mcfg->close.rx.mode);
    }
  }
  if (mcfg->close.tx.line != PAL_NOLINE) {
    if (mcfg->close.tx.mode == PAL_MODE_RESET) {
      palSetLine(mcfg->close.tx.line);
      palSetLineMode(mcfg->close.tx.line, PAL_MODE_OUTPUT_PUSHPULL);
    } else {
      palSetLineMode(mcfg->close.tx.line, mcfg->close.tx.mode);
    }
  }
  /* Release port lock. */
  chBSemSignal(&smd->msem);
  return msg;
}

#endif /* HAL_USE_SERIAL */

/** @} */
