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
SerialMuxDriver MSD1;
#endif

/** @brief USART2 serial driver identifier.*/
#if STM32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
SerialMuxDriver MSD2;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
SerialMuxDriver MSD3;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
SerialMuxDriver MSD4;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
SerialMuxDriver MSD5;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
SerialMuxDriver MSD6;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
SerialMuxDriver MSD7;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
SerialMuxDriver MSD8;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_HAS_LPUART1 || defined(__DOXYGEN__)
#if STM32_SERIAL_USE_LPUART1
SerialMuxDriver MLPSD1;
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
  chBSemObjectInit(&MSD1.msem, false);
  MSD1.sd = &SD1;
  MSD1.esp = NULL;
#endif

#if STM32_SERIAL_USE_USART2
  chBSemObjectInit(&MSD2.msem, false);
  MSD2.sd = &SD2;
  MSD2.esp = NULL;
#endif

#if STM32_SERIAL_USE_USART3
  chBSemObjectInit(&MSD3.msem, false);
  MSD3.sd = &SD3;
  MSD3.esp = NULL;
#endif

#if STM32_SERIAL_USE_UART4
  chBSemObjectInit(&MSD4.msem, false);
  MSD4.sd = &SD4;
  MSD4.esp = NULL;
#endif

#if STM32_SERIAL_USE_UART5
  chBSemObjectInit(&MSD5.msem, false);
  MSD5.sd = &SD5;
  MSD5.esp = NULL;
#endif

#if STM32_SERIAL_USE_USART6
  chBSemObjectInit(&MSD6.msem, false);
  MSD6.sd = &SD6;
  MSD6.esp = NULL;
#endif

#if STM32_SERIAL_USE_UART7
  chBSemObjectInit(&MSD7.msem, false);
  MSD7.sd = &SD7;
  MSD7.esp = NULL;
#endif

#if STM32_SERIAL_USE_UART8
  chBSemObjectInit(&MSD8.msem, false);
  MSD8.sd = &SD8;
  MSD8.esp = NULL;
#endif

#if STM32_HAS_LPUART1
#if STM32_SERIAL_USE_LPUART1
  chBSemObjectInit(&MLPSD1.msem, false);
  MLPSD1.sd = &LPSD1;
  MLPSD1.esp = NULL;
#endif
#endif
}

/**
 * @brief Connect to multiplexed serial channel
 * @note  Only supports UART/USART based serial channels
 *
 * @param[in] smd       pointer to serial multiplexer driver.
 * @param[in] sdcfg     pointer to the serial (SD) configuration.
 * @param[in] mcfg      pointer the multiplex serial configuration.
 * @param[in] timeout   the number of ticks before the operation times out.
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 *
 * @return              A message specifying the result of the operation.
 * @retval MSG_OK       if the serial channel has been connected.
 * @retval MSG_RESET    if the multiplex control semaphore has been reset.
 * @retval MSG_TIMEOUT  if the multiplex control was not acquired within the
 *                      time out.
 *
 * @api
 */
msg_t pktOpenMuxedSerial(SerialMuxDriver *const smd,
                         const SerialConfig *sdcfg,
                         const SerialMuxConfig *mcfg,
                         const sysinterval_t timeout) {
  chDbgCheck(smd != NULL);
  chDbgCheck(mcfg != NULL);

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
 *        Data to be written when closing the serial channel can be specified.
 *        When data out is specified the End Of Transmission is monitored.
 *        The serial driver will be stopped at EOT or timeout waiting for EOT.
 *
 * @param[in] smd       pointer to serial multiplexer driver.
 * @param[in] out       pointer to data to be written to serial at close.
 *                      can be NULL (no data is written and EOT is not checked)
 * @param[in] len       the number of bytes to write to serial channel.
 * @param[in] timeout   the number of ticks before the operation times out.
 *                      If no data is to be written timeout does not apply.
 *                      The special values are handled as follow:
 *                      - @a TIME_INFINITE is allowed but interpreted as a
 *                        normal time specification.
 *                      - @a TIME_IMMEDIATE this value does not wait for the
 *                        serial transmission to end after writing data.
 *
 * @return  status of the operation
 * @retval  MSG_OK      Channel close successfully
 * @retval  MGS_TIMEOUT Channel did not get EOT within timeout after send
 *
 * @api
 */
msg_t pktCloseMuxedSerial(SerialMuxDriver *const smd,
                          const SerialMuxConfig *mcfg,
                          const uint8_t *out,
                          const size_t len,
                          const sysinterval_t timeout) {
  chDbgCheck(smd != NULL);
  chDbgCheck(mcfg != NULL);

  msg_t msg = MSG_OK;
  if (out != NULL) {
    smd->esp = chnGetEventSource((BaseAsynchronousChannel *)smd->sd);

    /* Register for CHN_TRANSMISSION_END. No thread event bit is selected.
       When the source broadcasts it will set no event bit in this thread. */
    chEvtRegisterMaskWithFlags(smd->esp, &smd->el, (eventmask_t)0,
                               CHN_TRANSMISSION_END);

    /* Write the data to the serial channel. */
    sdWrite(smd->sd, out, len);

    if (timeout != TIME_IMMEDIATE) {
      /* Wait for event or timeout. */
      (void)chEvtWaitAnyTimeout((eventmask_t)0, timeout);
      if ((chEvtGetAndClearFlags(&smd->el) | CHN_TRANSMISSION_END) == 0) {
        /* No CHN_TRANSMISSION_END flag set at source so this is a timeout. */
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
  /* Release multiplex port lock. */
  chBSemSignal(&smd->msem);
  return msg;
}

#endif /* HAL_USE_SERIAL */

/** @} */
