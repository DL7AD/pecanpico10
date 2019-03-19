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
#include "portab.h"

#if (HAL_USE_SERIAL && STM32_SERIAL_USE_MUX) || defined(__DOXYGEN__)

//#include "pktconf.h"

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
/* Driver local functions.                                                   */
/*===========================================================================*/

#if 0
/**
 * @brief   Low level serial multiplexer driver initialization.
 *
 * @notapi
 */
static SerialMuxDriver* smd_lld_get_driver(SerialDriver *sd) {

#if STM32_SERIAL_USE_USART1
  if (sd == &SD1) {
    return &MSD1;
  }
#endif

#if STM32_SERIAL_USE_USART2
  if (sd == &SD2) {
    return &MSD2;
  }
#endif

#if STM32_SERIAL_USE_USART3
  if (sd == &SD3) {
    return &MSD3;
  }
#endif

#if STM32_SERIAL_USE_UART4
  if (sd == &SD4) {
    return &MSD4;
  }
#endif

#if STM32_SERIAL_USE_UART5
  if (sd == &SD5) {
    return &MSD5;
  }
#endif

#if STM32_SERIAL_USE_USART6
  if (sd == &SD6) {
    return &MSD6;
  }
#endif

#if STM32_SERIAL_USE_UART7
  if (sd == &SD7) {
    return &MSD7;
  }
#endif

#if STM32_SERIAL_USE_UART8
  if (sd == &SD8) {
    return &MSD8;
  }
#endif

#if STM32_HAS_LPUART1
#if STM32_SERIAL_USE_LPUART1
  if (sd == &LPSD1) {
    return &MLPSD1;
  }
#endif
#endif
  return NULL;
}
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
 * @param[in] msd       pointer to serial multiplex driver.
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
 * @retval MSG_TIMEOUT  if multiplex control was not acquired within the
 *                      specified time out.
 *
 * @api
 */
msg_t pktOpenMuxedSerial(SerialMuxDriver *const msd,
                         const SerialMuxConfig *mcfg,
                         const sysinterval_t timeout) {
  chDbgCheck(msd != NULL);
  chDbgCheck(mcfg != NULL);

  /* Acquire the multiplexed serial channel. */
  msg_t msg = chBSemWaitTimeout(&msd->msem, timeout);
  if (msg != MSG_OK) {
    return msg;
  }

  /* Save the configuration. */
  msd->mcfg = mcfg;

  /* Configure the GPIO to map to the multiplexed UART. */
  if (mcfg->open.rx.line != PAL_NOLINE) {
    palSetLineMode(mcfg->open.rx.line, mcfg->open.rx.mode);
  }
  if (mcfg->open.tx.line != PAL_NOLINE) {
    palSetLineMode(mcfg->open.tx.line, mcfg->open.tx.mode);
  }
  sdStart(msd->sd, &mcfg->sdcfg);
  return MSG_OK;
}

/**
 * @brief Disconnect multiplexed serial channel
 * @note  Only supports UART based serial channels
 *        Data to be written when closing the serial channel can be specified.
 *        When data out is specified the End Of Transmission is monitored.
 *        The serial driver will be stopped at EOT or timeout waiting for EOT.
 *
 * @param[in] msd       pointer to serial multiplex driver.
 * @param[in] out       pointer to data to be written to serial at close.
 *                      can be NULL (no data is written and EOT is not checked)
 * @param[in] len       the number of bytes to write to serial channel.
 * @param[in] timeout   the number of ticks before the operation times out.
 *                      If no data is to be written timeout does not apply.
 *                      The special values are handled as follow:
 *                      - @a TIME_INFINITE is allowed but interpreted as a
 *                        normal time specification.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *
 * @return  status of the operation
 * @retval  MSG_OK      Channel closed successfully
 * @retval  MGS_TIMEOUT Channel did not get EOT within timeout after send or
 *                      TIME_IMMEDIATE was specified for timeout.
 *                      Serial channel will be closed and any queued data in
 *                      for serial channel will not be sent.
 *
 * @api
 */
msg_t pktCloseMuxedSerial(SerialMuxDriver *const msd,
                          const uint8_t *out,
                          const size_t len,
                          const sysinterval_t timeout) {
  chDbgCheck(msd != NULL);

  msg_t msg = MSG_OK;
  if (out != NULL && len != 0) {
    /* Data to be written. */
    if (timeout != TIME_IMMEDIATE) {
      msd->esp = chnGetEventSource((BaseAsynchronousChannel *)msd->sd);

      /* Register for CHN_TRANSMISSION_END. No thread event bit is selected.
         When the source broadcasts it will set no event bit in this thread. */
      chEvtRegisterMaskWithFlags(msd->esp, &msd->el, (eventmask_t)0,
                                 CHN_TRANSMISSION_END);

      /* Write the data to the serial channel and wait for EOT. */
      sdWrite(msd->sd, out, len);

      /* Wait for event or timeout. */
      (void) chEvtWaitAnyTimeout((eventmask_t)0, timeout);
      if ((chEvtGetAndClearFlags(&msd->el) | CHN_TRANSMISSION_END) == 0) {
        /* No CHN_TRANSMISSION_END flag set at source so this is a timeout. */
        msg = MSG_TIMEOUT;
      }
      chEvtUnregister(msd->esp, &msd->el);
    } else {
      msg = MSG_TIMEOUT;
    }
  }

  /* Stop serial driver. */
  sdStop(msd->sd);

  /* Remove the GPIO map to the UART. */
  if (msd->mcfg->close.rx.line != PAL_NOLINE) {
    if (msd->mcfg->close.rx.mode == PAL_MODE_RESET) {
      palSetLineMode(msd->mcfg->close.rx.line, PAL_MODE_INPUT);
    } else {
      palSetLineMode(msd->mcfg->close.rx.line, msd->mcfg->close.rx.mode);
    }
  }
  if (msd->mcfg->close.tx.line != PAL_NOLINE) {
    if (msd->mcfg->close.tx.mode == PAL_MODE_RESET) {
      palSetLine(msd->mcfg->close.tx.line);
      palSetLineMode(msd->mcfg->close.tx.line, PAL_MODE_OUTPUT_PUSHPULL);
    } else {
      palSetLineMode(msd->mcfg->close.tx.line, msd->mcfg->close.tx.mode);
    }
  }

  /* Release multiplex port lock. */
  msd->mcfg = NULL;
  chBSemSignal(&msd->msem);
  return msg;
}

/*
 *
 */
void msdStart(SerialMuxDriver *msd, SerialMuxConfig *msdcfg) {
  (void) msd;
  (void) msdcfg;
}

/*
 *
 */
void msdStop(SerialMuxDriver *msd) {
  (void) msd;
}

#endif /* HAL_USE_SERIAL */

/** @} */
