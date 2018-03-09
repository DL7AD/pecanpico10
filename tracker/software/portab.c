/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/


/**
 * @file    portab.c
 * @brief   Application portability module code.
 *
 * @addtogroup application_portability
 * @{
 */

#include "hal.h"
#include "chprintf.h"
#include "portab.h"
#include <stdarg.h>

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

binary_semaphore_t callback_sem;

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/


/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

const SerialConfig debug_config = {
  115200,
  0,
  0,
  0
};

void pktConfigSerialDiag(void) {
#if ENABLE_EXTERNAL_I2C == FALSE
  /* USART3 TX.       */
  palSetLineMode(LINE_USART3_TX, PAL_MODE_ALTERNATE(7));
  /* USART3 RX.       */
  palSetLineMode(LINE_USART3_RX, PAL_MODE_ALTERNATE(7));
#endif
}

void pktConfigSerialPkt(void) {

}

void pktSetLineModeICU(void) {
  palSetLineMode(LINE_ICU, PAL_MODE_INPUT | PAL_MODE_ALTERNATE(2));
}

void pktSerialStart(void) {
  pktConfigSerialDiag();
  pktConfigSerialPkt();
#if ENABLE_EXTERNAL_I2C == FALSE
  sdStart(SERIAL_CFG_DEBUG_DRIVER, &debug_config);
#endif
  //sdStart(SERIAL_CFG_PACKET_DRIVER, &debug_config);

  /* Setup diagnostic resource access semaphore. */
  chBSemObjectInit(&callback_sem, false);
}

void dbgWrite(uint8_t level, uint8_t *buf, uint32_t len) {
  (void)level;
#if ENABLE_EXTERNAL_I2C == FALSE
  chnWrite((BaseSequentialStream*)SERIAL_CFG_DEBUG_DRIVER, buf, len);
#else
  (void)buf;
  (void)len;
#endif
}

int dbgPrintf(uint8_t level, const char *format, ...) {
  (void)level;
#if ENABLE_EXTERNAL_I2C == FALSE
  va_list arg;
  int done;

  va_start(arg, format);
  done = chprintf((BaseSequentialStream*)SERIAL_CFG_DEBUG_DRIVER, format, arg);
  va_end(arg);

  return done;
#else
  (void)format;
  return 0;
#endif
}

void pktWrite(uint8_t *buf, uint32_t len) {
  chnWrite((BaseSequentialStream*)SERIAL_CFG_DEBUG_DRIVER, buf, len);
}

void pktConfigureRadioGPIO() {
  // Configure Radio pins
  palSetLineMode(LINE_SPI_SCK, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST);     // SCK
  palSetLineMode(LINE_SPI_MISO, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST);    // MISO
  palSetLineMode(LINE_SPI_MOSI, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST);    // MOSI
  palSetLineMode(LINE_RADIO_CS, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); // RADIO CS
  palSetLineMode(LINE_RADIO_SDN, PAL_MODE_OUTPUT_PUSHPULL);                           // RADIO SDN

  // Pull CS HIGH
  palSetLine(LINE_RADIO_CS);

  // Reset radio
  palSetLine(LINE_RADIO_SDN);
  chThdSleep(TIME_MS2I(10));

  // Power up transceiver
  palClearLine(LINE_RADIO_SDN);   // Radio SDN low (power up transceiver)
  chThdSleep(TIME_MS2I(10));      // Wait for transceiver to power up
}

void pktDeconfigureRadioGPIO() {

  palSetLineMode(LINE_SPI_SCK, PAL_MODE_INPUT_PULLDOWN);      // SCK
  palSetLineMode(LINE_SPI_MISO, PAL_MODE_INPUT_PULLDOWN);     // MISO
  palSetLineMode(LINE_SPI_MOSI, PAL_MODE_INPUT_PULLDOWN);     // MOSI
  palSetLineMode(LINE_RADIO_CS, PAL_MODE_INPUT_PULLDOWN);     // RADIO CS
  palSetLineMode(LINE_RADIO_SDN, PAL_MODE_INPUT_PULLDOWN);    // RADIO SDN

}

/** @} */

