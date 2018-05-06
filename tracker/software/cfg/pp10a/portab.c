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
#include "pkttypes.h"
#include "portab.h"
#include "usb.h"
#include <stdarg.h>

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

typedef struct SysProviders {

} providers_t;

//binary_semaphore_t diag_out_sem;

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
  /* USART3 TX.       */
  palSetLineMode(LINE_USART3_TX, PAL_MODE_ALTERNATE(7));
  /* USART3 RX.       */
  palSetLineMode(LINE_USART3_RX, PAL_MODE_ALTERNATE(7));
}

void pktConfigSerialPkt(void) {

}

void pktSetLineModeICU(void) {
  palSetLineMode(LINE_ICU, PAL_MODE_INPUT | PAL_MODE_ALTERNATE(2));
}

void pktSerialStart(void) {
#if ENABLE_EXTERNAL_I2C == FALSE
  pktConfigSerialDiag();
  pktConfigSerialPkt();
  sdStart(SERIAL_CFG_DEBUG_DRIVER, &debug_config);
#endif
  /* Setup diagnostic resource access semaphore. */
  extern binary_semaphore_t diag_out_sem;
  chBSemObjectInit(&diag_out_sem, false);
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

void pktPowerUpRadio(radio_unit_t radio) {
  /* TODO: Implement hardware mapping. */
  (void)radio;
  /*
   * NOTE: RADIO_CS and RADIO_SDN pins are configured in board.h
   * RADIO_SDN is configured to open drain as it is pulled up on PCB by 100K.
   * The radio powers up in SDN mode.
   *
   * CS is set as push-pull and initialized to HIGH.
   */

  // Power up transceiver
  palClearLine(LINE_RADIO_SDN);   // Radio SDN low (power up transceiver)
  chThdSleep(TIME_MS2I(10));      // Wait for transceiver to power up
}

void pktPowerDownRadio(radio_unit_t radio) {
  /* TODO: Implement hardware mapping. */
  (void)radio;

  /*
   * Put radio in shutdown mode.
   * All registers are lost.
   */
  palSetLine(LINE_RADIO_SDN);
}

void sysConfigureCoreIO(void) {
  /* Setup SPI3. */
  palSetLineMode(LINE_SPI_SCK, PAL_MODE_ALTERNATE(6)
                 | PAL_STM32_OSPEED_HIGHEST);     // SCK
  palSetLineMode(LINE_SPI_MISO, PAL_MODE_ALTERNATE(6)
                 | PAL_STM32_OSPEED_HIGHEST);    // MISO
  palSetLineMode(LINE_SPI_MOSI, PAL_MODE_ALTERNATE(6)
                 | PAL_STM32_OSPEED_HIGHEST);    // MOSI

  /* Setup I2C1. */
  palSetLineMode(LINE_I2C_SDA, PAL_MODE_ALTERNATE(4)
                 | PAL_STM32_OSPEED_HIGHEST
                 | PAL_STM32_OTYPE_OPENDRAIN); // SDA
  palSetLineMode(LINE_I2C_SCL, PAL_MODE_ALTERNATE(4)
                 | PAL_STM32_OSPEED_HIGHEST
                 | PAL_STM32_OTYPE_OPENDRAIN); // SCL

  #if ACTIVATE_USB
  startUSB();
  #endif
}

/** @} */

