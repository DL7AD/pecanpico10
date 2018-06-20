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
#include "types.h"
#include <stdarg.h>

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

const radio_band_t band_2m = {
  .start    = BAND_MIN_2M_FREQ,
  .end      = BAND_MAX_2M_FREQ,
  .step     = BAND_STEP_2M_HZ,
  .def_aprs = BAND_DEF_2M_APRS
};

const radio_band_t band_70cm = {
  .start    = BAND_MIN_70CM_FREQ,
  .end      = BAND_MAX_70CM_FREQ,
  .step     = BAND_STEP_70CM_HZ,
  .def_aprs = BAND_DEF_70CM_APRS
};

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

typedef struct SysProviders {

} providers_t;

const radio_config_t radio_list[NUM_PKT_RADIOS] = {
  { /* Radio #1 */
    .unit = PKT_RADIO_1,
    .type = SI4464,
    .band = {
             (radio_band_t * const)&band_2m,
              NULL
            }
  } /* End radio1 */
};


const SerialConfig debug_config = {
  115200,
  0,
  0,
  0
};

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

void pktConfigSerialDiag(void) {
  /* USART3 TX.       */
  palSetLineMode(LINE_USART3_TX, PAL_MODE_ALTERNATE(7));
  /* USART3 RX.       */
  palSetLineMode(LINE_USART3_RX, PAL_MODE_ALTERNATE(7));
}

void pktConfigSerialPkt(void) {

}

/**
 * TODO: Move this into pktconf.h and use general GPIO to setup.
 */
void pktSetLineModeICU(void) {
  palSetLineMode(LINE_ICU, PAL_MODE_INPUT | PAL_MODE_ALTERNATE(2));
}

/*
 * Read GPIO that are used for:
 * a) general use or
 * b) UART and s/w I2C external.
 *
 * @return State of lines regardless of general or specific use.
 */
uint8_t pktReadIOlines() {
  return palReadLine(LINE_GPIO_PIN)
      | palReadLine(LINE_IO_TXD) << 1
      | palReadLine(LINE_IO_RXD) << 2;
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

/*
 * Return a single radio parameter record pointer
 * The radio parameter picks a single records.
 * The current system does not work if the same radio is listed multiple times.
 * TODO: Have an enumeration and check radio array on startup.
 */
radio_config_t *pktGetRadioParameters(radio_unit_t radio) {
  (void)radio;
  return NULL;
}

/*
 *
 */
radio_freq_t pktCheckAllowedFrequency(radio_unit_t radio, radio_freq_t freq) {
  /* Check validity. */
  uint8_t radios = NUM_PKT_RADIOS/*sizeof(radio_list) / sizeof(radio_param_t)*/;
  for(uint8_t i = 0; i < radios; i++) {
    if(radio_list[i].unit != radio)
      continue;
    for(uint8_t x = 0; x < NUM_BANDS_PER_RADIO; x++) {
      if(radio_list[i].band[x] == NULL)
        /* No more bands in this radio. */
          return FREQ_RADIO_INVALID;
      if(radio_list[i].band[x]->start <= freq
          && freq < radio_list[i].band[x]->end)
        return freq;
    } /* End for bands */
  } /* End for radios*/
  return FREQ_RADIO_INVALID;
}

/** @} */

