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
#include "console.h"
#include "types.h"
#include "si446x.h"
#include "pktradio.h"
#include <stdarg.h>

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   Configuration for radio(s) on this board.
 * @details The hardware assigned to a radio is defined.
 * @details The mapping between the radio and MCU is defined.
 */
const si446x_mcucfg_t radio1_cfg = {
  .gpio0    = LINE_RADIO1_GPIO0,
  .gpio1    = LINE_RADIO1_GPIO1,
  .gpio2    = PAL_NOLINE,
  .gpio3    = PAL_NOLINE,
  .nirq     = LINE_RADIO1_NIRQ,
  .sdn      = LINE_RADIO1_SDN,
  .cs       = LINE_RADIO1_CS,
  .spi      = &PKT_RADIO1_SPI,
  .init     = {
    .gpio       = {
     .gpio0 = 00,          /**< DONOTHING. */
     .gpio1 = 00,          /**< DONOTHING. */
     .gpio2 = 0x21,        /**< RX_STATE. */
     .gpio3 = 0x20,        /**< TX_STATE. */
     .nirq  = 02,          /**< DRIVE0. */
     .sdo   = 00,          /**< DONOTHING. */
     .cfg   = 00           /**< HIGH DRIVE. */
    }
  },
  .rafsk    = {
    .gpio       = {
      .gpio0 = 00,          /**< DONOTHING. */
      .gpio1 = 0x15,        /**< RAW_RX_DATA. */
      .gpio2 = 00,          /**< DONOTHING. */
      .gpio3 = 00,          /**< DONOTHING. */
      .nirq  = 0x1B,        /**< CCA. */
      .sdo   = 00,          /**< DONOTHING. */
      .cfg   = 00           /**< HIGH DRIVE. */
    },
    .pwm     = {
               .line = &radio1_cfg.gpio1,
               .mode = (PAL_MODE_INPUT | PAL_MODE_ALTERNATE(2))
    },
    .cca     = {
               .line = &radio1_cfg.nirq,
               .mode = PAL_MODE_INPUT_PULLUP
    },
    .icu     = &PKT_RADIO1_PWM_ICU,
    .cfg     = {
       ICU_INPUT_ACTIVE_HIGH,
       PWM_ICU_COUNT_FREQUENCY,   /**< ICU clock frequency. */
       pktRadioICUWidth,          /**< ICU width callback. */
       pktRadioICUPeriod,         /**< ICU period callback. */
       pktRadioICUOverflow,       /**< ICU overflow callback. */
       ICU_CHANNEL_1,             /**< Timer channel. */
       0                          /**< DIER bits. */
    }
  },
  .tafsk    = {
    .gpio       = {
     .gpio0 = 00,          /**< DONOTHING. */
     .gpio1 = 00,          /**< DONOTHING. */
     .gpio2 = 00,          /**< DONOTHING. */
     .gpio3 = 00,          /**< DONOTHING. */
     .nirq  = 0x1B,        /**< CCA. */
     .sdo   = 00,          /**< DONOTHING. */
     .cfg   = 00           /**< HIGH DRIVE. */
    },
    .cca     = {
               .line = &radio1_cfg.nirq,
               .mode = PAL_MODE_INPUT_PULLUP
    },
  },
  .r2fsk    = {
    .gpio       = {
     .gpio0 = 00,          /**< DONOTHING. */
     .gpio1 = 00,          /**< DONOTHING. */
     .gpio2 = 00,          /**< DONOTHING. */
     .gpio3 = 00,          /**< DONOTHING. */
     .nirq  = 0x1B,        /**< CCA. */
     .sdo   = 00,          /**< DONOTHING. */
     .cfg   = 00           /**< HIGH DRIVE. */
    },
    .cca     = {
               .line = &radio1_cfg.nirq,
               .mode = PAL_MODE_INPUT_PULLUP
    },
  },
  .t2fsk    = {
    .gpio       = {
     .gpio0 = 00,          /**< DONOTHING. */
     .gpio1 = 00,          /**< DONOTHING. */
     .gpio2 = 00,          /**< DONOTHING. */
     .gpio3 = 00,          /**< DONOTHING. */
     .nirq  = 0x1B,        /**< CCA. */
     .sdo   = 00,          /**< DONOTHING. */
     .cfg   = 00           /**< HIGH DRIVE. */
    },
    .cca     = {
               .line = &radio1_cfg.nirq,
               .mode = PAL_MODE_INPUT_PULLUP
    },
  }
};

/* Variable data for a radio. */
si446x_data_t radio1_dat = {
        .lastTemp = 0x7FFF,
        .radio_clock = Si446x_CLK + Si446x_CLK_ERROR,
        .radio_part = 0,
        .radio_rom_rev = 0,
        .radio_patch = 0
};


/* List of bands in this radio. */
const radio_band_t *const radio1_bands[] = {
                (radio_band_t *const)&band_2m,
                 NULL
};

/* List of indicators controlled by this radio. */
const indicator_io_t radio1_ind[] = {
  {
   .ind = PKT_INDICATOR_DECODE,
   .type = PKT_IND_GPIO_LINE,
   .address.line = LINE_IO_BLUE,
   .driver.mode = PAL_MODE_OUTPUT_PUSHPULL
  },
  {
   .ind = PKT_INDICATOR_NONE
  }
};

/* Radios on this board. */
const radio_config_t radio_list[] = {
  { /* Radio #1 */
    .unit       = PKT_RADIO_1,
    .type       = SI446X,
    .pkt        = (packet_svc_t *const)&RPKTD1,
    .afsk       = (AFSKDemodDriver *const)&AFSKD1,
    .cfg        = (si446x_mcucfg_t *const)&radio1_cfg,
    .dat        = (si446x_data_t *)&radio1_dat,
    .bands      = (radio_band_t **const)radio1_bands,
    .ind_set    = (indicator_io_t *const)radio1_ind
  }, /* End radio1 */
  {
     .unit = PKT_RADIO_NONE
  }
};

/**
 * Debug serial port setting.
 */
const SerialConfig debug_config = {
  230400,
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

/*
 * Read GPIO that are used for:
 * a) general use
 *  or
 * b) UART and s/w I2C external.
 *
 * @return State of lines regardless of general or specific use.
 */
uint8_t pktReadIOlines() {
  return palReadLine(LINE_GPIO_PIN1)
      | palReadLine(LINE_IO_TXD) << 1
      | palReadLine(LINE_IO_RXD) << 2
      | palReadLine(LINE_GPIO_PIN2);
}

void pktSerialStart(void) {
#if ENABLE_SERIAL_DEBUG == TRUE
  pktConfigSerialDiag();
  sdStart(&SERIAL_DEBUG_DRIVER, &debug_config);
#endif
  /* Setup diagnostic resource access semaphore. */
  //extern binary_semaphore_t debug_out_sem;
  //chBSemObjectInit(&debug_out_sem, false);
}

void dbgWrite(uint8_t level, uint8_t *buf, uint32_t len) {
  (void)level;
#if ENABLE_SERIAL_DEBUG == TRUE
  chnWrite((BaseSequentialStream*)&SERIAL_DEBUG_DRIVER, buf, len);
#else
  (void)buf;
  (void)len;
#endif
}

int dbgPrintf(uint8_t level, const char *format, ...) {
  (void)level;
#if ENABLE_SERIAL_DEBUG == TRUE
  va_list arg;
  int done;

  va_start(arg, format);
  done = chvprintf((BaseSequentialStream*)&SERIAL_DEBUG_DRIVER, format, arg);
  va_end(arg);

  return done;
#else
  (void)format;
  return 0;
#endif
}

void pktWrite(uint8_t *buf, uint32_t len) {
#if ENABLE_SERIAL_DEBUG == TRUE
  chnWrite((BaseSequentialStream*)&SERIAL_DEBUG_DRIVER, buf, len);
#else
  (void)buf;
  (void)len;
#endif
}

void pktConfigureCoreIO(void) {
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

}

/** @} */

