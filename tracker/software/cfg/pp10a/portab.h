/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef PORTAB_H_
#define PORTAB_H_

#include "usbcfg2.h"


/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/* Board and hardware capabilities settings. */
#define PKT_HARDWARE_SUPPORTS_CAM   TRUE
#define PKT_HARDWARE_SUPPORTS_USB   TRUE
#define PKT_HARDWARE_SUPPORTS_PWR   TRUE

#define PKT_MEMORY_USE_CCM          TRUE

/* To use IO_TXD/IO_RXD as a UART serial channel. */
#define ENABLE_UART_SERIAL          TRUE

/* To direct diagnostic to a serial channel. */
#define ENABLE_SERIAL_STREAM        FALSE

/* To direct trace to a serial channel. */
#define ENABLE_SERIAL_TRACE         TRUE

/* If set to true, the console will be started. */
#define ACTIVATE_CONSOLE            TRUE

/* Console selection. */
#if !defined(USE_UART_FOR_CONSOLE)
#define USE_UART_FOR_CONSOLE        FALSE
#endif

/* The external port can be used for bit bang I2C. */
#define ENABLE_EXTERNAL_I2C         FALSE

/* External BME fitting setting. */
#define BME280_E1_IS_FITTED         FALSE
#define BME280_E2_IS_FITTED         TRUE

/*
 *  Configure PWM stream data.
 *  Packed 12 bit saves memory but has reduced PWM range.
 */
#define USE_12_BIT_PWM               TRUE

/*
 * Allocate PWM buffers from a CCM heap/pool.
 * Implements fragmented queue/buffer objects.
 * PWM side swaps in new queue/buffer as each fills with PWM stream from radio.
 * Decoder side swaps queue/buffer on in-band message.
 * The retired buffer is reticulated to the pool ready for re-use.
 */
#define USE_CCM_BASED_PWM_HEAP      TRUE
#define TRACE_PWM_BUFFER_STATS      FALSE

/* Test of re-assigning CCA and NIRQ on radio. */
#define USE_GPIO0_OF_RADIO_FOR_CCA  TRUE
#define USE_NIRQ_OF_RADIO_FOR_NIRQ  TRUE

#if PKT_HARDWARE_SUPPORTS_USB
#include "usbcfg2.h"
#endif

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*
 * Serial port definitions
 */
#define SERIAL_UART_DRIVER          SD3
#define SERIAL_USB1_DRIVER          SDU1
#define SERIAL_USB2_DRIVER          SDU2

/* Serial channel mapping. */
#if USE_UART_FOR_CONSOLE == TRUE
#define SERIAL_CONSOLE_DRIVER       SERIAL_UART_DRIVER
#define SERIAL_TRACE_DRIVER         SERIAL_USB2_DRIVER
#else
#define SERIAL_CONSOLE_DRIVER       SERIAL_USB2_DRIVER
#define SERIAL_TRACE_DRIVER         SERIAL_UART_DRIVER
#endif
#define SERIAL_DEBUG_DRIVER         SERIAL_UART_DRIVER
#define SERIAL_STREAM_DRIVER        SERIAL_USB1_DRIVER

/*
 * SPI definitions
 */
#define SPI_BUS1_DRIVER             SPID3

/*
 * Radio SPI definitions.
 */
#define PKT_RADIO1_SPI              SPI_BUS1_DRIVER

/**
 * I2C definitions
 */
#define I2C_BUS1_DRIVER             I2CD1

/**
 * UBLOX IO definition
 */
#define PKT_GPS_I2C                 I2C_BUS1_DRIVER
#define PKT_GPS_UART                SD5

/**
 * OV5640 I2C definition
 */
#define PKT_CAM_I2C                 I2C_BUS1_DRIVER

/**
 * BME280 I2C definition
 */
#define PKT_BMEI_I2C                I2C_BUS1_DRIVER
#define PKT_BMEE_I2C                NULL

// Camera pins
#define LINE_CAM_XCLK               PAL_LINE(GPIOC, 9U)
#define LINE_CAM_PCLK               PAL_LINE(GPIOC, 6U)
#define LINE_CAM_VSYNC              PAL_LINE(GPIOC, 5U)
#define LINE_CAM_D2                 PAL_LINE(GPIOA, 0U)
#define LINE_CAM_D3                 PAL_LINE(GPIOA, 1U)
#define LINE_CAM_D4                 PAL_LINE(GPIOA, 2U)
#define LINE_CAM_D5                 PAL_LINE(GPIOA, 3U)
#define LINE_CAM_D6                 PAL_LINE(GPIOA, 4U)
#define LINE_CAM_D7                 PAL_LINE(GPIOA, 5U)
#define LINE_CAM_D8                 PAL_LINE(GPIOA, 6U)
#define LINE_CAM_D9                 PAL_LINE(GPIOA, 7U)
#define LINE_CAM_EN                 PAL_LINE(GPIOC, 7U)
#define LINE_CAM_RESET              PAL_LINE(GPIOB, 0U)

// SD Card pins
#define LINE_SD_CS                  PAL_LINE(GPIOC, 0U)
#define LINE_SD_DET                 PAL_LINE(GPIOC, 8U)

// ADC
#define LINE_ADC_VSOL               PAL_LINE(GPIOC, 2U)
#define LINE_ADC_VBAT               PAL_LINE(GPIOB, 1U)
#define LINE_ADC_VUSB               PAL_LINE(GPIOC, 4U)

// USB
#define LINE_USB_ID                 PAL_LINE(GPIOA, 10U)
#define LINE_USB_VBUS               PAL_LINE(GPIOA,  9U)
#define LINE_USB_DM                 PAL_LINE(GPIOA, 11U)
#define LINE_USB_DP                 PAL_LINE(GPIOA, 12U)

// LED
#define LINE_IO_BLUE                PAL_LINE(GPIOC, 1U)
#define LINE_IO_GREEN               PAL_LINE(GPIOC, 3U)

// I2C
#define LINE_I2C_SCL                PAL_LINE(GPIOB, 8U)
#define LINE_I2C_SCL_MODE           (PAL_MODE_ALTERNATE(4) |                 \
                                     PAL_STM32_OSPEED_HIGHEST |              \
                                     PAL_STM32_OTYPE_OPENDRAIN)
#define LINE_I2C_SDA                PAL_LINE(GPIOB, 9U)
#define LINE_I2C_SDA_MODE           (PAL_MODE_ALTERNATE(4) |                 \
                                     PAL_STM32_OSPEED_HIGHEST |              \
                                     PAL_STM32_OTYPE_OPENDRAIN)

// GPS
#define LINE_GPS_EN                 PAL_LINE(GPIOA, 15U)
#define LINE_GPS_RESET              PAL_LINE(GPIOB, 14U)
#define LINE_GPS_TXD                PAL_LINE(GPIOB, 13U)
#define LINE_GPS_TXD_MODE           PAL_MODE_ALTERNATE(11)
#define LINE_GPS_RXD                PAL_LINE(GPIOB, 12U)
#define LINE_GPS_RXD_MODE           PAL_MODE_ALTERNATE(11)
#define LINE_GPS_TIMEPULSE          PAL_LINE(GPIOB, 15U)

// IO
#define LINE_GPIO_PIN               PAL_LINE(GPIOA, 8U)
#define LINE_IO_TXD                 PAL_LINE(GPIOB, 10U)
#define LINE_IO_RXD                 PAL_LINE(GPIOC, 11U)

// APRS IO lines
#define LINE_IO1                    LINE_GPIO_PIN
#define LINE_IO2                    LINE_IO_TXD
#define LINE_IO3                    LINE_IO_RXD
#define LINE_IO4                    PAL_NOLINE
#define LINE_IO5                    PAL_NOLINE
#define LINE_IO6                    PAL_NOLINE
#define LINE_IO7                    PAL_NOLINE
#define LINE_IO8                    PAL_NOLINE

/*
 * Radio GPIO definitions.
 */
#define LINE_RADIO1_CS              PAL_LINE(GPIOC, 12U)
#define LINE_RADIO1_SDN             PAL_LINE(GPIOC, 10U)
#define LINE_RADIO1_NIRQ            PAL_LINE(GPIOD, 2U)
#define LINE_RADIO1_GPIO0           PAL_LINE(GPIOB, 7U)
#define LINE_RADIO1_GPIO1           PAL_LINE(GPIOB, 6U)

// SPI
#define LINE_SPI_SCK                PAL_LINE(GPIOB, 3U)
#define LINE_SPI_SCK_MODE           (PAL_MODE_ALTERNATE(6) |                 \
                                     PAL_STM32_OSPEED_HIGHEST)
#define LINE_SPI_MISO               PAL_LINE(GPIOB, 4U)
#define LINE_SPI_MISO_MODE          (PAL_MODE_ALTERNATE(6) |                 \
                                     PAL_STM32_OSPEED_HIGHEST)
#define LINE_SPI_MOSI               PAL_LINE(GPIOB, 5U)
#define LINE_SPI_MOSI_MODE          (PAL_MODE_ALTERNATE(6) |                 \
                                     PAL_STM32_OSPEED_HIGHEST)

/* TODO: Move into pktradio.h? */
#define BAND_MIN_2M_FREQ	    	144000000				/* Minimum allowed frequency in Hz */
#define BAND_MAX_2M_FREQ			148000000				/* Maximum allowed frequency in Hz */
#define BAND_STEP_2M_HZ             12500
#define BAND_DEF_2M_APRS            144800000               /* Default frequency in Hz.        */
#define BAND_MIN_70CM_FREQ          420000000               /* Minimum allowed frequency in Hz */
#define BAND_MAX_70CM_FREQ          450000000               /* Maximum allowed frequency in Hz */
#define BAND_STEP_70CM_HZ           12500
#define BAND_DEF_70CM_APRS          439100000               /* Default frequency in Hz.        */

#define DEFAULT_OPERATING_FREQ      144800000
#if DEFAULT_OPERATING_FREQ < BAND_MIN_2M_FREQ
#error "Default operating frequency must be an absolute value in Hz"
#endif

/* TCXO calibrator setup. */
#define PKT_TCXO_TIMER              ICUD12
#define PKT_TCXO_TIMER_CLOCK        STM32_TIMCLK1
#define PKT_TCXO_TIMER_CHANNEL      ICU_CHANNEL_2
#define PKT_TCXO_CLOCK              STM32_HSECLK
#define PKT_TCXO_DEFAULT_ERROR_HZ   650      /**< Manual error adjust in Hz  */



/* Si446x clock setup. */
#define Si446x_CLK					PKT_TCXO_CLOCK          /* Clock in Hz */
#define Si446x_CLK_ERROR			PKT_TCXO_DEFAULT_ERROR_HZ  /* Error in Hz */
#define Si446x_CLK_TCXO_EN			true                    /* Xtal or external. */
#define Si446x_XO_TUNE              0x40                    /* Xtal trim. */

/* LED status indicators (set to PAL_NOLINE if not available). */
#define LINE_OVERFLOW_LED           PAL_NOLINE
#define LINE_DECODER_LED            LINE_IO_BLUE
#define LINE_SQUELCH_LED            PAL_NOLINE
#define LINE_NO_FIFO_LED            PAL_NOLINE
#define LINE_NO_BUFF_LED            PAL_NOLINE
#define LINE_PWM_ERROR_LED          PAL_NOLINE

/* Diagnostic PWM mirror port. */
#define LINE_PWM_MIRROR             PAL_NOLINE

//#define LINE_UART4_TX               PAL_LINE(GPIOA, 12U)
//#define LINE_UART4_RX               PAL_LINE(GPIOA, 11U)


#define EI2C_SCL                    LINE_IO_TXD /* SCL */
#define EI2C_SDA                    LINE_IO_RXD /* SDA */

#if ENABLE_EXTERNAL_I2C == TRUE && ENABLE_UART_SERIAL == TRUE
#error "Cannot enable serial debug and external I2C together"
#elif ENABLE_EXTERNAL_I2C == FALSE && ENABLE_UART_SERIAL == TRUE
#define LINE_USART3_TX              LINE_IO_TXD
#define LINE_USART3_RX              LINE_IO_RXD
#endif

/**
 *  PWM ICU related definitions.
 */
#define PKT_RADIO1_PWM_ICU              ICUD4
#if    ((defined(ICUD1) && PKT_RADIO1_PWM_ICU == ICUD1))                     \
    || ((defined(ICUD8) && PKT_RADIO1_PWM_ICU == ICUD8))                     \
    || ((defined(ICUD9) && PKT_RADIO1_PWM_ICU == ICUD9))
/**
 * Clock of ICU timers connected to APB2 (Timers 1, 8, 9).
 */

#define PWM_ICU_RADIO1_CLK              STM32_TIMCLK2
#else
/**
 * Clock of ICU timers connected to APB1 (Timers 2, 3, 4, 5, 6, 7, 12).
 */
#define PWM_ICU_RADIO1_CLK              STM32_TIMCLK1

#endif

/* ICU counter frequency. */
/*
 * TODO: This should be calculated using timer clock.
 * ICU has to run at an integer divide from APBx clock.
 */
#if USE_12_BIT_PWM
#define PWM_ICU_COUNT_FREQUENCY         2000000U
#else
#define PWM_ICU_COUNT_FREQUENCY         6000000U
#endif

#if ((PWM_ICU_RADIO1_CLK % PWM_ICU_COUNT_FREQUENCY) != 0)
#error "Invalid ICU frequency for APBx clock setting"
#endif

/* Definitions for ICU FIFO implemented using chfactory. */
/* Use factory FIFO as stream control with separate chained PWM buffers. */
#define NUMBER_PWM_FIFOS            5U
/* Number of PWM data entries (stream symbols) per queue object. */
#define PWM_DATA_SLOTS              200
/* Number of PWM queue objects in total. */
#define PWM_DATA_BUFFERS            30


/* Number of frame receive buffers. */
#define NUMBER_RX_PKT_BUFFERS       5U
#define USE_CCM_HEAP_RX_BUFFERS     TRUE

/*
 * Number of general AX25/APRS processing & frame send buffers.
 * Can be configured as being in CCM.
 */
#define NUMBER_COMMON_PKT_BUFFERS       30U
#define RESERVE_BUFFERS_FOR_INTERNAL    10U
#if (NUMBER_COMMON_PKT_BUFFERS - RESERVE_BUFFERS_FOR_INTERNAL) < 2
#error "Insufficient buffers available for send."
#endif
#define MAX_BUFFERS_FOR_BURST_SEND      5U
#if (MAX_BUFFERS_FOR_BURST_SEND >                                            \
    (NUMBER_COMMON_PKT_BUFFERS - RESERVE_BUFFERS_FOR_INTERNAL))
#error "Can not allocate requested buffers for burst send"
#endif

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

#define PKT_SVC_USE_RADIO1  TRUE
#define PKT_SVC_USE_RADIO2  FALSE

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern const radio_band_t band_2m;
extern const radio_band_t band_70cm;
extern const radio_config_t radio_list[];
extern packet_svc_t RPKTD1;
extern AFSKDemodDriver AFSKD1;
extern SerialUSBDriver SDU1;
extern SerialUSBDriver SDU2;

#ifdef __cplusplus
extern "C" {
#endif
  void      pktConfigSerialDiag(void);
  void      pktConfigSerialPkt(void);
  void      pktConfigureCoreIO(void);
  //ioline_t  pktSetLineModeICU(const radio_unit_t radio);
  //ioline_t  pktSetLineModeRadioGPIO1(const radio_unit_t radio);
  //ioline_t  pktSetLineModeRadioGPIO0(const radio_unit_t radio);
  void      pktSerialStart(void);
  void      strmWrite(uint8_t level, uint8_t *buf, uint32_t len);
  int       strmPrintf(uint8_t level, const char *format, ...);
  void      pktWrite(uint8_t *buf, uint32_t len);
  uint8_t   pktReadIOlines(void);
  void      pktRadioICUWidth(ICUDriver *icup);
  void      pktRadioICUPeriod(ICUDriver *icup);
  void      pktRadioICUOverflow(ICUDriver *icup);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/


#endif /* PORTAB_H_ */

/** @} */
