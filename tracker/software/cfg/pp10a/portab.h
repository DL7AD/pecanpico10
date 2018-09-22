/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef PORTAB_H_
#define PORTAB_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*
 * Serial port definitions
 */
#define SERIAL_DEBUG_DRIVER		    SD3
#define SERIAL_CONSOLE_DRIVER       SDU1

/*
 * SPI definitions
 */
#define SPI_BUS1_DRIVER             &SPID3

/*
 * Radio SPI definitions.
 */
#define PKT_RADIO1_SPI              SPI_BUS1_DRIVER

/**
 * I2C definitions
 */
#define I2C_BUS1_DRIVER             &I2CD1

/**
 * UBLOX I2C definition
 */
#define PKT_GPS_I2C                 I2C_BUS1_DRIVER

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
#define LINE_I2C_SDA                PAL_LINE(GPIOB, 9U)

// GPS
#define LINE_GPS_EN                 PAL_LINE(GPIOA, 15U)
#define LINE_GPS_RESET              PAL_LINE(GPIOB, 14U)
#define LINE_GPS_TXD                PAL_LINE(GPIOB, 13U)
#define LINE_GPS_RXD                PAL_LINE(GPIOB, 12U)
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
#define LINE_SPI_MISO               PAL_LINE(GPIOB, 4U)
#define LINE_SPI_MOSI               PAL_LINE(GPIOB, 5U)

/* TODO: Move into pktradio.h */
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

/* Si446x clock setup. */
#define Si446x_CLK					STM32_HSECLK			/* Oscillator frequency in Hz */
#define Si446x_CLK_OFFSET			22						/* Oscillator frequency drift in ppm */
#define Si446x_CLK_TCXO_EN			true					/* Set this true, if a TCXO is used, false for XTAL */

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

/* The external port can be used for bit bang I2C. */
#define ENABLE_EXTERNAL_I2C         TRUE

#define EI2C_SCL                    LINE_IO_TXD /* SCL */
#define EI2C_SDA                    LINE_IO_RXD /* SDA */

/* To use IO_TXD/IO_RXD for UART debug channel. */
#define ENABLE_SERIAL_DEBUG         FALSE

#if ENABLE_EXTERNAL_I2C == TRUE && ENABLE_SERIAL_DEBUG == TRUE
#error "Cannot enable serial debug and external I2C together"
#elif ENABLE_EXTERNAL_I2C == FALSE && ENABLE_SERIAL_DEBUG == TRUE
#define LINE_USART3_TX              LINE_IO_TXD
#define LINE_USART3_RX              LINE_IO_RXD
#endif

/* If set to true, the console using USB interface will be switched on.
 * The tracker is also switched to 3V, because USB would not work at 1.8V.
 * Note that the transmission power is increased too when operating at 3V.
 * This option will also run the STM32 at 48MHz (AHB) permanently.
 * USB needs 48MHz speed to operate.
 */
#define ACTIVATE_CONSOLE            TRUE

/**
 *  ICU related definitions.
 */
#define PKT_RADIO1_ICU              &ICUD4

#define PWM_ICU_CLK                 STM32_TIMCLK1

/* ICU counter frequency. */
/*
 * TODO: This should be calculated using timer clock.
 * ICU has to run at an integer divide from APBx clock.
 */

#define ICU_COUNT_FREQUENCY         6000000U

#if ((PWM_ICU_CLK % ICU_COUNT_FREQUENCY) != 0)
#error "Invalid ICU frequency for APBx clock setting"
#endif

#define USE_12_BIT_PWM              FALSE

/*
 * Allocate PWM buffers from a CCM heap/pool.
 * Implements fragmented queue/buffer objects.
 * PWM side swaps in new queue/buffer as each fills with PWM stream from radio.
 * Decoder side swaps queue/buffer on in-band message.
 * The retired buffer is reticulated to the pool ready for re-use.
 */
#define USE_HEAP_PWM_BUFFER         TRUE
#define USE_CCM_BASED_PWM_HEAP      TRUE
#define TRACE_PWM_BUFFER_STATS      FALSE

/* Definitions for ICU FIFO implemented using chfactory. */
#if USE_HEAP_PWM_BUFFER == TRUE
/* Use factory FIFO as stream control with separate chained PWM buffers. */
#define NUMBER_PWM_FIFOS            5U
/* Number of PWM data entries per queue object. */
#define PWM_DATA_SLOTS              200
/* Number of PWM queue objects in total. */
#define PWM_DATA_BUFFERS            30
#else /* USE_HEAP_PWM_BUFFER != TRUE */
/* Use factory FIFO as stream control with integrated PWM buffer. */
#define NUMBER_PWM_FIFOS            3U
#define PWM_DATA_SLOTS              6000
#endif /* USE_HEAP_PWM_BUFFER == TRUE */

/* Number of frame receive buffers. */
#define NUMBER_RX_PKT_BUFFERS       5U
#define USE_CCM_HEAP_RX_BUFFERS     TRUE

#define PKT_RX_RLS_USE_NO_FIFO      TRUE

/*
 * Number of general AX25/APRS processing & frame send buffers.
 * Can configured as being in CCM to save system core memory use.
 */
#define NUMBER_COMMON_PKT_BUFFERS       20U
#define RESERVE_BUFFERS_FOR_INTERNAL    5U
#define MAX_BUFFERS_FOR_BURST_SEND      5U
#if (MAX_BUFFERS_FOR_BURST_SEND >                                            \
    (NUMBER_COMMON_PKT_BUFFERS - RESERVE_BUFFERS_FOR_INTERNAL))
#warning "Can not allocate requested buffers for burst send - set to 50%"
#undef MAX_BUFFERS_FOR_BURST_SEND
#define MAX_BUFFERS_FOR_BURST_SEND   (NUMBER_COMMON_PKT_BUFFERS / 2)
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
  ioline_t  pktSetLineModeICU(const radio_unit_t radio);
  ioline_t  pktSetLineModeRadioGPIO1(const radio_unit_t radio);
  ioline_t  pktSetLineModeRadioGPIO0(const radio_unit_t radio);
  void      pktSerialStart(void);
  void      dbgWrite(uint8_t level, uint8_t *buf, uint32_t len);
  int       dbgPrintf(uint8_t level, const char *format, ...);
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
