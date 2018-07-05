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
#define SERIAL_CFG_DEBUG_DRIVER		&SD3
//#define SERIAL_CFG_PACKET_DRIVER	&SD4


#define USE_SPI_ATTACHED_RADIO      TRUE
#define DUMP_PACKET_TO_SERIAL       FALSE

/*
 * TODO: Need to use radio unit ID to set assigned GPIO & SPI.
 * Only if there is a multi radio board...
 */

/*
 * Radio SPI definitions.
 */
#define PKT_RADIO_SPI               &SPID3

/*
 * Radio GPIO definitions.
 */
#define LINE_RADIO_CS               PAL_LINE(GPIOC, 12U)
#define LINE_RADIO_SDN              PAL_LINE(GPIOC, 10U)
#define LINE_RADIO_IRQ              PAL_LINE(GPIOD, 2U)
#define LINE_RADIO_GPIO0            PAL_LINE(GPIOB, 7U)
#define LINE_RADIO_GPIO1            PAL_LINE(GPIOB, 6U)
#define LINE_SPI_SCK                PAL_LINE(GPIOB, 3U)
#define LINE_SPI_MISO               PAL_LINE(GPIOB, 4U)
#define LINE_SPI_MOSI               PAL_LINE(GPIOB, 5U)

#define BAND_MIN_2M_FREQ	    	144000000				/* Minimum allowed frequency in Hz */
#define BAND_MAX_2M_FREQ			148000000				/* Maximum allowed frequency in Hz */
#define BAND_STEP_2M_HZ             12500
#define BAND_DEF_2M_APRS            144800000               /* Default frequency in Hz.        */
#define BAND_MIN_70CM_FREQ          420000000               /* Minimum allowed frequency in Hz */
#define BAND_MAX_70CM_FREQ          450000000               /* Maximum allowed frequency in Hz */
#define BAND_STEP_70CM_HZ           25000
#define BAND_DEF_70CM_APRS          439100000               /* Default frequency in Hz.        */

#define DEFAULT_OPERATING_FREQ      144800000

#define Si446x_CLK					STM32_HSECLK			/* Oscillator frequency in Hz */
#define Si446x_CLK_OFFSET			22						/* Oscillator frequency drift in ppm */
#define Si446x_CLK_TCXO_EN			true					/* Set this true, if a TCXO is used, false for XTAL */

//#define NUM_PKT_RADIOS              1
#define NUM_BANDS_PER_RADIO         2

/* LED status indicators (set to PAL_NOLINE if not available). */
#define LINE_OVERFLOW_LED           PAL_NOLINE
#define LINE_DECODER_LED            LINE_IO_BLUE
#define LINE_SQUELCH_LED            PAL_NOLINE
#define LINE_NO_FIFO_LED            PAL_NOLINE
#define LINE_NO_BUFF_LED            PAL_NOLINE

/* Diagnostic PWM mirror port. */
#define LINE_PWM_MIRROR             PAL_NOLINE

/* Radio ports. */
#define LINE_CCA                    LINE_RADIO_IRQ
#define LINE_ICU                    LINE_RADIO_GPIO1

//#define LINE_UART4_TX               PAL_LINE(GPIOA, 12U)
//#define LINE_UART4_RX               PAL_LINE(GPIOA, 11U)

/* The external port can be used for bit bang I2C. */
#define ENABLE_EXTERNAL_I2C         TRUE

#if ENABLE_EXTERNAL_I2C == FALSE
#define LINE_USART3_TX              LINE_IO_TXD
#define LINE_USART3_RX              LINE_IO_RXD
#endif

/* If set to true, the USB interface will be switched on. The tracker is also switched to
 * 3V, because USB would not work at 1.8V. Note that the transmission power is increased
 * too when operating at 3V. This option will also run the STM32 at 48MHz (AHB) permanently
 * because USB needs that speed, otherwise it is running at 6MHz which saves a lot of power.
 */
#define ACTIVATE_USB                TRUE

/**
 *  ICU related definitions.
 */
#define PWM_ICU                     ICUD4
#define PWM_TIMER_CHANNEL           0
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
#define USE_CCM_BASED_HEAP          TRUE
#define TRACE_PWM_BUFFER_STATS      FALSE

/* Definitions for ICU FIFO implemented using chfactory. */
#if USE_HEAP_PWM_BUFFER == TRUE
/* Use factory FIFO as stream control with separate chained PWM buffers. */
#define NUMBER_PWM_FIFOS            3U
/* Number of PWM data entries per queue object. */
#define PWM_DATA_SLOTS              200
/* Number of PWM queue objects in total. */
#define PWM_DATA_BUFFERS            30
#else
/* Use factory FIFO as stream control with integrated PWM buffer. */
#define NUMBER_PWM_FIFOS            3U
#define PWM_DATA_SLOTS              6000
#endif

/* Number of frame receive buffers. */
#define NUMBER_RX_PKT_BUFFERS      3U
#define USE_CCM_HEAP_RX_BUFFERS    TRUE

#define PKT_RX_RLS_USE_NO_FIFO     TRUE
/*
 * Number of general AX25/APRS processing & frame send buffers.
 * Can configured as being in CCM to save system core memory use.
 */
#define NUMBER_COMMON_PKT_BUFFERS       10U
#define RESERVE_BUFFERS_FOR_INTERNAL    2U
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

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef struct radioBand {
  radio_freq_t  start;
  radio_freq_t  end;
  channel_hz_t  step;
  radio_freq_t  def_aprs;
} radio_band_t;

typedef struct radioConfig {
  radio_unit_t  unit;
  radio_type_t  type;
  radio_band_t  *band[NUM_BANDS_PER_RADIO];
} radio_config_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

//extern const radio_param_t radio_list[NUM_PKT_RADIOS];

#ifdef __cplusplus
extern "C" {
#endif
  void pktConfigSerialDiag(void);
  void pktConfigSerialPkt(void);
  void pktConfigureCoreIO(void);
  void pktSetLineModeICU(void);
  void pktSerialStart(void);
  void dbgWrite(uint8_t level, uint8_t *buf, uint32_t len);
  int  dbgPrintf(uint8_t level, const char *format, ...);
  void pktWrite(uint8_t *buf, uint32_t len);
  void pktPowerUpRadio(radio_unit_t radio);
  void pktPowerDownRadio(radio_unit_t radio);
  radio_freq_t pktCheckAllowedFrequency(radio_unit_t radio, radio_freq_t freq);
  uint8_t pktReadIOlines(void);
  uint8_t pktGetNumRadios(void);
  const radio_config_t *pktGetRadioList(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* PORTAB_H_ */

/** @} */
