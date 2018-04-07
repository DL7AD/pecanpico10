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
#define DUMP_PACKET_TO_SERIAL       TRUE

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

#define Si446x_MIN_2M_FREQ	    	144000000				/* Minimum allowed frequency in Hz */
#define Si446x_MAX_2M_FREQ			148000000				/* Maximum allowed frequency in Hz */
#define Si446x_CLK					STM32_HSECLK			/* Oscillator frequency in Hz */
#define Si446x_CLK_OFFSET			22						/* Oscillator frequency drift in ppm */
#define Si446x_CLK_TCXO_EN			true					/* Set this true, if a TCXO is used, false for XTAL */

//#define LINE_OVERFLOW_LED         LINE_LED3
#define LINE_DECODER_LED            LINE_IO_BLUE
//#define LINE_SQUELCH_LED            LINE_IO_GREEN

#define LINE_CCA                    LINE_RADIO_IRQ
#define LINE_ICU                    LINE_RADIO_GPIO1

//#define LINE_UART4_TX               PAL_LINE(GPIOA, 12U)
//#define LINE_UART4_RX               PAL_LINE(GPIOA, 11U)

#if ENABLE_EXTERNAL_I2C == FALSE
#define LINE_USART3_TX              LINE_IO_TXD
#define LINE_USART3_RX              LINE_IO_RXD
#endif

//#define LINE_PWM_MIRROR             PAL_LINE(GPIOA, 8U)

/**
 *  ICU related definitions.
 */
#define PWM_ICU                     ICUD4
#define PWM_TIMER_CHANNEL           0

/* ICU counter frequency (2.88MHz). */
/*
 * TODO: This should be calculated using SYSTEM CLOCK.
 * ICU has to run at an integer divide from SYSTEM CLOCK.
 */

#define ICU_COUNT_FREQUENCY         6000000U

#define USE_12_BIT_PWM              FALSE
#define USE_HEAP_PWM_BUFFER         FALSE

/* Definitions for ICU FIFO implemented using chfactory. */
#define NUMBER_PWM_FIFOS            3U
#define PWM_DATA_SLOTS              6000

/* Number of frame receive buffers. */
#define NUMBER_RX_PKT_BUFFERS        3U
/* Number of frame send buffers. */
#define NUMBER_COMMON_PKT_BUFFERS    10U

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

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

#ifdef __cplusplus
extern "C" {
#endif
  void pktConfigSerialDiag(void);
  void pktConfigSerialPkt(void);
  void pktSetLineModeICU(void);
  void pktSerialStart(void);
  void dbgWrite(uint8_t level, uint8_t *buf, uint32_t len);
  int dbgPrintf(uint8_t level, const char *format, ...);
  void pktWrite(uint8_t *buf, uint32_t len);
  void pktConfigureRadioGPIO(radio_unit_t radio);
  void pktDeconfigureRadioGPIO(radio_unit_t radio);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* PORTAB_H_ */

/** @} */
