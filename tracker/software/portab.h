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

//#define LINE_OVERFLOW_LED   LINE_LED3
#define LINE_DECODER_LED    LINE_IO_BLUE
//#define LINE_SQUELCH_LED    LINE_IO_BLUE

#define LINE_CCA            PAL_LINE(GPIOD, 2U)
#define LINE_ICU            PAL_LINE(GPIOB, 6U)

#define LINE_UART4_TX       PAL_LINE(GPIOA, 12U)
#define LINE_UART4_RX       PAL_LINE(GPIOA, 11U)

//#define LINE_PWM_MIRROR     PAL_LINE(GPIOA, 8U)

#define PWM_ICU             ICUD4
#define PWM_TIMER_CHANNEL   0

/* Definitions for ICU FIFO implemented using chfactory. */
#define NUMBER_PWM_FIFOS    4
#define PWM_BUFFER_SLOTS    6000

/* Number of AX25 output buffers. */
#define NUMBER_PKT_FIFOS    2U

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
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* PORTAB_H_ */

/** @} */
