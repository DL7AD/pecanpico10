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

/* TODO: Set LED line associations - will need to remove some for PP10. */
//#define LINE_BUTTON         PAL_LINE(GPIOA, 0U)
//#define LINE_ONBOARD_LED    PAL_LINE(GPIOB, 0U)
//#define LINE_YELLOW_LED     PAL_LINE(GPIOA, 5U)
//#define LINE_RED_LED        PAL_LINE(GPIOA, 7U)
//#define LINE_BLUE_LED       PAL_LINE(GPIOB, 1U)
#define LINE_BLUE_LED       PAL_LINE(GPIOC, 1U) /* PP10 blue LED. */
//#define LINE_GREEN_LED      PAL_LINE(GPIOA, 3U)
//#define LINE_CCA            PAL_LINE(GPIOC, 1U)
#define LINE_CCA            PAL_LINE(GPIOD, 2U)
//#define LINE_ICU            PAL_LINE(GPIOA, 6U)
#define LINE_ICU            PAL_LINE(GPIOB, 6U) /* PP10 si4464 GPIO1. */

#define LINE_USART3_TX      PAL_LINE(GPIOB, 10U)
#define LINE_USART3_RX      PAL_LINE(GPIOB, 11U)
#define LINE_UART4_TX       PAL_LINE(GPIOC, 10U)
#define LINE_UART4_RX       PAL_LINE(GPIOC, 11U)

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
  void pktSetLineModeICU(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* PORTAB_H_ */

/** @} */
