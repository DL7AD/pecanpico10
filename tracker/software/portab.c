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
#include "portab.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

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
  //palSetLineMode(LINE_USART3_TX, PAL_MODE_OUTPUT_PUSHPULL | PAL_MODE_ALTERNATE(3));
  /* USART3 RX.       */
  //palSetLineMode(LINE_USART3_RX, PAL_MODE_ALTERNATE(3));
}

void pktConfigSerialPkt(void) {
  /* UART4 TX.       */
  palSetLineMode(LINE_UART4_TX, PAL_MODE_OUTPUT_PUSHPULL | PAL_MODE_ALTERNATE(11));
  /* UART4 RX.       */
  palSetLineMode(LINE_UART4_RX, PAL_MODE_INPUT | PAL_MODE_ALTERNATE(11));
}

void pktSetLineModeICU(void) {
  palSetLineMode(LINE_ICU, PAL_MODE_INPUT | PAL_MODE_ALTERNATE(2));
}

/** @} */
