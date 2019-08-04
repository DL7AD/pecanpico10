/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    dbguart.h
 * @brief   Definitions for UART4.
 *
 * @addtogroup IODevices
 * @{
 */

#ifndef DEVICES_DBGUART_H_
#define DEVICES_DBGUART_H_

#include "portab.h"

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#define DBG_ERROR	1
#define DBG_WARN	2
#define DBG_INFO	3
#define DBG_DEBUG	4

#ifdef __cplusplus
extern "C" {
#endif

  extern BaseSequentialStream* pkt_out;
#ifdef __cplusplus
}
#endif

#endif /* DEVICES_DBGUART_H_ */

/** @} */

