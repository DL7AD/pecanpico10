/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/*===========================================================================*/
/* Aerospace Decoder configuration definition and ChibiOS system includes.   */
/*===========================================================================*/

#include "pktconf.h"
#include "dbguart.h"

/**
 * @file    dbguart.c
 * @brief   Serial channels for debug.
 *
 * @addtogroup IODevices
 * @{
 */

binary_semaphore_t debug_out_sem;

//BaseSequentialStream* pkt_out = (BaseSequentialStream*)SERIAL_CFG_DEBUG_DRIVER;

/** @} */
