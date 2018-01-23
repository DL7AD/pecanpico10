/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    crc_calc.h
 * @brief   Calculates CCITT-CRC16.
 *
 * @addtogroup protocols
 * @{
 */
#ifndef PROTOCOLS_CRC_CALC_H_
#define PROTOCOLS_CRC_CALC_H_

/**
 * @brief   A constant for computed result for data + CRC in CCITT-CRC16.
 * @notes   Use this value to compare to calculation encompassing data + CRC.
 */
#define CRC_INCLUSIVE_CONSTANT    0x0F47

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  uint16_t calc_crc16 (ax25char_t *data, uint16_t offset, uint16_t len);
#ifdef __cplusplus
}
#endif

#endif /* PROTOCOLS_CRC_CALC_H_ */

/** @} */
