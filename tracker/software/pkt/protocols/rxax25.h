/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    rxax25.h
 * @brief   Definitions for AX25 protocol for packet receiver.
 *
 * @addtogroup protocols
 * @{
 */


#ifndef PKT_PROTOCOLS_RXAX25_H_
#define PKT_PROTOCOLS_RXAX25_H_


/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/* AX25 definitions for packet data useage. */
#define PKT_MAX_REPEATERS      8
/* Destination & Source. */
#define PKT_MIN_ADDRS          2
/*
 *  Destination & Source + 8 digipeater addresses.
 *  A destination address may specify a generic APRS digipeater path.
 *  In such case the digipeater address fields are overridden by that path.
 */
#define PKT_MAX_ADDRS          10
/* Address positions in frame. */
#define PKT_DESTINATION        0
#define PKT_SOURCE             1
#define PKT_REPEATER_1         2
#define PKT_REPEATER_2         3
#define PKT_REPEATER_3         4
#define PKT_REPEATER_4         5
#define PKT_REPEATER_5         6
#define PKT_REPEATER_6         7
#define PKT_REPEATER_7         8
#define PKT_REPEATER_8         9

/*
 * The maximum address length should be 6 letters, dash, 2 digits and null.
 * Making a total of 10.
 * However, object labels can be 10 characters.
 * So add 2 extra margin bytes.
 */
#define PKT_DS_ADDRESS_LEN     7
#define PKT_MAX_ADDR_LEN       12
#define PKT_CONTROL_LEN        1
#define PKT_PROTOCOL_LEN       1
#define PKT_CRC_LEN            2
#define PKT_FLAG_LEN           1
#define PKT_MIN_INFO_LEN       0


/* An AX.25 packet can have a control byte and no protocol. */
#define PKT_MIN_PACKET_LEN     (PKT_MIN_ADDRS * PKT_DS_ADDRESS_LEN        \
                                  + PKT_CONTROL_LEN)

/*
 * Maximum size for APRS.
 * The payload excluding CRC.
 */
#define PKT_MAX_RX_INFO_LEN       512

/* An AX.25 receive packet maximum - closing flag is not included. */
#define PKT_MAX_RX_PACKET_LEN     (PKT_MAX_ADDRS * PKT_DS_ADDRESS_LEN       \
                                  + PKT_CONTROL_LEN                         \
                                  + PKT_PROTOCOL_LEN                        \
                                  + PKT_MAX_RX_INFO_LEN                     \
                                  + PKT_CRC_LEN)

#define PKT_MIN_FRAME          ((PKT_MIN_PACKET_LEN) + PKT_CRC_LEN)
#define PKT_MAX_FRAME          ((PKT_MAX_PACKET_LEN) + PKT_CRC_LEN)


/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of characters in AX25 packet.
 *
 * @note    Only used in buffers.
 */

typedef int16_t ax25size_t;

#endif /* PKT_PROTOCOLS_RXAX25_H_ */

/** @} */
