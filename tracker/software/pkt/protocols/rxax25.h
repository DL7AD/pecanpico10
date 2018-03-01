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
#define PKT_MAX_INFO_LEN       2048

/* An AX.25 packet maximum - closing flag is not included. */
#define PKT_MAX_PACKET_LEN     (PKT_MAX_ADDRS * PKT_DS_ADDRESS_LEN          \
                                  + PKT_CONTROL_LEN                         \
                                  + PKT_PROTOCOL_LEN                        \
                                  + PKT_MAX_INFO_LEN                        \
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
typedef uint8_t ax25char_t;
typedef int16_t ax25size_t;

#ifdef PKT_IS_TEST_PROJECT
/* TODO: Create a chFactory FIFO to manage these objects. */
struct packet_s {

    int magic1;     /* for error checking. */

    int seq;        /* unique sequence number for debugging. */

    double release_time;    /* Time stamp in format returned by dtime_now(). */
                /* When to release from the SATgate mode delay queue. */

#define MAGIC 0x41583235

    struct packet_s *nextp; /* Pointer to next in queue. */

    int num_addr;       /* Number of addresses in frame. */
                /* Range of AX25_MIN_ADDRS .. AX25_MAX_ADDRS for AX.25. */
                /* It will be 0 if it doesn't look like AX.25. */
                /* -1 is used temporarily at allocation to mean */
                /* not determined yet. */



                /*
                 * The 7th octet of each address contains:
                     *
                 * Bits:   H  R  R  SSID  0
                 *
                 *   H      for digipeaters set to 0 intially.
                 *      Changed to 1 when position has been used.
                 *
                 *      for source & destination it is called
                 *      command/response.  Normally both 1 for APRS.
                 *      They should be opposites for connected mode.
                 *
                 *   R  R   Reserved.  Normally set to 1 1.
                 *
                 *   SSID   Substation ID.  Range of 0 - 15.
                 *
                 *   0      Usually 0 but 1 for last address.
                 */


#define SSID_H_MASK 0x80
#define SSID_H_SHIFT    7

#define SSID_RR_MASK    0x60
#define SSID_RR_SHIFT   5

#define SSID_SSID_MASK  0x1e
#define SSID_SSID_SHIFT 1

#define SSID_LAST_MASK  0x01


    int frame_len;      /* Frame length without CRC. */

    int modulo;     /* I & S frames have sequence numbers of either 3 bits (modulo 8) */
                /* or 7 bits (modulo 128).  This is conveyed by either 1 or 2 */
                /* control bytes.  Unfortunately, we can't determine this by looking */
                /* at an isolated frame.  We need to know about the context.  If we */
                /* are part of the conversation, we would know.  But if we are */
                /* just listening to others, this would be more difficult to determine. */

                /* For U frames:    set to 0 - not applicable */
                /* For I & S frames:    8 or 128 if known.  0 if unknown. */

    unsigned char frame_data[PKT_MAX_PACKET_LEN + 1];
                /* Raw frame contents, without the CRC. */


    int magic2;     /* Will get stomped on if above overflows. */
};




typedef struct packet_s *packet_t;

#endif

#endif /* PKT_PROTOCOLS_RXAX25_H_ */

/** @} */
