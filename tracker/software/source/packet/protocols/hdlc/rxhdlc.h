/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef PKT_PROTOCOLS_RXHDLC_H_
#define PKT_PROTOCOLS_RXHDLC_H_

#define HDLC_SYNC_USE_COUNTER TRUE

/* HDLC bit pattern definitions. */
#define HDLC_BIT_MASK       0x7FU
#define HDLC_FLAG           0x7EU
#define HDLC_RESET          0x7FU
#define HDLC_ZERO           0x00U

#if HDLC_SYNC_USE_COUNTER == TRUE
/* Count of flags after initial detection met which is for PLL settle. */
#define HDLC_PLL_SYNC_COUNT 16
#endif

/* Frame bounding. */
#define HDLC_SYNC_MASK_A    0xFFFFFFFFU
#define HDLC_SYNC_OPEN_A    0x7E7E7E7EU
#define HDLC_SYNC_MASK_B    0xFFFFFFFFU
#define HDLC_SYNC_OPEN_B    0x00007E7EU

/* RLL encoding. */
#define HDLC_RLL_BIT        0x3EU

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef uint8_t     hdlc_octet_t;
typedef uint32_t    hdlc_stream_t;

/* Result token codes returned from HDLC receive processing. */
typedef enum HDLCToken {
  HDLC_TOK_SYNC,
  HDLC_TOK_FLAG,
  HDLC_TOK_RESET,
  HDLC_TOK_DATA,
  HDLC_TOK_OPEN,
  HDLC_TOK_FEED,
  HDLC_TOK_RLL
} hdlc_token_t;

/* Structure containing the HDLC decode control. */
typedef struct decodeHDLC {
  enum {
    HDLC_FLAG_SEARCH,
    HDLC_FRAME_SYNC,
    HDLC_FRAME_OPEN
  } frame_state;
  hdlc_stream_t     hdlc_bits;
#if HDLC_SYNC_USE_COUNTER == TRUE
  uint16_t           sync_count;
#endif
  uint32_t          bit_index;  /*<< AX25 data bit index. */
  ax25char_t        current_byte;
  tone_t            tone_freq;
  tone_t            prior_freq;
  hdlc_token_t      last_token;
} pkt_hdlc_decode_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

#define isHDLCFrameOpen(driver)                                             \
                          (driver->rx_hdlc.frame_state == HDLC_FRAME_OPEN)
#define isHDLCSynchronising(driver)                                         \
                          (driver->rx_hdlc.frame_state == HDLC_FLAG_SEARCH)
#define getHDLCDataByte(driver) (driver->rx_hdlc.current_byte)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

  #ifdef __cplusplus
  extern "C" {
  #endif
    hdlc_token_t pktExtractHDLCfromAFSK(pkt_hdlc_decode_t *myHDLC);
  #ifdef __cplusplus
  }
  #endif

#endif /* PKT_PROTOCOLS_RXHDLC_H_ */
