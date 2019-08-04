/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    pktradio.h
 * @brief   Generic radio definitions.
 *
 * @addtogroup managers
 * @{
 */


#ifndef PKT_PROTOCOLS_TXHDLC_H_
#define PKT_PROTOCOLS_TXHDLC_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define HDLC_RLL_SEQUENCE       0x1FU

#define ITERATOR_MAX_QTY        0xFFFF

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum {
  ITERATE_INIT,
  ITERATE_PREAMBLE,
  ITERATE_FRAME,
  ITERATE_CRC,
  ITERATE_CLOSE,
  ITERATE_TAIL,
  ITERATE_FINAL,
  ITERATE_END
} txit_state_t;

typedef struct txIterator {
  txit_state_t  state;
  bool          no_write;
  uint16_t      qty;
  uint16_t      out_count;
  uint8_t       hdlc_count;
  uint8_t       hdlc_post;
  uint8_t       hdlc_tail;
  ax25char_t*   data_buff;
  uint16_t      data_size;
  hdlc_octet_t* out_buff;
  uint8_t       hdlc_code;
  uint8_t       crc[sizeof(uint16_t)];
  uint8_t       nrzi_hist;
  uint8_t       hdlc_hist;
  uint32_t      inp_index;
  uint32_t      out_index;
  uint8_t       rll_count;
  bool          scramble;
  uint32_t      lfsr;
} tx_iterator_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  uint16_t  pktStreamEncodingIterator(tx_iterator_t *iterator,
                                     uint8_t *stream, uint16_t qty);
  void      pktStreamIteratorInit(tx_iterator_t *iterator,
                             ax25char_t* pkt,
                             size_t len,
                             //packet_t pp,
                             uint8_t pre,
                             uint8_t post,
                             uint8_t tail,
                             bool scramble);
#ifdef __cplusplus
}
#endif


#endif /* PKT_PROTOCOLS_TXHDLC_H_ */

/** @} */
