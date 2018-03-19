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

#define HDLC_RLL_SEQUENCE        0x1FU

#define AX25_WRITE_BIT(data, size) { \
    data[size >> 3] |= (1 << (size & 7)); \
}

#define AX25_CLEAR_BIT(data, size) { \
    data[size >> 3] &= ~(1 << (size & 7)); \
}

typedef enum {
  ITERATE_INIT,
  ITERATE_PREAMBLE,
  ITERATE_FRAME,
  ITERATE_CRC,
  ITERATE_CLOSE
} txit_state_t;

typedef struct {
  //packet_t      pp;
  txit_state_t  state;
  uint16_t      pre_count;
  uint8_t      *data_buff;
  uint16_t      data_size;
  uint8_t       *out_buff;
  uint16_t      out_size;
  uint8_t       nrzi_last;
  uint8_t       hdlc_data;
  uint32_t      inp_index;
  uint32_t      out_index;
  uint16_t      rll_count;
  bool          scramble;
  uint32_t      lfsr;
} tx_iterator_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  int32_t pktIterateSendStream(tx_iterator_t *iterator, size_t qty);
#ifdef __cplusplus
}
#endif


#endif /* PKT_PROTOCOLS_TXHDLC_H_ */

/** @} */
