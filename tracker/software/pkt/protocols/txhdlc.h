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

#define AX25_WRITE_BIT(data, size) { \
    data[size >> 3] |= (1 << (size & 7)); \
}

#define AX25_CLEAR_BIT(data, size) { \
    data[size >> 3] &= ~(1 << (size & 7)); \
}

typedef struct {
  packet_t      pp;
  uint16_t      pre_count;
  uint16_t      data_size;
  uint8_t       hdlc_data;
  uint32_t      bit_index;
  uint8_t       rll_count;
  bool          prior_nrz;
  bool          scramble;
  uint32_t      lfsr;
} tx_composer_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  uint32_t pktStreamDataForSend(tx_composer_t *composer,
                                uint8_t *buf, size_t size);
#ifdef __cplusplus
}
#endif


#endif /* PKT_PROTOCOLS_TXHDLC_H_ */

/** @} */
