/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include "pktconf.h"

uint32_t lfsr;
uint8_t scramble_bit(uint8_t _in) {
    uint8_t x = (_in ^ (lfsr >> 16) ^ (lfsr >> 11)) & 1;
    lfsr = (lfsr << 1) | (x & 1);
    return x;
}

/**
  * Scrambling for 2GFSK
  */
void scramble(uint8_t *data, size_t size) {

    // Scramble
    lfsr = 0;
    for(uint32_t i = 0; i < size; i++) {
        uint8_t bit = scramble_bit((data[i >> 3] >> (i & 0x7)) & 0x1);
        if(bit) {
            AX25_WRITE_BIT(data, i);
        } else {
            AX25_CLEAR_BIT(data, i);
        }
    }
}


/* TODO: Make this a macro. */
static bool pktGetBitAsNRZI(bool bit, bool *prior) {
    if((bit & 0x1) == 0)
        *prior = !*prior;
    return *prior;
}

/*
 * Create stream of bytes of encoded link level data.
 * The calling function may request chunk sizes from 1 up.
 * The function returns the number of bytes encoded.
 *
 */
uint32_t pktStreamDataForSend(tx_composer_t *composer,
                              uint8_t *buf, size_t size) {

  if(size == 0)
    return 0;
  size_t cnt = size;

  /*
   * Output preamble bytes of specified quantity in requested chunk size.
   * RLL encoding is not used as these are HDLC flags.
   */
  while(composer->pre_count > 0) {
    uint8_t i = 0, f = HDLC_FLAG;
    do {
      /* Shift up prior bit. */
      composer->hdlc_data <<= 1;
      /* Get a data bit. */
      composer->hdlc_data |= pktGetBitAsNRZI(f & 0x1,
                                             &composer->prior_nrz);
      f <<= 1;
    } while(++i < 8);
    *buf++ = composer->hdlc_data;
    if(--cnt == 0)
      return size;
    composer->pre_count--;
  }

  /*
   * Output frame bytes in requested chunk size.
   * CRC has been added to the data already.
   */
  while(composer->data_size > 0) {
    uint8_t i = 0;
    do {
      /* Shift up prior bit (and shift in a zero). */
      composer->hdlc_data <<= 1;
      /* Check RLL encoding. */
      if((composer->hdlc_data & HDLC_RLL_BIT) == HDLC_RLL_BIT) {
        /* Include the inserted 0 bit in the stream count. */
        i++;
      } else {
      /* Get a data bit. */
        composer->hdlc_data |= pktGetBitAsNRZI(
                composer->pp->frame_data[composer->bit_index >> 3] & 0x1,
                &composer->prior_nrz);
      }

      /* Count the added bit. */
      composer->bit_index++;
    } while(++i < 8);

    /* Save the HDLC byte. */
    *buf++ = composer->hdlc_data;
    if(--cnt == 0)
      return size;
    composer->data_size--;
  }

  /*
   * Output closing flag.
   * RLL encoding is not used as this is an HDLC flag.
   */
  uint8_t i = 0, f = HDLC_FRAME_CLOSE;
  do {
    /* Shift up prior bit. */
    composer->hdlc_data <<= 1;
    /* Get a data bit. */
    composer->hdlc_data |= pktGetBitAsNRZI(f & 0x1,
                                           &composer->prior_nrz);
    f <<= 1;
  } while(++i < 8);
  *buf++ = composer->hdlc_data;
  composer->bit_index += i;
    return --cnt;
}

/** @} */
