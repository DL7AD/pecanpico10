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


/*
 * Write NRZI data to output buffer.
 * Byte complete = true else false.
 *
 */
static bool pktIteratorWriteNRZI(tx_iterator_t *iterator, uint8_t bit) {
  /* If new byte clear it first. */
  if(iterator->out_index % 8 == 0)
    iterator->out_buff[iterator->out_index >> 3] = 0;

  /* clean up bit. */
  bit &= 0x1;
  /* Keep track of HDLC for RLL detection. */
  iterator->hdlc_data <<= 1;
  iterator->hdlc_data |= bit;

  /* Remember last NRZI state (keep history for debug purposes). */
  iterator->nrzi_last <<= 1;
  iterator->nrzi_last |= (bit == 0)
      ? (((iterator->nrzi_last >> 1 ) ^ 0x1) & 0x1)
      : 1;

  /* Write NRZI bit to current byte. */
  iterator->out_buff[iterator->out_index >> 3] |=
      (iterator->nrzi_last & 0x1) << (iterator->out_index % 8);

  /* If byte is full return true. */
  return ((++iterator->out_index % 8) == 0);
}

/*
 * Create stream of bytes of encoded link level data.
 * The calling function may request chunk sizes from 1 byte up.
 * The function returns the number of bytes encoded.
 * When return < request there is no more to encode.
 * When return is -1 then the output buffer is full.
 */
int32_t pktIterateSendStream(tx_iterator_t *iterator, size_t qty) {

  if(qty == 0)
    return 0;
  size_t cnt = 0;

  /*
   * Output preamble bytes of specified quantity in requested chunk size.
   * RLL encoding is not used as these are HDLC flags.
   */
  while(iterator->pre_count > 0) {
    /* TODO: Check for output buffer exhausted. */
    uint8_t i;
    for(i = 0; !pktIteratorWriteNRZI(iterator, (HDLC_FLAG >> i) & 0x1); i++);
    iterator->pre_count--;
    if(++cnt == qty)
      return qty;
  } /* End while. */

  /*
   * Output frame bytes in requested chunk size.
   * CRC and closing HDLC flag must be included in the frame data.
   */
  while(iterator->data_size > 0) {
    do {
      /*
       * RLL encoding is enabled for the packet data pay load.
       * Except the last data byte which is the HDLC flag.
       */


      if(((iterator->hdlc_data & HDLC_RLL_SEQUENCE) == HDLC_RLL_SEQUENCE)
         && (iterator->data_size > 1)) {
        /* Insert RLL 0 to HDLC output stream. */
        if(pktIteratorWriteNRZI(iterator, 0)) {
          if(++cnt == qty)
            return qty;
        }
      }
      /* Get data bit. */
      uint8_t byte = iterator->data_buff[iterator->inp_index >> 3];
      uint8_t bit = (byte >> (iterator->inp_index % 8)) & 0x1;

      /* Show bit is consumed and write it to steam. */
      iterator->inp_index++;
      if(pktIteratorWriteNRZI(iterator, bit)) {
        if(++cnt == qty)
        return qty;
      }
    } while((iterator->inp_index % 8) != 0);
    /* Consumed an input byte. */
    --iterator->data_size;
  } /* End while. */

  /* TODO: Put CRC and closing flag insertion in here versus in main code. */

  /* Round up for partial byte. */

  return (++cnt);
}

/** @} */
