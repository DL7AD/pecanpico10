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
 * Initialize the iterator object.
 * TODO: Preamble count to come from config.
 */
void pktStreamIteratorInit(tx_iterator_t *iterator,
                           packet_t pp, bool scramble) {
  memset(iterator, 0, sizeof(tx_iterator_t));
  iterator->flag_count = 50;
  iterator->scramble = scramble;
  iterator->data_buff = pp->frame_data;
  iterator->data_size = pp->frame_len;
  uint16_t crc = calc_crc16(pp->frame_data, 0, pp->frame_len);
  iterator->crc[0] = crc & 0xFF;
  iterator->crc[1] = crc >> 8;
  iterator->state = ITERATE_PREAMBLE;
}


/*
 * Write NRZI data to output buffer.
 * Scrambling is applied if selected.
 *
 */
static bool pktIteratorWriteStream(tx_iterator_t *iterator, uint8_t bit) {
  /* If new byte clear it first. */
  if(iterator->out_index % 8 == 0)
    iterator->out_buff[iterator->out_index >> 3] = 0;

  /* Normalize to bit 0. */
  if(bit != 0) bit = 1;

  /* Keep track of HDLC for RLL detection. */
  iterator->hdlc_hist <<= 1;
  iterator->hdlc_hist |= bit;

  if(iterator->scramble) {
    iterator->lfsr <<= 1;
    iterator->lfsr |= (bit ^ (iterator->lfsr >> 17)
        ^ (iterator->lfsr >> 12)) & 0x1;
    bit = iterator->lfsr & 0x1;
  }

  /* NRZI encode bit. */
  iterator->nrzi_hist ^= (bit == 0) ? 0x1 : 0x0;

  /* Write NRZI bit to current byte. */
  iterator->out_buff[iterator->out_index >> 3] |=
      (iterator->nrzi_hist & 0x1) << (iterator->out_index % 8);

  /* If byte is full and required quantity, reached return true. */
  if((++iterator->out_index % 8) == 0)
    return (++iterator->out_count == iterator->qty);
  else
    return false;
}

/*
 *
 */
static bool pktEncodeFrameFlag(tx_iterator_t *iterator) {
  do {
    if(pktIteratorWriteStream(iterator,
           ((HDLC_FLAG >> iterator->flag_bit++) & 0x1)
           << (iterator->out_index % 8)))
        return true;
  } while(iterator->flag_bit < 8);
  iterator->flag_bit = 0;
  return false;
}


/*
 *
 */
static bool pktEncodeFrameData(tx_iterator_t *iterator) {
  do {

    /*
     * RLL encoding is enabled for the packet data pay load.
     * Except the last data byte which is the HDLC flag.
     */
    if((iterator->hdlc_hist & HDLC_RLL_SEQUENCE) == HDLC_RLL_SEQUENCE) {
      iterator->rll_count++;
      /* Insert RLL 0 to HDLC output stream. */
      if(pktIteratorWriteStream(iterator, 0)) {
        //if(++iterator->out_count == iterator->qty)
          return true;
      }
    }
    /* Get data bit. */
    uint8_t byte = iterator->data_buff[iterator->inp_index >> 3];
    uint8_t bit = (byte >> (iterator->inp_index % 8)) & 0x1;

    /* Indicate data input bit is consumed. Write to stream. */
    iterator->inp_index++;
    if(pktIteratorWriteStream(iterator, bit)) {
      //if(++iterator->out_count == iterator->qty)
      return true;
    }
  } while((iterator->inp_index % 8) != 0);
  return false;
}

/*
 * Create stream of bytes of encoded link level data.
 * The calling function may request chunk sizes from 1 byte up.
 * The function returns the number of bytes encoded.
 *
 * It is the calling functions responsibility to allocate the sized buffer.
 * The iterator will write stream bytes up to the buffer size specified.
 */
uint16_t pktStreamEncodingIterator(tx_iterator_t *iterator,
                                   uint8_t *stream, size_t qty) {

  if(qty == 0)
    return 0;

  iterator->out_count = 0;
  iterator->qty = qty;

  chDbgAssert(stream != NULL, "no stream buffer allocated");

  iterator->out_buff = stream;

  while(true) {
    switch(iterator->state) {
    case ITERATE_INIT:
    case ITERATE_FINAL:
      return 0;

    case ITERATE_PREAMBLE: {
      /*
       * Output preamble bytes of specified quantity in requested chunk size.
       * RLL encoding is not used as these are HDLC flags.
       */
      while(iterator->flag_count > 0) {
        if(pktEncodeFrameFlag(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
        /* False means a byte has been consumed from flags. */
        --iterator->flag_count;
        continue;
      } /* End while. */
      iterator->state = ITERATE_FRAME;
      break;
      } /* End case ITERATE_PREAMBLE. */

    case ITERATE_FRAME: {
      /*
       * Output frame data bytes in requested chunk size.
       */
      while(iterator->data_size > 0) {
        /* Consume bytes until count reached. */
        if(pktEncodeFrameData(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
        /* False means a byte has been consumed from input. */
        --iterator->data_size;
        continue;
      }
      /* All frame data input consumed. */
      iterator->state = ITERATE_CRC;
      iterator->data_buff = iterator->crc;
      iterator->data_size = sizeof(iterator->crc);
      iterator->inp_index = 0;
      break;
      } /* End case ITERATE_FRAME. */

    case ITERATE_CRC: {
      /*
       * Output frame CRC bytes in requested chunk size.
       */
      while(iterator->data_size > 0) {
        /* Consume bytes until count reached. */
        if(pktEncodeFrameData(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
        /* False means a byte has been consumed from input. */
        --iterator->data_size;
        continue;
      }
      /* Frame CRC consumed. */
      iterator->state = ITERATE_CLOSE;
      iterator->flag_count = 1;
      iterator->flag_bit = 0;
      break;
      }

    case ITERATE_CLOSE: {
      /*
       * Output closing flag.
       * RLL encoding is not used as this is an HDLC flag.
       */
      while(iterator->flag_count > 0) {
        if(pktEncodeFrameFlag(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
        /* False means a byte has been consumed from flags. */
        --iterator->flag_count;
        continue;
      } /* End while. */
      iterator->state = ITERATE_FINAL;
      /* Round up to include any partial bits in next byte. */
      if(iterator->out_index % 8 != 0)
        ++iterator->out_count;
      return (iterator->out_count);
      }
    } /* End switch on state. */
  } /* End while. */
} /* End function. */

/** @} */
