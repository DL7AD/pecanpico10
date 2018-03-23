/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include "pktconf.h"

/*
 * Initialize the iterator object.
 * TODO: Preamble count to come from config.
 */
void pktStreamIteratorInit(tx_iterator_t *iterator,
                           packet_t pp, bool scramble) {
  memset(iterator, 0, sizeof(tx_iterator_t));
  iterator->hdlc_code = HDLC_FLAG;
  iterator->hdlc_count = 50;
  iterator->scramble = scramble;
  iterator->data_buff = pp->frame_data;
  iterator->data_size = pp->frame_len;
  uint16_t crc = calc_crc16(pp->frame_data, 0, pp->frame_len);
  iterator->crc[0] = crc & 0xFF;
  iterator->crc[1] = crc >> 8;
  iterator->no_write = false;
  iterator->state = ITERATE_PREAMBLE;
}


/*
 * Write NRZI data to output buffer.
 * Scrambling is applied if selected.
 *
 */
static bool pktIteratorWriteStream(tx_iterator_t *iterator, uint8_t bit) {
  /* If new output buffer byte clear it first if write is enabled. */
  if((iterator->out_index % 8 == 0) && (iterator->no_write == false))
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
  if(iterator->no_write == false)
    iterator->out_buff[iterator->out_index >> 3] |=
        (iterator->nrzi_hist & 0x1) << (iterator->out_index % 8);

  /* If byte was completed check quantity status. */
  if((iterator->inp_index % 8) == 0) {
    if((++iterator->out_count) == iterator->qty) {
      //iterator->out_index = 0;
      return true;
    }
  }
  return false;
}

/*
 *
 */
static bool pktEncodeFrameHDLC(tx_iterator_t *iterator) {
  do {
    iterator->inp_index++;
    if(pktIteratorWriteStream(iterator,
           ((iterator->hdlc_code >> iterator->hdlc_bit++) & 0x1)
           << (iterator->out_index % 8))) {
      if(iterator->hdlc_bit == 8)
        iterator->hdlc_bit = 0;
      return true;
    }
  } while(iterator->hdlc_bit < 8);
  iterator->hdlc_bit = 0;
  return false;
}


/*
 *
 */
static bool pktEncodeFrameData(tx_iterator_t *iterator) {
  do {

    /* Get data bit. */
    uint8_t byte = iterator->data_buff[iterator->inp_index >> 3];
    uint8_t bit = (byte >> (iterator->inp_index % 8)) & 0x1;

    /* Indicate data input bit is consumed. Write to stream. */
    iterator->inp_index++;
    if(pktIteratorWriteStream(iterator, bit))
      return true;

    /* RLL encoding is enabled for the packet data pay load. */
    if((iterator->hdlc_hist & HDLC_RLL_SEQUENCE) == HDLC_RLL_SEQUENCE) {
      iterator->rll_count++;
      /* Insert RLL 0 to output stream. */
      if(pktIteratorWriteStream(iterator, 0))
          return true;
    }
  } while((iterator->inp_index % 8) != 0);
  return false;
}

/**
 * @brief   Encode frame stream for transmission.
 * @pre     The iterator has to be initialized before use.
 * @post    When the stream is complete the iterator may be re-used.
 * @notes   The iterator allows a frame to be encoded in chunks.
 * @notes   The calling function may request chunk sizes from 1 byte up.
 * @notes   A quantity of 0 will return the number of bytes pending only.
 * @notes   In this case no data is actually written to the stream.
 *
 * @param[in]   iterator   pointer to an @p iterator object.
 * @param[in]   stream     pointer to buffer to write stream data.
 * @param[in]   qty        the requested number of stream bytes.
 *                         requesting 0 will return the stream bytes remaining.
 *
 * @return  number of bytes encoded.
 * @retval  zero indicates the iterator is not initialized or is finished.
 * @retval  requested size is returned while encoding continues.
 * @retval  less than requested size is returned when encoding is ending.
 *
 * @api
 */
uint16_t pktStreamEncodingIterator(tx_iterator_t *iterator,
                                   uint8_t *stream, uint16_t qty) {

  if(qty == 0) {
    tx_iterator_t saved;

    /* Save state. */
    saved = *iterator;
    iterator->no_write = true;

    /* Count the number of bytes remaining to output to the stream. */
    uint16_t remain = pktStreamEncodingIterator(iterator, NULL, 0xFFFF);

    /* Restore state. */
    *iterator = saved;
    return remain;
  }

  /* Each call has a new quantity and buffer which starts from index 0. */
  iterator->out_count = 0;
  iterator->qty = qty;
  iterator->out_index = 0;

  chDbgAssert((stream != NULL) || (iterator->no_write == true),
              "no stream buffer allocated");

  iterator->out_buff = stream;

  while(true) {
    switch(iterator->state) {
    case ITERATE_INIT:
      return 0;

    case ITERATE_END:
      return 0;

    case ITERATE_PREAMBLE: {
      /*
       * Output preamble bytes of specified quantity in requested chunk size.
       * RLL encoding is not used as these are HDLC flags.
       */
      while(iterator->hdlc_count-- > 0) {
        if(pktEncodeFrameHDLC(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
      } /* End while. */
      iterator->state = ITERATE_FRAME;
      continue;
      } /* End case ITERATE_PREAMBLE. */

    case ITERATE_FRAME: {
      /*
       * Output frame data bytes in requested chunk size.
       */
      while(iterator->data_size-- > 0) {
        /* Consume bytes until count reached. */
        if(pktEncodeFrameData(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
      }
      /* All frame data input consumed. Switch to CRC fields. */
      iterator->state = ITERATE_CRC;
      iterator->data_buff = iterator->crc;
      iterator->data_size = sizeof(iterator->crc);
      iterator->inp_index = 0;
      continue;
      } /* End case ITERATE_FRAME. */

    case ITERATE_CRC: {
      /*
       * Output frame CRC bytes in requested chunk size.
       */
      while(iterator->data_size-- > 0) {
        /* Consume bytes until count reached. */
        if(pktEncodeFrameData(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
      }
      /* Frame CRC consumed. */
      iterator->state = ITERATE_CLOSE;
      iterator->hdlc_count = 10;
      iterator->hdlc_code = HDLC_FLAG;
      iterator->hdlc_bit = 0;
      iterator->inp_index = 0;
      continue;
      } /* End case ITERATE_CRC. */

    case ITERATE_CLOSE: {
      /*
       * Output closing flags.
       * RLL encoding is not used as these are HDLC flags.
       */
      while(iterator->hdlc_count-- > 0) {
        if(pktEncodeFrameHDLC(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
      } /* End while. */
      iterator->state = ITERATE_TAIL;
      /* Tail length. */
      iterator->hdlc_count = 10;
      iterator->hdlc_code = HDLC_ZERO;
      continue;
      } /* End case ITERATE_CLOSE. */

    case ITERATE_TAIL: {
      /*
       * Output tail.
       * RLL encoding is not used as these are idle flags.
       * This keeps the data tail clean for the receiver.
       */
      while(iterator->hdlc_count-- > 0) {
        if(pktEncodeFrameHDLC(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
      } /* End while. */
      iterator->state = ITERATE_FINAL;
      continue;
      } /* End case ITERATE_TAIL. */

    case ITERATE_FINAL: {
      iterator->state = ITERATE_END;
      /* Check for extra bits due to RLL. */
      /* TODO: Calculate if RLL bits > 7 and feed out in QTY chunks. */
      if((iterator->out_index % 8) != 0)
        iterator->out_count++;
      return iterator->out_count;
      } /* End case ITERATE_FINAL. */
    } /* End switch on state. */
  } /* End while. */
} /* End function. */

/** @} */
