/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include "pktconf.h"

/**
 * @brief   Initialize an NRZI stream iterator.
 * @post    The iterator is ready for use.
 *
 * @param[in]   iterator    pointer to an @p iterator object.
 * @param[in]   packet      packet buffer reference pointer.
 * @param[in]   len         packet length.
 * @param[in]   pre         length of HDLC (flags) preamble
 * @param[in]   post        length of HDLC (flags) closing
 * @param[in]   tail        length of HDLC (0 data) tail flags
 * @param[in]   scramble    determines if scrambling (whitening) is applied.
 *
 * @api
 */
void pktStreamIteratorInit(tx_iterator_t *iterator,
                           ax25char_t*  pkt,
                           size_t       len,
                           uint8_t      pre,
                           uint8_t      post,
                           uint8_t      tail,
                           bool         scramble) {
  memset(iterator, 0, sizeof(tx_iterator_t));
  iterator->hdlc_code = HDLC_FLAG;
  iterator->hdlc_count = pre;
  iterator->hdlc_post = post;
  iterator->hdlc_tail = tail;
  iterator->scramble = scramble;
  iterator->data_buff = pkt;
  iterator->data_size = len;
  uint16_t crc = calc_crc16(pkt, 0, len);
  iterator->crc[0] = crc & 0xFF;
  iterator->crc[1] = crc >> 8;
  iterator->no_write = false;
  iterator->state = ITERATE_PREAMBLE;
}


/**
 * @brief   Write NRZI stream data to buffer.
 * @post    NRZI encoded bits are written to the stream.
 * @notes   If counting only is active data is not written but simply counted.
 *
 * @param[in]   iterator    pointer to an @p iterator object.
 * @param[in]   bit         the bit to be written.
 *
 * @return  status.
 * @retval  true indicates the requested quantity of output bytes was reached.
 * @retval  false indicates the requested quantity of output bytes not yet reached.
 *
 * @notapi
 */
static bool pktIteratorWriteStreamBit(tx_iterator_t *iterator, uint8_t bit) {
  /* If new output buffer byte clear it first if write is enabled. */
  if ((iterator->out_index % 8 == 0) && (iterator->no_write == false))
    iterator->out_buff[iterator->out_index >> 3] = 0;

  /* Mask to bit 0 only. */
  bit &= 1;

  /* Keep track of HDLC for RLL detection. */
  iterator->hdlc_hist <<= 1;
  iterator->hdlc_hist |= bit;

  if (iterator->scramble) {
    iterator->lfsr <<= 1;
    /* Scramble is ^16 and ^11 but we shifted up one. */
    iterator->lfsr |= (bit ^ (iterator->lfsr >> 17)
        ^ (iterator->lfsr >> 12)) & 0x1;
    bit = iterator->lfsr & 0x1;
  }

  /* NRZI encode bit. */
  iterator->nrzi_hist ^= (bit == 0) ? 0x1 : 0x0;

  /* Write NRZI bit to current byte. */
  if (iterator->no_write == false)
    iterator->out_buff[iterator->out_index >> 3] |=
        (iterator->nrzi_hist & 0x1) << (iterator->out_index % 8);

  /* If byte was filled then check quantity status. */
  if ((++iterator->out_index % 8) == 0) {
    if ((++iterator->out_count) == iterator->qty)
      return true;
  }
  return false;
}

/**
 * @brief   Encode frame HDLC byte.
 * @pre     Iterator object initialized and buffer pointer set.
 * @post    HDLC octet is written to the stream unless counting only is active.
 *
 * @param[in]   iterator   pointer to an @p iterator object.
 *
 * @return  status.
 * @retval  true indicates the requested quantity of bytes has been reached.
 * @retval  false indicates the requested quantity of bytes not reached.
 *
 * @notapi
 */
static bool pktEncodeFrameHDLC(tx_iterator_t *iterator) {
  do {
    uint8_t bit = (iterator->hdlc_code >> (iterator->inp_index++ % 8)) & 0x1;
    if ((iterator->inp_index % 8) == 0)
      iterator->hdlc_count--;
    if (pktIteratorWriteStreamBit(iterator, bit))
      return true;
  } while ((iterator->inp_index % 8) != 0);
  return false;
}

/**
 * @brief   Encode frame data byte.
 * @pre     Iterator object initialized and buffer pointer set.
 * @post    Data out is written to the stream unless counting only is active.
 * @notes   Data out size may expand due to RLL encoding.
 * @notes   The required output quantity may be reached on an RLL inserted bit.
 *
 * @param[in]   iterator   pointer to an @p iterator object.
 *
 * @return  status.
 * @retval  true indicates the requested quantity of bytes has been reached.
 * @retval  false indicates the requested quantity of bytes not reached.
 *
 * @notapi
 */
static bool pktEncodeFrameData(tx_iterator_t *iterator) {
  do {
    /* Next apply RLL encoding for the packet data pay load. */
    if ((iterator->hdlc_hist & HDLC_RLL_SEQUENCE) == HDLC_RLL_SEQUENCE) {
      iterator->rll_count++;
      /* Insert RLL 0 to output stream. */
      if (pktIteratorWriteStreamBit(iterator, 0))
          return true;
    }
    /* Get data bit and advance index. */
    uint8_t byte = iterator->data_buff[iterator->inp_index >> 3];
    uint8_t bit = (byte >> (iterator->inp_index++ % 8)) & 0x1;
    if ((iterator->inp_index % 8) == 0)
      iterator->data_size--;
    /* Write data bit to stream. */
    if (pktIteratorWriteStreamBit(iterator, bit))
      return true;
  } while ((iterator->inp_index % 8) != 0);
  /* End of input byte. */
  return false;
}

/**
 * @brief   Encode frame stream for transmission.
 * @pre     The iterator has to be initialized before use.
 * @post    When the stream is complete the iterator may be re-used.
 * @notes   The iterator allows a frame to be encoded in chunks.
 * @notes   The calling function may request output chunk sizes from 1 byte up.
 * @notes   A quantity of 0 will return the number of bytes pending only.
 * @notes   In this case no data is actually written to the stream.
 *
 * @param[in]   iterator   pointer to an @p iterator object.
 * @param[in]   stream     pointer to buffer to write stream data.
 * @param[in]   qty        the requested quantity of stream out bytes.
 *                         requesting 0 will return the stream
 *                         output pending size.
 *
 * @return  number of bytes encoded or remaining to be encoded.
 * @retval  zero indicates the iterator is not initialized or is finished.
 * @retval  requested size is returned while encoding continues.
 * @retval  less than requested size is returned when encoding is ending.
 *          Or...
 * @retval  number of bytes remaining to be encoded for quantity zero request.
 *
 * @api
 */
uint16_t pktStreamEncodingIterator(tx_iterator_t *iterator,
                                   uint8_t *stream, uint16_t qty) {

  if (qty == 0) {
    tx_iterator_t saved;

    /* Save state. */
    saved = *iterator;
    iterator->no_write = true;

    /* Count the number of bytes remaining to output to the stream. */
    uint16_t remain = pktStreamEncodingIterator(iterator, NULL,
                                                ITERATOR_MAX_QTY);

    /* Restore state. */
    *iterator = saved;
    return remain;
  }

  /*
   * Each call specifies a quantity and stream buffer.
   * The stream data is written from index 0 of the buffer.
   */
  iterator->out_count = 0;
  iterator->qty = qty;
  iterator->out_index = 0;

  chDbgAssert((stream != NULL) || (iterator->no_write == true),
              "no stream buffer allocated");

  iterator->out_buff = stream;

  while (true) {
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
      while (iterator->hdlc_count > 0) {
        if (pktEncodeFrameHDLC(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
      } /* End while. */
      iterator->inp_index = 0;
      iterator->state = ITERATE_FRAME;
      continue;
      } /* End case ITERATE_PREAMBLE. */

    case ITERATE_FRAME: {
      /*
       * Output frame data bytes in requested chunk size.
       */
      while (iterator->data_size > 0) {
        /* Consume input bytes until count reached. */
        if (pktEncodeFrameData(iterator))
          /* True means the requested output count has been reached. */
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
      while (iterator->data_size > 0) {
        /* Consume bytes until count reached. */
        if (pktEncodeFrameData(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
      }
      /* Frame CRC consumed. */
      iterator->state = ITERATE_CLOSE;
      iterator->hdlc_count = iterator->hdlc_post;
      iterator->hdlc_code = HDLC_FLAG;
      iterator->inp_index = 0;
      continue;
      } /* End case ITERATE_CRC. */

    case ITERATE_CLOSE: {
      /*
       * Output closing flags.
       * RLL encoding is not used as these are HDLC flags.
       */
      while (iterator->hdlc_count > 0) {
        if (pktEncodeFrameHDLC(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
      } /* End while. */
      iterator->state = ITERATE_TAIL;
      /* Tail length. */
      iterator->hdlc_count = iterator->hdlc_tail;
      iterator->hdlc_code = HDLC_ZERO;
      iterator->inp_index = 0;
      continue;
      } /* End case ITERATE_CLOSE. */

    case ITERATE_TAIL: {
      /*
       * Output tail.
       * RLL encoding is not used as these are idle flags.
       * This keeps the data tail clean for the receiver.
       */
      while (iterator->hdlc_count > 0) {
        if (pktEncodeFrameHDLC(iterator))
          /* True means the requested count has been reached. */
          return iterator->qty;
      } /* End while. */
      /* Account for RLL inserted bits. */
      iterator->hdlc_count = ((iterator->rll_count + 7) / 8);
      iterator->state = ITERATE_FINAL;
      continue;
    } /* End case ITERATE_TAIL. */

    case ITERATE_FINAL: {
      if (iterator->hdlc_count <= iterator->qty) {
        iterator->state = ITERATE_END;
        return (iterator->out_count + iterator->hdlc_count);
      } else {
        iterator->hdlc_count -= iterator->qty;
        return iterator->qty;
      }
    } /* End case ITERATE_FINAL. */
    } /* End switch on state. */
  } /* End while. */
} /* End function. */

/** @} */
