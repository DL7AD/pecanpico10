/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include "pktconf.h"

/**
 * @brief   Extract HDLC from AFSK.
 * @post    The HDLC decode control object will be updated.
 *
 * @param[in]   myDriver   pointer to an @p decodeHDLC structure.
 *
 * @return  status of operation as a token code
 *
 * @api
 */
hdlc_token_t pktExtractHDLCfromAFSK(pkt_hdlc_decode_t *myHDLC) {

  /* Shift prior HDLC bits up before adding new bit. */
  myHDLC->hdlc_bits <<= 1;
  /* Same tone indicates a 1. */
  myHDLC->hdlc_bits |= (myHDLC->tone_freq == myHDLC->prior_freq) ? 0x01 : 0x00;
  /* Update the prior frequency. */
  myHDLC->prior_freq = myHDLC->tone_freq;

  if((myHDLC->hdlc_bits & HDLC_BIT_MASK) == HDLC_RLL_BIT) {
    /*
     * The stuffed bit is discarded.
     * We just wait for next HDLC bit.
     */
    return HDLC_TOK_RLL;
  }

  /* Shift the prior AX25 bits and add the new bit. */
  myHDLC->current_byte >>= 1;
  myHDLC->current_byte &= 0x7F;
  myHDLC->current_byte |= (myHDLC->hdlc_bits & 0x01) ? 0x80 : 0x00;

  /* Check if a byte has been accumulated or if still searching for sync. */
  if((++myHDLC->bit_index % 8U == 0)
      || (myHDLC->frame_state == HDLC_FRAME_SEARCH)) {

    /* Process HDLC stream based on frame state.  */
    switch(myHDLC->frame_state) {

    /* Frame opening sync pattern searching. */
    case HDLC_FRAME_SEARCH: {
      /*
       *  Frame start not yet detected.
       * Check for opening HDLC flag sequence.
       */
      if(
          ((myHDLC->hdlc_bits & HDLC_SYNC_MASK_A) == HDLC_SYNC_OPEN_A)
          ||
          ((myHDLC->hdlc_bits & HDLC_SYNC_MASK_B) == HDLC_SYNC_OPEN_B)
      ) {


        /* Reset data bit/byte index. */
        myHDLC->bit_index = 0;

        /*
         * Contiguous HDLC flags in the preamble will be handled in SYNC state.
         */
        myHDLC->frame_state = HDLC_FRAME_SYNC;
        return HDLC_TOK_SYNC;
      }
      return HDLC_TOK_FEED;
    } /* End case FRAME_SEARCH. */

    /* A frame opening sync pattern has been detected. */
    case HDLC_FRAME_SYNC: {
      switch(myHDLC->hdlc_bits & HDLC_BIT_MASK) {
      case HDLC_FLAG: {
        /*
         * Another preamble HDLC flag. Continue waiting for data.
         */
        return HDLC_TOK_FEED;
      } /* End case HDLC_FLAG. */

      case HDLC_RESET: {
        /*
         *  Can be a real HDLC reset or more likely incorrect bit sync.
         *  Since the decoder is in sync phase just go back to search.
         *  No data has been captured.
         */
        myHDLC->frame_state = HDLC_FRAME_SEARCH;
        return HDLC_TOK_RESET;
      } /* End case HDLC_FRAME_SYNC. */

      default:
        /* If not an HDLC pattern then transition to open state. */
        myHDLC->frame_state = HDLC_FRAME_OPEN;
        /* A data byte is available. */
        return HDLC_TOK_DATA;
      } /* End switch on HDLC code. */
    } /* End case HDLC_FRAME_SYNC. */

    /* Data (non HDLC) has been processed. Handle both data and HDLC now. */
    case HDLC_FRAME_OPEN: {
      switch(myHDLC->hdlc_bits & HDLC_BIT_MASK) {
      case HDLC_FLAG: {

        /*
         * An HDLC flag here should close the frame.
         * If the frame has sufficient data the AFSK decoder can close it.
         * If not assume sync will be restarted.
         */
        myHDLC->frame_state = HDLC_FRAME_SEARCH;

        /* Reset HDLC processor data bit/byte index. */
        myHDLC->bit_index = 0;
        return HDLC_TOK_FLAG;
      } /* End case HDLC_FRAME_OPEN. */

      case HDLC_RESET: {
        /*
         *  Can be a real HDLC reset or more likely incorrect bit sync.
         *  The decoder is in data state so go back to search.
         *  The AFSK decoder will decide what to do.
         */
        myHDLC->frame_state = HDLC_FRAME_SEARCH;

        /* Reset HDLC processor data bit/byte index. */
        myHDLC->bit_index = 0;

        return HDLC_TOK_RESET;
      } /* End case HDLC_RESET. */

      default:
        /* Otherwise indicate there is a data byte. */
        return HDLC_TOK_DATA;
      } /* End switch on HDLC code. */
    } /* End case HDLC_FRAME_OPEN. */
    } /* End switch on decoder state. */
  } /* End if byte. */
  return HDLC_TOK_FEED;
}

/** @} */
