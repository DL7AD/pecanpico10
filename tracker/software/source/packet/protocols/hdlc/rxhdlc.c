/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include "pktconf.h"

/**
 * Array for looking up model name
 */
static const char *token[] = {HDLC_TOKEN_NAMES};

/**
 * Get pointer to token name as string
 */
const char *pktGetHDLCTokenName(uint8_t index) {
  return (index > HDLC_TOKEN_MAX ? "INVALID" : token[index]);
}

/**
 * @brief   Reset HDLC processor.
 * @post    The HDLC decode control object will be reset.
 *
 * @param[in]   myDriver   pointer to an @p decodeHDLC structure.
 *
 * @api
 */
void pktResetHDLCProcessor(pkt_hdlc_decode_t *myHDLC) {
    myHDLC->frame_state = HDLC_FLAG_SEARCH;
#if HDLC_SYNC_USE_COUNTER == TRUE
    myHDLC->flag_count = 0;
    myHDLC->lead_type = HDLC_LEAD_NONE;
#endif
 myHDLC->tone_freq = TONE_NONE;
 myHDLC->prior_freq = TONE_NONE;
 myHDLC->bit_index = 0;
 myHDLC->hdlc_bits = (int32_t)-1;
}

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
    return (myHDLC->last_token = HDLC_TOK_RLL);
  }

  /* Shift the prior AX25 bits and add the new bit. */
  myHDLC->current_byte >>= 1;
  myHDLC->current_byte &= 0x7F;
  myHDLC->current_byte |= (myHDLC->hdlc_bits & 0x01) ? 0x80 : 0x00;

  /* Check if a byte has been accumulated or if still searching for sync. */
  if((++myHDLC->bit_index % 8U == 0)
      || (myHDLC->frame_state == HDLC_FLAG_SEARCH)) {

    /* Process HDLC stream based on frame state.  */
    switch(myHDLC->frame_state) {

    /* Frame opening sync pattern searching. The decoder PLL will search at
       an aggressive rate for bit sync when HDLC processor is in this state.
       Two search patterns are looked for. One with pure HDLC flags the other
       with leading zeros followed by HDLC flags. Leading zeros are used by
       some TNCs as that results in constant NRZI transitions. */
    case HDLC_FLAG_SEARCH: {
      /* Search bit pattern as it slides by looking for a sync sequence. */
      if ((myHDLC->hdlc_bits & HDLC_SYNC_MASK_FLAG) == HDLC_SYNC_OPEN_FLAG)
        myHDLC->lead_type = HDLC_LEAD_FLAG;
      if ((myHDLC->hdlc_bits & HDLC_SYNC_MASK_ZERO) == HDLC_SYNC_OPEN_ZERO)
        myHDLC->lead_type = HDLC_LEAD_ZERO;
      if (myHDLC->lead_type != HDLC_LEAD_NONE) {
        /* Reset data bit/byte index. */
        myHDLC->bit_index = 0;
#if HDLC_SYNC_USE_COUNTER == TRUE
        /* Reset PLL sync counter. */
        myHDLC->flag_count = 0;
#endif
        /*
         * Contiguous HDLC flags in the preamble will be handled in SYNC state.
         */
        myHDLC->frame_state = HDLC_FRAME_SYNC;
        return (myHDLC->last_token = HDLC_TOK_SYNC);
      }
      return (myHDLC->last_token = HDLC_TOK_FEED);
    } /* End case FRAME_SEARCH. */

    /* An opening bit sync pattern has been detected. */
    case HDLC_FRAME_SYNC: {
      switch (myHDLC->hdlc_bits & HDLC_BIT_MASK) {
      case HDLC_FLAG: {
        /*
         * Another preamble HDLC flag. Continue waiting for data.
         */
#if HDLC_SYNC_USE_COUNTER == TRUE
        myHDLC->flag_count++;
#endif
        return (myHDLC->last_token = HDLC_TOK_FEED);
      } /* End case HDLC_FLAG. */

      case HDLC_RESET: {
        /*
         *  Can be a real HDLC reset or more likely incorrect bit sync.
         *  Since the decoder is in sync phase just go back to search.
         *  No data has been captured.
         */
        myHDLC->frame_state = HDLC_FLAG_SEARCH;
        return (myHDLC->last_token = HDLC_TOK_RESET);
      } /* End case HDLC_FRAME_SYNC. */

      default:
        /* Not an HDLC pattern. */
#if HDLC_SYNC_USE_COUNTER == TRUE
        /* Check number of contiguous flags received. This sequence is
           intended to settle the decoder PLL. */
        if (
            (myHDLC->lead_type == HDLC_LEAD_ZERO && myHDLC->flag_count >= HDLC_SYNC_COUNT_ZERO)
            ||
            (myHDLC->lead_type == HDLC_LEAD_FLAG && myHDLC->flag_count >= HDLC_SYNC_COUNT_FLAG)
            ) {
          /* A data byte is available. */
          myHDLC->frame_state = HDLC_FRAME_OPEN;
          return (myHDLC->last_token = HDLC_TOK_OPEN);
        }
        else {
        /* Discard as this is likely junk. Go back to bit level sync */
        myHDLC->frame_state = HDLC_FLAG_SEARCH;
        return (myHDLC->last_token = HDLC_TOK_FEED);
      }
#else
        /* A data byte is available. */
        myHDLC->frame_state = HDLC_FRAME_OPEN;
        return (myHDLC->last_token = HDLC_TOK_OPEN);
#endif
      } /* End switch on HDLC code. */
    } /* End case HDLC_FRAME_SYNC. */

    /* Data (non HDLC) has been processed. Handle both data and HDLC now. */
    case HDLC_FRAME_OPEN: {
      switch(myHDLC->hdlc_bits & HDLC_BIT_MASK) {
      case HDLC_FLAG: {

        /*
         * An HDLC flag here should close the frame.
         * If the frame has sufficient data the AFSK decoder can close it.
         * If not then the decoder will reset itself and HDLC processor.
         * In the case that a decode is valid we setup to consume the HDLC tail.
         */
        myHDLC->frame_state = HDLC_FRAME_TAIL;

        /* Reset HDLC processor data bit/byte index. */
        myHDLC->bit_index = 0;
        return (myHDLC->last_token = HDLC_TOK_FLAG);
      } /* End case HDLC_FRAME_OPEN. */

      case HDLC_RESET: {
        /*
         *  Can be a real HDLC reset or more likely incorrect bit sync.
         *  The HDLC processor stays in frame open state.
         *  The AFSK decoder will decide what to do.
         */
        //myHDLC->frame_state = HDLC_FLAG_SEARCH;

        /* Reset HDLC processor data bit/byte index. */
        myHDLC->bit_index = 0;

        return (myHDLC->last_token = HDLC_TOK_RESET);
      } /* End case HDLC_RESET. */

      default:
        /* Otherwise indicate there is a data byte. */
        return (myHDLC->last_token = HDLC_TOK_DATA);
      } /* End switch on HDLC code. */
    } /* End case HDLC_FRAME_OPEN. */

    /* Frame closing flag processed. */
    case HDLC_FRAME_TAIL: {
      switch(myHDLC->hdlc_bits & HDLC_BIT_MASK) {
      case HDLC_FLAG: {
        /* Consume the frame tail flags. */
        return (myHDLC->last_token = HDLC_TOK_FEED);
      } /* End case HDLC_FRAME_OPEN. */

      default:
        /* If not a flag then go into bit sync. */
        myHDLC->frame_state = HDLC_FLAG_SEARCH;
        return (myHDLC->last_token = HDLC_TOK_FEED);
      } /* End switch on HDLC code. */
    } /* End case HDLC_FRAME_TAIL. */
    } /* End switch on decoder state. */
  } /* End if byte. */
  return (myHDLC->last_token = HDLC_TOK_FEED);
}

/** @} */
