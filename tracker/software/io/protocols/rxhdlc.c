/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include "pktconf.h"

/*
 * Here we handle the generation of HDLC bits and data bits.
 * HDLC flag is detected at the raw link level.
 * RLL encoding (bit stuffing) is also handled and removed from the data stream.
 *
 */
bool pktExtractHDLCfromAFSK(AFSKDemodDriver *myDriver) {

  packet_rx_t *myHandler = myDriver->packet_handler;

  /* Shift prior HDLC bits up before adding new bit. */
  myDriver->hdlc_bits <<= 1;
  myDriver->hdlc_bits &= 0xFE;
  /* Same tone indicates a 1. */
  if(myDriver->tone_freq == myDriver->prior_freq) {
    myDriver->hdlc_bits |= 1;
  }
  /* Update the prior frequency. */
  myDriver->prior_freq = myDriver->tone_freq;

  /*
   * Check if we are in AX25 data capture mode.
   * If so check and act on HDLC codes otherwise just store data.
   */
  if(myDriver->frame_state == FRAME_OPEN) {
    switch(myDriver->hdlc_bits & HDLC_CODE_MASK) {
      case HDLC_FLAG: {
        /*
         * An HDLC flag after minimum packet size terminates the AX25 frame.
         */
        if(myHandler->active_packet_object->packet_size >= AX25_MIN_FRAME) {
          /*
           * Frame size is valid.
           * Dump any bits already put into the AX25 byte.
           */
          myDriver->bit_index = 0;
          myHandler->packet_count++;
          /* Inform thread loop of end of frame. */
          myDriver->frame_state = FRAME_CLOSE;
          return true;
        } /* End AX25 frame size check. */

        /*
         * Frame size is not valid.
         * HDLC sync still in progress.
         * Reset AX25 counts and wait for next HDLC bit.
         */
        pktResetDataBuffer(myHandler->active_packet_object);
        myDriver->bit_index = 0;
        return true;
      } /* End case. */

      case HDLC_RESET: {
        /*
         *  Can be a real HDLC reset or most likely incorrect bit sync.
         *  TODO: Figure out correct handling...
         */
        myDriver->frame_state = FRAME_RESET;
        myDriver->active_demod_object->status |= EVT_HDLC_RESET_RCVD;
        pktAddEventFlags(myHandler, EVT_HDLC_RESET_RCVD);
        return false;
      } /* End case. */

      default: {
       /* Check for RLL encoding inserted ("stuffed") bit in the bit stream. */
       if((myDriver->hdlc_bits & HDLC_RLL_MASK) == HDLC_RLL_BIT) {
         /*
          * The stuffed bit is discarded.
          * We just wait for next HDLC bit.
          */
         return true;
       }
       /*
        * Else we have a non-special pattern.
        * Just put the bit into the AX25 data.
        * AX25 data bits arrive MSB -> LSB.
        */
       myDriver->current_byte &= 0x7F;
       if((myDriver->hdlc_bits & 0x01) == 1) {
         myDriver->current_byte |= 0x80;
       }
       /* Check if we have a byte accumulated. */
       if(++myDriver->bit_index == 8U) {
         myDriver->bit_index = 0;
         if(pktStoreBufferData(myHandler->active_packet_object,
                         myDriver->current_byte)) {
           return true;
         }
         pktAddEventFlags(myHandler, EVT_AX25_BUFFER_FULL);
         myDriver->active_demod_object->status |= EVT_AX25_BUFFER_FULL;
         return false;
       }
       /* Else shift the prior bit to make space for next bit. */
       myDriver->current_byte >>= 1;
       //myDriver->current_byte &= 0x7F;
       return true;
      } /* End case default. */
    } /* End switch. */
  } /* Else not frame_open... */
  /*
   *  Frame start not yet detected.
   * Check for opening HDLC flag sequence.
   */
  if(
      ((myDriver->hdlc_bits & HDLC_FRAME_MASK_A) == HDLC_FRAME_OPEN_A)
      ||
      ((myDriver->hdlc_bits & HDLC_FRAME_MASK_B) == HDLC_FRAME_OPEN_B)
    ) {
    myDriver->frame_state = FRAME_OPEN;

    /* Reset AX25 data indexes. */
    myHandler->active_packet_object->packet_size = 0;
    myDriver->bit_index = 0;

    /*
     * AX25 data buffering is now enabled.
     * Data bytes will be written to the AX25 buffer.
     */
  }
  return true;
} /* End function. */

/** @} */
