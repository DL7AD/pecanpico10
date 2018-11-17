/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    ax25_dump.c
 * @brief   Packet dump utility.
 *
 * @addtogroup DSP
 * @{
 */

#include "pktconf.h"

/* Buffer and size params for serial terminal output. */
char serial_buf[1024];
int serial_out;

/* Access control semaphore. */
extern binary_semaphore_t stream_out_sem;

void pktDumpAX25Frame(ax25char_t *frame_buffer,
                      ax25size_t frame_size, ax25_select_t which) {
  if(which == AX25_DUMP_ALL || which == AX25_DUMP_RAW) {
    ax25size_t bufpos;
    ax25size_t bufpos_a = 0;
    /* Write out a buffer line as hex first. */
    for(bufpos = 0; bufpos < frame_size; bufpos++) {
        if((bufpos + 1) % DUMP_LINE_LENGTH == 0) {
            serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
                                    "%02x\r\n", frame_buffer[bufpos]);
            pktWrite((uint8_t *)serial_buf, serial_out);
            /* Write out full line of converted ASCII under hex.*/
            bufpos_a = (bufpos + 1) - DUMP_LINE_LENGTH;
            do {
                char asciichar = frame_buffer[bufpos_a];
                if(asciichar == 0x7e) {
                    asciichar = '^';
                } else {
                    asciichar >>= 1;
                    if(!((asciichar >= 0x70 && asciichar < 0x7a)
                        || (asciichar > 0x2f && asciichar < 0x3a)
                        || (asciichar > 0x40 && asciichar < 0x5b))) {
                        asciichar = 0x20;
                    } else if(asciichar >= 0x70 && asciichar < 0x7a) {
                        asciichar &= 0x3f;
                    }
                }
                if((bufpos_a + 1) % DUMP_LINE_LENGTH == 0) {
                    serial_out = chsnprintf(serial_buf,
                                            sizeof(serial_buf),
                                            " %c\r\n", asciichar);
                } else {
                    serial_out = chsnprintf(serial_buf,
                                            sizeof(serial_buf),
                                            " %c ", asciichar);
                }
                pktWrite((uint8_t *)serial_buf, serial_out);
            } while(bufpos_a++ < bufpos);
        } else {
            serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
                                    "%02x ", frame_buffer[bufpos]);
            pktWrite((uint8_t *)serial_buf, serial_out);
        }
    } /* End for(bufpos = 0; bufpos < frame_size; bufpos++). */
    serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "\r\n");
    pktWrite((uint8_t *)serial_buf, serial_out);
    /* Write out remaining partial line of converted ASCII under hex. */
    do {
        char asciichar = frame_buffer[bufpos_a];
        if(asciichar == 0x7e) {
            asciichar = '^';
        } else {
            asciichar >>= 1;
            if(!((asciichar >= 0x70 && asciichar < 0x7a)
                || (asciichar > 0x2f && asciichar < 0x3a)
                || (asciichar > 0x40 && asciichar < 0x5b))) {
                asciichar = 0x20;
            } else if(asciichar >= 0x70 && asciichar < 0x7a) {
                asciichar &= 0x3f;
            }
        }
        serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
                                " %c ", asciichar);
        pktWrite((uint8_t *)serial_buf, serial_out);
    } while(++bufpos_a < bufpos);
    serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "\r\n");
    pktWrite((uint8_t *)serial_buf, serial_out);
  } /* End raw dump. */
}


void pktDiagnosticOutput(packet_svc_t *packetHandler,
                         pkt_data_object_t *myPktFIFO) {
  //chMtxLock(&debug_mtx);
  chBSemWait(&stream_out_sem);
  /* Buffer and size params for serial terminal output. */
    char serial_buf[1024];
    int serial_out;

  /* Packet buffer. */
  ax25char_t *frame_buffer = myPktFIFO->buffer;
  uint16_t frame_size = myPktFIFO->packet_size;

  if(pktIsBufferValidAX25Frame(myPktFIFO)) {

      uint16_t magicCRC = calc_crc16(frame_buffer, 0, frame_size);

      float32_t good = (float32_t)packetHandler->good_count
          / (float32_t)packetHandler->valid_count;
      serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
          "AFSK... mode: %s, factory: %s, status: %x"
          ", packet count: %u sync count: %u"
          " valid frames: %u"
          " good frames: %u (%.2f%%), bytes: %u"
          ", CRCm: %04x\r\n",
          ((packetHandler->usr_callback == NULL) ? "polling" : "callback"),
          packetHandler->pbuff_name,
          myPktFIFO->status,
          packetHandler->sync_count,
          packetHandler->frame_count,
          packetHandler->valid_count,
          packetHandler->good_count,
          (good * 100),
          frame_size,
          magicCRC
      );
      strmWrite(DBG_INFO, (uint8_t *)serial_buf, serial_out);
      /* Dump the frame contents out. */
      pktDumpAX25Frame(frame_buffer, frame_size, AX25_DUMP_RAW);
  } else { /* End if valid frame. */
    serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
                        "Invalid frame, status %x, bytes %u\r\n",
                        myPktFIFO->status, myPktFIFO->packet_size);
    strmWrite(DBG_INFO, (uint8_t *)serial_buf, serial_out);
  }

  chBSemSignal(&stream_out_sem);
  //chMtxUnlock(&debug_mtx);
}


/** @} */
