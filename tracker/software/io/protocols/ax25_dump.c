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

void pktDumpAX25Frame(ax25char_t *frame_buffer,
                      ax25size_t frame_size, ax25_select_t which) {
  if(which == AX25_DUMP_ALL || which == AX25_DUMP_RAW) {
    ax25size_t bufpos;
    ax25size_t bufpos_a = 0;
    /* Write out a buffer line as hex first. */
    for(bufpos = 0; bufpos < frame_size; bufpos++) {
        if((bufpos + 1) % LINE_LENGTH == 0) {
            serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
                                    "%02x\r\n", frame_buffer[bufpos]);
            chnWrite(diag_out, (uint8_t *)serial_buf, serial_out);
            /* Write out full line of converted ASCII under hex.*/
            bufpos_a = (bufpos + 1) - LINE_LENGTH;
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
                if((bufpos_a + 1) % LINE_LENGTH == 0) {
                    serial_out = chsnprintf(serial_buf,
                                            sizeof(serial_buf),
                                            " %c\r\n", asciichar);
                } else {
                    serial_out = chsnprintf(serial_buf,
                                            sizeof(serial_buf),
                                            " %c ", asciichar);
                }
                chnWrite(diag_out, (uint8_t *)serial_buf, serial_out);
            } while(bufpos_a++ < bufpos);
        } else {
            serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
                                    "%02x ", frame_buffer[bufpos]);
            chnWrite(diag_out, (uint8_t *)serial_buf, serial_out);
        }
    } /* End for(bufpos = 0; bufpos < frame_size; bufpos++). */
    serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "\r\n");
    chnWrite(diag_out, (uint8_t *)serial_buf, serial_out);
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
        chnWrite(diag_out, (uint8_t *)serial_buf, serial_out);
    } while(++bufpos_a < bufpos);
    serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "\r\n");
    chnWrite(diag_out, (uint8_t *)serial_buf, serial_out);
  } /* End raw dump. */

  if(which == AX25_DUMP_ALL || which == AX25_DUMP_APRS) {
    uint16_t magicCRC = calc_crc16(frame_buffer, 0, frame_size);
    if(magicCRC == CRC_INCLUSIVE_CONSTANT) {
      /* CRC is good => decode APRS packet */
      packet_t pp = ax25_from_frame(frame_buffer, frame_size - 2);
      if(pp != NULL) {
        char rec[1024];
        unsigned char *pinfo;
        ax25_format_addrs(pp, rec);
        ax25_get_info(pp, &pinfo);
        serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
                                "%s", rec);
        chnWrite(diag_out, (uint8_t *)serial_buf, serial_out);
        for(uint32_t i=0; pinfo[i]; i++) {
          if(pinfo[i] < 32 || pinfo[i] > 126) {
            /* Printable char */
            serial_out = chsnprintf(serial_buf,
                                    sizeof(serial_buf),
                                    "<0x%02x>", pinfo[i]);
          } else {
            serial_out = chsnprintf(serial_buf,
                                    sizeof(serial_buf),
                                    "%c", pinfo[i]);
          }
          chnWrite(diag_out, (uint8_t *)serial_buf, serial_out);
        }
        serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
                                "\r\n");
        chnWrite(diag_out, (uint8_t *)serial_buf, serial_out);
        ax25_delete(pp);
      } else {
          serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
                                  "APRS: Error in packet\r\n");
          chnWrite(diag_out, (uint8_t *)serial_buf, serial_out);
      }
    } else {
        serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
                                "APRS: Bad CRC\r\n");
        chnWrite(diag_out, (uint8_t *)serial_buf, serial_out);
    } /* End CRC check. */
  } /* End APRS dump. */
}

/** @} */
