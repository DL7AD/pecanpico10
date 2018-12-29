/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/* produce intel hex file output */

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#define MAX_HEX_LINE 32   /* the maximum number of bytes to put in one line */

void hexout(BaseSequentialStream * fhex, uint8_t byte, bool end) {
  static bool writing_in_progress = false;
  static uint8_t buffer_pos, byte_buffer[MAX_HEX_LINE];
  static int8_t sum;
  static uint16_t output_addr;

  if (!writing_in_progress) {
      /* initial condition setup */
      buffer_pos = 0;
      output_addr = 0;
      sum = 0;
      writing_in_progress = true;
  }

  if ((buffer_pos == MAX_HEX_LINE) || ((end) && (buffer_pos > 0)) ) {
      /* it's time to dump the buffer to a line in the file */
      chprintf(fhex, ":%02x%04x00", buffer_pos, output_addr);
      sum = buffer_pos + (output_addr & 0xFF) + ((output_addr >> 8) & 0xFF);
      uint8_t i;
      for (i = 0; i < buffer_pos; i++) {
          chprintf(fhex, "%02x", byte_buffer[i]);
          sum += byte_buffer[i];
      }
      chprintf(fhex, "%02x\r\n", (-sum) & 0xFF);
      output_addr += MAX_HEX_LINE;
      buffer_pos = 0;
      sum = 0;
  }

  if (end) {
      chprintf(fhex, ":00000001FF\r\n");  /* end of file marker */
      writing_in_progress = false;
      return;
  }
  byte_buffer[buffer_pos++] = byte;
  sum += byte;
}

