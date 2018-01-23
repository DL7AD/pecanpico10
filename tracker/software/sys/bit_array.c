/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include "bit_array.h"

inline int bindex(int b) { return b / BIT_WORD_SIZE; }
inline int boffset(int b) { return b % BIT_WORD_SIZE; }

void set_bit(int b, bit_word_t data[]) {
  data[bindex(b)] |= 1 << (boffset(b));
}

void clear_bit(int b, bit_word_t data[]) {
  data[bindex(b)] &= ~(1 << (boffset(b)));
}

void copy_bit(int b, bit_word_t data[], int v) {
  data[bindex(b)] |= (v & 1) << (boffset(b));
}

int get_bit(int b, bit_word_t data[]) {
  return (data[bindex(b)] >> boffset(b)) & 1;
}

void write_bit(int b, int d, bit_word_t data[]) {
  if(d & 1)
    set_bit(b, data);
  else
    clear_bit(b, data);
}

/*
 * Get a masked set of bits from index b of array.
 * The mask does not need to be contiguous bits.
 * The mask size may be up to bit_word_t bits in size.
 * The bits can be extracted across bit_word boundaries.
 */
bit_word_t mask_get_bits(int b, bit_word_t data[], bit_word_t mask) {
  int r = 0;
  while(mask != 0) {
    r |= (data[bindex(b)] >> boffset(b)) & (mask & 1);
    b++;
    r <<= 1;
    mask >>= 1;
  }
  return r;
}

void shift_all_bits(bit_word_t data[], size_t bits_in_array) {
  size_t i;
  for(i = BIT_ARRAY_SIZE(bits_in_array) - 1; i > 0; i--) {
    data[i] <<= 1U;
    data[i] |= (data[i - 1] >> (BIT_WORD_SIZE - 1)) & 1U;
  }
  data[0] <<= 1U;
}

void clear_all_bits(bit_word_t data[], size_t bits_in_array) {
  /* set all elements of data to zero */
  size_t i;
  for(i = 0; i < BIT_ARRAY_SIZE(bits_in_array); i++) {
    data[i] = 0;
  }
}

void set_all_bits(bit_word_t data[], size_t bits_in_array) {
  /* set all elements of data to zero */
  size_t i;
  for(i = 0; i < BIT_ARRAY_SIZE(bits_in_array); i++) {
    data[i] = (bit_word_t)-1;
  }
}

