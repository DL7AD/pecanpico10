/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef SYS_BIT_ARRAY_H_
#define SYS_BIT_ARRAY_H_

#include "ch.h"
#include <limits.h>

/* Bit array testing, shifting and other helper macros. */
typedef uint32_t bit_word_t;

typedef bit_word_t bit_buffer_t;

/* Type (mask to make safe) just to make it clear what is expected. */
typedef int8_t bit_t;

/* Use CHAR_BIT out of limits.h for bits per byte definition. */
enum { BIT_WORD_SIZE = sizeof(bit_word_t) * CHAR_BIT };

/* Use this macro in bit array size definition. */
#define BIT_ARRAY_SIZE(number) ((number / BIT_WORD_SIZE) + 1)

/*===========================================================================*/
/* Other Macro declarations.                                                 */
/*===========================================================================*/
#define SELECT_DATA_BIT(data, index) (uint8_t)((data >> index) & 1)
#define BITS_DIFFER(bits1, bits2) (((bits1)^(bits2)) & 0x01)
#define DUAL_XOR(bits1, bits2) ((((bits1)^(bits2)) & 0x03) == 0x03)

#define SIGNAL_TRANSITIONED(bits) DUAL_XOR((bits), (bits) >> 2)
#define TRANSITION_FOUND(bits) BITS_DIFFER((bits), (bits) >> 1)


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  //inline int bindex(int b);
  //inline int boffset(int b);
  void set_bit(int b, bit_word_t data[]);
  void clear_bit(int b, bit_word_t data[]);
  void copy_bit(int b, bit_word_t data[], int v);
  int get_bit(int b, bit_word_t data[]);
  void write_bit(int b, int d, bit_word_t data[]);
  bit_word_t mask_get_bits(int b, bit_word_t data[], bit_word_t mask);
  void shift_all_bits(bit_word_t data[], size_t bits_in_array);
  void clear_all_bits(bit_word_t data[], size_t bits_in_array);
  void set_all_bits(bit_word_t data[], size_t bits_in_array);
#ifdef __cplusplus
}
#endif

#endif /* SYS_BIT_ARRAY_H_ */
