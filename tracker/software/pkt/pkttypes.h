/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    pkttypes.h
 * @brief   Types definitions.
 * @details Include this file in each source code module.
 *
 * @addtogroup pktconfig
 * @details Decoder type definitions.
 * @{
 */

#ifndef PKT_PKTTYPES_H_
#define PKT_PKTTYPES_H_

#ifdef PKT_IS_TEST_PROJECT
/* Modulation type. */
typedef enum {
  MOD_NONE,
  MOD_AFSK,
  MOD_2FSK
} mod_t;

#endif

#ifdef PKT_IS_TEST_PROJECT

inline const char *getModulation(uint8_t key) {
    const char *val[] = {"NONE", "AFSK", "2FSK"};
    return val[key];
};
#endif

#endif /* PKT_PKTTYPES_H_ */
