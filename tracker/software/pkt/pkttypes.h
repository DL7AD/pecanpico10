/*
 * pkttypes.h
 *
 *  Created on: 15 Mar 2018
 *      Author: Bob
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
