/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    pkttypes.h
 * @brief   Types definitions.
 * @details Include this file as required in a source code module.
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

/* Radio parameters. */

/* Radio frequency in Hz. */
typedef uint32_t radio_freq_t;

/* Channel step in Hz. */
typedef uint16_t channel_hz_t;

/* Channel selector for radio frequency. */
typedef uint8_t radio_ch_t;

/* Radio squelch setting. */
typedef uint8_t radio_squelch_t;

typedef int8_t  radio_pwr_t;

typedef int8_t  radio_signal_t;

/**
 * @brief   Definition of radio unit ID.
 * @details Defines the radio unit used in configuring and enabling a radio.
 *
 */
typedef enum radioUnit {
  PKT_RADIO_NONE = 0,
  PKT_RADIO_1
} radio_unit_t;

typedef enum radioTypes {
  SI4464,
  SI4463
} radio_type_t;

typedef enum radioMode {
  RADIO_OFF,
  RADIO_RX,
  RADIO_TX
} radio_mode_t;

typedef uint8_t ax25char_t;

typedef int32_t gps_coord_t;

typedef int32_t gps_alt_t;

typedef uint16_t aprs_sym_t;

typedef uint32_t link_speed_t;

typedef uint16_t volt_level_t;

#endif /* PKT_PKTTYPES_H_ */
