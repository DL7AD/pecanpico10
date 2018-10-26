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

typedef uint16_t deviation_hz_t;

/**
 * @brief   Definition of radio unit ID.
 * @details Defines the radio unit used in configuring and enabling a radio.
 *
 */
typedef enum radioUnit {
  PKT_RADIO_NONE = 0,
  PKT_RADIO_1
} radio_unit_t;

typedef uint16_t    radio_part_t;

typedef uint8_t     radio_rev_t;

typedef uint16_t    radio_patch_t;

/*
 * Specify radio family.
 * Specific radio in family is identified dynamically.
 */
typedef enum radioTypes {
  SI446X
} radio_type_t;

typedef enum radioMode {
  RADIO_OFF,
  RADIO_RX,
  RADIO_TX,
  RADIO_ALL
} radio_mode_t;

/* Forward declaration. */
//typedef struct radioBand radio_band_t;
typedef struct packetHandlerData packet_svc_t;
typedef struct AFSK_data AFSKDemodDriver;
typedef struct indicatorIO indicator_io_t;

/* Type for a radio band. */
typedef struct radioBand {
  radio_freq_t  start;
  radio_freq_t  end;
  channel_hz_t  step;
} radio_band_t;

typedef struct radioConfig {
  radio_unit_t      unit;
  radio_type_t      type;
  packet_svc_t      *pkt;
  AFSKDemodDriver   *afsk;
  void              *cfg;
  void              *dat;
  radio_freq_t      def_aprs;       /**< A frequency in one of the bands. */
  radio_band_t      **bands;
  indicator_io_t    *ind_set;
} radio_config_t;

typedef uint8_t ax25char_t;

typedef int32_t gps_coord_t;

typedef int32_t gps_alt_t;

typedef uint16_t aprs_sym_t;

typedef uint32_t link_speed_t;

typedef uint16_t volt_level_t;

typedef uint32_t xtal_osc_t;

#endif /* PKT_PKTTYPES_H_ */
