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

/**
 * @brief Radio operating parameters.
 */

typedef uint32_t    radio_freq_hz_t;    /**< Base radio frequency.          */
typedef uint16_t    radio_chan_hz_t;    /**< Radio channel step size.       */
typedef uint8_t     radio_ch_t;         /**< Radio channel number.          */
typedef uint8_t     radio_squelch_t;    /**< RX squelch threshold.          */
typedef int8_t      radio_pwr_t;        /**< Radio power level (0- 0x7F)    */
typedef uint8_t     radio_signal_t;     /**< RX signal level.               */
typedef uint16_t    radio_dev_hz_t;     /**< TX deviation.                  */
typedef uint32_t    radio_clock_t;      /**< XO frequency for radio chip.   */
typedef int16_t     radio_temp_t;       /**< Chip temperature.              */
typedef uint16_t    radio_payload_t;    /**< Packet payload size.           */

/**
 * @brief   Definition of radio unit ID.
 * @details Defines the radio unit used in configuring and enabling a radio.
 *
 */
typedef enum radioUnit {
  PKT_RADIO_NONE = 0,
  PKT_RADIO_1
} radio_unit_t;

/* Radio chip details. */
typedef uint16_t    radio_part_t;       /**< Radio part number.             */
typedef uint8_t     radio_rev_t;        /**< Radio part revision.           */
typedef uint16_t    radio_patch_t;      /**< Radio patch ID.                */

typedef struct radioData {
  radio_part_t  radio_part;
  radio_rev_t   radio_rom_rev;
  radio_patch_t radio_patch;
} radio_data_t;

/*
 * Specify radio family.
 * Specific radio in family is identified dynamically.
 */
typedef enum radioTypes {
  SI446X
} radio_type_t;

/* Used as common enum in actions performed on radio (e.g. lock). */
typedef enum radioMode {
  RADIO_RX,
  RADIO_TX
} radio_mode_t;

/* Forward declaration. */
//typedef struct radioBand radio_band_t;
typedef struct packetHandlerData packet_svc_t;
typedef struct AFSK_data AFSKDemodDriver;
typedef struct indicatorIO indicator_io_t;

/* Type for a radio band. */
typedef struct radioBand {
  radio_freq_hz_t  start;
  radio_freq_hz_t  end;
  radio_chan_hz_t  step;
} radio_band_t;

typedef struct radioConfig {
  radio_unit_t      unit;
  radio_type_t      type;
  packet_svc_t      *pkt;
  AFSKDemodDriver   *afsk;
  void              *cfg;
  void              *dat;           /**< TODO: Make this struct a common minimum of radio data. */
  radio_freq_hz_t   def_aprs;       /**< A frequency in one of the bands. */
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

typedef uint32_t statusflags_t;

#endif /* PKT_PKTTYPES_H_ */
