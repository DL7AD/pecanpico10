/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __APRS_H__
#define __APRS_H__

#include "config.h"
#include "si446x.h"
#include "ax25_pad.h"
#include "collector.h"

#define GSP_FIX_OLD						0x0
#define GSP_FIX_CURRENT					0x1

#define NMEA_SRC_OTHER					0x0
#define NMEA_SRC_GLL					0x1
#define NMEA_SRC_GGA					0x2
#define NMEA_SRC_RMC					0x3

#define ORIGIN_COMPRESSED				0x0
#define ORIGIN_TNC_BTEXT				0x1
#define ORIGIN_SOFTWARE					0x2
#define ORIGIN_RESERVED					0x3
#define ORIGIN_KPC3						0x4
#define ORIGIN_PICO						0x5
#define ORIGIN_OTHER_TRACKER			0x6
#define ORIGIN_DIGIPEATER_CONVERSION	0x7

#define APRS_DEVICE_CALLSIGN	    	"APECAN" // APExxx = Pecan device
//#define APRS_DEST_SSID					0

#define SYM_BALLOON						0x2F4F
#define SYM_SMALLAIRCRAFT				0x2F27
#define SYM_SATELLITE					0x5C53
#define SYM_CAR							0x2F3E
#define SYM_SHIP						0x2F73
#define SYM_DIGIPEATER					0x2F23
#define SYM_ANTENNA                     0x2F72

#define APRS_HEARD_LIST_SIZE            20

#define APRS_MAX_MSG_ARGUMENTS          10

typedef struct APRSIdentity {
  char      num[8];
  char      src[AX25_MAX_ADDR_LEN];
  char      call[AX25_MAX_ADDR_LEN];
  char      path[16];
  uint16_t  symbol;
  uint32_t  freq;
  uint8_t   pwr;
  mod_t     mod;
  uint8_t   cca;
} aprs_identity_t;

/**
 * @brief   Command handler function type.
 */
typedef msg_t (*aprscmd_t)(aprs_identity_t *id, int argc, char *argv[]);

/**
 * @brief   APRS command entry type.
 */
typedef struct {
  const char            *ac_name;           /**< @brief Command name.       */
  aprscmd_t             ac_function;        /**< @brief Command function.   */
} APRSCommand;

/* Temporary. Will be deprecated when fixed station feature is implemented. */
extern bool test_gps_enabled;

#ifdef __cplusplus
extern "C" {
#endif
  void      aprs_debug_getPacket(packet_t pp, char* buf, uint32_t len);
  packet_t  aprs_encode_position(const char *callsign, const char *path,
                                uint16_t symbol, dataPoint_t *dataPoint);
  packet_t  aprs_encode_telemetry_configuration(const char *originator,
                                               const char *path,
                                               const char *destination,
                                               uint8_t type);
  packet_t  aprs_encode_message(const char *callsign, const char *path,
                               const char *receiver, const char *text,
                               const bool ack);
  packet_t  aprs_encode_data_packet(const char *callsign, const char *path,
                                   char packetType, uint8_t *data);
  packet_t  aprs_compose_aprsd_message(const char *callsign, const char *path,
                                   const char *receiver);
  void      aprs_decode_packet(packet_t pp);
  msg_t     aprs_send_position_beacon(aprs_identity_t *id,
                                  int argc, char *argv[]);
  msg_t     aprs_send_aprsd_message(aprs_identity_t *id,
                                        int argc, char *argv[]);
  msg_t     aprs_send_aprsh_message(aprs_identity_t *id,
                                   int argc, char *argv[]);
  msg_t     aprs_execute_gpio_command(aprs_identity_t *id,
                                   int argc, char *argv[]);
  msg_t     aprs_handle_gps_command(aprs_identity_t *id,
                                   int argc, char *argv[]);
  msg_t     aprs_execute_config_command(aprs_identity_t *id,
                                   int argc, char *argv[]);
  msg_t     aprs_execute_config_save(aprs_identity_t *id,
                                  int argc, char *argv[]);
  msg_t     aprs_execute_img_command(aprs_identity_t *id,
                                   int argc, char *argv[]);
  msg_t aprs_execute_system_reset(aprs_identity_t *id,
                                  int argc, char *argv[]);
#ifdef __cplusplus
}
#endif
#endif

