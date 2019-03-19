/**
  * @see https://github.com/thasti/utrak
  */

#include "ch.h"
#include "hal.h"
#include "ublox.h"
#include "debug.h"
#include "config.h"
#include "collector.h"
#include "serialmux.h"
#include "pktconf.h"
#include "portab.h"

bool gps_enabled = false;
uint8_t gps_model = GPS_MODEL_AUTOMOTIVE;

/* Serial driver configuration for GPS. */
const SerialConfig gps_config =
{
    9600,   // baud rate
    0,      // CR1 register
    0,      // CR2 register
    0       // CR3 register
};

/* Configuration if using multiplexed serial port. */
#if GPS_USE_SERIAL_MUX == TRUE
/* Configuration for open/use and close/release. */
const SerialMuxConfig mcfg = {
      .open = {
               .rx = {
                      .line = LINE_GPS_RXD,
                      .mode = LINE_GPS_RXD_MODE
               },
               .tx = {
                      .line = LINE_GPS_TXD,
                      .mode = LINE_GPS_TXD_MODE
               }
      },
      .close = {
                .rx = {
                       .line = LINE_GPS_RXD,
                       .mode = PAL_MODE_RESET
                },
                .tx = {
                       .line = LINE_GPS_TXD,
                       .mode = PAL_MODE_RESET
                }
      },
      .sdcfg = {
                  9600,   // baud rate
                  0,      // CR1 register
                  0,      // CR2 register
                  0       // CR3 register
               }
};
#endif

/**
 * Array for looking up model name
 */
static const char *model[] = {GPS_MODEL_NAMES};

/**
 * Get pointer to model name as string
 */
const char *gps_get_model_name(uint8_t index) {
  return (index > GPS_MODEL_MAX ? "INVALID" : model[index]);
}

/*
 * Calculate checksum and inserts into buffer.
 * Calling function must allocate space in message buff for csum.
 *
 */
static bool gps_calc_ubx_csum(uint8_t *mbuf, const uint16_t mlen) {

  uint16_t i;
  uint8_t ck_a = 0, ck_b = 0;
  /* Counting sync bytes there must be at least one byte to checksum. */
  if (mlen < 5)
    return false;

  for (i = 2; i < mlen - 2; i++) {
      ck_b += (ck_a += mbuf[i]);
  }
  mbuf[mlen - 2] = ck_a;
  mbuf[mlen - 1] = ck_b;
  return true;
}

/**
 * Transmits a string of bytes to the GPS
 */
static uint8_t gps_transmit_string(uint8_t *const cmd, const uint8_t length) {
  gps_calc_ubx_csum(cmd, length);
  return sdWrite(&PKT_GPS_UART, cmd, length);
}

/**
 * Receives a single byte from the GPS and assigns to supplied pointer.
 * Returns false is there is no byte available else true
 */
static bool gps_receive_byte(uint8_t *data) {
	return sdAsynchronousRead(&PKT_GPS_UART, data, 1);
}

/**
 * @brief Get the ACK/NAK response from the GPS
 * @note  Waits a timeout for reception of an ACK/NAK message from the GPS.
 *
 * @return status of received message
 * @retval UBX_COMMAND_ACK value or UBX_COMMAND_NAK value
 *
 * @notapi
 *
 */
static uint8_t gps_receive_ack(uint8_t class_id, uint8_t msg_id,
                               sysinterval_t timeout) {

  chDbgCheck(timeout != TIME_IMMEDIATE && timeout != TIME_INFINITE);

  uint8_t match_index = 0;
  uint8_t msg_ack = 0;
  uint8_t rx_byte;

  /* Two forms of ACK message to be captured. */
  uint8_t ack[] = {0xB5, 0x62, 0x05, UBX_COMMAND_ACK, 0x02, 0x00, class_id, msg_id, 0x00, 0x00};
  uint8_t nak[] = {0xB5, 0x62, 0x05, UBX_COMMAND_NAK, 0x02, 0x00, class_id, msg_id, 0x00, 0x00};

  /* Insert checksum bytes. */
  gps_calc_ubx_csum(ack, sizeof(ack));
  gps_calc_ubx_csum(nak, sizeof(nak));

  /* Get ACK/NAK packet. */
  systime_t sNow = chVTGetSystemTime();

  while (chVTTimeElapsedSinceX(sNow) < timeout) {

    /* Receive one byte. */
    if (!gps_receive_byte(&rx_byte)) {
      chThdSleep(TIME_MS2I(1));
      continue;
    }

    /* Process one byte. */
    if (rx_byte != ack[match_index] && rx_byte != nak[match_index]) {
      match_index = 0;
      continue;
    }

    if (match_index == 3) {
      /* Capture ACK/NAK byte */
      msg_ack = rx_byte;
    }

    if (++match_index == sizeof(ack)) {
      /* Response including checksum now processed. */
      return msg_ack;
    }
  } /* End while. */
  return UBX_COMMAND_NAK;
}

/**
 * gps_receive_payload
 *
 * retrieves the payload of a packet with a given class and message-id with the retrieved length.
 * the caller has to ensure suitable buffer length!
 *
 * returns the length of the payload
 *
 */
static uint16_t gps_receive_payload(uint8_t class_id, uint8_t msg_id,
                                    uint8_t *payload, size_t size,
                                    sysinterval_t timeout) {

  chDbgCheck(size != (size_t)0 && payload != NULL);
  chDbgCheck(timeout != TIME_IMMEDIATE && timeout != TIME_INFINITE);

  uint8_t rx_byte;
  enum {UBX_A, UBX_B, CLASSID, MSGID, LEN_A, LEN_B, PAYLOAD, CK_A, CK_B} state = UBX_A;
  uint16_t payload_cnt;
  uint16_t payload_len;

  systime_t sNow = chVTGetSystemTime();

  while (chVTTimeElapsedSinceX(sNow) < timeout) {

    /* Receive one byte. */
    if (!gps_receive_byte(&rx_byte)) {
      chThdSleep(TIME_MS2I(1));
      continue;
    }

    /* Process the incoming byte. */
    switch (state) {
    case UBX_A:
      if (rx_byte == 0xB5) {
        state = UBX_B;
      }
      continue;

    case UBX_B:
      if (rx_byte == 0x62) {
        state = CLASSID;
        continue;
      }
      break;

    case CLASSID:
      if (rx_byte == class_id) {
        state = MSGID;
        continue;
      }
      break;

    case MSGID:
      if (rx_byte == msg_id) {
        state = LEN_A;
        payload_len = 0;
        continue;
      }
      break;

    case LEN_A:
      payload_len = rx_byte;
      state = LEN_B;
      continue;

    case LEN_B:
      payload_len |= ((uint16_t)rx_byte << 8);
      if (payload_len == 0) {
        return 0;
      }
      state = PAYLOAD;
      payload_cnt = 0;
      continue;

    case PAYLOAD:
      payload[payload_cnt++] = rx_byte;
      /* Check for buffer overflow. */
      if (payload_cnt > size) {
        return 0;
      }
      if (payload_cnt == payload_len) {
        state = CK_A;
      }
      continue;

    case CK_A:
      state = CK_B;
      continue;

    case CK_B:
      return payload_len;
    } /* End switch. */
    state = UBX_A;
  } /* End while. */
  return 0;
}

/**
 * @brief Execute a UBX protocol transaction.
 *
 * @param[in] cmd       pointer to array containing the command data
 * @param[in] length    length of the command data
 * @param[in] ack       boolean indicating this command produces an ACK message
 * @param[in] payload   pointer to a buffer to receive a response payload
 * @param[in] size      size of the payload buffer
 * @param[in] timeout   system ticks after which the operation times out
 *
 * @return   status of the operation
 * @retval   ACK(1) if an acknowledge request received a NAK
 * @retval   NAK(0) if an acknowledge request received a NAK
 * @retval   0 if the operation failed for a payload response
 * @retval   otherwise size of the received payload
 *
 * @notapi
 */
static uint16_t gps_ubx_transaction(uint8_t *const cmd, const uint8_t length,
                             const bool ack, uint8_t *const payload,
                             const size_t size, const sysinterval_t timeout) {

  chDbgCheck(cmd != NULL && length != 0);
  chDbgCheck(!(payload != NULL && ack == true));

#if GPS_USE_SERIAL_MUX
  uint16_t result = UBX_COMMAND_ACK;


  /* Open the serial channel. */
  msg_t msg = pktOpenMuxedSerial(&PKT_GPS_UART,
                                 &gps_config,
                                 &mcfg,
                                 timeout);
  if (msg != MSG_OK) {
    return UBX_COMMAND_NAK;
  }
  /* Proceed with the transaction now. */
  gps_transmit_string(cmd, length);
  if (payload == NULL) {
    /* No payload to be retrieved. */
    if (ack) {
      /* This command produces an ACK/NAK. */
      result = gps_receive_ack(cmd[2], cmd[3], timeout);
    }

  } else {
    /* This command returns a payload. */
    result = gps_receive_payload(cmd[2], cmd[3], payload, size, timeout);
  }

  /* Close the serial channel. */
  (void) pktCloseMuxedSerial(&PKT_GPS_UART,
                            NULL,
                            0,
                            timeout);

  return result;
#else
  if (gps_transmit_string(cmd, length) != length) {
    TRACE_ERROR("GPS  > Incomplete send of command string");
    return UBX_COMMAND_NAK;
  }
  if (payload == NULL) {
    if (ack) {
      /* This command produces an ACK/NAK. */
      return gps_receive_ack(cmd[2], cmd[3], timeout);
    }
    return UBX_COMMAND_ACK;
  }
    /* This command produces a return payload. */
    return gps_receive_payload(cmd[2], cmd[3], payload, size, timeout);
#endif
}

/**
  * gps_get_timepulse_info
  *
  */
bool gps_get_timepulse_info(tpidx_t tp, gps_tp5_t *tp5, size_t size) {
  if (!gps_enabled)
    return false;

  // Transmit request
  uint8_t tp5_req[] = {0xB5, 0x62, 0x06, 0x31, 0x01, 0x00, tp & 0x01, 0xFF, 0xFF};
  if (!gps_ubx_transaction(tp5_req, sizeof(tp5_req), false,
                           (uint8_t *)tp5, size, TIME_S2I(5))) {
    TRACE_ERROR("GPS  > NAV-SVINFO Polling FAILED");
    return false;
  }
  return true;
}

/**
  * gps_get_sv_info
  *
  */
bool gps_get_sv_info(gps_svinfo_t *svinfo, size_t size) {
  if (!gps_enabled)
    return false;

  // Transmit request
  uint8_t navsvinfo_req[] = {0xB5, 0x62, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00};
  if (gps_ubx_transaction(navsvinfo_req, sizeof(navsvinfo_req), false,
                           (uint8_t *)svinfo, size, TIME_S2I(10)) == 0) {
    TRACE_ERROR("GPS  > NAV-SVINFO Polling FAILED");
    return false;
  }
  return true;
}

/**
  * gps_get_nav_sat_info
  *
  */
bool gps_get_nav_sat_info(gps_svinfo_t *nsinfo, size_t size) {
  if (!gps_enabled)
    return false;

  // Transmit request
  uint8_t navsatinfo_req[] = {0xB5, 0x62, 0x01, 0x35, 0x00, 0x00, 0x00, 0x00};
  if (!gps_ubx_transaction(navsatinfo_req, sizeof(navsatinfo_req), false,
                           (uint8_t *)nsinfo, size, TIME_S2I(5))) {
    TRACE_ERROR("GPS  > NAV-SAT Polling FAILED");
    return false;
  }
  return true;
}

/**
 * gps_get_nav_status
 *
 */
bool gps_get_nav_status(gps_navinfo_t *navinfo, size_t size) {
  if (!gps_enabled)
    return false;
  uint8_t navstatus_req[] = {0xB5, 0x62, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
  if (gps_ubx_transaction(navstatus_req, sizeof(navstatus_req), false,
                           (uint8_t *)navinfo, size, TIME_S2I(5)) == 0) {
    TRACE_ERROR("GPS  > NAV-PVT Polling FAILED");
    return false;
  }
  return true;
}

/**
  * gps_get_fix
  *
  * retrieves a GPS fix from the module.
  * if validity flag is not set, date/time and position/altitude are
  * assumed not to be reliable!
  *
  */
bool gps_get_fix(gpsFix_t *fix) {
    static uint8_t navpvt[128];
    static uint8_t navstatus[32];

    // Transmit request
    uint8_t navpvt_req[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00};
    if (gps_ubx_transaction(navpvt_req, sizeof(navpvt_req), false,
                            navpvt, sizeof(navpvt), TIME_S2I(5)) == 0) {
      TRACE_ERROR("GPS  > NAV-PVT Polling FAILED");
      return false;
    }
    uint8_t navstatus_req[] = {0xB5, 0x62, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    if (gps_ubx_transaction(navstatus_req, sizeof(navstatus_req), false,
                            navstatus, sizeof(navstatus), TIME_S2I(5)) == 0) {
      TRACE_ERROR("GPS  > NAV-STATUS Polling FAILED");
        return false;
    }
    // Extract data from message
    fix->fixOK = navstatus[5] & 0x1;
    fix->pdop = navpvt[76] + (navpvt[77] << 8);

    fix->num_svs = navpvt[23];
    fix->type = navpvt[20];

    fix->time.year = navpvt[4] + (navpvt[5] << 8);
    fix->time.month = navpvt[6];
    fix->time.day = navpvt[7];
    fix->time.hour = navpvt[8];
    fix->time.minute = navpvt[9];
    fix->time.second = navpvt[10];

    fix->lat = (int32_t) (
            (uint32_t)(navpvt[28])
            + ((uint32_t)(navpvt[29]) << 8)
            + ((uint32_t)(navpvt[30]) << 16)
            + ((uint32_t)(navpvt[31]) << 24)
            );
    fix->lon = (int32_t) (
            (uint32_t)(navpvt[24])
            + ((uint32_t)(navpvt[25]) << 8)
            + ((uint32_t)(navpvt[26]) << 16)
            + ((uint32_t)(navpvt[27]) << 24)
            );
    int32_t alt_tmp = (((int32_t)
            ((uint32_t)(navpvt[36])
                + ((uint32_t)(navpvt[37]) << 8)
                + ((uint32_t)(navpvt[38]) << 16)
                + ((uint32_t)(navpvt[39]) << 24))
            ) / 1000);
    if (alt_tmp <= 0) {
        fix->alt = 1;
    } else if (alt_tmp > 50000) {
        fix->alt = 50000;
    } else {
        fix->alt = (uint16_t)alt_tmp;
    }
    fix->model = gps_model;
      TRACE_DEBUG("GPS  > Polling OK time=%04d-%02d-%02d %02d:%02d:%02d lat=%d.%05d lon=%d.%05d alt=%dm sats=%d fixOK=%d pDOP=%02d.%02d model=%s",
      fix->time.year, fix->time.month, fix->time.day, fix->time.hour, fix->time.minute, fix->time.second,
      fix->lat/10000000, (fix->lat > 0 ? 1:-1)*(fix->lat/100)%100000, fix->lon/10000000, (fix->lon > 0 ? 1:-1)*(fix->lon/100)%100000,
      fix->alt, fix->num_svs, fix->fixOK, fix->pdop/100, fix->pdop%100, gps_get_model_name(fix->model)
    );

    return true;
}

/**
  * gps_disable_nmea_output
  *
  * disables all NMEA messages to be output from the GPS.
  * even though the parser can cope with NMEA messages and ignores them, it 
  * may save power to disable them completely.
  *
  * returns ACK/NAK result
  *
  */

uint8_t gps_disable_nmea_output(void) {
    uint8_t nonmea[] = {
        0xB5, 0x62, 0x06, 0x00, 20, 0x00,   // UBX-CFG-PRT
        0x01, 0x00, 0x00, 0x00,             // UART1, reserved, no TX ready
        0xD0, 0x08, 0x00, 0x00,             // UART mode (8N1)
        0x80, 0x25, 0x00, 0x00,             // UART baud rate (9600)
        0x01, 0x00,                         // input protocols (uBx only)
        0x01, 0x00,                         // output protocols (uBx only)
        0x00, 0x00,                         // flags
        0x00, 0x00,                         // reserved
        0x00, 0x00                          // CRC place holders
    };
    return (uint8_t)gps_ubx_transaction(nonmea, sizeof(nonmea), true,
                                        NULL, 0, TIME_S2I(5));
}

/**
  * gps_set_stationary_model
  *
  * tells the GPS to use the stationary positioning model.
  *
  * returns ACK/NAK result
  *
  */
uint8_t gps_set_stationary_model(void) {
    uint8_t model6[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,     // UBX-CFG-NAV5
        0xFF, 0xFF,                             // parameter bitmask
        GPS_MODEL_STATIONARY,                   // dynamic model
        0x03,                                   // fix mode
        0x00, 0x00, 0x00, 0x00,                 // 2D fix altitude
        0x10, 0x27, 0x00, 0x00,                 // 2D fix altitude variance
        0x05,                                   // minimum elevation
        0x00,                                   // reserved
        0xFA, 0x00,                             // position DOP
        0xFA, 0x00,                             // time DOP
        0x64, 0x00,                             // position accuracy
        0x2C, 0x01,                             // time accuracy
        0x00,                                   // static hold threshold
        0x3C,                                   // DGPS timeout
        0x00,                                   // min. SVs above C/No thresh
        0x00,                                   // C/No threshold
        0x00, 0x00,                             // reserved
        0xc8, 0x00,                             // static hold max. distance
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // reserved
        0x00, 0x00                              // CRC place holders
    };
    return (uint8_t)gps_ubx_transaction(model6, sizeof(model6), true,
                                        NULL, 0, TIME_S2I(5));
}

/**
  * gps_set_low_alt_model
  *
  * tells the GPS to use the low altitude positioning model.
  *
  * returns ACK/NAK result
  *
  */
uint8_t gps_set_low_alt_model(gps_hp_model_t model) {
    uint8_t model6[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,     // UBX-CFG-NAV5
        0xFF, 0xFF,                             // parameter bitmask
        model,                                  // dynamic model
        0x03,                                   // fix mode
        0x00, 0x00, 0x00, 0x00,                 // 2D fix altitude
        0x10, 0x27, 0x00, 0x00,                 // 2D fix altitude variance
        0x05,                                   // minimum elevation
        0x00,                                   // reserved
        0xFA, 0x00,                             // position DOP
        0xFA, 0x00,                             // time DOP
        0x64, 0x00,                             // position accuracy
        0x2C, 0x01,                             // time accuracy
        0x00,                                   // static hold threshold
        0x3C,                                   // DGPS timeout
        0x00,                                   // min. SVs above C/No thresh
        0x00,                                   // C/No threshold
        0x00, 0x00,                             // reserved
        0xc8, 0x00,                             // static hold max. distance
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // reserved
        0x00, 0x00                              // CRC place holders
    };
    return (uint8_t)gps_ubx_transaction(model6, sizeof(model6), true,
                                        NULL, 0, TIME_S2I(5));
}

/**
  * gps_set_high_alt_model
  *
  * tells the GPS to use the high altitude positioning model.
  *
  * returns ACK/NAK result
  *
  */
uint8_t gps_set_high_alt_model(gps_lp_model_t model) {
    uint8_t model6[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,     // UBX-CFG-NAV5
        0xFF, 0xFF,                             // parameter bitmask
        model,                                  // dynamic model
        0x03,                                   // fix mode
        0x00, 0x00, 0x00, 0x00,                 // 2D fix altitude
        0x10, 0x27, 0x00, 0x00,                 // 2D fix altitude variance
        0x05,                                   // minimum elevation
        0x00,                                   // reserved
        0xFA, 0x00,                             // position DOP
        0xFA, 0x00,                             // time DOP
        0x64, 0x00,                             // position accuracy
        0x2C, 0x01,                             // time accuracy
        0x00,                                   // static hold threshold
        0x3C,                                   // DGPS timeout
        0x00,                                   // min. SVs above C/No thresh
        0x00,                                   // C/No threshold
        0x00, 0x00,                             // reserved
        0xc8, 0x00,                             // static hold max. distance
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // reserved
        0x00, 0x00                              // CRC place holders
    };
    return (uint8_t)gps_ubx_transaction(model6, sizeof(model6), true,
                                        NULL, 0, TIME_S2I(5));
}

/**
  * gps_set_portable_model
  *
  * tells the GPS to use the portable positioning model.
  *
  * returns ACK/NAK result
  *
  */
uint8_t gps_set_portable_model(void) {
    uint8_t model6[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,     // UBX-CFG-NAV5
        0xFF, 0xFF,                             // parameter bitmask
        GPS_MODEL_PORTABLE,                     // dynamic model
        0x03,                                   // fix mode
        0x00, 0x00, 0x00, 0x00,                 // 2D fix altitude
        0x10, 0x27, 0x00, 0x00,                 // 2D fix altitude variance
        0x05,                                   // minimum elevation
        0x00,                                   // reserved
        0xFA, 0x00,                             // position DOP
        0xFA, 0x00,                             // time DOP
        0x64, 0x00,                             // position accuracy
        0x2C, 0x01,                             // time accuracy
        0x00,                                   // static hold threshold
        0x3C,                                   // DGPS timeout
        0x00,                                   // min. SVs above C/No thresh
        0x00,                                   // C/No threshold
        0x00, 0x00,                             // reserved
        0xc8, 0x00,                             // static hold max. distance
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // reserved
        0x00, 0x00                              // CRC place holders
    };
    return (uint8_t)gps_ubx_transaction(model6, sizeof(model6), true,
                                        NULL, 0, TIME_S2I(5));
}

/**
  * gps_set_airborne_model
  *
  * tells the GPS to use the airborne positioning model. Should be used to
  * get stable lock up to 50km altitude
  *
  * returns ACK/NAK result
  *
  */
uint8_t gps_set_airborne_model(void) {
    uint8_t model6[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,     // UBX-CFG-NAV5
        0xFF, 0xFF,                             // parameter bitmask
        GPS_MODEL_AIRBORNE1G,                   // dynamic model
        0x03,                                   // fix mode
        0x00, 0x00, 0x00, 0x00,                 // 2D fix altitude
        0x10, 0x27, 0x00, 0x00,                 // 2D fix altitude variance
        0x05,                                   // minimum elevation
        0x00,                                   // reserved
        0xFA, 0x00,                             // position DOP
        0xFA, 0x00,                             // time DOP
        0x64, 0x00,                             // position accuracy
        0x2C, 0x01,                             // time accuracy
        0x00,                                   // static hold threshold
        0x3C,                                   // DGPS timeout
        0x00,                                   // min. SVs above C/No thresh
        0x00,                                   // C/No threshold
        0x00, 0x00,                             // reserved
        0xc8, 0x00,                             // static hold max. distance
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // reserved
        0x00, 0x00                              // CRC place holders
    };
    return (uint8_t)gps_ubx_transaction(model6, sizeof(model6), true,
                                        NULL, 0, TIME_S2I(5));
}

/**
  * gps_set_power_save
  *
  * enables cyclic tracking on the uBlox M8Q
  *
  * returns ACK/NAK result
  *
  */
uint8_t gps_set_power_options(void) {
    uint8_t poweroptions[] = {
        0xB5, 0x62, 0x06, 0x3B, 44, 0,      // UBX-CFG-PM2
        0x01, 0x00, 0x00, 0x00,             // v1, reserved 1..3
        0x00, 0b00010000, 0b00000010, 0x00, // cyclic tracking, update ephemeris
        0x10, 0x27, 0x00, 0x00,             // update period, ms
        0x10, 0x27, 0x00, 0x00,             // search period, ms
        0x00, 0x00, 0x00, 0x00,             // grid offset
        0x00, 0x00,                         // on-time after first fix
        0x01, 0x00,                         // minimum acquisition time
        0x00, 0x00, 0x00, 0x00,             // reserved 4,5
        0x00, 0x00, 0x00, 0x00,             // reserved 6
        0x00, 0x00, 0x00, 0x00,             // reserved 7
        0x00, 0x00, 0x00, 0x00,             // reserved 8,9,10
        0x00, 0x00, 0x00, 0x00,             // reserved 11
        0x00, 0x00                          // CRC place holders
    };
    return (uint8_t)gps_ubx_transaction(poweroptions, sizeof(poweroptions), true,
                                        NULL, 0, TIME_S2I(5));
}

/**
  * gps_switch_power_save_mode
  *
  * enables or disables the power save mode (which was configured before)
  *
  * returns ACK/NAK result
  */
uint8_t gps_switch_power_save_mode(bool on) {
#if UBLOX_ALLOW_POWER_SAVE == FALSE
  return UBX_COMMAND_NAK;
#endif
    uint8_t recvmgmt[] = {
        0xB5, 0x62, 0x06, 0x11, 2, 0,   // UBX-CFG-RXM
        0x08, on ? 0x01 : 0x00,         // reserved, enable power save mode
        0x00, 0x00                      // CRC place holders
    };
    return (uint8_t)gps_ubx_transaction(recvmgmt, sizeof(recvmgmt), true,
                                        NULL, 0, TIME_S2I(5));
}

/**
  * gps_set_model
  *
  * Selects nav model based on air pressure
  */
bool gps_set_model(bool dynamic) {
  uint8_t cntr;
  bool status;

  dataPoint_t tp;
  getSensors(&tp);
  setSystemStatus(&tp);
  if (dynamic && tp.sen_i1_press/10 != 0 && (tp.sen_i1_press/10 > conf_sram.gps_pressure)) {
    if (gps_model == conf_sram.gps_low_alt)
      return true;
    /* Set low altitude model. */
    cntr = 3;
    while ((status =
        gps_set_low_alt_model(conf_sram.gps_low_alt)) == false && cntr--);
    if (status) {
      gps_model = conf_sram.gps_low_alt;
      return true;
    }
    TRACE_ERROR("GPS  > Communication Error [set low altitude model]");
    return false;
  } /* Else use high altitude specified or BMEi1 not functional. */

  /* Default to high altitude model. */
  if (gps_model == conf_sram.gps_high_alt)
    return true;
  cntr = 3;
  while ((status =
      gps_set_high_alt_model(conf_sram.gps_high_alt)) == false && cntr--);
  if (status) {
    gps_model = conf_sram.gps_high_alt;
    return true;
  }
  TRACE_ERROR("GPS  > Communication Error [set high altitude model]");
  return false;
}

/*
 *
 */
bool GPS_Init(void) {
    // Initialize pins
    TRACE_DEBUG("GPS  > Init GPS pins");
    pktSetGPIOlineMode(LINE_GPS_RESET, PAL_MODE_OUTPUT_PUSHPULL);   // GPS reset
    pktSetGPIOlineMode(LINE_GPS_EN, PAL_MODE_OUTPUT_PUSHPULL);      // GPS off

#if GPS_USE_SERIAL_MUX == FALSE
    // Init and start UART
    TRACE_DEBUG("GPS  > Init GPS UART");
    /* UBLOX UART definition in portab. */
    pktSetGPIOlineMode(LINE_GPS_RXD, LINE_GPS_RXD_MODE);       // UART RXD
    pktSetGPIOlineMode(LINE_GPS_TXD, LINE_GPS_TXD_MODE);       // UART TXD
    sdStart(&PKT_GPS_UART, &gps_config);
#endif /* GPS_USE_SERIAL_MUX == FALSE */

    // Switch power
    TRACE_DEBUG("GPS  > Power up GPS");
    pktWriteGPIOline(LINE_GPS_RESET, PAL_HIGH); // Pull up GPS reset
    chThdSleep(TIME_MS2I(10));
    pktWriteGPIOline(LINE_GPS_EN, PAL_HIGH);    // Switch on GPS

    // Wait for GPS startup
    chThdSleep(TIME_S2I(1));

    gps_model = GPS_MODEL_AUTOMOTIVE;
    // Configure GPS
    TRACE_DEBUG("GPS  > Transmit config to GPS");

    uint8_t cntr = 3;
    bool status;
    while((status = gps_disable_nmea_output()) == false && cntr--);
    if(status) {
      TRACE_DEBUG("GPS  > ... Disable NMEA output OK");
    } else {
        TRACE_ERROR("GPS  > Communication Error [disable NMEA]");
        return false;
    }
    cntr = 3;
    while((status = gps_set_power_options()) == false && cntr--);
    if(status) {
      TRACE_DEBUG("GPS  > ... Set power options OK");
    } else {
        TRACE_ERROR("GPS  > Communication Error [power options]");
        return false;
    }
    gps_enabled = true;
    return true;
}

/*
 *
 */
void GPS_Deinit(void)
{
  // Switch MOSFET
  TRACE_DEBUG("GPS  > Power down GPS");
  pktWriteGPIOline(LINE_GPS_EN, PAL_LOW);

#if GPS_USE_SERIAL_MUX == FALSE
  /* Stop and deinit UART. */
  TRACE_DEBUG("GPS  > Stop GPS UART");
  sdStop(&PKT_GPS_UART);
  pktSetGPIOlineMode(LINE_GPS_RXD, PAL_MODE_INPUT);    // UART RXD HI Z
  pktSetGPIOlineMode(LINE_GPS_TXD, PAL_MODE_INPUT);    // UART TXD HI Z
#endif /* GPS_USE_SERIAL_MUX == FALSE */
  gps_model = GPS_MODEL_AUTOMOTIVE;
  gps_enabled = false;
}

