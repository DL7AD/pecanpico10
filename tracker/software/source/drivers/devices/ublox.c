/**
  * @see https://github.com/thasti/utrak
  */

#include "ch.h"
#include "hal.h"

#include "ublox.h"
#include "pi2c.h"
#include "debug.h"
#include "config.h"
#include "collector.h"
#include "portab.h"

bool gps_enabled = false;
uint8_t gps_model = GPS_MODEL_PORTABLE;

#if defined(UBLOX_UART_CONNECTED)
// Serial driver configuration for GPS
const SerialConfig gps_config =
{
	9600,	// baud rate
	0,		// CR1 register
	0,		// CR2 register
	0		// CR3 register
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

/**
  * Transmits a string of bytes to the GPS
  */
void gps_transmit_string(uint8_t *cmd, uint8_t length) {
  gps_calc_ubx_csum(cmd, length);
#if UBLOX_USE_I2C == TRUE
  I2C_writeN(PKT_GPS_I2C, UBLOX_MAX_ADDRESS, cmd, length);
#elif defined(UBLOX_UART_CONNECTED)
  sdWrite(&PKT_GPS_UART, cmd, length);
#endif
}

/**
  * Receives a single byte from the GPS and assigns to supplied pointer.
  * Returns false is there is no byte available else true
  */
bool gps_receive_byte(uint8_t *data) {
#if UBLOX_USE_I2C == TRUE
	uint16_t len;
	I2C_read16(PKT_GPS_I2C, UBLOX_MAX_ADDRESS, 0xFD, &len);
	if(len) {
		I2C_read8(PKT_GPS_I2C, UBLOX_MAX_ADDRESS, 0xFF, data);
		return true;
	}
#elif defined(UBLOX_UART_CONNECTED)
	return sdReadTimeout(&PKT_GPS_UART, data, 1, TIME_IMMEDIATE);
#else
    (void) data;
#endif
    return false;
}

/**
  * gps_receive_ack
  *
  * waits for transmission of an ACK/NAK message from the GPS.
  *
  * returns 1 if ACK was received, 0 if NAK was received or timeout
  *
  */
uint8_t gps_receive_ack(uint8_t class_id, uint8_t msg_id, uint16_t timeout) {
	int match_count = 0;
	int msg_ack = 0;
	uint8_t rx_byte;
	uint8_t ack[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x00, 0x00};
	uint8_t nak[] = {0xB5, 0x62, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00};
	ack[6] = class_id;
	nak[6] = class_id;
	ack[7] = msg_id;
	nak[7] = msg_id;

	// runs until ACK/NAK packet is received
	sysinterval_t sTimeout = chVTGetSystemTimeX() + TIME_MS2I(timeout);
	while(sTimeout >= chVTGetSystemTimeX()) {

		// Receive one byte
		if(!gps_receive_byte(&rx_byte)) {
			chThdSleep(TIME_MS2I(10));
			continue;
		}

		// Process one byte
		if (rx_byte == ack[match_count] || rx_byte == nak[match_count]) {
			if (match_count == 3) {	/* test ACK/NAK byte */
				if (rx_byte == ack[match_count]) {
					msg_ack = 1;
				} else {
					msg_ack = 0;
				}
			}
			if (match_count == 7) { 
				return msg_ack;
			}
			match_count++;
		} else {
			match_count = 0;
		}

	}

	return 0;
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
uint16_t gps_receive_payload(uint8_t class_id, uint8_t msg_id,
                             unsigned char *payload, size_t size,
                             sysinterval_t timeout) {
	uint8_t rx_byte;
	enum {UBX_A, UBX_B, CLASSID, MSGID, LEN_A, LEN_B, PAYLOAD} state = UBX_A;
	uint16_t payload_cnt = 0;
	uint16_t payload_len = 0;

	systime_t sNow = chVTGetSystemTime();
	systime_t sEnd = chTimeAddX(sNow, timeout);

	while(chVTIsSystemTimeWithin(sNow, sEnd)) {

		// Receive one byte
      if(!gps_receive_byte(&rx_byte)) {
			chThdSleep(TIME_MS2I(1));
			continue;
		}

		// Process one byte
		switch (state) {
			case UBX_A:
				if(rx_byte == 0xB5)	   state = UBX_B;
				break;
			case UBX_B:
				if(rx_byte == 0x62)	    state = CLASSID;
				else			state = UBX_A;
				break;
			case CLASSID:
				if(rx_byte == class_id) state = MSGID;
				else			state = UBX_A;
				break;
			case MSGID:
				if(rx_byte == msg_id)	state = LEN_A;
				else			state = UBX_A;
				break;
			case LEN_A:
				payload_len = rx_byte;
				state = LEN_B;
				break;
			case LEN_B:
				payload_len |= ((uint16_t)rx_byte << 8);
				state = PAYLOAD;
				break;
			case PAYLOAD:
				payload[payload_cnt++] = rx_byte;
                if(payload_cnt == payload_len)
                  return payload_len;
				if(payload_cnt > size)
				  return 0;
				break;
			default:
				state = UBX_A;
		}
	}
	return 0;
}

/**
  * gps_get_timepulse_info
  *
  *
  */
bool gps_get_timepulse_info(tpidx_t tp, gps_tp5_t *tp5, size_t size) {
  if(!gps_enabled)
    return false;

  // Transmit request
  uint8_t tp5_req[] = {0xB5, 0x62, 0x06, 0x31, 0x01, 0x00, tp & 0x01, 0xFF, 0xFF};
  gps_transmit_string(tp5_req, sizeof(tp5_req));

  if(!gps_receive_payload(0x06, 0x31, (unsigned char*)tp5, size,
                          TIME_MS2I(3000))) { // Receive request
    TRACE_ERROR("GPS  > CFG-TP5 Polling FAILED");
    return false;
  }
  return true;
}

/**
  * gps_get_sv_info
  *
  *
  */
bool gps_get_sv_info(gps_svinfo_t *svinfo, size_t size) {
  if(!gps_enabled)
    return false;

  // Transmit request
  uint8_t navsvinfo_req[] = {0xB5, 0x62, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00};
  gps_transmit_string(navsvinfo_req, sizeof(navsvinfo_req));

  if(!gps_receive_payload(0x01, 0x30, (unsigned char*)svinfo, size,
                          TIME_MS2I(3000))) { // Receive request
    TRACE_ERROR("GPS  > NAV-SVINFO Polling FAILED");
    return false;
  }
  return true;
}

/**
 * gps_get_nav_status
 */
bool gps_get_nav_status(gps_navinfo_t *navinfo, size_t size) {
  if(!gps_enabled)
    return false;
  uint8_t navstatus_req[] = {0xB5, 0x62, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
  gps_transmit_string(navstatus_req, sizeof(navstatus_req));

  if(!gps_receive_payload(0x01, 0x03, (unsigned char*)navinfo, size,
                          TIME_MS2I(3000))) { // Receive request
      TRACE_ERROR("GPS  > NAV-STATUS Polling FAILED");
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
	gps_transmit_string(navpvt_req, sizeof(navpvt_req));

	if(!gps_receive_payload(0x01, 0x07, navpvt, sizeof(navpvt),
	                        TIME_MS2I(3000))) { // Receive request
		TRACE_ERROR("GPS  > NAV-PVT Polling FAILED");
		return false;
	}

	uint8_t navstatus_req[] = {0xB5, 0x62, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
	gps_transmit_string(navstatus_req, sizeof(navstatus_req));

	if(!gps_receive_payload(0x01, 0x03, navstatus, sizeof(navstatus),
	                        TIME_MS2I(3000))) { // Receive request
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
		0xB5, 0x62, 0x06, 0x00, 20, 0x00,	// UBX-CFG-PRT
		0x01, 0x00, 0x00, 0x00, 			// UART1, reserved, no TX ready
		0xe0, 0x08, 0x00, 0x00,				// UART mode (8N1)
		0x80, 0x25, 0x00, 0x00,				// UART baud rate (9600)
		0x01, 0x00,							// input protocols (uBx only)
		0x01, 0x00,							// output protocols (uBx only)
		0x00, 0x00,							// flags
		0x00, 0x00,							// reserved
		0x00, 0x00		                    // CRC place holders
	};

	gps_transmit_string(nonmea, sizeof(nonmea));
	return gps_receive_ack(0x06, 0x00, 1000);
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

    gps_transmit_string(model6, sizeof(model6));
    return gps_receive_ack(0x06, 0x24, 1000);
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

    gps_transmit_string(model6, sizeof(model6));
    return gps_receive_ack(0x06, 0x24, 1000);
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

    gps_transmit_string(model6, sizeof(model6));
    return gps_receive_ack(0x06, 0x24, 1000);
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

    gps_transmit_string(model6, sizeof(model6));
    return gps_receive_ack(0x06, 0x24, 1000);
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
		0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 	// UBX-CFG-NAV5
		0xFF, 0xFF, 							// parameter bitmask
		GPS_MODEL_AIRBORNE1G,					// dynamic model
		0x03, 									// fix mode
		0x00, 0x00, 0x00, 0x00, 				// 2D fix altitude
		0x10, 0x27, 0x00, 0x00,					// 2D fix altitude variance
		0x05, 									// minimum elevation
		0x00, 									// reserved
		0xFA, 0x00, 							// position DOP
		0xFA, 0x00, 							// time DOP
		0x64, 0x00, 							// position accuracy
		0x2C, 0x01, 							// time accuracy
		0x00,									// static hold threshold 
		0x3C, 									// DGPS timeout
		0x00, 									// min. SVs above C/No thresh
		0x00, 									// C/No threshold
		0x00, 0x00, 							// reserved
		0xc8, 0x00,								// static hold max. distance
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 	// reserved
		0x00, 0x00								// CRC place holders
	};

	gps_transmit_string(model6, sizeof(model6));
	return gps_receive_ack(0x06, 0x24, 1000);
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
		0xB5, 0x62, 0x06, 0x3B, 44, 0,		// UBX-CFG-PM2
		0x01, 0x00, 0x00, 0x00, 			// v1, reserved 1..3
		0x00, 0b00010000, 0b00000010, 0x00,	// cyclic tracking, update ephemeris
		0x10, 0x27, 0x00, 0x00,				// update period, ms
		0x10, 0x27, 0x00, 0x00,				// search period, ms
		0x00, 0x00, 0x00, 0x00,				// grid offset
		0x00, 0x00,							// on-time after first fix
		0x01, 0x00,							// minimum acquisition time
		0x00, 0x00, 0x00, 0x00,				// reserved 4,5
		0x00, 0x00, 0x00, 0x00,				// reserved 6
		0x00, 0x00, 0x00, 0x00,				// reserved 7
		0x00, 0x00, 0x00, 0x00,				// reserved 8,9,10
		0x00, 0x00, 0x00, 0x00,				// reserved 11
		0x00, 0x00                          // CRC place holders
	};

	gps_transmit_string(poweroptions, sizeof(poweroptions));
	return gps_receive_ack(0x06, 0x3B, 1000);
}

/**
  * gps_switch_power_save_mode
  *
  * enables or disables the power save mode (which was configured before)
  *
  * returns ACK/NAK result
  */
uint8_t gps_switch_power_save_mode(bool on) {
	uint8_t recvmgmt[] = {
		0xB5, 0x62, 0x06, 0x11, 2, 0,	// UBX-CFG-RXM
		0x08, on ? 0x01 : 0x00,	        // reserved, enable power save mode
		0x00, 0x00                      // CRC place holders
	};

	gps_transmit_string(recvmgmt, sizeof(recvmgmt));
	return gps_receive_ack(0x06, 0x11, 1000);
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
  if(dynamic && ((tp.sys_error & BMEI1_STATUS_MASK) == BME_OK_VALUE)
      && (tp.sen_i1_press/10 > conf_sram.gps_pressure)) {
    if(gps_model == conf_sram.gps_low_alt)
      return true;
    /* Set low altitude model. */
    cntr = 3;
    while((status =
        gps_set_low_alt_model(conf_sram.gps_low_alt)) == false && cntr--);
    if(status) {
      gps_model = conf_sram.gps_low_alt;
      return true;
    }
    TRACE_ERROR("GPS  > Communication Error [set low altitude model]");
    return false;
  } /* Else use high altitude specified or BMEi1 not functional. */

  /* Default to high altitude model. */
  if(gps_model == conf_sram.gps_high_alt)
    return true;
  cntr = 3;
  while((status =
      gps_set_high_alt_model(conf_sram.gps_high_alt)) == false && cntr--);
  if(status) {
    gps_model = conf_sram.gps_high_alt;
    return true;
  }
  TRACE_ERROR("GPS  > Communication Error [set high altitude model]");
  return false;
}

/*
 *
 */
bool GPS_Init() {
	// Initialize pins
    TRACE_DEBUG("GPS  > Init GPS pins");
	palSetLineMode(LINE_GPS_RESET, PAL_MODE_OUTPUT_PUSHPULL);	// GPS reset
	palSetLineMode(LINE_GPS_EN, PAL_MODE_OUTPUT_PUSHPULL);		// GPS off


#if defined(UBLOX_UART_CONNECTED) && UBLOX_USE_I2C == FALSE
    // Init and start UART
	TRACE_DEBUG("GPS  > Init GPS UART");
    /* TODO: Put UBLOX UART definition in portab. */
    palSetLineMode(LINE_GPS_RXD, PAL_MODE_ALTERNATE(11));       // UART RXD
    palSetLineMode(LINE_GPS_TXD, PAL_MODE_ALTERNATE(11));       // UART TXD
	sdStart(&PKT_GPS_UART, &gps_config);
#endif

	// Switch MOSFET
	TRACE_DEBUG("GPS  > Power up GPS");
	palSetLine(LINE_GPS_RESET);	// Pull up GPS reset
	palSetLine(LINE_GPS_EN);	// Switch on GPS
	
	// Wait for GPS startup
	chThdSleep(TIME_S2I(1));

	gps_model = GPS_MODEL_PORTABLE;
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
	palClearLine(LINE_GPS_EN);

#if defined(UBLOX_UART_CONNECTED) && UBLOX_USE_I2C == FALSE
    // Stop and deinit UART
	TRACE_DEBUG("GPS  > Stop GPS UART");
    sdStop(&PKT_GPS_UART);
    palSetLineMode(LINE_GPS_RXD, PAL_MODE_INPUT);       // UART RXD
    palSetLineMode(LINE_GPS_TXD, PAL_MODE_INPUT);       // UART TXD
#endif
    gps_model = GPS_MODEL_PORTABLE;
    gps_enabled = false;
}

/*
 * Calculate checksum and inserts into buffer.
 * Calling function must allocate space in message buff for csum.
 *
 */
bool gps_calc_ubx_csum(uint8_t *mbuf, uint16_t mlen) {

  uint16_t i;
  uint8_t ck_a = 0, ck_b = 0;
  /* Counting sync bytes there must be at least one byte to checksum. */
  if(mlen < 5)
    return false;

  for (i = 2; i < mlen - 2; i++) {
      ck_b += (ck_a += mbuf[i]);
  }
  mbuf[mlen - 2] = ck_a;
  mbuf[mlen - 1] = ck_b;

return true;
}
