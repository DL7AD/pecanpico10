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
#include "config.h"
#include "aprs.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "debug.h"
#include "base91.h"

#define METER_TO_FEET(m) (((m)*26876) / 8192)

static uint16_t msg_id;

/**
 * Transmit APRS position packet. The comments are filled with:
 * - Static comment (can be set in config.h)
 * - Battery voltage in mV
 * - Solar voltage in mW (if tracker is solar-enabled)
 * - Temperature in Celcius
 * - Air pressure in Pascal
 * - Number of satellites being used
 * - Number of cycles where GPS has been lost (if applicable in cycle)
 */
packet_t aprs_encode_position(const aprs_conf_t *config, trackPoint_t *trackPoint)
{
	// Latitude
	uint32_t y = 380926 * (90 - trackPoint->gps_lat/10000000.0);
	uint32_t y3  = y   / 753571;
	uint32_t y3r = y   % 753571;
	uint32_t y2  = y3r / 8281;
	uint32_t y2r = y3r % 8281;
	uint32_t y1  = y2r / 91;
	uint32_t y1r = y2r % 91;

	// Longitude
	uint32_t x = 190463 * (180 + trackPoint->gps_lon/10000000.0);
	uint32_t x3  = x   / 753571;
	uint32_t x3r = x   % 753571;
	uint32_t x2  = x3r / 8281;
	uint32_t x2r = x3r % 8281;
	uint32_t x1  = x2r / 91;
	uint32_t x1r = x2r % 91;

	// Altitude
	uint32_t a = logf(METER_TO_FEET(trackPoint->gps_alt)) / logf(1.002f);
	uint32_t a1  = a / 91;
	uint32_t a1r = a % 91;

	uint8_t gpsFix = trackPoint->gps_lock == GPS_LOCKED1 || trackPoint->gps_lock == GPS_LOCKED2 ? GSP_FIX_CURRENT : GSP_FIX_OLD;
	uint8_t src = NMEA_SRC_GGA;
	uint8_t origin = ORIGIN_PICO;

	char xmit[256];
	uint32_t len = chsnprintf(xmit, sizeof(xmit), "%s-%d>%s,%s:!", config->callsign, config->ssid, APRS_DEST_CALLSIGN, config->path);

	xmit[len+0]  = (config->symbol >> 8) & 0xFF;
	xmit[len+1]  = y3+33;
	xmit[len+2]  = y2+33;
	xmit[len+3]  = y1+33;
	xmit[len+4]  = y1r+33;
	xmit[len+5]  = x3+33;
	xmit[len+6]  = x2+33;
	xmit[len+7]  = x1+33;
	xmit[len+8]  = x1r+33;
	xmit[len+9]  = config->symbol & 0xFF;
	xmit[len+10] = a1+33;
	xmit[len+11] = a1r+33;
	xmit[len+12] = ((gpsFix << 5) | (src << 3) | origin) + 33;

	// Comments
	uint32_t len2 = base91_encode((uint8_t*)trackPoint, (uint8_t*)&xmit[len+13], sizeof(trackPoint_t));

	xmit[len+len2+13] = '|';

	// Sequence ID
	uint32_t t = trackPoint->id & 0x1FFF;
	xmit[len+len2+14] = t/91 + 33;
	xmit[len+len2+15] = t%91 + 33;

	// Telemetry parameter
	for(uint8_t i=0; i<5; i++) {
		switch(i) {
			case 0: t = trackPoint->adc_vbat;				break;
			case 1: t = trackPoint->adc_vsol;				break;
			case 2: t = trackPoint->pac_pbat+4096;			break;
			case 3: t = trackPoint->sen_i1_temp/10 + 1000;	break;
			case 4: t = trackPoint->sen_i1_press/125 - 40;	break;
		}

		xmit[len+len2+16+i*2]   = t/91 + 33;
		xmit[len+len2+16+i*2+1] = t%91 + 33;
	}

	xmit[len+len2+26] = '|';
	xmit[len+len2+27] = 0;

	return ax25_from_text(xmit, 1);
}

packet_t aprs_encode_data_packet(char packetType, const aprs_conf_t *config, uint8_t *data)
{
	char xmit[256];
	chsnprintf(xmit, sizeof(xmit), "%s-%d>%s,%s:{{%c%s", config->callsign, config->ssid, APRS_DEST_CALLSIGN, config->path, packetType, data);

	return ax25_from_text(xmit, 1);
}

/**
 * Transmit message packet
 */
packet_t aprs_encode_message(const aprs_conf_t *config, const char *receiver, const char *text, const bool noCounter)
{
	char xmit[256];
	if(noCounter)
		chsnprintf(xmit, sizeof(xmit), "%s-%d>%s,%s::%-9s:%s", config->callsign, config->ssid, APRS_DEST_CALLSIGN, config->path, receiver, text);
	else
		chsnprintf(xmit, sizeof(xmit), "%s-%d>%s,%s::%-9s:%s{%d", config->callsign, config->ssid, APRS_DEST_CALLSIGN, config->path, receiver, text, ++msg_id);

	return ax25_from_text(xmit, 1);
}

/**
 * Transmit APRS telemetry configuration
 */
packet_t aprs_encode_telemetry_configuration(const aprs_conf_t *config, uint8_t type)
{
	char dest[16];
	chsnprintf(dest, sizeof(dest), "%s-%d", config->callsign, config->ssid);
	switch(type)
	{
		case 0:	return aprs_encode_message(config, dest, "PARM.Vbat,Vsol,Pbat,Temperature,Airpressure", true);
		case 1: return aprs_encode_message(config, dest, "UNIT.V,V,W,degC,Pa", true);
		case 2: return aprs_encode_message(config, dest, "EQNS.0,.001,0,0,.001,0,0,.001,-4.096,0,.1,-100,0,12.5,500", true);
		case 3: return aprs_encode_message(config, dest, "BITS.11111111,", true);
		default: return NULL;
	}
}

