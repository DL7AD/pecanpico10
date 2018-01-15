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
#include "ax25.h"
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
void aprs_encode_position(ax25_t* packet, const aprs_conf_t *config, trackPoint_t *trackPoint)
{
	char temp[128];

	// Encode header
	ax25_send_header(packet, config->callsign, config->ssid, config->path, packet->size > 0 ? 0 : config->preamble);
	ax25_send_byte(packet, '!');

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

	temp[0]  = (config->symbol >> 8) & 0xFF;
	temp[1]  = y3+33;
	temp[2]  = y2+33;
	temp[3]  = y1+33;
	temp[4]  = y1r+33;
	temp[5]  = x3+33;
	temp[6]  = x2+33;
	temp[7]  = x1+33;
	temp[8]  = x1r+33;
	temp[9]  = config->symbol & 0xFF;
	temp[10] = a1+33;
	temp[11] = a1r+33;
	temp[12] = ((gpsFix << 5) | (src << 3) | origin) + 33;
	temp[13] = 0;

	ax25_send_string(packet, temp);

	// Comments
	base91_encode((uint8_t*)trackPoint, (uint8_t*)temp, sizeof(trackPoint_t));
	ax25_send_string(packet, temp);

	ax25_send_byte(packet, '|');

	// Sequence ID
	uint32_t t = trackPoint->id & 0x1FFF;
	temp[0] = t/91 + 33;
	temp[1] = t%91 + 33;
	temp[2] = 0;
	ax25_send_string(packet, temp);

	// Telemetry parameter
	for(uint8_t i=0; i<5; i++) {
		switch(i) {
			case 0: t = trackPoint->adc_vbat;				break;
			case 1: t = trackPoint->adc_vsol;				break;
			case 2: t = trackPoint->pac_pbat+4096;			break;
			case 3: t = trackPoint->sen_i1_temp/10 + 1000;	break;
			case 4: t = trackPoint->sen_i1_press/125 - 40;	break;
		}

		temp[0] = t/91 + 33;
		temp[1] = t%91 + 33;
		ax25_send_string(packet, temp);
	}

	ax25_send_byte(packet, '|');

	// Encode footer
	ax25_send_footer(packet);
}

/**
 * Transmit custom data packet (the methods aprs_encode_data allow multiple APRS packets in a row without preable being sent)
 */
void aprs_encode_init(ax25_t* packet, uint8_t* buffer, uint16_t size, mod_t mod)
{
	packet->data = buffer;
	packet->max_size = size;
	packet->mod = mod;

	// Encode APRS header
	ax25_init(packet);
}
void aprs_encode_data_packet(ax25_t* packet, char packetType, const aprs_conf_t *config, uint8_t *data, size_t size)
{
	// Encode header
	ax25_send_header(packet, config->callsign, config->ssid, config->path, packet->size > 0 ? 0 : config->preamble);
	ax25_send_string(packet, "{{");
	ax25_send_byte(packet, packetType);

	// Encode message
	for(uint16_t i=0; i<size; i++)
		ax25_send_byte(packet, data[i]);

	// Encode footer
	ax25_send_footer(packet);
}
uint32_t aprs_encode_finalize(ax25_t* packet)
{
	scramble(packet);
	nrzi_encode(packet);
	return packet->size;
}

/**
 * Transmit message packet
 */
void aprs_encode_message(ax25_t* packet, const aprs_conf_t *config, const char *receiver, const char *text)
{
	char temp[10];

	// Encode header
	ax25_send_header(packet, config->callsign, config->ssid, config->path, packet->size > 0 ? 0 : config->preamble);
	ax25_send_byte(packet, ':');

	chsnprintf(temp, sizeof(temp), "%-9s", receiver);
	ax25_send_string(packet, temp);

	ax25_send_byte(packet, ':');
	ax25_send_string(packet, text);
	ax25_send_byte(packet, '{');

	chsnprintf(temp, sizeof(temp), "%d", ++msg_id);
	ax25_send_string(packet, temp);

	// Encode footer
	ax25_send_footer(packet);
}

/**
 * Transmit APRS telemetry configuration
 */
void aprs_encode_telemetry_configuration(ax25_t* packet, const aprs_conf_t *config)
{
	char temp[4];

	for(uint8_t type=0; type<5; type++)
	{
		// Encode header
		ax25_send_header(packet, config->callsign, config->ssid, config->path, packet->size > 0 ? 0 : config->preamble);
		ax25_send_byte(packet, ':'); // Message flag

		// Callsign
		ax25_send_string(packet, config->callsign);
		ax25_send_byte(packet, '-');
		chsnprintf(temp, sizeof(temp), "%d", config->ssid);
		ax25_send_string(packet, temp);

		// Padding
		uint8_t length = strlen(config->callsign) + (config->ssid/10);
		for(uint8_t i=length; i<7; i++)
			ax25_send_string(packet, " ");

		ax25_send_string(packet, ":"); // Message separator

		switch(type) {
			case 0: // Telemetry parameter names
				ax25_send_string(packet, "PARM.Vbat,Vsol,Pbat,Temperature,Airpressure");
				break;

			case 1: // Telemetry units
				ax25_send_string(packet, "UNIT.V,V,W,degC,Pa");
				break;

			case 2: // Telemetry conversion parameters
				ax25_send_string(packet, "EQNS.0,.001,0,0,.001,0,0,.001,-4.096,0,.1,-100,0,12.5,500");
				break;

			case 3:
				ax25_send_string(packet, "BITS.11111111,");
				ax25_send_string(packet, config->tel_comment);
				break;
		}

		// Encode footer
		ax25_send_footer(packet);
	}
}

