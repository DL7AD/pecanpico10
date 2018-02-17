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
#include "digipeater.h"
#include "dedupe.h"
#include "radio.h"

#define METER_TO_FEET(m) (((m)*26876) / 8192)

typedef struct {
	sysinterval_t time;
	char call[AX25_MAX_ADDR_LEN];
} heard_t;


static uint16_t msg_id;
char alias_re[] = "WIDE[4-7]-[1-7]|CITYD";
char wide_re[] = "WIDE[1-7]-[1-7]";
enum preempt_e preempt = PREEMPT_OFF;
static heard_t heard_list[20];
static bool dedupe_initialized;

void aprs_debug_getPacket(packet_t pp, char* buf, uint32_t len)
{
	// Decode packet
	char rec[256];
	unsigned char *pinfo;
	ax25_format_addrs(pp, rec);
	ax25_get_info(pp, &pinfo);

	// Print decoded packet
	uint32_t out = chsnprintf(buf, len, "%s", rec);
	for(uint32_t i=0; pinfo[i]; i++) {
		if(pinfo[i] < 32 || pinfo[i] > 126) {
			out += chsnprintf(&buf[out], len-out, "<0x%02x>", pinfo[i]);
		} else {
			out += chsnprintf(&buf[out], len-out, "%c", pinfo[i]);
		}
	}
}

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
packet_t aprs_encode_position(const char *callsign, const char *path, uint16_t symbol, trackPoint_t *trackPoint)
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
	uint32_t len = chsnprintf(xmit, sizeof(xmit), "%s>%s,%s:!", callsign, APRS_DEST_CALLSIGN, path);

	xmit[len+0]  = (symbol >> 8) & 0xFF;
	xmit[len+1]  = y3+33;
	xmit[len+2]  = y2+33;
	xmit[len+3]  = y1+33;
	xmit[len+4]  = y1r+33;
	xmit[len+5]  = x3+33;
	xmit[len+6]  = x2+33;
	xmit[len+7]  = x1+33;
	xmit[len+8]  = x1r+33;
	xmit[len+9]  = symbol & 0xFF;
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

packet_t aprs_encode_data_packet(const char *callsign, const char *path, char packetType, uint8_t *data)
{
	char xmit[256];
	chsnprintf(xmit, sizeof(xmit), "%s>%s,%s:{{%c%s", callsign, APRS_DEST_CALLSIGN, path, packetType, data);

	return ax25_from_text(xmit, 1);
}

/**
 * Transmit message packet
 */
packet_t aprs_encode_message(const char *callsign, const char *path, const char *receiver, const char *text, const bool noCounter)
{
	char xmit[256];
	if(noCounter)
		chsnprintf(xmit, sizeof(xmit), "%s>%s,%s::%-9s:%s", callsign, APRS_DEST_CALLSIGN, path, receiver, text);
	else
		chsnprintf(xmit, sizeof(xmit), "%s>%s,%s::%-9s:%s{%d", callsign, APRS_DEST_CALLSIGN, path, receiver, text, ++msg_id);

	return ax25_from_text(xmit, 1);
}

packet_t aprs_encode_query_answer_aprsd(const char *callsign, const char *path, const char *receiver)
{
	char buf[256] = "Directs=";
	uint32_t out = 8;
	for(uint8_t i=0; i<20; i++) {
		if(heard_list[i].time && heard_list[i].time + TIME_S2I(600) >= chVTGetSystemTime() && heard_list[i].time <= chVTGetSystemTime())
			out += chsnprintf(&buf[out], sizeof(buf)-out, "%s ", heard_list[i].call);
	}
	buf[out-1] = 0; // Remove last spacer

	return aprs_encode_message(callsign, path, receiver, buf, true);
}

static bool aprs_decode_message(packet_t pp)
{
	// Get Info field
	char src[256];
	unsigned char *pinfo;
	ax25_get_info(pp, &pinfo);
	ax25_format_addrs(pp, src);

	// Decode destination callsign
	char dest[AX25_MAX_ADDR_LEN];
	uint8_t i=0;

	while(i < sizeof(dest)-1) {
		if(pinfo[i+1] == ':' || pinfo[i+1] == ' ') {
			dest[i++] = 0;
			break;
		}
		dest[i] = pinfo[i+1];
		i++;
	}

	// Decode source callsign
	for(uint32_t i=0; i < sizeof(src); i++) {
		if(src[i] == '>') {
			src[i] = 0;
			break;
		}
	}

	// Try to find out if this message is meant for us
	if(pinfo[10] == ':' && !strcmp(conf_sram.rx.call, dest))
	{
		char msg_id_rx[8];
		memset(msg_id_rx, 0, sizeof(msg_id_rx));

		// Cut off control chars
		for(uint16_t i=11; pinfo[i] != 0 && i<0xFFFF; i++) {
			if(pinfo[i] == '{') {
				// Copy ACK ID
				memcpy(msg_id_rx, &pinfo[i+1], sizeof(msg_id_rx)-1);
				// Cut off non-printable chars
				for(uint8_t j=0; j<sizeof(msg_id_rx); j++) {
					if(msg_id_rx[j] < 32 || msg_id_rx[j] > 126) {
						msg_id_rx[j] = 0;
						break;
					}
				}
						

				pinfo[i] = 0; // Mark end of message
			}
			if(pinfo[i] == '\r' || pinfo[i] == '\n') {
				pinfo[i] = 0;
			}
		}

		// Trace
		TRACE_INFO("APRS > Received message from %s (ID=%s): %s", src, msg_id_rx, &pinfo[11]);

		char *command = strupr((char*)&pinfo[11]);

		// Do control actions
		if(!strcmp(command, "?GPIO PA8:1")) { // Switch on pin

			TRACE_INFO("Message: GPIO query PA8 HIGH");
			palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL);
			palSetPad(GPIOA, 8);

		} else if(!strcmp(command, "?GPIO PA8:0")) { // Switch off pin

			TRACE_INFO("Message: GPIO query PA8 LOW");
			palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL);
			palClearPad(GPIOA, 8);

		} else if(!strcmp(command, "?APRSP")) { // Transmit position

			TRACE_INFO("Message: Position query");
			trackPoint_t* trackPoint = getLastTrackPoint();
			packet_t pp = aprs_encode_position(conf_sram.rx.call, conf_sram.rx.path, conf_sram.rx.symbol, trackPoint);
			transmitOnRadio(pp, conf_sram.rx.radio_conf.freq, conf_sram.rx.radio_conf.pwr, conf_sram.rx.radio_conf.mod);

		} else if(!strcmp(command, "?APRSD")) { // Transmit position

			TRACE_INFO("Message: Directs query");
			packet_t pp = aprs_encode_query_answer_aprsd(conf_sram.rx.call, conf_sram.rx.path, src);
			transmitOnRadio(pp, conf_sram.rx.radio_conf.freq, conf_sram.rx.radio_conf.pwr, conf_sram.rx.radio_conf.mod);

		} else if(!strcmp(command, "?RESET")) { // Transmit position

			TRACE_INFO("Message: System Reset");
			char buf[16];
			chsnprintf(buf, sizeof(buf), "ack%s", msg_id_rx);
			packet_t pp = aprs_encode_message(conf_sram.rx.call, conf_sram.rx.path, src, buf, true);
			transmitOnRadio(pp, conf_sram.rx.radio_conf.freq, conf_sram.rx.radio_conf.pwr, conf_sram.rx.radio_conf.mod);
			chThdSleep(TIME_S2I(5)); // Give some time to send the message

			NVIC_SystemReset();

		} else {
			TRACE_ERROR("Command Message not understood");
		}

		if(msg_id_rx[0]) { // Message ID has been sent which has to be acknowledged
			char buf[16];
			chsnprintf(buf, sizeof(buf), "ack%s", msg_id_rx);
			packet_t pp = aprs_encode_message(conf_sram.rx.call, conf_sram.rx.path, src, buf, true);
			transmitOnRadio(pp, conf_sram.rx.radio_conf.freq, conf_sram.rx.radio_conf.pwr, conf_sram.rx.radio_conf.mod);
		}

		return false; // Mark that message dont has to be digipeated
	}

	return true; // Mark that message has to be digipeated
}

static void aprs_digipeat(packet_t pp)
{
	if(!dedupe_initialized) {
		dedupe_init(TIME_S2I(10));
		dedupe_initialized = true;
	}

	if(!dedupe_check(pp, 0)) { // Last identical packet older than 10 seconds
		packet_t result = digipeat_match(0, pp, conf_sram.rx.call, conf_sram.rx.call, alias_re, wide_re, 0, preempt, NULL);
		if(result != NULL) { // Should be digipeated
			dedupe_remember(result, 0);
			transmitOnRadio(result, conf_sram.rx.radio_conf.freq, conf_sram.rx.radio_conf.pwr, conf_sram.rx.radio_conf.mod);
		}
	}
}

/**
 * Transmit APRS telemetry configuration
 */
packet_t aprs_encode_telemetry_configuration(const char *callsign, const char *path, uint8_t type)
{
	switch(type)
	{
		case 0:	return aprs_encode_message(callsign, path, callsign, "PARM.Vbat,Vsol,Pbat,Temperature,Airpressure", true);
		case 1: return aprs_encode_message(callsign, path, callsign, "UNIT.V,V,W,degC,Pa", true);
		case 2: return aprs_encode_message(callsign, path, callsign, "EQNS.0,.001,0,0,.001,0,0,.001,-4.096,0,.1,-100,0,12.5,500", true);
		case 3: return aprs_encode_message(callsign, path, callsign, "BITS.11111111,", true);
		default: return NULL;
	}
}

void aprs_decode_packet(packet_t pp)
{
	// Get heard callsign
	char call[AX25_MAX_ADDR_LEN];
	int8_t v = -1;
	do {
		v++;
		ax25_get_addr_with_ssid(pp, ax25_get_heard(pp)-v, call);
	} while(ax25_get_heard(pp)-v >= AX25_SOURCE && (!strncmp("WIDE", call, 4) || !strncmp("TRACE", call, 5)));

	// Fill/Update direct list
	sysinterval_t first_time = 0xFFFFFFFF;	// Timestamp of oldest heard list entry
	uint8_t first_id = 0;					// ID of oldest heard list entry

	for(uint8_t i=0; i<=20; i++) {
		if(i < 20) {
			// Search for callsign in list
			if(!strcmp(heard_list[i].call, call)) { // Callsign found in list
				heard_list[i].time = chVTGetSystemTime(); // Update time the callsign was last heard
				break;
			}

			// Find oldest entry
			if(first_time > heard_list[i].time) {
				first_time = heard_list[i].time;
				first_id = i;
			}
		} else { // Callsign not in list
			// Overwrite old entry/ use empty entry
			memcpy(heard_list[first_id].call, call, sizeof(heard_list[first_id].call));
			heard_list[first_id].time = chVTGetSystemTime();
		}
	}

	// Decode message packets
	bool digipeat = true;
	unsigned char *pinfo;
	ax25_get_info(pp, &pinfo);
	if(pinfo[0] == ':') digipeat = aprs_decode_message(pp); // ax25_get_dti(pp)

	// Digipeat packet
	if(conf_sram.dig_active && digipeat) {
		aprs_digipeat(pp);
	}
}

