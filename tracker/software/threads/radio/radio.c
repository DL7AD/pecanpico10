#include "ch.h"
#include "hal.h"

#include "debug.h"
#include "si446x.h"
#include "geofence.h"
#include "aprs.h"
#include "pktconf.h"


static const char *getModulation(uint8_t key) {
	const char *val[] = {"AFSK", "2FSK"};
	return val[key];
};

static void handlePacket(uint8_t *buf, uint32_t len) {
	// Decode APRS frame
	packet_t pp = ax25_from_frame(buf, len);

	if(pp != NULL) {
		char serial_buf[512];
		aprs_debug_getPacket(pp, serial_buf, sizeof(serial_buf));
		TRACE_INFO("RX   > %s", serial_buf);

		aprs_decode_packet(pp);
		ax25_delete(pp);
	} else {
		TRACE_DEBUG("RX    > Error in packet");
	}
}

void start_rx_thread(uint32_t freq, uint8_t rssi) {
	uint32_t f;
	if(freq == FREQ_APRS_DYNAMIC)
		f = getAPRSRegionFrequency(); // Get transmission frequency by geofencing
	else
		f = config.rx.radio_conf.freq;

	// Start transceiver
	Si446x_receive(f, rssi, MOD_AFSK);

	// Start decoder
	Si446x_startDecoder(handlePacket);
}

bool transmitOnRadio(packet_t pp, uint32_t freq, uint8_t pwr, mod_t mod)
{
	if(freq == FREQ_APRS_DYNAMIC)
		freq = getAPRSRegionFrequency(); // Get transmission frequency by geofencing

	uint8_t *c;
	uint32_t len = ax25_get_info(pp, &c);

	if(len) // Message length is not zero
	{
		TRACE_INFO(	"RAD  > Transmit %d.%03d MHz, Pwr %d, %s, %d byte",
					freq/1000000, (freq%1000000)/1000, pwr,
					getModulation(mod), len
		);

		char buf[1024];
		aprs_debug_getPacket(pp, buf, sizeof(buf));
		TRACE_INFO("TX   > %s", buf);

		switch(mod)
		{
			case MOD_2FSK:
				Si446x_send2FSK(/*pp->frame_data, pp->frame_len*/pp, freq, pwr, 9600);
				break;
			case MOD_AFSK:
				Si446x_sendAFSK(/*pp->frame_data, pp->frame_len*/pp, freq, pwr);
				break;
		}

	} else {

		TRACE_ERROR("RAD  > It is nonsense to transmit 0 bits, %d.%03d MHz, Pwr dBm, %s, %d byte",
					freq/1000000, (freq%1000000)/1000, pwr, getModulation(mod), len
		);

	}

	return true;
}


