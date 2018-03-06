#include "ch.h"
#include "hal.h"

#include "debug.h"
#include "si446x.h"
#include "geofence.h"
#include "aprs.h"
#include "pktconf.h"
#include "radio.h"

static void handlePacket(uint8_t *buf, uint32_t len) {
  /* Remove CRC from frame. */
  if(len > 2) {
    len -= 2;
    // Decode APRS frame
	packet_t pp = ax25_from_frame(buf, len);

	if(pp != NULL) {
		char serial_buf[512];
		aprs_debug_getPacket(pp, serial_buf, sizeof(serial_buf));
		TRACE_INFO("RX   > %s", serial_buf);

		if(pp->num_addr > 0) {
	      aprs_decode_packet(pp);
		} else {
	      TRACE_DEBUG("RX   > No addresses in packet");
		}
		ax25_delete(pp);
	} else {
		TRACE_DEBUG("RX    > Error in packet");
	}
	return;
  }
  TRACE_DEBUG("RX    > Packet data length < 2");
}

void start_rx_thread(uint32_t freq, uint8_t rssi) {

	if(freq == FREQ_APRS_DYNAMIC)
		freq = getAPRSRegionFrequency(); // Get transmission frequency by geofencing

	// Start decoder
	Si446x_startDecoder(freq, rssi, handlePacket);

}

bool transmitOnRadio(packet_t pp, uint32_t freq, uint8_t pwr, mod_t mod)
{
	if(freq == FREQ_APRS_DYNAMIC)
		freq = getAPRSRegionFrequency(); // Get transmission frequency by geofencing

	uint8_t *c;
	uint32_t len = ax25_get_info(pp, &c);

	if(len) // Message length is not zero
	{
		TRACE_INFO(	"RAD  > Transmit %d.%03d MHz, Chn %d, Pwr %d, %s, %d byte",
					freq/1000000, (freq%1000000)/1000, Si446x_getChannel(),
					pwr, getModulation(mod), len
		);

		char buf[1024];
		aprs_debug_getPacket(pp, buf, sizeof(buf));
		TRACE_INFO("TX   > %s", buf);

		switch(mod)
		{
			case MOD_2FSK:
				Si446x_send2FSK(pp, freq, pwr, 9600);
				break;
			case MOD_AFSK:
				Si446x_sendAFSK(pp, freq, pwr);
				break;
		}

	} else {

		TRACE_ERROR("RAD  > It is nonsense to transmit 0 bits, %d.%03d MHz, Pwr dBm, %s, %d byte",
					freq/1000000, (freq%1000000)/1000, pwr, getModulation(mod), len
		);

	}

	return true;
}


