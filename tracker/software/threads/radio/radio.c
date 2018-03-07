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
	      TRACE_DEBUG("RX   > No addresses in packet - dropped");
		}
		ax25_delete(pp);
	} else {
		TRACE_DEBUG("RX    > Error in packet - dropped");
	}
	return;
  }
  TRACE_DEBUG("RX    > Packet dropped due to data length < 2");
}

void start_rx_thread(uint32_t freq, uint16_t step,
                     radio_ch_t chan, uint8_t rssi) {

	if(freq == FREQ_APRS_DYNAMIC) {
		freq = getAPRSRegionFrequency(); // Get transmission frequency by geofencing
		/* If using geofence ignore channel and step for now. */
		chan = 0;
		step = 0;
	}

	// Start decoder
	Si446x_startPacketReception(freq, step, chan, rssi, handlePacket);

}

/*
 *
 */
bool transmitOnRadio(packet_t pp, uint32_t freq, uint16_t step, uint8_t chan,
                     uint8_t pwr, mod_t mod)
{
	if(freq == FREQ_APRS_DYNAMIC)
		freq = getAPRSRegionFrequency(); // Get transmission frequency by geofencing

	uint8_t *c;
	uint32_t len = ax25_get_info(pp, &c);

	if(len) // Message length is not zero
	{

        /* Set band and step size in 446x. */
      /* TODO: Check for success/fail from band set. */
      if(!Si446x_setBandParameters(freq, step, RADIO_TX)) {

        TRACE_ERROR("RAD  > Transmit base frequency of %d.%03d MHz is invalid",
                      freq/1000000, (freq%1000000)/1000);
        return false;
      }
      uint32_t op_freq = Si446x_computeOperatingFrequency(chan, RADIO_TX);
		TRACE_INFO(	"RAD  > Transmit packet on %d.%03d MHz (ch %d), Pwr %d, %s, %d byte",
					op_freq/1000000, (op_freq%1000000)/1000, Si446x_getChannel(),
					pwr, getModulation(mod), len
		);

		char buf[1024];
		aprs_debug_getPacket(pp, buf, sizeof(buf));
		TRACE_INFO("TX   > %s", buf);

		switch(mod)
		{
			case MOD_2FSK:
				Si446x_send2FSK(pp, freq, step, chan, pwr, 9600);
				break;
			case MOD_AFSK:
				Si446x_sendAFSK(pp, freq, step, chan, pwr);
				break;
		}

	} else {

		TRACE_ERROR("RAD  > It is nonsense to transmit 0 bits, %d.%03d MHz, Pwr dBm, %s, %d byte",
					freq/1000000, (freq%1000000)/1000, pwr,
					getModulation(mod), len);
	}

	return true;
}


