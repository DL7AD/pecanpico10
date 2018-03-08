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
	if(freq == FREQ_APRS_DYNAMIC) {
		freq = getAPRSRegionFrequency(); // Get transmission frequency by geofencing
		step = 0;
		chan = 0;
	}

	uint8_t *c;
	uint32_t len = ax25_get_info(pp, &c);

	if(len) // Message length is not zero
	{
	  /* Check frequency. */
	  if(!Si446x_isFrequencyInBand(freq, step, chan)) {

        TRACE_ERROR("RAD  > Transmit base frequency of %d.%03d MHz is invalid",
                      freq/1000000, (freq%1000000)/1000);
        return false;
      }

      uint32_t op_freq = Si446x_computeOperatingFrequency(freq, step, chan);
		TRACE_INFO(	"RAD  > Transmit packet on %d.%03d MHz (ch %d),"
		            " Pwr %d, %s, %d byte",
					op_freq/1000000, (op_freq%1000000)/1000,
					Si446x_getChannel(),
					pwr, getModulation(mod), len
		);

		char buf[1024];
		aprs_debug_getPacket(pp, buf, sizeof(buf));
		TRACE_INFO("TX   > %s", buf);

        /*
         * TODO: The following is an interim setup.
         * The management of TX is only partially integrated.
         * Packet services also has WIP in handler <> radio mapping, etc.
         */

		extern packet_svc_t RPKTD1;
		packet_svc_t *handler = &RPKTD1;

		if(!(handler->state == PACKET_OPEN || handler->state == PACKET_RUN)) {
          TRACE_ERROR("RAD  > Packet services are not open for transmit");
		  return false;
		}

		/* Update  the saved radio data with this new request. */
        radio_task_object_t rt = handler->radio_tx_config;

        rt.handler = handler;
		rt.command = PKT_RADIO_TX_SEND;
		rt.type = mod;
		rt.base_frequency = freq;
		rt.step_hz = step;
		rt.channel = chan;
		rt.tx_power = pwr;
		rt.tx_speed = (mod == MOD_2FSK ? 9600 : 1200);
		rt.packet_out = pp;
		rt.callback = NULL;

		/* Save the current data. */
		handler->radio_tx_config = rt;

        msg_t msg = pktSendRadioCommand(rt.handler, &rt);
        if(msg != MSG_OK)
          return false;

/*		switch(mod)
		{
			case MOD_2FSK:
				Si446x_send2FSK(pp, freq, step, chan, pwr, 9600);
				break;
			case MOD_AFSK:
				Si446x_sendAFSK(pp, freq, step, chan, pwr);
				break;
		}*/

	} else {

		TRACE_ERROR("RAD  > It is nonsense to transmit 0 bits, %d.%03d MHz, Pwr dBm, %s, %d byte",
					freq/1000000, (freq%1000000)/1000, pwr,
					getModulation(mod), len);
	}

	return true;
}


