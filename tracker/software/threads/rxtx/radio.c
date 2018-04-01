#include "ch.h"
#include "hal.h"

#include "debug.h"
#include "si446x.h"
#include "geofence.h"
#include "aprs.h"
#include "pktconf.h"
#include "radio.h"

static void processPacket(uint8_t *buf, uint32_t len) {
  /* Remove CRC from frame. */
  if(len > 2) {
    len -= 2;
    // Decode APRS frame
	packet_t pp = ax25_from_frame(buf, len);

	if(pp != NULL) {
	    uint8_t *c;
	    uint32_t len = ax25_get_info(pp, &c);
	    if(len == 0) {
	        TRACE_INFO("RX   > Invalid packet structure - dropped");
#if USE_NEW_PKT_TX_ALLOC == TRUE
      pktReleaseOutgoingBuffer(pp);
#else
      ax25_delete (this_p);
#endif
	        return;
	    }
		char serial_buf[512];
		aprs_debug_getPacket(pp, serial_buf, sizeof(serial_buf));
		TRACE_INFO("RX   > %s", serial_buf);

		if(pp->num_addr > 0) {
	      aprs_decode_packet(pp);
		} else {
	      TRACE_INFO("RX   > No addresses in packet - dropped");
		}
#if USE_NEW_PKT_TX_ALLOC == TRUE
      pktReleaseOutgoingBuffer(pp);
#else
      ax25_delete (this_p);
#endif
	} else {
		TRACE_INFO("RX    > Error in packet - dropped");
	}
	return;
  }
  TRACE_INFO("RX    > Packet dropped due to data length < 2");
}

void mapCallback(pkt_data_object_t *pkt_buff) {
  /* Packet buffer. */
  ax25char_t *frame_buffer = pkt_buff->buffer;
  ax25size_t frame_size = pkt_buff->packet_size;

  /* FIXME: This is a quick diagnostic implementation only. */
#if DUMP_PACKET_TO_SERIAL == TRUE && ENABLE_EXTERNAL_I2C != TRUE
  pktDiagnosticOutput(pkt_buff->handler, pkt_buff);
#endif
if(pktIsBufferValidAX25Frame(pkt_buff)) {
  /* Perform the callback. */
  processPacket(frame_buffer, frame_size);
  } else {
    TRACE_INFO("RX   > Invalid frame - dropped");
  }
}

void start_rx_thread(radio_unit_t radio, radio_freq_t base_freq,
                     channel_hz_t step,
                     radio_ch_t chan, radio_squelch_t rssi) {

	if(base_freq == FREQ_APRS_DYNAMIC) {
		base_freq = getAPRSRegionFrequency(); // Get transmission frequency by geofencing
		/* If using geofence ignore channel and step for now. */
		chan = 0;
		step = 0;
	}

    /* Open packet radio service. */
    msg_t omsg = pktOpenRadioReceive(radio,
                         MOD_AFSK,
                         base_freq,
                         step);

    if(omsg != MSG_OK) {
      TRACE_DEBUG("RX   > Open of radio service failed");
      return;
    }

    /* Start the decoder. */
    msg_t smsg = pktStartDataReception(radio,
                           chan,
                           rssi,
                           mapCallback);
    if(smsg != MSG_OK) {
      pktCloseRadioReceive(radio);
      TRACE_DEBUG("RX   > Start of radio packet reception failed");
    }
}

/*
 *
 */
bool transmitOnRadio(packet_t pp, radio_freq_t base_freq,
                     channel_hz_t step, radio_ch_t chan,
                     radio_pwr_t pwr, mod_t mod, radio_squelch_t rssi) {
  /* TODO: This should select a radio by frequency. For now just use 1. */
  radio_unit_t radio = PKT_RADIO_1;

  if(!pktIsTransmitOpen(radio)) {
    TRACE_WARN( "RAD  > Transmit is not open on radio");
#if USE_NEW_PKT_TX_ALLOC == TRUE
      pktReleaseOutgoingBuffer(pp);
#else
      ax25_delete (this_p);
#endif
    return false;
  }
	if(base_freq == FREQ_APRS_DYNAMIC) {
		base_freq = getAPRSRegionFrequency(); // Get transmission frequency by geofencing
		step = 0;
		chan = 0;
	}

	if(base_freq == FREQ_APRS_RECEIVE) {
	  /* TODO: Get current RX frequency (if valid) and use that. */
	}

	//uint8_t *c;
	uint32_t len = ax25_get_info(pp, /*&c*/NULL);

	if(len) // Message length is not zero
	{
	  /* Check frequency. */
	  if(!Si446x_isFrequencyInBand(radio, base_freq, step, chan)) {

        TRACE_ERROR("RAD  > Transmit base frequency of %d.%03d MHz is invalid",
                      base_freq/1000000, (base_freq%1000000)/1000);
        return false;
      }

	  radio_freq_t op_freq = pktComputeOperatingFrequency(base_freq, step, chan);
		TRACE_INFO(	"RAD  > Transmit packet on %d.%03d MHz (ch %d),"
		            " Pwr %d, %s, %d byte",
					op_freq/1000000, (op_freq%1000000)/1000,
					chan, pwr, getModulation(mod), len
		);

		/* TODO: Check size of buf. */
		char buf[1024];
		aprs_debug_getPacket(pp, buf, sizeof(buf));
		TRACE_INFO("TX   > %s", buf);

		/* Check if packet services available for transmit. */
		if(!pktIsTransmitOpen(radio)) {
          TRACE_ERROR("RAD  > Packet services are not open for transmit");
#if USE_NEW_PKT_TX_ALLOC == TRUE
      pktReleaseOutgoingBuffer(pp);
#else
      ax25_delete (this_p);
#endif
		  return false;
		}

		/* The service object. */
        packet_svc_t *handler = pktGetServiceObject(radio);

		/* Update  the saved radio data with this new request. */
        radio_task_object_t rt = handler->radio_tx_config;

        rt.handler = handler;
		rt.command = PKT_RADIO_TX_SEND;
		rt.type = mod;
		rt.base_frequency = base_freq;
		rt.step_hz = step;
		rt.channel = chan;
		rt.tx_power = pwr;
		rt.tx_speed = (mod == MOD_2FSK ? 9600 : 1200);
		rt.squelch = rssi;
		rt.packet_out = pp;
		rt.callback = NULL;

		/* Save the current data. */
		//handler->radio_tx_config = rt;

        msg_t msg = pktSendRadioCommand(radio, &rt);
        if(msg != MSG_OK) {
          TRACE_ERROR("RAD  > Failed to post radio task");
#if USE_NEW_PKT_TX_ALLOC == TRUE
      pktReleaseOutgoingBuffer(pp);
#else
      ax25_delete (this_p);
#endif
          return false;
        }

	} else {

		TRACE_ERROR("RAD  > It is nonsense to transmit 0 bits, %d.%03d MHz, Pwr dBm, %s, %d byte",
					base_freq/1000000, (base_freq%1000000)/1000, pwr,
					getModulation(mod), len);
#if USE_NEW_PKT_TX_ALLOC == TRUE
      pktReleaseOutgoingBuffer(pp);
#else
      ax25_delete (this_p);
#endif
	}

	return true;
}


