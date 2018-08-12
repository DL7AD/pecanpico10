#include "ch.h"
#include "hal.h"

#include "debug.h"
#include "si446x.h"
#include "geofence.h"
#include "aprs.h"
#include "pktconf.h"
#include "radio.h"

static void processPacket(uint8_t *buf, uint32_t len) {

  if(len < 3) {
    /*
     *  Incoming packet was too short.
     *  Don't yet have a general packet so nothing to do.
     */
    TRACE_INFO("RX    > Packet dropped due to data length < 2");
    return;
  }
  /* Remove CRC from frame. */
  len -= 2;

  /* Decode APRS frame. */
  packet_t pp = ax25_from_frame(buf, len);

  if(pp == NULL) {
    TRACE_INFO("RX   > Error in packet - dropped");
    return;
  }
  /* Continue packet analysis. */
  uint8_t *c;
  uint32_t ilen = ax25_get_info(pp, &c);
  if(ilen == 0) {
    TRACE_INFO("RX   > Invalid packet structure - dropped");
    pktReleasePacketBuffer(pp);
    return;
  }
  /* Output packet as text. */
  char serial_buf[512];
  aprs_debug_getPacket(pp, serial_buf, sizeof(serial_buf));
  TRACE_MON("RX   > %s", serial_buf);

  if(pp->num_addr > 0) {
    aprs_decode_packet(pp);
  }
  else {
    TRACE_INFO("RX   > No addresses in packet - dropped");
  }
  pktReleasePacketBuffer(pp);
}

void mapCallback(pkt_data_object_t *pkt_buff) {
  /* Packet buffer. */
  ax25char_t *frame_buffer = pkt_buff->buffer;
  ax25size_t frame_size = pkt_buff->packet_size;

  if(pktGetAX25FrameStatus(pkt_buff)) {

  /* Perform the callback. */
  processPacket(frame_buffer, frame_size);
  } else {
    TRACE_INFO("RX   > Frame has bad CRC - dropped");
  }
}
/**
 * TODO: Select a radio based on frequency and start that.
 */
void start_aprs_threads(radio_unit_t radio, radio_freq_t base_freq,
                     channel_hz_t step,
                     radio_ch_t chan, radio_squelch_t rssi) {

    if(base_freq == FREQ_RX_APRS) {
      TRACE_ERROR("RX   > Cannot specify FREQ_RX_APRS for receiver");
      return;
    }

    /* Open packet radio service.
     * TODO: The parameter should be channel not step.
     */
    msg_t omsg = pktOpenRadioReceive(radio,
                         MOD_AFSK,
                         base_freq,
                         step);

    if(omsg != MSG_OK) {
      TRACE_ERROR("RX   > Open of radio service failed");
      return;
    }

    /* Start the decoder. */
    msg_t smsg = pktEnableDataReception(radio,
                           chan,
                           rssi,
                           mapCallback);
    if(smsg != MSG_OK) {
      pktCloseRadioReceive(radio);
      TRACE_ERROR("RX   > Start of radio packet reception failed");
    }
}

/*
 *
 */
bool transmitOnRadio(packet_t pp, const radio_freq_t base_freq,
                     const channel_hz_t step, radio_ch_t chan,
                     const radio_pwr_t pwr, const mod_t mod,
                     const radio_squelch_t cca) {
  /* Select a radio by frequency. */
  radio_unit_t radio = pktSelectRadioForFrequency(base_freq,
                                                  step,
                                                  chan,
                                                  RADIO_TX);

  if(radio == PKT_RADIO_NONE) {
    char code_s[100];
    pktDisplayFrequencyCode(base_freq, code_s, sizeof(code_s));
    TRACE_WARN( "RAD  > No radio available to transmit on base %s "
                                                    "at channel %d)",
                                                    code_s, chan);
    pktReleaseBufferChain(pp);
    return false;
  }

  if(!pktIsTransmitOpen(radio)) {
    TRACE_WARN( "RAD  > Transmit is not open on radio");
    pktReleaseBufferChain(pp);
    return false;
  }

  radio_freq_t op_freq = pktComputeOperatingFrequency(radio,
                                                      base_freq,
                                                      step,
                                                      chan,
                                                      RADIO_TX);
  if(op_freq == FREQ_INVALID) {
    TRACE_ERROR("RAD  > Transmit operating frequency of %d.%03d MHz is invalid",
                op_freq/1000000, (op_freq%1000000)/1000);
      pktReleaseBufferChain(pp);
      return false;
  }

  /* Channel is only used with absolute base frequencies. */
  if(base_freq < FREQ_CODES_END) {
    chan = 0;
  }

  uint16_t len = ax25_get_info(pp, NULL);

  /* Check information size. */
  if(AX25_MIN_INFO_LEN < len && len <= AX25_MAX_INFO_LEN) {

    TRACE_INFO( "RAD  > %s transmit on %d.%03d MHz (ch %d),"
        " PWR %d, %s, CCA %d, data %d",
        (pp->nextp != NULL) ? "Burst" : "Packet",
            op_freq/1000000, (op_freq%1000000)/1000,
            chan, pwr, getModulation(mod), cca, len
    );

    /* TODO: Check size of buf. */
    char buf[1024];
    aprs_debug_getPacket(pp, buf, sizeof(buf));
    TRACE_INFO("TX   > %s", buf);

    /* The service object. */
    packet_svc_t *handler = pktGetServiceObject(radio);

    /* Get  the saved radio data for this new request. */
    radio_task_object_t rt = handler->radio_tx_config;

    rt.handler = handler;
    rt.command = PKT_RADIO_TX_SEND;
    rt.type = mod;
    rt.base_frequency = op_freq;
    rt.step_hz = step;
    rt.channel = chan;
    rt.tx_power = pwr;
    rt.tx_speed = (mod == MOD_2FSK ? 9600 : 1200);
    rt.squelch = cca;
    rt.packet_out = pp;

    /* Update the task mirror. */
    handler->radio_tx_config = rt;

    msg_t msg = pktSendRadioCommand(radio, &rt, NULL);
    if(msg != MSG_OK) {
      TRACE_ERROR("RAD  > Failed to post radio task");
      pktReleaseBufferChain(pp);
      return false;
    }

  } else {

    TRACE_ERROR("RAD  > Information size is invalid for transmission, %d.%03d MHz, "
        "Pwr dBm, %s, %d byte",
                base_freq/1000000, (base_freq%1000000)/1000, pwr,
                getModulation(mod), len);

    pktReleaseBufferChain(pp);
    return false;
  }
  return true;
}


