#include "ch.h"
#include "hal.h"

#include "debug.h"
#include "si446x.h"
#include "geofence.h"
#include "aprs.h"
#include "pktconf.h"
#include "radio.h"
#include "sleep.h"
#include "threads.h"

static void processPacket(uint8_t *buf, uint32_t len) {

  if(len < 3) {
    /*
     *  Incoming packet was too short.
     *  Nothing to do.
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
    pktReleaseCommonPacketBuffer(pp);
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
  pktReleaseCommonPacketBuffer(pp);
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

/*
 *
 */
bool transmitOnRadio(packet_t pp, const radio_freq_t base_freq,
                     const channel_hz_t step, radio_ch_t chan,
                     const radio_pwr_t pwr, const radio_mod_t mod,
                     const radio_squelch_t cca) {

  return transmitOnRadioWithCallback(pp,base_freq, step,
                                     chan, pwr, mod, cca, NULL);
}

/*
 *
 */
bool transmitOnRadioWithCallback(packet_t pp, const radio_freq_t base_freq,
                     const channel_hz_t step, radio_ch_t chan,
                     const radio_pwr_t pwr, const radio_mod_t mod,
                     const radio_squelch_t cca,
                     const radio_task_cb_t cb) {
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

  if(!pktIsTransmitAvailable(radio)) {
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
    TRACE_MON("TX   > %s", buf);

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
    rt.squelch = cca;
    rt.packet_out = pp;

    /* Serial number for this TX. */
    rt.tx_seq_num++;

    /* Update the task mirror. */
    handler->radio_tx_config = rt;

    msg_t msg = pktSendRadioCommand(radio, &rt, TIME_S2I(10),
                                    (radio_task_cb_t)cb);
    if(msg == MSG_TIMEOUT) {
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

/**
 *
 */
THD_FUNCTION(aprsThread, arg) {
  thd_aprs_conf_t* conf = (thd_aprs_conf_t*)arg;

  radio_unit_t radio = PKT_RADIO_1;

  if(conf->rx.svc_conf.init_delay) chThdSleep(conf->rx.svc_conf.init_delay);
  systime_t time = chVTGetSystemTime();
  do {
    if(conf->rx.radio_conf.freq == FREQ_RX_APRS) {
      TRACE_ERROR("RX   > Cannot specify FREQ_RX_APRS for receive");
      break;
    }
#if 0
    /* Open packet radio receive service. */
    TRACE_INFO("RX   > Opening receive on radio %d", PKT_RADIO_1);
    msg_t omsg = pktOpenRadioReceive(radio,
                         MOD_AFSK,
                         conf->rx.radio_conf.freq,
                         0, // Step is 0 for now. Get from radio record.
                         TIME_S2I(5));

    if(omsg != MSG_OK) {
      TRACE_ERROR("RX   > Open of radio receive service failed with error %d",
                  omsg);
      time = waitForTrigger(time, conf->rx.svc_conf.cycle);
      continue;
    }

    /* Start the decoder. */
    msg_t smsg = pktEnableDataReception(radio,
                           0, // Chan = 0 for now
                           conf->rx.radio_conf.rssi,
                           mapCallback,
                           TIME_S2I(10));
    if(smsg != MSG_OK) {
      pktCloseRadioReceive(radio);
      TRACE_ERROR("RX   > Start of radio %d packet reception failed with error %d",
                  radio, smsg);
      time = waitForTrigger(time, conf->rx.svc_conf.cycle);
      continue;
    }
    TRACE_INFO("RX   > Radio %d now active", radio);
/*    if(conf->rx.svc_conf.cycle == CYCLE_CONTINUOUSLY)
      break;*/
#else
    msg_t msg = pktStartDataReception(radio,
                                     MOD_AFSK,
                                     conf->rx.radio_conf.freq,
                                     0, // Step is 0 for now. Get from radio record.
                                     0, // Chan = 0 for now
                                     conf->rx.radio_conf.rssi,
                                     mapCallback,
                                     TIME_S2I(10));
    if(msg != MSG_OK) {
      TRACE_ERROR("RX   > Start of radio %d packet reception failed with error %d",
                  radio, msg);
      time = waitForTrigger(time, conf->rx.svc_conf.cycle);
      continue;
    }
#endif
    /*
     * Check if there is a "listening" interval.
     * In that case turn the receive off after that time.
     */
    if(conf->rx.svc_conf.interval != TIME_IMMEDIATE) {
      chThdSleep(conf->rx.svc_conf.interval);
      TRACE_INFO("RX   > Closing receive on radio %d", radio);
      msg = pktDisableDataReception(radio);
      if(msg != MSG_OK) {
        pktCloseRadioReceive(radio);
        TRACE_ERROR("RX   > Stop of radio packet reception failed");
      }
      msg = pktCloseRadioReceive(radio);
      if(msg != MSG_OK) {
        pktCloseRadioReceive(radio);
        TRACE_ERROR("RX   > Close of radio packet reception failed");
      }
    }
    /* Start reception at next run time (which may be immediately). */
    time = waitForTrigger(time, conf->rx.svc_conf.cycle);
  } while(true);
  /*
   * If there is no cycle time then run continuously by terminating thread.
   * If there is a cycle time and duration then turn the radio off after that duration.
   * Then turn the radio on after cycle time
   * If duration is TIME_INFINITE then the thread is active but sleeps forever.
   * Hence the radio stays active.
   */
  pktThdTerminateSelf();
}

/**
 * TODO: Start radio manager for each radio.
 */
thread_t *start_aprs_threads(thd_aprs_conf_t *conf, const char *name) {

  thread_t *th = chThdCreateFromHeap(NULL,
                               THD_WORKING_AREA_SIZE(PKT_APRS_MAIN_WA_SIZE),
                               name, LOWPRIO, aprsThread, conf);
  if(!th) {
    TRACE_ERROR("APRS > Could not start thread (insufficient memory)");
  }
  return th;
}
