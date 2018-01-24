#include "ch.h"
#include "hal.h"

#include "tracking.h"
#include "debug.h"
#include "radio.h"
#include "geofence.h"
#include "modulation.h"
#include "si4464.h"

// Thread
static thread_t* si4464_rx_thd = NULL;
static THD_WORKING_AREA(si4464_rx_wa, 32*1024);

static const char *getModulation(uint8_t key) {
	const char *val[] = {"unknown", "2FSK", "AFSK"};
	return val[key];
};

bool transmitOnRadio(packet_t packet, freq_conf_t *freq_conf, uint8_t pwr, mod_t mod)
{
	uint32_t freq = getFrequency(freq_conf); // Get transmission frequency
	uint8_t *c;
	uint32_t len = ax25_get_info(packet, &c);

	if(len) // Message length is not zero
	{
		lockRadio(); // Lock radio

		TRACE_INFO(	"RAD  > Transmit %d.%03d MHz, Pwr %d, %s, %d byte",
					freq/1000000, (freq%1000000)/1000, pwr,
					getModulation(mod), len
		);

		switch(mod)
		{
			case MOD_2FSK:
				init2FSK();
				send2FSK(packet, freq, pwr);
				break;
			case MOD_AFSK:
				initAFSK();
				sendAFSK(packet, freq, pwr);
				break;
			case MOD_NOT_SET:
				TRACE_ERROR("RAD  > Modulation not set");
				break;
		}

		unlockRadio(); // Unlock radio

	} else {

		TRACE_ERROR("RAD  > It is nonsense to transmit 0 bits, %d.%03d MHz, Pwr dBm, %s, %d byte",
					freq/1000000, (freq%1000000)/1000, pwr, getModulation(mod), len
		);

	}

	return true;
}

#include "pktconf.h"

THD_FUNCTION(si_receiver, arg)
{
	(void)arg;

	chRegSetThreadName("radio_receiver");




	init145_175();
	//setFrequency(144800000, 0);
	startRx();
	palSetPadMode(GPIOA,8, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetLineMode(LINE_RADIO_GPIO1, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);
	while(1) {
		palWritePad(GPIOA,8,palReadLine(LINE_RADIO_GPIO1));
	}



  /* Buffer and size params for serial terminal output. */
  char serial_buf[1024];

#if SUSPEND_HANDLING != NO_SUSPEND
  /*
   * Register for serial diag_out events.
   */
  event_listener_t d_listener;
  event_listener_t p_listener;

#endif

  /*
   * Setup the parameters for the AFSK decoder thread.
   * TODO: Radio configuration to be implemented in pktOpenReceiveChannel().
   */

  radio_config_t afsk_radio = { PKT_RADIO_1 };

  /* set packet instance assignment(s). */
  pktInitReceiveChannels();

  packet_rx_t *packetHandler = pktOpenReceiveChannel(DECODE_AFSK, &afsk_radio);
  chDbgAssert(packetHandler != NULL, "invalid packet type");

  thread_t *the_decoder =
      ((AFSKDemodDriver *)packetHandler->link_controller)->decoder_thd;
  chDbgAssert(the_decoder != NULL, "no decoder assigned");

  event_source_t *events = pktGetEventSource(packetHandler);

  chsnprintf(serial_buf, sizeof(serial_buf),
                  "\r\nStarting main\r\n");
  TRACE_DEBUG(serial_buf);

  /* Test thread start. Decoder thread will start and be in WAIT state. */
  chThdSleep(TIME_S2I(10));

  /* Start the decoder. */
  msg_t pstart = pktStartDataReception(packetHandler);

  chsnprintf(serial_buf, sizeof(serial_buf),
                  "Starting decoder: start status %i, event source @ %x\r\n",
                  pstart, events);
  TRACE_DEBUG(serial_buf);

  /* Main loop. */
  while (true) {
    pkt_data_fifo_t *myPktFIFO =
        pktReceiveDataBufferTimeout(packetHandler, TIME_MS2I(1000));
    if(myPktFIFO == NULL) {
      continue;
    }
    /* Packet buffer sent via FIFO. */
    ax25char_t *frame_buffer = myPktFIFO->buffer;
    uint16_t frame_size = myPktFIFO->packet_size;
    eventmask_t the_events;
    packetHandler->frame_count++;
    if(pktIsBufferValidAX25Frame(myPktFIFO) == MSG_OK) {
      the_events = EVT_DIAG_OUT_END | EVT_PKT_OUT_END;
      uint16_t actualCRC = frame_buffer[frame_size - 2]
        | (frame_buffer[frame_size - 1] << 8);
      uint16_t computeCRC = calc_crc16(frame_buffer, 0, frame_size - 2);
      uint16_t magicCRC = calc_crc16(frame_buffer, 0, frame_size);
      if(magicCRC == CRC_INCLUSIVE_CONSTANT)
        packetHandler->valid_count++;
      float32_t good = (float32_t)packetHandler->valid_count
          / (float32_t)packetHandler->packet_count;
    /* Write out the buffer data.
     * TODO: Have a define to put diagnostic data into AX25 buffer object.
     */
    chsnprintf(serial_buf, sizeof(serial_buf),
      "AFSK capture: status %x"
      ", packet count %u frame count %u valid frames %u (%.2f%%) bytes %u"
      ", CRCr %04x, CRCc %04x, CRCm %04x\r\n"
      /*", phase correction %i, phase drift %i\r\n"*/,
      myPktFIFO->status,
      packetHandler->packet_count,
      packetHandler->frame_count,
      packetHandler->valid_count,
      (good * 100),
      frame_size,
      actualCRC,
      computeCRC,
      magicCRC/*,
      myPktFIFO->correction,
      myPktFIFO->drift*/);
/*      ((qcorr_decoder_t *)((AFSKDemodDriver *)
          packetHandler->link_controller)->tone_decoder)->phase_correction,
      ((qcorr_decoder_t *)((AFSKDemodDriver *)
          packetHandler->link_controller)->tone_decoder)->pll_locked_integrator
          / QCORR_PLL_COMB_SIZE);*/
    TRACE_DEBUG(serial_buf);

#define LINE_LENGTH 60U
    uint16_t bufpos;
    uint16_t bufpos_a = 0;
    /* Write out a buffer line as hex first. */
    for(bufpos = 0; bufpos < frame_size; bufpos++) {
      if((bufpos + 1) % LINE_LENGTH == 0) {
        chsnprintf(serial_buf, sizeof(serial_buf),
                                "%02x\r\n", frame_buffer[bufpos]);
        TRACE_DEBUG(serial_buf);
        /* Write out full line of converted ASCII under hex.*/
        bufpos_a = (bufpos + 1) - LINE_LENGTH;
        do {
          char asciichar = frame_buffer[bufpos_a];
          if(asciichar == 0x7e) {
            asciichar = '^';
          } else {
            asciichar >>= 1;
            if(!((asciichar >= 0x70 && asciichar < 0x7a)
              || (asciichar > 0x2f && asciichar < 0x3a)
              || (asciichar > 0x40 && asciichar < 0x5b))) {
              asciichar = 0x20;
            } else if(asciichar >= 0x70 && asciichar < 0x7a) {
              asciichar &= 0x3f;
            }
          }
          if((bufpos_a + 1) % LINE_LENGTH == 0) {
            chsnprintf(serial_buf, sizeof(serial_buf),
                                    " %c\r\n", asciichar);
          }
          else {
            chsnprintf(serial_buf, sizeof(serial_buf),
                                    " %c ", asciichar);
          }
          TRACE_DEBUG(serial_buf);
        } while(bufpos_a++ < bufpos);
      }
      else {
        chsnprintf(serial_buf, sizeof(serial_buf),
                                "%02x %c", frame_buffer[bufpos], frame_buffer[bufpos]);
        TRACE_DEBUG(serial_buf);
      }
    } /* End for(bufpos = 0; bufpos < frame_size; bufpos++). */
    chsnprintf(serial_buf, sizeof(serial_buf), "\r\n");
    TRACE_DEBUG(serial_buf);
    /* Write out remaining partial line of converted ASCII under hex. */
    do {
      char asciichar = frame_buffer[bufpos_a];
      if(asciichar == 0x7e) {
        asciichar = '^';
      } else {
        asciichar >>= 1;
        if(!((asciichar >= 0x70 && asciichar < 0x7a)
          || (asciichar > 0x2f && asciichar < 0x3a)
          || (asciichar > 0x40 && asciichar < 0x5b))) {
          asciichar = 0x20;
        } else if(asciichar >= 0x70 && asciichar < 0x7a) {
          asciichar &= 0x3f;
        }
      }
      chsnprintf(serial_buf, sizeof(serial_buf), " %c ",
                              asciichar);
      //TRACE_DEBUG(serial_buf);
      } while(++bufpos_a < bufpos);
      chsnprintf(serial_buf, sizeof(serial_buf), "\r\n");
      //TRACE_DEBUG(serial_buf);

      /* Now dump the packet data. */
      uint16_t hx;
      for (hx = 0; hx <= frame_size; hx++) {
        hexout((BaseSequentialStream*)&SD3, frame_buffer[hx], (hx == frame_size));
      }

    } else {/* End if valid frame. */
      the_events = EVT_DIAG_OUT_END;
      chsnprintf(serial_buf,
       sizeof(serial_buf), "Invalid frame,"
       " status %x, bytes %u\r\n",
/*       ", phase correction %i, phase drift %i\r\n"*/
       myPktFIFO->status, myPktFIFO->packet_size/*,
       myPktFIFO->correction,
       myPktFIFO->drift*/);
/*       ((qcorr_decoder_t *)((AFSKDemodDriver *)
           packetHandler->link_controller)->tone_decoder)->phase_correction,
       ((qcorr_decoder_t *)((AFSKDemodDriver *)
           packetHandler->link_controller)->tone_decoder)->pll_locked_integrator
           / QCORR_PLL_COMB_SIZE);*/
      TRACE_DEBUG(serial_buf);
    }

#if SUSPEND_HANDLING == RELEASE_ON_OUTPUT
    /*
     *  Wait for end of transmission on diagnostic channel.
     */
    eventmask_t evt = chEvtWaitAllTimeout(the_events, TIME_S2I(10));
    if (!evt) {
      chsnprintf(serial_buf, sizeof(serial_buf),
                       "FAIL: Timeout waiting for EOT from serial channels\r\n");
      TRACE_DEBUG(serial_buf);
    }
    chEvtSignal(the_decoder, EVT_SUSPEND_EXIT);
#else
    (void)the_events;
#endif
    pktReleaseDataBuffer(packetHandler, myPktFIFO);
    if(packetHandler->packet_count % 10 == 0
        && packetHandler->packet_count != 0) {
      /* Stop the decoder. */
      msg_t pmsg = pktStopDataReception(packetHandler);
      chsnprintf(serial_buf, sizeof(serial_buf),
                              "Decoder STOP %i\r\n", pmsg);
      TRACE_DEBUG(serial_buf);
      if(packetHandler->packet_count % 20 == 0
          && packetHandler->packet_count != 0) {
        chThdSleep(TIME_S2I(5));
        pmsg = pktCloseReceiveChannel(packetHandler);
        chsnprintf(serial_buf, sizeof(serial_buf),
                                "Decoder CLOSE %i\r\n", pmsg);
        TRACE_DEBUG(serial_buf);
        chThdSleep(TIME_S2I(5));
        packetHandler = pktOpenReceiveChannel(DECODE_AFSK, &afsk_radio);
        chsnprintf(serial_buf, sizeof(serial_buf),
                                "Decoder OPEN %x\r\n", packetHandler);
        TRACE_DEBUG(serial_buf);
      }
      chThdSleep(TIME_S2I(5));
      pmsg = pktStartDataReception(packetHandler);
      chsnprintf(serial_buf, sizeof(serial_buf),
                              "Decoder START %i\r\n", pmsg);
      TRACE_DEBUG(serial_buf);
    }
  }






}

void startReceiver(void)
{
	if(si4464_rx_thd == NULL)
		si4464_rx_thd = chThdCreateStatic(si4464_rx_wa, sizeof(si4464_rx_wa), HIGHPRIO+1, si_receiver, NULL);
}

