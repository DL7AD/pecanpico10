#include "ch.h"
#include "hal.h"

#include "tracking.h"
#include "debug.h"
#include "radio.h"
#include "geofence.h"
#include "modulation.h"
#include "si446x.h"

// Thread
static thread_t* si446x_rx_thd = NULL;
static THD_WORKING_AREA(si446x_rx_wa, 32*1024);

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

#include <stdlib.h>
#include <string.h>
#include <ctype.h>	/* for isdigit, isupper */

#include "pktconf.h"
#include <regex.h>
#include "dedupe.h"
#include "digipeater.h"

char serial_buf[1024];

THD_FUNCTION(si_receiver, arg)
{
	(void)arg;

	chRegSetThreadName("radio_receiver");


	Si446x_init();
	Si446x_setModemAFSK_RX();
	Si446x_receive(144800000, 0x4F);

	//init144_800();
	//startRx();

	//palSetPadMode(GPIOA,8, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	//palSetLineMode(LINE_RADIO_GPIO1, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);
	//while(1) {
	//	palWritePad(GPIOA,8,palReadLine(LINE_RADIO_GPIO1));
	//}


	/*char mycall[] = "DL7AD-12";
	regex_t alias_re;
	regex_t wide_re;

	int e;
	char message[256];

	dedupe_init(TIME_S2I(4));

	e = regcomp(&alias_re, "^WIDE[4-7]-[1-7]|CITYD$", REG_EXTENDED|REG_NOSUB);
	if(e != 0) {
		regerror(e, &alias_re, message, sizeof(message));
		TRACE_DEBUG("\n%s\n\n", message);
	}

	e = regcomp(&wide_re, "^WIDE[1-7]-[1-7]$|^TRACE[1-7]-[1-7]$|^MA[1-7]-[1-7]$", REG_EXTENDED|REG_NOSUB);
	if(e != 0) {
		regerror(e, &wide_re, message, sizeof(message));
		TRACE_DEBUG("\n%s\n\n", message);
	}*/








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

  TRACE_DEBUG("Starting main");

  /* Test thread start. Decoder thread will start and be in WAIT state. */
  chThdSleep(TIME_S2I(1));

  /* Start the decoder. */
  msg_t pstart = pktStartDataReception(packetHandler);

  TRACE_DEBUG("Starting decoder: start status %i, event source @ %x", pstart, events);

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
    TRACE_DEBUG("AFSK capture: status %x"
      ", packet count %u frame count %u valid frames %u (%.2f%%) bytes %u"
      ", CRCr %04x, CRCc %04x, CRCm %04x",
      myPktFIFO->status,
      packetHandler->packet_count,
      packetHandler->frame_count,
      packetHandler->valid_count,
      (good * 100),
      frame_size,
      actualCRC,
      computeCRC,
      magicCRC);
    

#define LINE_LENGTH 60U
    uint16_t bufpos;
    uint16_t bufpos_a = 0;
    /* Write out a buffer line as hex first. */
    for(bufpos = 0; bufpos < frame_size; bufpos++) {
      if((bufpos + 1) % LINE_LENGTH == 0) {
        //TRACE_DEBUG("%02x\r\n", frame_buffer[bufpos]);
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
            //TRACE_DEBUG(" %c\r\n", asciichar);
          }
          else {
            //TRACE_DEBUG(" %c ", asciichar);
          }
        } while(bufpos_a++ < bufpos);
      }
      else {
        //TRACE_DEBUG("%02x %c", frame_buffer[bufpos], frame_buffer[bufpos]);
      }
    } /* End for(bufpos = 0; bufpos < frame_size; bufpos++). */

    /* Write out remaining partial line of converted ASCII under hex. */
    /*do {
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
      TRACE_DEBUG(" %c ",
                              asciichar);

      } while(++bufpos_a < bufpos);*/





	packet_t pp = ax25_from_frame(frame_buffer, bufpos);
    if(pp != NULL) {
		char rec[1024];
		unsigned char *pinfo;
		ax25_format_addrs (pp, rec);
		ax25_get_info (pp, &pinfo);
		strlcat(rec, (char*)pinfo, sizeof(rec));

		//packet_t result = digipeat_match(0, pp, mycall, mycall, &alias_re, &wide_re, 0, PREEMPT_OFF, NULL);
		//if(result != NULL) {
			//char xmit[1024];

			//dedupe_remember(result, 0);
			//ax25_format_addrs(result, xmit);
			//info_len = ax25_get_info(result, &pinfo);
			//strlcat(xmit, (char*)pinfo, sizeof(xmit));
			//*frame_out_len = ax25_pack(result, frame_out);
			//ax25_delete(result);

			//TRACE_DEBUG("Xmit %s", xmit);
		//}


		TRACE_DEBUG("Rec %s", rec);
	}
	ax25_delete(pp);




      /* Now dump the packet data. */
      /*uint16_t hx;
      for (hx = 0; hx <= frame_size; hx++) {
        hexout((BaseSequentialStream*)&SDU1, frame_buffer[hx], (hx == frame_size));
      }*/

    }

#if SUSPEND_HANDLING == RELEASE_ON_OUTPUT
    /*
     *  Wait for end of transmission on diagnostic channel.
     */
    eventmask_t evt = chEvtWaitAllTimeout(the_events, TIME_S2I(10));
    if (!evt) {
      TRACE_DEBUG("FAIL: Timeout waiting for EOT from serial channels");
    }
    chEvtSignal(the_decoder, EVT_SUSPEND_EXIT);
#else
    (void)the_events;
#endif
    pktReleaseDataBuffer(packetHandler, myPktFIFO);
    if(packetHandler->packet_count % 50 == 0
        && packetHandler->packet_count != 0) {
      /* Stop the decoder. */
      msg_t pmsg = pktStopDataReception(packetHandler);
      TRACE_DEBUG("Decoder STOP %i", pmsg);
      if(packetHandler->packet_count % 100 == 0
          && packetHandler->packet_count != 0) {
        chThdSleep(TIME_S2I(5));
        pmsg = pktCloseReceiveChannel(packetHandler);
        TRACE_DEBUG("Decoder CLOSE %i", pmsg);
        chThdSleep(TIME_S2I(5));
        packetHandler = pktOpenReceiveChannel(DECODE_AFSK, &afsk_radio);
        TRACE_DEBUG("Decoder OPEN %x", packetHandler);
      }
      chThdSleep(TIME_S2I(5));
      pmsg = pktStartDataReception(packetHandler);
      TRACE_DEBUG("Decoder START %i", pmsg);
    }
  }












}

void startReceiver(void)
{
	if(si446x_rx_thd == NULL)
		si446x_rx_thd = chThdCreateStatic(si446x_rx_wa, sizeof(si446x_rx_wa), HIGHPRIO, si_receiver, NULL);
}

