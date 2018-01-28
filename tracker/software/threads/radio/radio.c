#include "ch.h"
#include "hal.h"

#include "tracking.h"
#include "debug.h"
#include "radio.h"
#include "geofence.h"
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
		TRACE_INFO(	"RAD  > Transmit %d.%03d MHz, Pwr %d, %s, %d byte",
					freq/1000000, (freq%1000000)/1000, pwr,
					getModulation(mod), len
		);

		switch(mod)
		{
			case MOD_2FSK:
				send2FSK(packet, freq, pwr);
				break;
			case MOD_AFSK:
				sendAFSK(packet, freq, pwr);
				break;
			case MOD_NOT_SET:
				TRACE_ERROR("RAD  > Modulation not set");
				break;
		}

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
#include "regex.h"
#include "dedupe.h"
#include "digipeater.h"

THD_FUNCTION(si_receiver, arg)
{
	(void)arg;

	chRegSetThreadName("radio_receiver");

	receiveAFSK(144800000, 0x2F);

	//palSetLineMode(LINE_IO_RXD, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	//palSetLineMode(LINE_IO_TXD, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	//while(1) {
	//	palWriteLine(LINE_IO_RXD,palReadLine(LINE_RADIO_GPIO1));
	//	palWriteLine(LINE_IO_TXD,palReadLine(LINE_RADIO_IRQ));
	//}













	/* Buffer and size params for serial terminal output. */
	char serial_buf[1024];
	int serial_out;

	/*
	 * Setup the parameters for the AFSK decoder thread.
	 * TODO: Radio configuration to be implemented in pktOpenReceiveChannel().
	 */

	radio_config_t afsk_radio = { PKT_RADIO_1 };

	/* set packet instance assignment(s). */
	pktInitReceiveChannels();

	packet_rx_t *packetHandler = pktOpenReceiveChannel(DECODE_AFSK, &afsk_radio);
	chDbgAssert(packetHandler != NULL, "invalid packet type");

	thread_t *the_decoder = ((AFSKDemodDriver *)packetHandler->link_controller)->decoder_thd;
	chDbgAssert(the_decoder != NULL, "no decoder assigned");

	event_source_t *events = pktGetEventSource(packetHandler);

	/* Start the decoder. */
	msg_t pstart = pktStartDataReception(packetHandler);

	serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "Starting decoder: start status %i, event source @ %x\r\n", pstart, events);
	chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);

	/* Main loop. */
	while (true) {
		pkt_data_fifo_t *myPktFIFO = pktReceiveDataBufferTimeout(packetHandler, TIME_MS2I(1000));
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
			uint16_t actualCRC = frame_buffer[frame_size - 2] | (frame_buffer[frame_size - 1] << 8);
			uint16_t computeCRC = calc_crc16(frame_buffer, 0, frame_size - 2);
			uint16_t magicCRC = calc_crc16(frame_buffer, 0, frame_size);
			if(magicCRC == CRC_INCLUSIVE_CONSTANT)
				packetHandler->valid_count++;
			float32_t good = (float32_t)packetHandler->valid_count / (float32_t)packetHandler->packet_count;
			/* Write out the buffer data.
			 * TODO: Have a define to put diagnostic data into AX25 buffer object.
			 */
			serial_out = chsnprintf(serial_buf, sizeof(serial_buf),
				"AFSK capture: status %x"
				", packet count %u frame count %u valid frames %u (%.2f%%) bytes %u"
				", CRCr %04x, CRCc %04x, CRCm %04x\r\n",
				myPktFIFO->status,
				packetHandler->packet_count,
				packetHandler->frame_count,
				packetHandler->valid_count,
				(good * 100),
				frame_size,
				actualCRC,
				computeCRC,
				magicCRC
			);
			chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);

#define LINE_LENGTH 60U
			uint16_t bufpos;
			uint16_t bufpos_a = 0;
			/* Write out a buffer line as hex first. */
			for(bufpos = 0; bufpos < frame_size; bufpos++) {
				if((bufpos + 1) % LINE_LENGTH == 0) {
					serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "%02x\r\n", frame_buffer[bufpos]);
					chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
					/* Write out full line of converted ASCII under hex.*/
					bufpos_a = (bufpos + 1) - LINE_LENGTH;
					do {
						char asciichar = frame_buffer[bufpos_a];
						if(asciichar == 0x7e) {
							asciichar = '^';
						} else {
							asciichar >>= 1;
							if(!((asciichar >= 0x70 && asciichar < 0x7a) || (asciichar > 0x2f && asciichar < 0x3a) || (asciichar > 0x40 && asciichar < 0x5b))) {
								asciichar = 0x20;
							} else if(asciichar >= 0x70 && asciichar < 0x7a) {
								asciichar &= 0x3f;
							}
						}
						if((bufpos_a + 1) % LINE_LENGTH == 0) {
							serial_out = chsnprintf(serial_buf, sizeof(serial_buf), " %c\r\n", asciichar);
						} else {
							serial_out = chsnprintf(serial_buf, sizeof(serial_buf), " %c ", asciichar);
						}
						chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
					} while(bufpos_a++ < bufpos);
				} else {
					serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "%02x ", frame_buffer[bufpos]);
					chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
				}
			} /* End for(bufpos = 0; bufpos < frame_size; bufpos++). */
			serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "\r\n");
			chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
			/* Write out remaining partial line of converted ASCII under hex. */
			do {
				char asciichar = frame_buffer[bufpos_a];
				if(asciichar == 0x7e) {
					asciichar = '^';
				} else {
					asciichar >>= 1;
					if(!((asciichar >= 0x70 && asciichar < 0x7a) || (asciichar > 0x2f && asciichar < 0x3a) || (asciichar > 0x40 && asciichar < 0x5b))) {
						asciichar = 0x20;
					} else if(asciichar >= 0x70 && asciichar < 0x7a) {
						asciichar &= 0x3f;
					}
				}
				serial_out = chsnprintf(serial_buf, sizeof(serial_buf), " %c ", asciichar);
				chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
			} while(++bufpos_a < bufpos);
			serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "\r\n");
			chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);

			if(actualCRC == computeCRC) {

				packet_t pp = ax25_from_frame(frame_buffer, bufpos-2);
				if(pp != NULL) {
					char rec[1024];
					unsigned char *pinfo;
					ax25_format_addrs(pp, rec);
					ax25_get_info(pp, &pinfo);

					serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "%s", rec);
					chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
					for(uint32_t i=0; pinfo[i]; i++) {
						if(pinfo[i] < 32 || pinfo[i] > 126) {
							serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "<0x%02x>", pinfo[i]);
						} else {
							serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "%c", pinfo[i]);
						}
						chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
					}
					serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "\r\n");
					chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
				} else {
					serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "Error in packet");
					chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
				}
				ax25_delete(pp);

			} else {
				serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "Bad CRC");
				chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
			}

		} else {/* End if valid frame. */
			the_events = EVT_DIAG_OUT_END;
			serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "Invalid frame, status %x, bytes %u\r\n", myPktFIFO->status, myPktFIFO->packet_size);
			chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
		}

#if SUSPEND_HANDLING == RELEASE_ON_OUTPUT
		/*
		 *  Wait for end of transmission on diagnostic channel.
		 */
		eventmask_t evt = chEvtWaitAllTimeout(the_events, TIME_S2I(10));
		if (!evt) {
			serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "FAIL: Timeout waiting for EOT from serial channels\r\n");
			chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
		}
		chEvtSignal(the_decoder, EVT_SUSPEND_EXIT);
#else
		(void)the_events;
#endif
		pktReleaseDataBuffer(packetHandler, myPktFIFO);
		if(packetHandler->packet_count % 100 == 0 && packetHandler->packet_count != 0) {
			/* Stop the decoder. */
			msg_t pmsg = pktStopDataReception(packetHandler);
			serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "Decoder STOP %i\r\n", pmsg);
			chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
			if(packetHandler->packet_count % 1000 == 0 && packetHandler->packet_count != 0) {
				chThdSleep(TIME_S2I(5));
				pmsg = pktCloseReceiveChannel(packetHandler);
				serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "Decoder CLOSE %i\r\n", pmsg);
				chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
				chThdSleep(TIME_S2I(5));
				packetHandler = pktOpenReceiveChannel(DECODE_AFSK, &afsk_radio);
				serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "Decoder OPEN %x\r\n", packetHandler);
				chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
			}
			chThdSleep(TIME_S2I(5));
			pmsg = pktStartDataReception(packetHandler);
			serial_out = chsnprintf(serial_buf, sizeof(serial_buf), "Decoder START %i\r\n", pmsg);
			chnWrite((BaseSequentialStream*)&SDU1, (uint8_t *)serial_buf, serial_out);
		}
	}
}

void startReceiver(void)
{
	if(si446x_rx_thd == NULL)
		si446x_rx_thd = chThdCreateStatic(si446x_rx_wa, sizeof(si446x_rx_wa), HIGHPRIO, si_receiver, NULL);
}

