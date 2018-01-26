#include "ch.h"
#include "hal.h"

#include "si446x.h"
#include "debug.h"
#include <string.h>
#include "ax25_pad.h"
#include "fcs_calc.h"

// AFSK
#define PLAYBACK_RATE		13200
#define BAUD_RATE			1200									/* APRS AFSK baudrate */
#define SAMPLES_PER_BAUD	(PLAYBACK_RATE / BAUD_RATE)				/* Samples per baud (192kHz / 1200baud = 160samp/baud) */
#define PHASE_DELTA_1200	(((2 * 1200) << 16) / PLAYBACK_RATE)	/* Delta-phase per sample for 1200Hz tone */
#define PHASE_DELTA_2200	(((2 * 2200) << 16) / PLAYBACK_RATE)	/* Delta-phase per sample for 2200Hz tone */

static uint32_t phase_delta;			// 1200/2200 for standard AX.25
static uint32_t phase;					// Fixed point 9.7 (2PI = TABLE_SIZE)
static uint32_t packet_pos;				// Next bit to be sent out
static uint32_t current_sample_in_baud;	// 1 bit = SAMPLES_PER_BAUD samples
static uint8_t current_byte;
static uint8_t ctone = 0;

// Thread
static thread_t* feeder_thd = NULL;
static THD_WORKING_AREA(si_fifo_feeder_wa, 4096);

// Mutex
static mutex_t radio_mtx;				// Radio mutex
static bool nextTransmissionWaiting;	// Flag that informs the feeder thread to keep the radio switched on
static bool radio_mtx_init = false;

// Modulation and buffer
static mod_t active_mod = MOD_NOT_SET;
static packet_t radio_packet;
static uint32_t radio_freq;
static uint8_t radio_pwr;

 void shutdownRadio(void)
{
	// Wait for PH to finish transmission
	while(Si446x_getState() == Si446x_STATE_TX)
		chThdSleep(TIME_MS2I(1));

	if(!nextTransmissionWaiting) { // No thread is waiting for radio, so shutdown radio
		TRACE_INFO("RAD  > Transmission finished");
		TRACE_INFO("RAD  > Shutdown radio");
		Si446x_shutdown();
		active_mod = MOD_NOT_SET;
	} else {
		TRACE_INFO("RAD  > Transmission finished");
		TRACE_INFO("RAD  > Keep radio switched on");
	}
}

/* ======================================================================== Locking ========================================================================= */

void lockRadio(void)
{
	// Initialize mutex
	if(!radio_mtx_init)
		chMtxObjectInit(&radio_mtx);
	radio_mtx_init = true;

	chMtxLock(&radio_mtx);
	nextTransmissionWaiting = true;

	// Wait for old feeder thread to terminate
	if(feeder_thd != NULL) // No waiting on first use
		chThdWait(feeder_thd);
}

/* This method is only called by image.c. It's not different to lockRadio() with
 * the exception that the radio it shutdown after transmission (if there is any) */
void lockRadioByCamera(void)
{
	// Initialize mutex
	if(!radio_mtx_init)
		chMtxObjectInit(&radio_mtx);
	radio_mtx_init = true;

	chMtxLock(&radio_mtx);

	// Wait for old feeder thread to terminate
	if(feeder_thd != NULL) // No waiting on first use
		chThdWait(feeder_thd);
}

void unlockRadio(void)
{
	nextTransmissionWaiting = false;
	chMtxUnlock(&radio_mtx);
}

/* ========================================================================== AFSK ========================================================================== */

void initAFSK(void) {
	if(active_mod == MOD_AFSK)
		return;

	// Initialize radio
	Si446x_init();
	Si446x_setModemAFSK_TX();
	active_mod = MOD_AFSK;
}

static bool encode_nrzi(bool bit)
{
	if((bit & 0x1) == 0)
		ctone = !ctone;
	return ctone;
}

static uint32_t afsk_pack(packet_t pp, uint8_t* buf, uint32_t buf_len)
{
	memset(buf, 0, buf_len); // Clear buffer
	uint32_t blen = 0;

	// Preamble
	for(uint8_t i=0; i<30; i++) {
		for(uint8_t j=0; j<8; j++) {

			if(blen >> 3 >= buf_len) { // Buffer overflow
				TRACE_ERROR("Packet too long");
				return blen;
			}

			buf[blen >> 3] |= encode_nrzi((0x7E >> j) & 0x1) << (blen % 8);
			blen++;
		}
	}

	// Insert CRC to buffer
	uint16_t crc = fcs_calc(pp->frame_data, pp->frame_len);
	pp->frame_data[pp->frame_len++] = crc & 0xFF;
	pp->frame_data[pp->frame_len++] = crc >> 8;

	uint32_t pos = 0;
	uint8_t bitstuff_cntr = 0;

	while(pos < (uint32_t)pp->frame_len*8)
	{
		if(blen >> 3 >= buf_len) { // Buffer overflow
			TRACE_ERROR("Packet too long");
			return blen;
		}

		bool bit;
		if(bitstuff_cntr < 5) { // Normale bit

			bit = (pp->frame_data[pos >> 3] >> (pos%8)) & 0x1;
			if(bit == 1) {
				bitstuff_cntr++;
			} else {
				bitstuff_cntr = 0;
			}
			pos++;

		} else { // Fill stuffing bit

			bit = 0;
			bitstuff_cntr = 0;

		}

		// NRZ-I encode bit
		bool nrzi = encode_nrzi(bit);

		buf[blen >> 3] |= nrzi << (blen % 8);
		blen++;
	}

	// Final flag
	for(uint8_t i=0; i<10; i++)
	for(uint8_t j=0; j<8; j++) {

		if(blen >> 3 >= buf_len) { // Buffer overflow
			TRACE_ERROR("Packet too long");
			return blen;
		}

		buf[blen >> 3] |= encode_nrzi((0x7E >> j) & 0x1) << (blen % 8);
		blen++;
	}

	return blen;
}

static uint8_t getAFSKbyte(uint8_t* buf, uint32_t blen)
{
	if(packet_pos == blen) 	// Packet transmission finished
		return false;

	uint8_t b = 0;
	for(uint8_t i=0; i<8; i++)
	{
		if(current_sample_in_baud == 0) {
			if((packet_pos & 7) == 0) { // Load up next byte
				current_byte = buf[packet_pos >> 3];
			} else { // Load up next bit
				current_byte = current_byte / 2;
			}
		}

		// Toggle tone (1200 <> 2200)
		phase_delta = (current_byte & 1) ? PHASE_DELTA_1200 : PHASE_DELTA_2200;

		phase += phase_delta;			// Add delta-phase (delta-phase tone dependent)
		b |= ((phase >> 16) & 1) << i;	// Set modulation bit

		current_sample_in_baud++;

		if(current_sample_in_baud == SAMPLES_PER_BAUD) {	// Old bit consumed, load next bit
			current_sample_in_baud = 0;
			packet_pos++;
		}
	}

	return b;
}

THD_FUNCTION(si_fifo_feeder_afsk, arg)
{
	(void)arg;
	chRegSetThreadName("radio_afsk_feeder");

	uint8_t layer0[3072];
	uint32_t layer0_blen = afsk_pack(radio_packet, layer0, sizeof(layer0));

	// Initialize variables for timer
	phase_delta = PHASE_DELTA_1200;
	phase = 0;
	packet_pos = 0;
	current_sample_in_baud = 0;
	current_byte = 0;
	uint8_t localBuffer[129];
	uint16_t c = 129;
	uint16_t all = (layer0_blen*SAMPLES_PER_BAUD+7)/8;

	// Initial FIFO fill
	for(uint16_t i=0; i<c; i++)
		localBuffer[i] = getAFSKbyte(layer0, layer0_blen);
	Si446x_writeFIFO(localBuffer, c);

	// Start transmission
	Si446x_transmit(radio_freq, radio_pwr, all, 0x3F, TIME_S2I(3));

	while(c < all) { // Do while bytes not written into FIFO completely
		// Determine free memory in Si446x-FIFO
		uint8_t more = Si446x_freeFIFO();
		if(more > all-c) {
			if((more = all-c) == 0) // Calculate remainder to send
              break; // End if nothing left
		}

		for(uint16_t i=0; i<more; i++)
			localBuffer[i] = getAFSKbyte(layer0, layer0_blen);

		Si446x_writeFIFO(localBuffer, more); // Write into FIFO
		c += more;
		chThdSleep(TIME_MS2I(15));
	}
	// Shutdown radio (and wait for Si446x to finish transmission)
	shutdownRadio();




	// Delete packet
	ax25_delete(radio_packet);

	chThdExit(MSG_OK);
}

void sendAFSK(packet_t packet, uint32_t freq, uint8_t pwr) {
	// Set pointers for feeder
	radio_packet = packet;
	radio_freq = freq;
	radio_pwr = pwr;

	// Start/re-start FIFO feeder
	feeder_thd = chThdCreateStatic(si_fifo_feeder_wa, sizeof(si_fifo_feeder_wa), HIGHPRIO, si_fifo_feeder_afsk, NULL);

	// Wait for the transmitter to start (because it is used as mutex)
	while(Si446x_getState() != Si446x_STATE_TX)
		chThdSleep(TIME_MS2I(1));
}

/* ========================================================================== 2FSK ========================================================================== */

void init2FSK(void) {
	if(active_mod == MOD_2FSK)
		return;

	// Initialize radio
	Si446x_init();
	Si446x_setModem2FSK(9600);
	active_mod = MOD_2FSK;
}

THD_FUNCTION(si_fifo_feeder_fsk, arg)
{
	(void)arg;
	chRegSetThreadName("radio_2fsk_feeder");

	//uint8_t *frame = radio_packet->frame_data;
	//uint32_t len = radio_packet->frame_len;

	/*uint16_t c = 129;
	uint16_t all = (radio_msg.bin_len+7)/8;

	// Initial FIFO fill
	Si446x_writeFIFO(radio_msg.buffer, c);

	// Start transmission
	radioTune((uint32_t)frequency, 0, radio_msg.power, all);

	while(c < all) { // Do while bytes not written into FIFO completely
		// Determine free memory in Si446x-FIFO
		uint8_t more = Si446x_freeFIFO();
		if(more > all-c) {
			if((more = all-c) == 0) // Calculate remainder to send
              break; // End if nothing left
		}
		Si446x_writeFIFO(&radio_msg.buffer[c], more); // Write into FIFO
		c += more;
		chThdSleep(TIME_MS2I(15)); // That value is ok up to 96k
	}*/

	// Shutdown radio (and wait for Si446x to finish transmission)
	shutdownRadio();

	chThdExit(MSG_OK);
}

void send2FSK(packet_t packet, uint32_t freq, uint8_t pwr) {
	// Set pointers for feeder
	radio_packet = packet;
	radio_freq = freq;
	radio_pwr = pwr;

	// Start/re-start FIFO feeder
	feeder_thd = chThdCreateStatic(si_fifo_feeder_wa, sizeof(si_fifo_feeder_wa), HIGHPRIO, si_fifo_feeder_fsk, NULL);

	// Wait for the transmitter to start (because it is used as mutex)
	while(Si446x_getState() != Si446x_STATE_TX)
		chThdSleep(TIME_MS2I(1));
}

