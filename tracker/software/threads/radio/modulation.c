#include "ch.h"
#include "hal.h"

#include "si4464.h"
#include "debug.h"
#include <string.h>

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

// Thread
static thread_t* feeder_thd = NULL;
static THD_WORKING_AREA(si_fifo_feeder_wa, 1024);

// Mutex
static mutex_t radio_mtx;				// Radio mutex
static bool nextTransmissionWaiting;	// Flag that informs the feeder thread to keep the radio switched on
static bool radio_mtx_init = false;

// Modulation and buffer
static mod_t active_mod = MOD_NOT_SET;
static radioMSG_t radio_msg;
static uint8_t radio_buffer[8192];

 void shutdownRadio(void)
{
	// Wait for PH to finish transmission
	while(Si4464_getState() == SI4464_STATE_TX)
		chThdSleepMilliseconds(1);

	if(!nextTransmissionWaiting) { // No thread is waiting for radio, so shutdown radio
		TRACE_INFO("RAD  > Transmission finished");
		TRACE_INFO("RAD  > Shutdown radio");
		Si4464_shutdown();
		active_mod = MOD_NOT_SET;
	} else {
		TRACE_INFO("RAD  > Transmission finished");
		TRACE_INFO("RAD  > Keep radio switched on");
	}
}

static void copyBuffer(radioMSG_t* msg)
{
	// Copy data
	memcpy(&radio_msg, msg, sizeof(radioMSG_t));
	memcpy(&radio_buffer, msg->buffer, sizeof(radio_buffer));
	radio_msg.buffer = radio_buffer;
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

void initAFSK(radioMSG_t* msg) {
	copyBuffer(msg);

	if(active_mod == MOD_AFSK)
		return;

	// Initialize radio
	Si4464_Init();
	setModemAFSK();
	active_mod = MOD_AFSK;
}

static uint8_t getAFSKbyte(void)
{
	if(packet_pos == radio_msg.bin_len) 	// Packet transmission finished
		return false;

	uint8_t b = 0;
	for(uint8_t i=0; i<8; i++)
	{
		if(current_sample_in_baud == 0) {
			if((packet_pos & 7) == 0) { // Load up next byte
				current_byte = radio_msg.buffer[packet_pos >> 3];
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

THD_FUNCTION(si_fifo_feeder_afsk, frequency)
{
	chRegSetThreadName("radio_afsk_feeder");

	// Initialize variables for timer
	phase_delta = PHASE_DELTA_1200;
	phase = 0;
	packet_pos = 0;
	current_sample_in_baud = 0;
	current_byte = 0;
	uint8_t localBuffer[129];
	uint16_t c = 129;
	uint16_t all = (radio_msg.bin_len*SAMPLES_PER_BAUD+7)/8;

	// Initial FIFO fill
	for(uint16_t i=0; i<c; i++)
		localBuffer[i] = getAFSKbyte();
	Si4464_writeFIFO(localBuffer, c);

	// Start transmission
	radioTune((uint32_t)frequency, 0, radio_msg.power, all);

	while(c < all) { // Do while bytes not written into FIFO completely
		// Determine free memory in Si4464-FIFO
		uint8_t more = Si4464_freeFIFO();
		if(more > all-c) {
			if((more = all-c) == 0) // Calculate remainder to send
              break; // End if nothing left
		}

		for(uint16_t i=0; i<more; i++)
			localBuffer[i] = getAFSKbyte();

		Si4464_writeFIFO(localBuffer, more); // Write into FIFO
		c += more;
		chThdSleepMilliseconds(15);
	}
	// Shutdown radio (and wait for Si4464 to finish transmission)
	shutdownRadio();

	chThdExit(MSG_OK);
}

void sendAFSK(uint32_t frequency) {
	// Start/re-start FIFO feeder
	feeder_thd = chThdCreateStatic(si_fifo_feeder_wa, sizeof(si_fifo_feeder_wa), HIGHPRIO+1, si_fifo_feeder_afsk, (void*)frequency);

	// Wait for the transmitter to start (because it is used as mutex)
	while(Si4464_getState() != SI4464_STATE_TX)
		chThdSleepMilliseconds(1);
}

/* ========================================================================== 2FSK ========================================================================== */

void init2FSK(radioMSG_t* msg) {
	copyBuffer(msg);

	if(active_mod == MOD_2FSK)
		return;

	// Initialize radio
	Si4464_Init();
	setModem2FSK(radio_msg.fsk_conf);
	active_mod = MOD_2FSK;
}

THD_FUNCTION(si_fifo_feeder_fsk, frequency)
{
	uint16_t c = 129;
	uint16_t all = (radio_msg.bin_len+7)/8;

	chRegSetThreadName("radio_2fsk_feeder");
	// Initial FIFO fill
	Si4464_writeFIFO(radio_msg.buffer, c);

	// Start transmission
	radioTune((uint32_t)frequency, 0, radio_msg.power, all);

	while(c < all) { // Do while bytes not written into FIFO completely
		// Determine free memory in Si4464-FIFO
		uint8_t more = Si4464_freeFIFO();
		if(more > all-c) {
			if((more = all-c) == 0) // Calculate remainder to send
              break; // End if nothing left
		}
		Si4464_writeFIFO(&radio_msg.buffer[c], more); // Write into FIFO
		c += more;
		chThdSleepMilliseconds(15); // That value is ok up to 96k
	}

	// Shutdown radio (and wait for Si4464 to finish transmission)
	shutdownRadio();

	chThdExit(MSG_OK);
}

void send2FSK(uint32_t frequency) {
	// Start/re-start FIFO feeder
	feeder_thd = chThdCreateStatic(si_fifo_feeder_wa, sizeof(si_fifo_feeder_wa), HIGHPRIO+1, si_fifo_feeder_fsk, (void*)frequency);

	// Wait for the transmitter to start (because it is used as mutex)
	while(Si4464_getState() != SI4464_STATE_TX)
		chThdSleepMilliseconds(1);
}

