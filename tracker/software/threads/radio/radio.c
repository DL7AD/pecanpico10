#include "ch.h"
#include "hal.h"

#include "tracking.h"
#include "debug.h"
#include "radio.h"
#include "geofence.h"
#include "modulation.h"

// Thread
static thread_t* si4464_rx_thd = NULL;
static THD_WORKING_AREA(si4464_rx_wa, 16*1024);

static const char *getModulation(uint8_t key) {
	const char *val[] = {"unknown", "2FSK", "AFSK"};
	return val[key];
};

bool transmitOnRadio(radioMSG_t *msg)
{
	uint32_t freq = getFrequency(msg->freq); // Get transmission frequency

	if(msg->bin_len > 0) // Message length is not zero
	{
		lockRadio(); // Lock radio

		TRACE_INFO(	"RAD  > Transmit %d.%03d MHz, Pwr %d, %s, %d bits",
					freq/1000000, (freq%1000000)/1000, msg->power,
					getModulation(msg->mod), msg->bin_len
		);

		switch(msg->mod)
		{
			case MOD_2FSK:
				init2FSK(msg);
				send2FSK(freq);
				break;
			case MOD_AFSK:
				initAFSK(msg);
				sendAFSK(freq);
				break;
			case MOD_NOT_SET:
				TRACE_ERROR("RAD  > Modulation not set");
				break;
		}

		unlockRadio(); // Unlock radio

	} else {

		TRACE_ERROR("RAD  > It is nonsense to transmit 0 bits, %d.%03d MHz, Pwr dBm, %s, %d bits",
					freq/1000000, (freq%1000000)/1000, msg->power, getModulation(msg->mod), msg->bin_len
		);

	}

	return true;
}

THD_FUNCTION(si_receiver, arg)
{
	(void)arg;

	chRegSetThreadName("radio_receiver");

	while(true)
	{
		chThdSleepMilliseconds(1000);
	}

}

void startReceiver(void)
{
	if(si4464_rx_thd == NULL)
		si4464_rx_thd = chThdCreateStatic(si4464_rx_wa, sizeof(si4464_rx_wa), HIGHPRIO+1, si_receiver, NULL);
}

