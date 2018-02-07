#include "ch.h"
#include "hal.h"

#include "debug.h"
#include "si446x.h"
#include "geofence.h"

static const char *getModulation(uint8_t key) {
	const char *val[] = {"AFSK", "2FSK"};
	return val[key];
};

bool transmitOnRadio(packet_t packet, uint32_t freq, uint8_t pwr, mod_t mod)
{
	if(freq == FREQ_APRS_DYNAMIC)
		freq = getAPRSRegionFrequency(); // Get transmission frequency by geofencing

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
		}

	} else {

		TRACE_ERROR("RAD  > It is nonsense to transmit 0 bits, %d.%03d MHz, Pwr dBm, %s, %d byte",
					freq/1000000, (freq%1000000)/1000, pwr, getModulation(mod), len
		);

	}

	return true;
}

