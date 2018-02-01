#include "ch.h"
#include "hal.h"

#include "debug.h"
#include "threads.h"
#include "config.h"
#include "radio.h"
#include "aprs.h"
#include "sleep.h"
#include "chprintf.h"
#include <string.h>
#include <math.h>
#include "watchdog.h"

THD_FUNCTION(posThread, arg)
{
	module_conf_t* conf = (module_conf_t*)arg;

	// Wait
	if(conf->init_delay) chThdSleep(TIME_MS2I(conf->init_delay));

	// Start tracking manager (if not running yet)
	init_tracking_manager(true);

	// Start position thread
	TRACE_INFO("POS  > Startup position thread");

	// Set telemetry configuration transmission variables
	sysinterval_t last_conf_transmission = chVTGetSystemTimeX() - TIME_S2I(conf->aprs_conf.tel_enc_cycle);
	sysinterval_t time = chVTGetSystemTimeX();

	while(true)
	{
		TRACE_INFO("POS  > Do module POSITION cycle");
		conf->wdg_timeout = chVTGetSystemTimeX() + TIME_S2I(600); // TODO: Implement more sophisticated method

		TRACE_INFO("POS  > Get last track point");
		trackPoint_t* trackPoint = getLastTrackPoint();

		if(!p_sleep(&conf->sleep_conf))
		{
			TRACE_INFO("POS  > Transmit position");

			// Encode/Transmit position packet
			packet_t packet = aprs_encode_position(&(conf->aprs_conf), trackPoint); // Encode packet
			transmitOnRadio(packet, &conf->frequency, conf->power, conf->modulation);
			chThdSleep(TIME_S2I(5));

			// Encode/Transmit APRSD packet
			packet_t pp = aprs_encode_query_answer_aprsd(&(config->aprs_conf), config->aprs_conf.callsign);
			transmitOnRadio(pp, &conf->frequency, conf->power, conf->modulation);

			// Telemetry encoding parameter transmission
			if(conf->aprs_conf.tel_enc_cycle != 0 && last_conf_transmission + TIME_S2I(conf->aprs_conf.tel_enc_cycle) < chVTGetSystemTimeX())
			{
				chThdSleep(TIME_S2I(5)); // Take a litte break between the packet transmissions

				TRACE_INFO("POS  > Transmit telemetry configuration");

				// Encode and transmit telemetry config packet
				for(uint8_t type=0; type<4; type++)
				{
					packet = aprs_encode_telemetry_configuration(&conf->aprs_conf, type);
					transmitOnRadio(packet, &conf->frequency, conf->power, conf->modulation);
					chThdSleep(TIME_S2I(5));
				}

				last_conf_transmission += TIME_S2I(conf->aprs_conf.tel_enc_cycle);
			}
		}

		time = waitForTrigger(time, &conf->trigger);
	}
}

void start_position_thread(module_conf_t *conf)
{
	chsnprintf(conf->name, sizeof(conf->name), "POS");
	thread_t *th = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(10*1024), "POS", NORMALPRIO, posThread, conf);
	if(!th) {
		// Print startup error, do not start watchdog for this thread
		TRACE_ERROR("POS  > Could not startup thread (not enough memory available)");
	} else {
		register_thread_at_wdg(conf);
		conf->wdg_timeout = chVTGetSystemTimeX() + TIME_S2I(1);
	}
}

