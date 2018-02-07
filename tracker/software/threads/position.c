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
	thd_pos_conf_t* conf = (thd_pos_conf_t*)arg;

	// Wait
	if(conf->thread_conf.init_delay) chThdSleep(conf->thread_conf.init_delay);

	// Start tracking manager (if not running yet)
	init_tracking_manager(true);

	// Start position thread
	TRACE_INFO("POS  > Startup position thread");

	// Set telemetry configuration transmission variables
	sysinterval_t last_conf_transmission = chVTGetSystemTime() - TIME_S2I(conf->tel_enc_cycle);
	sysinterval_t time = chVTGetSystemTime();

	while(true)
	{
		TRACE_INFO("POS  > Do module POSITION cycle");

		TRACE_INFO("POS  > Get last track point");
		trackPoint_t* trackPoint = getLastTrackPoint();

		if(!p_sleep(&conf->thread_conf.sleep_conf))
		{
			TRACE_INFO("POS  > Transmit position");

			// Encode/Transmit position packet
			packet_t packet = aprs_encode_position(conf->call, conf->path, conf->symbol, trackPoint);
			transmitOnRadio(packet, conf->radio_conf.freq, conf->radio_conf.pwr, conf->radio_conf.mod);
			chThdSleep(TIME_S2I(5));

			// Encode/Transmit APRSD packet
			packet_t pp = aprs_encode_query_answer_aprsd(conf->call, conf->path, conf->call);
			transmitOnRadio(pp, conf->radio_conf.freq, conf->radio_conf.pwr, conf->radio_conf.mod);

			// Telemetry encoding parameter transmission
			if(conf->tel_enc_cycle != 0 && last_conf_transmission + TIME_S2I(conf->tel_enc_cycle) < chVTGetSystemTime())
			{
				chThdSleep(TIME_S2I(5)); // Take a litte break between the packet transmissions

				TRACE_INFO("POS  > Transmit telemetry configuration");

				// Encode and transmit telemetry config packet
				for(uint8_t type=0; type<4; type++)
				{
					packet = aprs_encode_telemetry_configuration(conf->call, conf->path, type);
					transmitOnRadio(packet, conf->radio_conf.freq, conf->radio_conf.pwr, conf->radio_conf.mod);
					chThdSleep(TIME_S2I(5));
				}

				last_conf_transmission += TIME_S2I(conf->tel_enc_cycle);
			}
		}

		time = waitForTrigger(time, conf->thread_conf.cycle);
	}
}

void start_position_thread(thd_pos_conf_t *conf)
{
	thread_t *th = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(10*1024), "POS", NORMALPRIO, posThread, conf);
	if(!th) {
		// Print startup error, do not start watchdog for this thread
		TRACE_ERROR("POS  > Could not startup thread (not enough memory available)");
	}
}

