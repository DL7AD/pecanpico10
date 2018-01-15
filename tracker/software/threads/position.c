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
	if(conf->init_delay) chThdSleepMilliseconds(conf->init_delay);

	// Start tracking manager (if not running yet)
	init_tracking_manager(true);

	// Start position thread
	TRACE_INFO("POS  > Startup position thread");

	// Set telemetry configuration transmission variables
	systime_t last_conf_transmission = chVTGetSystemTimeX();
	systime_t time = chVTGetSystemTimeX();

	while(true)
	{
		TRACE_INFO("POS  > Do module POSITION cycle");
		conf->wdg_timeout = chVTGetSystemTimeX() + S2ST(600); // TODO: Implement more sophisticated method

		TRACE_INFO("POS  > Get last track point");
		trackPoint_t* trackPoint = getLastTrackPoint();

		if(!p_sleep(&conf->sleep_conf))
		{

			TRACE_INFO("POS  > Transmit position");

			radioMSG_t msg;
			uint8_t buffer[512];
			msg.buffer = buffer;
			msg.freq = &conf->frequency;
			msg.power = conf->power;

			if(isAPRS(conf->protocol)) {

				// Position transmission
				msg.mod = conf->protocol == PROT_APRS_AFSK ? MOD_AFSK : MOD_2FSK;
				msg.fsk_conf = &(conf->fsk_conf);
				msg.afsk_conf = &(conf->afsk_conf);

				ax25_t ax25_handle;

				// Encode and transmit position packet
				aprs_encode_init(&ax25_handle, buffer, sizeof(buffer), msg.mod);
				aprs_encode_position(&ax25_handle, &(conf->aprs_conf), trackPoint); // Encode packet
				msg.bin_len = aprs_encode_finalize(&ax25_handle);
				transmitOnRadio(&msg);
				if(conf->redundantTx) transmitOnRadio(&msg);

				// Telemetry encoding parameter transmission
				if(conf->aprs_conf.tel_enc_cycle != 0 && last_conf_transmission + S2ST(conf->aprs_conf.tel_enc_cycle) < chVTGetSystemTimeX())
				{
					chThdSleepMilliseconds(5000); // Take a litte break between the packet transmissions

					TRACE_INFO("POS  > Transmit telemetry configuration");

					// Encode and transmit telemetry config packet
					aprs_encode_init(&ax25_handle, buffer, sizeof(buffer), msg.mod);
					aprs_encode_telemetry_configuration(&ax25_handle, &conf->aprs_conf);
					msg.bin_len = aprs_encode_finalize(&ax25_handle);
					transmitOnRadio(&msg);
					if(conf->redundantTx) transmitOnRadio(&msg);
				}
					
			} else {

				TRACE_ERROR("POS  > Unsupported modulation/protocol selected for module POSITION");

			}
		}

		time = waitForTrigger(time, &conf->trigger);
	}
}

void start_position_thread(module_conf_t *conf)
{
	chsnprintf(conf->name, sizeof(conf->name), "POS");
	thread_t *th = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(5*1024), "POS", NORMALPRIO, posThread, conf);
	if(!th) {
		// Print startup error, do not start watchdog for this thread
		TRACE_ERROR("POS  > Could not startup thread (not enough memory available)");
	} else {
		register_thread_at_wdg(conf);
		conf->wdg_timeout = chVTGetSystemTimeX() + S2ST(1);
	}
}

