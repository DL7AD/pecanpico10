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

/*
 *
 */
THD_FUNCTION(posThread, arg)
{
	thd_pos_conf_t* conf = (thd_pos_conf_t*)arg;

	// Wait
	if(conf->thread_conf.init_delay) chThdSleep(conf->thread_conf.init_delay);

	// Start data collector (if not running yet)
	init_data_collector();

	// Start position thread
	TRACE_INFO("POS  > Startup position thread");

	// Set telemetry configuration transmission variables
	sysinterval_t last_conf_transmission =
	    chVTGetSystemTime() - conf->tel_enc_cycle;
	sysinterval_t time = chVTGetSystemTime();

	while(true) {
		TRACE_INFO("POS  > Do module POSITION cycle");

		TRACE_INFO("POS  > Get last data point");
		dataPoint_t* dataPoint = getLastDataPoint();

		if(!p_sleep(&conf->thread_conf.sleep_conf)) {
			TRACE_INFO("POS  > Transmit position");

			// Encode/Transmit position packet
			packet_t packet = aprs_encode_position(conf->call, conf->path,
			                                       conf->symbol, dataPoint);
            if(packet == NULL) {
              TRACE_WARN("POS  > No free packet objects"
                  " for position transmission");
            } else {
              if(!transmitOnRadio(packet,
                              conf->radio_conf.freq,
                              0,
                              0,
                              conf->radio_conf.pwr,
                              conf->radio_conf.mod,
                              conf->radio_conf.rssi)) {
                TRACE_ERROR("POS  > failed to transmit position data");
              }
              chThdSleep(TIME_S2I(5));
            }

			// Encode/Transmit APRSD packet
            /*
             * FIXME: When sending out an unsolicited APRSD who to send it to?
             * Should there be a callsign set in config that APRSD is sent to?
             * For now just send to self... doesn't make sense though.
             */
			bool rx = conf->aprs_msg;
			packet = aprs_encode_query_answer_aprsd(conf->call,
                                                 conf->path,
                                                 rx ? conf->call
                                                    : conf_sram.aprs.tx.call);
            if(packet == NULL) {
              TRACE_WARN("POS  > No free packet objects for "
                  "APRSD transmission");
            } else {
              if(!transmitOnRadio(packet,
                              conf->radio_conf.freq,
                              0,
                              0,
                              conf->radio_conf.pwr,
                              conf->radio_conf.mod,
                              conf->radio_conf.rssi)) {
                TRACE_ERROR("POS  > Failed to transmit APRSD data");
              }
              chThdSleep(TIME_S2I(5));
            }

			// Telemetry encoding parameter transmission
			if(conf->tel_enc_cycle != 0 && last_conf_transmission
			    + conf->tel_enc_cycle < chVTGetSystemTime()) {


				TRACE_INFO("POS  > Transmit telemetry configuration");

				// Encode and transmit telemetry config packet
				for(uint8_t type = 0; type < 4; type++) {
					packet = aprs_encode_telemetry_configuration(conf->call,
					                           conf->path, type);
		            if(packet == NULL) {
		              TRACE_WARN("POS  > No free packet objects for"
		                  " telemetry transmission");
		            } else {
                      if(!transmitOnRadio(packet,
                                      conf->radio_conf.freq,
                                      0,
                                      0,
                                      conf->radio_conf.pwr,
                                      conf->radio_conf.mod,
                                      conf->radio_conf.rssi)) {
                       TRACE_ERROR("POS  > Failed to transmit telemetry data");
                      }
                      chThdSleep(TIME_S2I(5));
		            }
				}

				last_conf_transmission += conf->tel_enc_cycle;
			}
		}
		time = waitForTrigger(time, conf->thread_conf.cycle);
	}
}

/*
 *
 */
void start_position_thread(thd_pos_conf_t *conf)
{
	thread_t *th = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(20*1024),
	                                   "POS", LOWPRIO, posThread, conf);
	if(!th) {
		// Print startup error, do not start watchdog for this thread
		TRACE_ERROR("POS  > Could not startup thread (not enough memory available)");
	}
}

