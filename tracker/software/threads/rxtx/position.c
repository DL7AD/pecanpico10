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

            // Telemetry encoding parameter transmission
            if(conf->tel_enc_cycle != 0 && last_conf_transmission
                + conf->tel_enc_cycle < chVTGetSystemTime()) {

                TRACE_INFO("POS  > Transmit telemetry configuration");

                // Encode and transmit telemetry config packet
                for(uint8_t type = 0; type < 4; type++) {
                    packet_t packet = aprs_encode_telemetry_configuration(
                                              conf->call,
                                              conf->path,
                                              APRS_DEVICE_CALLSIGN,
                                              type);
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
                                      conf->radio_conf.cca)) {
                       TRACE_ERROR("POS  > Failed to transmit telemetry data");
                      }
                      chThdSleep(TIME_S2I(5));
                    }
                }

                last_conf_transmission += conf->tel_enc_cycle;
            }
			// Encode/Transmit position packet
			packet_t packet = aprs_encode_position_and_telemetry(conf->call,
			                                       conf->path,
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
                              conf->radio_conf.cca)) {
                TRACE_ERROR("POS  > failed to transmit position data");
              }
              chThdSleep(TIME_S2I(5));
            }

            /*
             * Encode/Transmit APRSD packet.
             * This is a tracker originated message (not a reply to a request).
             * The message will be sent to the base station if set.
             * Else send it to device identity.
             */
            char *call = conf_sram.aprs.base.enabled
                ? conf_sram.aprs.base.call : APRS_DEVICE_CALLSIGN;
            /*
             * Send message from this device.
             * Use call sign and path as specified in base config.
             * There is no acknowledgment requested.
             */
            packet = aprs_compose_aprsd_message(
                              conf->call,
                              conf->path,
                              call);
            if(packet == NULL) {
              TRACE_WARN("POS  > No free packet objects "
                  "or badly formed APRSD message");
            } else {
              if(!transmitOnRadio(packet,
                              conf_sram.aprs.base.radio_conf.freq,
                              0,
                              0,
                              conf_sram.aprs.base.radio_conf.pwr,
                              conf_sram.aprs.base.radio_conf.mod,
                              conf_sram.aprs.base.radio_conf.cca
                              )) {
                TRACE_ERROR("POS  > Failed to transmit APRSD data");
              }
              chThdSleep(TIME_S2I(5));
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
	thread_t *th = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(10*1024),
	                                   "POS", LOWPRIO, posThread, conf);
	if(!th) {
		// Print startup error, do not start watchdog for this thread
		TRACE_ERROR("POS  > Could not startup thread (not enough memory available)");
	}
}

