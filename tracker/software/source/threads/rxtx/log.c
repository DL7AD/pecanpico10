/**
  * Logging module
  * 
  */

#include "ch.h"
#include "hal.h"

#include "debug.h"
#include "threads.h"
#include "base91.h"
#include "aprs.h"
#include "sleep.h"
#include "radio.h"
#include "log.h"
#include "pflash.h"

static uint16_t log_id = 0;

static dataPoint_t* getNextLogDataPoint(uint8_t density)
{
	// Determine sector
	dataPoint_t *tp;
	uint32_t i = 0;
	do {
		if((tp = flash_getLogBuffer(log_id))) {
			log_id += density;
		} else {
			log_id = 0;
			tp = flash_getLogBuffer(0);
		}
	} while(LOG_IS_EMPTY(tp) && i++ < LOG_FLASH_SIZE / sizeof(dataPoint_t));

	return LOG_IS_EMPTY(tp) ? NULL : tp;
}

THD_FUNCTION(logThread, arg)
{
	log_app_conf_t* conf = (log_app_conf_t*)arg;

	if(conf->svc_conf.init_delay) chThdSleep(conf->svc_conf.init_delay);
	TRACE_INFO("LOG  > Startup logging thread");

	systime_t time = chVTGetSystemTime();
	while(true)
	{
		TRACE_INFO("LOG  > Do module LOG cycle");

		if(!p_sleep(&conf->svc_conf.sleep_conf))
		{
			// Get log from memory
			dataPoint_t *log = getNextLogDataPoint(conf->density);

			if(log) {
				// Encode Base91
				uint8_t pkt_base91[BASE91LEN(sizeof(dataPoint_t))];
				base91_encode((uint8_t*)log, pkt_base91, sizeof(dataPoint_t));
				// Encode and transmit log packet
				packet_t packet = aprs_encode_data_packet(conf->call, conf->path, 'L', pkt_base91); // Encode packet
	            if(packet == NULL) {
	              TRACE_WARN("LOG  > No free packet objects for log transmission");
	            } else {
				// Transmit packet
                  pktTransmitOnRadio(packet,
                                  conf->radio_conf.freq,
                                  0,
                                  0,
                                  conf->radio_conf.pwr,
                                  conf->radio_conf.mod,
                                  conf->radio_conf.cca);
	            }
			} else {
				TRACE_INFO("LOG  > No log point in memory");
			}
		}

		time = waitForTrigger(time, conf->svc_conf.cycle);
	}
}

void start_logging_thread(log_app_conf_t *conf)
{
	thread_t *th = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(6*1024), "LOG", LOWPRIO, logThread, conf);
	if(!th) {
		// Print startup error, do not start watchdog for this thread
		TRACE_ERROR("LOG  > Could not startup thread (not enough memory available)");
	}
}

