#include "ch.h"
#include "hal.h"

#include "tracking.h"
#include "watchdog.h"
#include "pi2c.h"
#include "pac1720.h"
#include "si446x.h"

sysinterval_t watchdog_tracking;

void start_essential_threads(void)
{
	init_watchdog();				// Init watchdog
	pi2cInit();						// Initialize I2C
	pac1720_init();					// Initialize current measurement
	init_tracking_manager(false);	// Initialize tracking manager (without GPS, GPS is initialized if needed by position thread)
	chThdSleep(TIME_MS2I(300));		// Wait for tracking manager to initialize
}

void start_user_threads(void)
{
	if(config.pos_pri.thread_conf.active) start_position_thread(&config.pos_pri);
	if(config.pos_sec.thread_conf.active) start_position_thread(&config.pos_sec);

	if(config.img_pri.thread_conf.active) start_image_thread(&config.img_pri);
	if(config.img_sec.thread_conf.active) start_image_thread(&config.img_sec);

	if(config.log.thread_conf.active) start_logging_thread(&config.log);

	if(config.rx.thread_conf.active) start_rx_thread(config.rx.radio_conf.freq, config.rssi);
}

