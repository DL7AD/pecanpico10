#include "ch.h"
#include "hal.h"

#include "watchdog.h"
#include "pi2c.h"
#include "pac1720.h"
#include "radio.h"
#include "si446x.h"
#include "image.h"
#include "position.h"
#include "log.h"
#include "radio.h"
#include "ax25_pad.h"
#include "flash.h"

sysinterval_t watchdog_tracking;

void start_essential_threads(void)
{
	init_watchdog();				// Init watchdog
	pi2cInit();						// Initialize I2C
	pac1720_init();					// Initialize current measurement
	chThdSleep(TIME_MS2I(300));		// Wait for tracking manager to initialize
}

void start_user_threads(void)
{
	conf_t *conf_flash = (conf_t*)0x08060000;
	/* Check if a user update has been made to configuration in flash. */
	if(conf_flash->magic != CONFIG_MAGIC_UPDATED) {
	    if(conf_flash->magic != CONFIG_MAGIC_DEFAULT
	    || memcmp(&conf_flash, &conf_flash_default, sizeof(conf_t)) != 0) {
        /* No configuration found in flash memory or default config has changed. */
          flashSectorBegin(flashSectorAt((uint32_t)conf_flash));
          flashErase((uint32_t)conf_flash, 0x20000);
          flashWrite((uint32_t)conf_flash, (char*)&conf_flash_default, sizeof(conf_t));
          flashSectorEnd(flashSectorAt((uint32_t)conf_flash));
      }
	}

	// Copy 
	memcpy(&conf_sram, conf_flash, sizeof(conf_t));

	if(conf_sram.pos_pri.thread_conf.active) start_position_thread(&conf_sram.pos_pri);
	if(conf_sram.pos_sec.thread_conf.active) start_position_thread(&conf_sram.pos_sec);

	if(conf_sram.img_pri.thread_conf.active) start_image_thread(&conf_sram.img_pri);
	if(conf_sram.img_sec.thread_conf.active) start_image_thread(&conf_sram.img_sec);

	if(conf_sram.log.thread_conf.active) start_logging_thread(&conf_sram.log);

	if(conf_sram.rx.thread_conf.active)
	  start_rx_thread(conf_sram.rx.radio_conf.freq,
	                  conf_sram.rx.radio_conf.chan,
	                  conf_sram.rssi);
}

