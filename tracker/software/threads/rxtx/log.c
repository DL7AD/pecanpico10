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
#include "flash.h"
#include "sleep.h"
#include "radio.h"
#include "log.h"


#define LOG_POINTS_IN_SECTOR	(0x20000 / sizeof(dataPoint_t))
#define LOG_POS_IN_SECTOR(id)	((id) % LOG_POINTS_IN_SECTOR)
#define LOG_SECTOR_ID(id)		((id) / LOG_POINTS_IN_SECTOR)
#define LOG_RSTandID(tp)		(((uint64_t)(tp)->reset << 32) & (tp)->id)
#define LOG_IS_EMPTY(tp)		((tp)->id == 0xFFFFFFFF && (tp)->reset == 0xFFFF)

static uint16_t log_id = 0;

static dataPoint_t* getLogBuffer(uint16_t id)
{
	uint32_t addr = LOG_FLASH_ADDR + LOG_SECTOR_ID(id) * 0x20000 + LOG_POS_IN_SECTOR(id) * sizeof(dataPoint_t);
	if(addr >= LOG_FLASH_ADDR && addr <= LOG_FLASH_ADDR+LOG_FLASH_SIZE-sizeof(dataPoint_t))
		return (dataPoint_t*)addr;
	else
		return NULL; // Outside of memory address allocation
}

/**
  * Returns next free log entry address in memory. Returns 0 if all cells are
  * filled with data
  */
static dataPoint_t* getNextFreeLogAddress(void)
{
	dataPoint_t* tp;
	for(uint32_t i=0; (tp = getLogBuffer(i)) != NULL; i++)
		if(LOG_IS_EMPTY(tp))
			return tp;

	return NULL;
}

dataPoint_t* getNewestLogEntry(void)
{
	dataPoint_t* last_tp = NULL;
	uint64_t last_id = 0x0;
	dataPoint_t* tp;
	for(uint32_t i=0; (tp = getLogBuffer(i)) != NULL; i++) {
		if(!LOG_IS_EMPTY(tp) && last_id <= LOG_RSTandID(tp)) {
			last_id = LOG_RSTandID(tp);
			last_tp = tp;
		}
	}
	return last_tp;
}

dataPoint_t* getOldestLogEntry(void)
{
	dataPoint_t* first_tp = NULL;
	uint64_t first_id = 0xFFFFFFFFFFFFFFFF;
	dataPoint_t* tp;
	for(uint32_t i=0; (tp = getLogBuffer(i)) != NULL; i++) {
		if(!LOG_IS_EMPTY(tp) && first_id >= LOG_RSTandID(tp)) {
			first_id = LOG_RSTandID(tp);
			first_tp = tp;
		}
	}
	return first_tp;
}

/**
  * Erases oldest data
  */

static void eraseOldestLogData(void)
{
	uint32_t last_tp = (uint32_t)getOldestLogEntry();
	if(last_tp) {
		last_tp = (last_tp / 0x20000) * 0x20000; // Get start address of sector

		TRACE_INFO("LOG  > Erase flash %08x", last_tp);
		flashErase(last_tp, 0x20000);
	}
}

void writeLogDataPoint(dataPoint_t* tp)
{
	// Get address to write on
	dataPoint_t* address = getNextFreeLogAddress();
	if(address == NULL) // Memory completly used, erase oldest data
	{
		eraseOldestLogData();
		address = getNextFreeLogAddress();
	}
	if(address == NULL) // Something went wront at erasing the memory
	{
		TRACE_ERROR("LOG  > Erasing flash failed");
		return;
	}

	// Write data into flash
	TRACE_INFO("LOG  > Flash write (ADDR=%08x)", address);
	flashSectorBegin(flashSectorAt((uint32_t)address));
	flashWrite((uint32_t)address, (char*)tp, sizeof(dataPoint_t));
	flashSectorEnd(flashSectorAt((uint32_t)address));

	// Verify
	if(flashCompare((uint32_t)address, (char*)tp, sizeof(dataPoint_t))) {
		TRACE_INFO("LOG  > Flash write OK");
	} else {
		TRACE_ERROR("LOG  > Flash write failed");
	}
}

static dataPoint_t* getNextLogDataPoint(uint8_t density)
{
	// Determine sector
	dataPoint_t *tp;
	uint32_t i = 0;
	do {
		if((tp = getLogBuffer(log_id))) {
			log_id += density;
		} else {
			log_id = 0;
			tp = getLogBuffer(0);
		}
	} while(LOG_IS_EMPTY(tp) && i++ < LOG_FLASH_SIZE / sizeof(dataPoint_t));

	return LOG_IS_EMPTY(tp) ? NULL : tp;
}

THD_FUNCTION(logThread, arg)
{
	thd_log_conf_t* conf = (thd_log_conf_t*)arg;

	if(conf->thread_conf.init_delay) chThdSleep(conf->thread_conf.init_delay);
	TRACE_INFO("LOG  > Startup logging thread");

	sysinterval_t time = chVTGetSystemTime();
	while(true)
	{
		TRACE_INFO("LOG  > Do module LOG cycle");

		if(!p_sleep(&conf->thread_conf.sleep_conf))
		{
			// Get log from memory
			dataPoint_t *log = getNextLogDataPoint(conf->density);

			if(log) {
				// Encode Base91
				uint8_t pkt_base91[BASE91LEN(sizeof(dataPoint_t))];
				base91_encode((uint8_t*)log, pkt_base91, sizeof(dataPoint_t));

				// Encode and transmit log packet
				packet_t packet = aprs_encode_data_packet(conf->call, conf->path, 'L', pkt_base91); // Encode packet

				// Transmit packet
				transmitOnRadio(packet,
				                conf->radio_conf.freq,
	                            conf->radio_conf.step,
	                            conf->radio_conf.chan,
	                            conf->radio_conf.pwr,
	                            conf->radio_conf.mod);
			} else {
				TRACE_INFO("LOG  > No log point in memory");
			}
		}

		time = waitForTrigger(time, conf->thread_conf.cycle);
	}
}

void start_logging_thread(thd_log_conf_t *conf)
{
	thread_t *th = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(6*1024), "LOG", NORMALPRIO, logThread, conf);
	if(!th) {
		// Print startup error, do not start watchdog for this thread
		TRACE_ERROR("LOG  > Could not startup thread (not enough memory available)");
	}
}

