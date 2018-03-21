#ifndef __LOG_H__
#define __LOG_H__

#include "collector.h"

#define LOG_FLASH_ADDR				0x08080000	/* Log flash memory address */
#define LOG_FLASH_SIZE				0x100000	/* Log flash memory size */

void start_logging_thread(thd_log_conf_t *conf);

dataPoint_t* getNewestLogEntry(void);
dataPoint_t* getOldestLogEntry(void);
void writeLogDataPoint(dataPoint_t* tp);

#endif

