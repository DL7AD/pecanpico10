#ifndef __LOG_H__
#define __LOG_H__

#include "tracking.h"

#define LOG_FLASH_ADDR				0x08080000	/* Log flash memory address */
#define LOG_FLASH_SIZE				0x100000	/* Log flash memory size */

void start_logging_thread(thd_log_conf_t *conf);

trackPoint_t* getNewestLogEntry(void);
trackPoint_t* getOldestLogEntry(void);
void writeLogTrackPoint(trackPoint_t* tp);

#endif

