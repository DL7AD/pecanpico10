#ifndef __PFLASH_H__
#define __PFLASH_H__

#include "collector.h"

#define LOG_FLASH_ADDR			0x08080000	/* Log flash memory address */
#define LOG_FLASH_SIZE			0x100000	/* Log flash memory size */
#define LOG_SECTOR_SIZE			0x20000		/* Single sector size */

#define LOG_POINTS_IN_SECTOR	(LOG_SECTOR_SIZE / sizeof(dataPoint_t))
#define LOG_POS_IN_SECTOR(id)	((id) % LOG_POINTS_IN_SECTOR)
#define LOG_SECTOR_ID(id)		((id) / LOG_POINTS_IN_SECTOR)
#define LOG_RSTandID(tp)		(((uint64_t)(tp)->reset << 32) & (tp)->id)
#define LOG_IS_EMPTY(tp)		((tp)->id == 0xFFFFFFFF && (tp)->reset == 0xFFFF)


dataPoint_t* flash_getLogBuffer(uint16_t id);
dataPoint_t* flash_getNewestLogEntry(void);
dataPoint_t* flash_getOldestLogEntry(void);
void flash_writeLogDataPoint(dataPoint_t* tp);

#endif

