#include "flash.h"
#include "pflash.h"
#include "debug.h"

dataPoint_t* flash_getLogBuffer(uint16_t id)
{
	uint32_t addr = LOG_FLASH_ADDR
	    + (LOG_SECTOR_ID(id) * LOG_SECTOR_SIZE)
	    + (LOG_POS_IN_SECTOR(id) * sizeof(dataPoint_t));
	if(addr >= LOG_FLASH_ADDR
	    && addr <= (LOG_FLASH_ADDR + LOG_FLASH_SIZE - sizeof(dataPoint_t)))
		return (dataPoint_t*)addr;
	else
		return NULL; // Outside of memory address allocation
}

/**
  * Returns next free log entry address in memory.
  * Returns NULL if all entries are used.
  */
static dataPoint_t* flash_getNextFreeLogAddress(void) {
  dataPoint_t* tp;
  for(uint32_t i = 0; (tp = flash_getLogBuffer(i)) != NULL; i++) {
    if(LOG_IS_EMPTY(tp))
      return tp;
  }
  return NULL;
}

/*
 *
 */
dataPoint_t* flash_getNewestLogEntry(void) {
  dataPoint_t* last_tp = NULL;
  uint64_t last_id = 0x0;
  dataPoint_t* tp;
  for(uint32_t i=0; (tp = flash_getLogBuffer(i)) != NULL; i++) {
    if(!LOG_IS_EMPTY(tp) && last_id <= LOG_RSTandID(tp)) {
      last_id = LOG_RSTandID(tp);
      last_tp = tp;
    } else {
      break;
    }
  }
  return last_tp;
}

/*
 * Alternative version of flash_getNewestLogEntry(...)
 * All that really needs to be found is the next EMPTY entry?
 */
/*
dataPoint_t* flash_getNewestLogEntry(void) {
  dataPoint_t* tp;
  uint32_t i = 0;
  while((tp = flash_getLogBuffer(i++)) != NULL) {
    if(LOG_IS_EMPTY(tp))
      break;
  }
  return (i > 1 && tp != NULL ? flash_getLogBuffer(i - 1) : NULL);
}
*/

/*
 *
 */
dataPoint_t* flash_getOldestLogEntry(void) {
  dataPoint_t* first_tp = NULL;
  uint64_t first_id = 0xFFFFFFFFFFFFFFFF;
  dataPoint_t* tp;
  for(uint32_t i=0; (tp = flash_getLogBuffer(i)) != NULL; i++) {
    if(!LOG_IS_EMPTY(tp) && first_id >= LOG_RSTandID(tp)) {
      first_id = LOG_RSTandID(tp);
      first_tp = tp;
    } else {
      break;
    }
  }
  return first_tp;
}

/**
  * Erases oldest data
  */

static void flash_eraseOldestLogData(void)
{
	uint32_t last_tp = (uint32_t)flash_getOldestLogEntry();
	if(last_tp) {
		last_tp = (last_tp / LOG_SECTOR_SIZE) * LOG_SECTOR_SIZE; // Get start address of sector

		TRACE_DEBUG("LOG  > Erase flash %08x", last_tp);
		flashErase(last_tp, LOG_SECTOR_SIZE);
	}
}

void flash_writeLogDataPoint(dataPoint_t* tp)
{
	// Get address to write on
	dataPoint_t* address = flash_getNextFreeLogAddress();
	if(address == NULL) // Memory completely used, erase oldest data
	{
		flash_eraseOldestLogData();
		address = flash_getNextFreeLogAddress();
	}
	if(address == NULL) // Something went wrong at erasing the memory
	{
		TRACE_ERROR("LOG  > Erasing flash failed");
		return;
	}

	// Write data into flash
	TRACE_DEBUG("LOG  > Flash write (ADDR=%08x)", address);
	flashSectorBegin(flashSectorAt((uint32_t)address));
	flashWrite((uint32_t)address, (char*)tp, sizeof(dataPoint_t));
	flashSectorNext(flashSectorAt((uint32_t)address));

	// Verify
	if(flashCompare((uint32_t)address, (char*)tp, sizeof(dataPoint_t))) {
	  TRACE_DEBUG("LOG  > Flash write OK");
	} else {
		TRACE_ERROR("LOG  > Flash write failed");
	}
}

