#ifndef __CONFIG_H__
#define __CONFIG_H__

#define LOG_FLASH_ADDR1				0x080C0000	/* Log flash memory address 1 */
#define LOG_FLASH_ADDR2				0x080E0000	/* Log flash memory address 2 */
#define LOG_SECTOR_SIZE				0x20000		/* Log flash memory size */

#define TRACE_TIME					TRUE		/* Enables time tracing on debugging port */
#define TRACE_FILE					TRUE		/* Enables file and line tracing on debugging port */

#define ACTIVATE_USB				TRUE		/* If set to true, the USB interface will be switched on. The tracker is also switched to
												 * 3V, because USB would not work at 1.8V. Note that the transmission power is increased
												 * too when operating at 3V. This option will also run the STM32 at 48MHz (AHB) permanently
												 * because USB needs that speed, otherwise it is running at 6MHz which saves a lot of power. */

#include "types.h"

extern conf_t config;

#endif

