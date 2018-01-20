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

#include "ch.h"
#include "types.h"
#include "radio.h"

void start_user_modules(void);

extern module_conf_t config[7];

extern sysinterval_t track_cycle_time;
extern sysinterval_t log_cycle_time;
extern bool keep_cam_switched_on;
extern uint16_t gps_on_vbat;
extern uint16_t gps_off_vbat;
extern uint16_t gps_onper_vbat;

#endif

