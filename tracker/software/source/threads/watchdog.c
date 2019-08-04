#include "ch.h"
#include "hal.h"
#include "debug.h"
#include "portab.h"
#include "pktconf.h"

#if DISABLE_HW_WATCHDOG != TRUE
// Hardware Watchdog configuration
static const WDGConfig wdgcfg = {
#if STM32_IWDG_IS_WINDOWED
    .winr = STM32_IWDG_WIN_DISABLED,
#endif
	.pr =	STM32_IWDG_PR_256,
	.rlr =	STM32_IWDG_RL(10000)
};
#endif

static void flash_led(void) {
  pktWriteGPIOline(LINE_IO_GREEN, PAL_HIGH);
  chThdSleep(TIME_MS2I(50));
  pktWriteGPIOline(LINE_IO_GREEN, PAL_LOW);
  if (I2C_hasError()) {
    /* Double blink if I2C device has an error (PAC1720 primarily). */
    chThdSleep(TIME_MS2I(100));
    pktWriteGPIOline(LINE_IO_GREEN, PAL_HIGH);
    chThdSleep(TIME_MS2I(50));
    pktWriteGPIOline(LINE_IO_GREEN, PAL_LOW);
  }
}

THD_FUNCTION(wdgThread, arg) {
	(void)arg;

	// Setup LED
    pktSetGPIOlineMode(LINE_IO_GREEN, PAL_MODE_OUTPUT_PUSHPULL);

	uint8_t counter = 0;
	while(true) {
		chThdSleep(TIME_MS2I(500));

		bool healthy = true;
		// FIXME: Watchdog without functionality at the moment

		/*for(uint8_t i=0; i<threads_cnt; i++) {
			if(registered_threads[i]->wdg_timeout < chVTGetSystemTime())
			{
				TRACE_ERROR("WDG  > Thread %s not healthy", registered_threads[i]->name);
				healthy = false; // Threads reached timeout
			}
		}*/

		if(healthy)
#if DISABLE_HW_WATCHDOG != TRUE
			wdgReset(&WDGD1);	// Reset hardware watchdog at no error
#endif
		// Switch LEDs
		if(counter++ % (4*healthy) == 0)
		{
			flash_led();
		}
	}
}

/**
 *
 */
void init_watchdog(void)
{
#if DISABLE_HW_WATCHDOG != TRUE
	// Initialize Watchdog
	TRACE_INFO("WDG  > Initialize hardware watchdog");
	wdgStart(&WDGD1, &wdgcfg);
	wdgReset(&WDGD1);
#else
#warning "Hardware Watchdog is disabled"
    TRACE_INFO("WDG  > Hardware watchdog disabled");
#endif
	flash_led();

	TRACE_INFO("WDG  > Startup software watchdog thread");
	thread_t *th = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(256), "WDG", NORMALPRIO, wdgThread, NULL);
	if(!th) {
		// Print startup error, do not start watchdog for this thread
		TRACE_ERROR("WDG  > Could not startup thread (not enough memory available)");
	}
}

