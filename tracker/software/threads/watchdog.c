#include "ch.h"
#include "hal.h"
#include "debug.h"

// Hardware Watchdog configuration
static const WDGConfig wdgcfg = {
	.pr =	STM32_IWDG_PR_256,
	.rlr =	STM32_IWDG_RL(10000)
};

static void flash_led(void) {
	palSetLine(LINE_IO_GREEN);
	chThdSleep(TIME_MS2I(50));
	palClearLine(LINE_IO_GREEN);
}

THD_FUNCTION(wdgThread, arg) {
	(void)arg;

	// Setup LED
	palSetLineMode(LINE_IO_GREEN, PAL_MODE_OUTPUT_PUSHPULL);

	uint8_t counter = 0;
	while(true)
	{
		chThdSleep(TIME_MS2I(500));

		bool healthy = true;
		// FIXME: Watchdog without functionality at the moment

		/*for(uint8_t i=0; i<threads_cnt; i++) {
			if(registered_threads[i]->wdg_timeout < chVTGetSystemTime())
			{
				TRACE_ERROR("WDG  > Thread %s not healty", registered_threads[i]->name);
				healthy = false; // Threads reached timeout
			}
		}*/

		if(healthy)
			wdgReset(&WDGD1);	// Reset hardware watchdog at no error

		// Switch LEDs
		if(counter++ % (4*healthy) == 0)
		{
			flash_led();
		}
	}
}

void init_watchdog(void)
{
	// Initialize Watchdog
	TRACE_INFO("WDG  > Initialize Watchdog");
	wdgStart(&WDGD1, &wdgcfg);
	wdgReset(&WDGD1);

	flash_led();

	TRACE_INFO("WDG  > Startup Watchdog thread");
	thread_t *th = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(256), "WDG", NORMALPRIO, wdgThread, NULL);
	if(!th) {
		// Print startup error, do not start watchdog for this thread
		TRACE_ERROR("TRAC > Could not startup thread (not enough memory available)");
	}
}

