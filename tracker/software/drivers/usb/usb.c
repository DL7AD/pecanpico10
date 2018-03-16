#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "shell.h"
#include "commands.h"

static thread_t *shelltp;
static bool usb_initialized;

static const ShellConfig shell_cfg = {
	(BaseSequentialStream*)&SDU1,
	commands
};

void startUSB(void) {
	if(usb_initialized)
		return; // Avoid duplicate initialization

	// Start USB
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);

	usbDisconnectBus(serusbcfg.usbp);
	chThdSleep(TIME_MS2I(100));
	usbStart(serusbcfg.usbp, &usbcfg);
	usbConnectBus(serusbcfg.usbp);
	usb_initialized = true;

	// Initialize shell
	shelltp = NULL;
	event_listener_t shell_el;
	shellInit();
	chEvtRegister(&shell_terminated, &shell_el, 0);

	usb_initialized = true;
}

void startShell(void) {
	if(shelltp == NULL) {
		shelltp = chThdCreateFromHeap(NULL,
		                              THD_WORKING_AREA_SIZE(1024),
		                              "shell", NORMALPRIO + 1,
		                              shellThread,
		                              (void*)&shell_cfg);
	}
	if(chThdTerminatedX(shelltp)) {
		chThdRelease(shelltp);
		shelltp = NULL;
	}
}

bool isUSBInitialized(void) {
	return usb_initialized;
}
