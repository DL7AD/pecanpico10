#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "shell.h"
#include "commands.h"
#include "pktconf.h"

static thread_t *shelltp;
static bool usb_initialized;

event_listener_t shell_el;

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
	shellInit();

	usb_initialized = true;
}

void manageShell(void) {
	if(shelltp == NULL) {
		shelltp = chThdCreateFromHeap(NULL,
		                              THD_WORKING_AREA_SIZE(1024),
		                              "shell", NORMALPRIO + 1,
		                              shellThread,
		                              (void*)&shell_cfg);

	    chEvtRegister(&shell_terminated, &shell_el, USB_SHELL_EVT);
	}
    chEvtWaitAnyTimeout(EVENT_MASK(USB_SHELL_EVT), TIME_S2I(1));
	if(chThdTerminatedX(shelltp)) {
		chThdRelease(shelltp);
		shelltp = NULL;
	    chEvtUnregister(&shell_terminated, &shell_el);
	}
}

bool isUSBInitialized(void) {
	return usb_initialized;
}
