#ifndef __USB_H__
#define __USB_H__

#include "ch.h"
#include "hal.h"

#define isUSBactive() (SDU1.config->usbp->state == USB_ACTIVE)

void startUSB(void);
void manageShell(void);
bool isUSBInitialized(void);

#endif

