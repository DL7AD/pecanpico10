#ifndef __USB_H__
#define __USB_H__

#include "ch.h"
#include "hal.h"

typedef enum sduTermStates {
  TERM_SDU_INIT = 0,
  TERM_SDU_START,
  TERM_SDU_IDLE,
  TERM_SDU_OUT,
  TERM_SDU_SHELL,
  TERM_SDU_EXIT
} sdu_term_t;

#define isUSBactive() (SDU1.config->usbp->state == USB_ACTIVE)

void startUSB(void);
void startSDU(void);
void manageTraceAndShell(void);
bool isSDUAvailable(void);

#endif

