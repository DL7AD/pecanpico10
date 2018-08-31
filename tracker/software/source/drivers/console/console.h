#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include "ch.h"
//#include "hal.h"

typedef enum consoleStates {
  CON_CHN_INIT = 0,
  CON_CHN_READY,
  CON_CHN_IDLE,
  CON_CHN_OUT,
  CON_CHN_SHELL,
  CON_CHN_EXIT
} con_chn_state_t;

#define isUSBactive() (SDU1.config->usbp->state == USB_ACTIVE)

msg_t   pktStartConsole(void);
void    startSDU(void);
void    manageTraceAndShell(void);
bool    isConsoleOutputAvailable(void);

#endif /* __CONSOLE_H__ */

