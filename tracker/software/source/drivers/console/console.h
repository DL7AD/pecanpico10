#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include "ch.h"
//#include "hal.h"

typedef enum consoleStates {
  CON_CHN_INIT = 0,
  CON_CHN_IDLE,
  CON_CHN_WAIT,
  CON_CHN_TERM,
  CON_CHN_TRACE,
  CON_CHN_SHELL,
  CON_CHN_FLUSH,
  CON_CHN_CONNECT
} con_chn_state_t;

//#define isUSBactive() (SDU1.config->usbp->state == USB_ACTIVE)

msg_t   pktStartConsole(void);
void    startSDU(void);
bool    isConsoleOutputAvailable(void);

#endif /* __CONSOLE_H__ */

