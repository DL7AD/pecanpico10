#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include "ch.h"
//#include "hal.h"

#define  CON_RESUME_SHELL   FALSE
#define  CON_DEBUG_TRACE    FALSE

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

/* State limit. */
#define CONSOLE_STATE_MAX           CON_CHN_CONNECT

/**
 * @brief   Console states as array of strings.
 * @details Each element in an array initialized with this macro can be
 *          indexed using a numeric console state value.
 */
#define CONSOLE_STATE_NAMES                                                 \
  "INIT", "IDLE", "WAIT", "TERM", "TRACE", "SHELL", "FLUSH", "CONNECT"

msg_t   pktStartConsole(void);
void    startSDU(void);
bool    isConsoleOutputAvailable(void);

#endif /* __CONSOLE_H__ */

