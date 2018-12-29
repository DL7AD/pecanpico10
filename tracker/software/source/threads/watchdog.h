#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#if !defined(DISABLE_HW_WATCHDOG)
#define DISABLE_HW_WATCHDOG FALSE
#endif

void init_watchdog(void);

#endif

