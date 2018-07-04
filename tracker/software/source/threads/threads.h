#ifndef __THREADS_H__
#define __THREADS_H__

#include "ch.h"

void start_essential_threads(void);
void start_user_threads(void);
void pktThdTerminateSelf(void);
void pktIdleThread(void);

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

extern sysinterval_t watchdog_tracking; // Last update time for module TRACKING

#endif /* __THREADS_H__ */
