#ifndef __THREADS_H__
#define __THREADS_H__

#include "ch.h"

void start_essential_threads(void);
void start_user_threads(void);
void pktTerminateThread(thread_t *th);
void release_terminated_thread(thread_t *th);

extern sysinterval_t watchdog_tracking; // Last update time for module TRACKING

#endif /* __THREADS_H__ */
