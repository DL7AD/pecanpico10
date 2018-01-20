#ifndef __SLEEP_H__
#define __SLEEP_H__

#include "ch.h"
#include "hal.h"
#include "types.h"

#define WAIT_FOR_TRACKING_POINT		trigger_new_tracking_point
#define TX_CONTINUOSLY				trigger_immediately

bool p_sleep(const sleep_conf_t *config);
interval_t waitForTrigger(interval_t prev, trigger_conf_t *config);
void trigger_new_tracking_point(void);
void trigger_immediately(void);

#endif
