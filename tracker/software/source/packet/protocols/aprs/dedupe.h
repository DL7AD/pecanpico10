#ifndef __DEDUPE_H__
#define __DEDUPE_H__

#include "ch.h"
#include "hal.h"

void dedupe_init(sysinterval_t ttl);
void dedupe_remember(packet_t pp, radio_freq_hz_t freq);
bool dedupe_check(packet_t pp, radio_freq_hz_t freq);

#endif

