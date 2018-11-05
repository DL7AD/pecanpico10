#ifndef DIGIPEATER_H
#define DIGIPEATER_H 1

#include "regex.h"

#include "ax25_pad.h"		/* for packet_t */


enum preempt_e { PREEMPT_OFF, PREEMPT_DROP, PREEMPT_MARK, PREEMPT_TRACE };

packet_t digipeat_match (radio_freq_hz_t from_freq, packet_t pp,
                         char *mycall_rec, char *mycall_xmit,
                         char *alias, char *wide, radio_freq_hz_t to_freq,
                         enum preempt_e preempt, char *filter_str);

#endif 

