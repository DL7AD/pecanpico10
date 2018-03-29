#ifndef __RADIO_H__
#define __RADIO_H__

#include "ch.h"
#include "hal.h"
#include "config.h"
#include "ax25_pad.h"
#include "pktconf.h"

// APRS region frequencies
#define APRS_FREQ_OTHER				144800000
#define APRS_FREQ_AMERICA			144390000
#define APRS_FREQ_CHINA				144640000
#define APRS_FREQ_JAPAN				144660000
#define APRS_FREQ_SOUTHKOREA		144620000
#define APRS_FREQ_SOUTHEASTASIA		144390000
#define APRS_FREQ_AUSTRALIA			145175000
#define APRS_FREQ_NEWZEALAND		144575000
#define APRS_FREQ_ARGENTINA			144930000
#define APRS_FREQ_BRAZIL			145575000

void start_rx_thread(radio_unit_t radio, radio_freq_t freq, channel_hz_t step,
                     radio_ch_t chan, radio_squelch_t rssi);
bool transmitOnRadio(packet_t pp, radio_freq_t freq, channel_hz_t step,
                     radio_ch_t chan, radio_pwr_t pwr, mod_t mod);

inline const char *getModulation(uint8_t key) {
    const char *val[] = {"NONE", "AFSK", "2FSK"};
    return val[key];
};

#endif /* __RADIO_H__ */

