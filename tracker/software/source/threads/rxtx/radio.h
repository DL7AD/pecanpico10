#ifndef __RADIO_H__
#define __RADIO_H__

#include "ch.h"
#include "hal.h"
#include "config.h"
#include "ax25_pad.h"
#include "pktconf.h"

#define PKT_APRS_MAIN_WA_SIZE       1024

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

#define PKT_DUMP_BAD_PACKETS        TRUE

thread_t *pktStartAPRSthreads(thd_aprs_conf_t *conf, const char *name);
bool pktTransmitOnRadio(packet_t pp,
                        const radio_freq_hz_t freq,
                        const radio_chan_hz_t step,
                        const radio_ch_t chan,
                        const radio_pwr_t pwr,
                        const radio_mod_t mod,
                        const radio_squelch_t rssi);
bool pktTransmitOnRadioWithCallback(packet_t pp,
                                    const radio_freq_hz_t base_freq,
                                    const radio_chan_hz_t step,
                                    const radio_ch_t chan,
                                    const radio_pwr_t pwr,
                                    const radio_mod_t mod,
                                    const radio_squelch_t cca,
                                    const radio_task_cb_t cb);

inline const char *getModulation(radio_mod_t key) {
    const char *val[] = {"NONE", "CW", "AFSK", "2FSK 300", "2FSK 9k6", "2FSK 19k2",
                         "2FSK 38k4", "2FSK 57k6", "2FSK 76k8", "2FSK 96k",
                         "2FSK 115k2"};
    return val[key];
};

#endif /* __RADIO_H__ */

