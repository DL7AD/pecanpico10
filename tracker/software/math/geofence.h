#ifndef __GEOFENCE_H__
#define __GEOFENCE_H__

#include "ch.h"
#include "hal.h"
#include "types.h"

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

typedef struct {
	int32_t lat;
	int32_t lon;
} coord_t;

uint32_t getAPRSRegionFrequency(void);
uint32_t getFrequency(freq_conf_t *config);

#endif

