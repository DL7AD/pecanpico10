#ifndef __GEOFENCE_H__
#define __GEOFENCE_H__

#include "ch.h"
#include "hal.h"
#include "types.h"

// APRS region frequencies
#define FREQ_APRS_EUROPE			144800000
#define FREQ_APRS_AMERICA			144390000
#define FREQ_APRS_CHINA				144640000
#define FREQ_APRS_JAPAN				144660000
#define FREQ_APRS_SOUTHKOREA		144620000
#define FREQ_APRS_SOUTHEASTASIA		144390000
#define FREQ_APRS_AUSTRALIA			145175000
#define FREQ_APRS_NEWZEALAND		144575000
#define FREQ_APRS_ARGENTINA			144930000
#define FREQ_APRS_BRAZIL			145575000

#define FREQ_GEOFENCE_DEFAULT       144800000

typedef struct {
	int32_t lat;
	int32_t lon;
} coord_t;

uint32_t getAPRSRegionFrequency(void);

#endif

