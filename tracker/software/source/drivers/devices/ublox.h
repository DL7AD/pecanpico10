/**
  * @see https://github.com/thasti/utrak
  */

#ifndef __MAX_H__
#define __MAX_H__

#include "ch.h"
#include "hal.h"
#include "ptime.h"


/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

#define GPS_USE_SERIAL_MUX      FALSE
#define GPS_USE_UBX_ATOMIC      TRUE
#define UBLOX_ALLOW_POWER_SAVE  FALSE

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define UBX_COMMAND_ACK         1
#define UBX_COMMAND_NAK         0
/**
 * @brief   GPS model values.
 */
#define GPS_MODEL_PORTABLE      0
#define GPS_MODEL_STATIONARY    2
#define GPS_MODEL_PEDESTRIAN    3
#define GPS_MODEL_AUTOMOTIVE    4
#define GPS_MODEL_SEA           5
#define GPS_MODEL_AIRBORNE1G    6
#define GPS_MODEL_AIRBORNE2G    7
#define GPS_MODEL_AIRBORNE4G    8

/* Model limits. */
#define GPS_MODEL_MAX           GPS_MODEL_AIRBORNE4G

/**
 * @brief   GPS models as array of strings.
 * @details Each element in an array initialized with this macro can be
 *          indexed using a numeric GPS model value.
 */
#define GPS_MODEL_NAMES                                                     \
  "PORTABLE", "NONE", "STATIONARY", "PEDESTRIAN", "AUTOMOTIVE", "SEA",      \
  "AIRBORNE1G",  "AIRBORNE2G", "AIRBORNE4G"

typedef enum {
  GPS_PORTABLE      = GPS_MODEL_PORTABLE,
  GPS_STATIONARY    = GPS_MODEL_STATIONARY,
  GPS_PEDESTRIAN    = GPS_MODEL_PEDESTRIAN,
  GPS_AUTOMOTIVE    = GPS_MODEL_AUTOMOTIVE,
  GPS_SEA           = GPS_MODEL_SEA
} gps_hp_model_t;

typedef enum {
  GPS_AIRBORNE_1G   = GPS_MODEL_AIRBORNE1G,
  GPS_AIRBORNE_2G   = GPS_MODEL_AIRBORNE2G,
  GPS_AIRBORNE_4G   = GPS_MODEL_AIRBORNE4G
} gps_lp_model_t;

#define GPS_MAX_SV_CHANNELS     30

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#define isGPSLocked(pos) ((pos)->type == 3 && (pos)->num_svs >= 4 && (pos)->fixOK == true)

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/* UBLOX reply messages. */
typedef struct {
  uint8_t       chn;            // Channel number
  uint8_t       svid;           // Satellite ID
  uint8_t       flags;          // Flags.. svUsed, diffCorr, orbitAvail, orbitEph, unhealthy, orbitAlm, orbitAop, smoothed
  uint8_t       quality;        // Field.. Idle, searching, acquired, unusable, code lock, code & carrier lock
  uint8_t       cno;            // Carrier to noise ratio (signal strength) in dBHz
  int8_t        elev;           // Elevation in integer degrees
  int16_t       azim;           // Azimuth in integer degrees
  int32_t       prRes;          // Pseudo range residual in centimetres
} __attribute__((packed)) gps_svchn_t;

typedef struct {
	uint32_t    iTOW;		    // Time ms
	uint8_t     numCh;   	    // number of satellites in info
	uint8_t     globalFlags;    // chip hardware generation
	uint16_t    reserved2;      // reserved
	gps_svchn_t svinfo[GPS_MAX_SV_CHANNELS];
} __attribute__((packed)) gps_svinfo_t;

typedef struct {
    uint32_t    iTOW;           // Time ms
    uint8_t     gpsFix;         // gps fix type
    uint8_t     flags;          // gpsFixOk is bit 0
    uint8_t     fixStat;        // fix status information
    uint8_t     flags2;         // further navigation information
    uint32_t    ttff;           // time to first fix
    uint32_t    msss;           // milliseconds since startup
} __attribute__((packed)) gps_navinfo_t;

typedef uint8_t tpidx_t;

typedef struct {
    uint8_t     tpIdx;              // Timepulse selection
    uint8_t     reserved0;          // reserved
    uint16_t    reserved1;          // reserved
    int16_t     antCableDelay;      // Antenna cable delay (nS)
    int16_t     rfGroupDelay;       // RF group delay (nS)
    uint32_t    freqPeriod;         // Frequency or period time
    uint32_t    freqPeriodLock;     // Frequency or period when locked
    uint32_t    pulseLenRatio;      // Pulse length or duty cycle
    uint32_t    pulseLenRatioLock;  // Pulse length or duty cycle when locked
    int32_t     userConfigDelay;    // User configurable time pulse delay
    uint32_t    flags;              // Configuration flags
} __attribute__((packed)) gps_tp5_t;

/* Combination object (not a UBLOX reply). */
typedef struct {
    ptime_t     time;       // Time
    uint8_t     type;       // type of fix (validity)
    uint8_t     num_svs;    // number of satellites used for solution, range 0 .. 19
    int32_t     lat;        // latitude in deg * 10^7, range -90 .. +90 * 10^7
    int32_t     lon;        // longitude in deg * 10^7, range -180 .. +180 * 10^7
    int32_t     alt;        // altitude in m, range 0m, up to 50000m, clamped
    bool        fixOK;      // Flag that is set to true, when DOP is with the limits
    uint16_t    pdop;       // Position DOP
    uint8_t     model;      // Dynamic model
} gpsFix_t;


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  uint8_t gps_disable_nmea_output(void);
  bool    gps_set_model(bool dynamic);
  uint8_t gps_set_stationary_model(void);
  uint8_t gps_set_portable_model(void);
  uint8_t gps_set_airborne_model(void);
  uint8_t gps_set_power_options(void);
  uint8_t gps_switch_power_save_mode(bool on);
  bool    gps_get_fix(gpsFix_t *fix);
  bool    gps_get_sv_info(gps_svinfo_t *svinfo, size_t size);
  bool    gps_get_timepulse_info(tpidx_t tp, gps_tp5_t *tp5, size_t size);
  bool    gps_get_nav_status(gps_navinfo_t *navinfo, size_t size);
  bool    GPS_Init(void);
  void    GPS_Deinit(void);
  const char *gps_get_model_name(uint8_t index);
#ifdef __cplusplus
}
#endif

#endif /* __MAX_H__ */

