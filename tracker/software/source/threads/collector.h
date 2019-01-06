#ifndef __COLLECTOR_H__
#define __COLLECTOR_H__

#include "ch.h"
#include "hal.h"
#include "ptime.h"
#include "types.h"
#include "portab.h"

#define BME_STATUS_BITS         2
#define BME_STATUS_MASK         0x3
#define BME_OK_VALUE            0x0
#define BME_FAIL_VALUE          0x1
#define BME_NOT_FITTED_VALUE    0x2

#define BME_ALL_STATUS_MASK     0x3F
#define BME_ALL_STATUS_SHIFT    8

#define BMEI1_STATUS_SHIFT      BME_ALL_STATUS_SHIFT
#define BMEI1_STATUS_MASK       (BME_STATUS_MASK << BMEI1_STATUS_SHIFT)

#define BMEE1_STATUS_SHIFT      BMEI1_STATUS_SHIFT + BME_STATUS_BITS
#define BMEE1_STATUS_MASK       (BME_STATUS_MASK << BMEE1_STATUS_SHIFT)

#define BMEE2_STATUS_SHIFT      BMEI1_STATUS_SHIFT + BME_STATUS)BITS
#define BMEE2_STATUS_MASK       (BME_STATUS_MASK << BMEE2_STATUS_SHIFT)

//#define BME280_E1_IS_FITTED     FALSE
//#define BME280_E2_IS_FITTED     TRUE

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   GPS states as array of strings.
 * @details Each element in an array initialized with this macro can be
 *          indexed using a numeric GPS model value.
 */
#define GPS_STATE_NAMES                                                     \
  "LOCKED1", "LOCKED2", "LOSS", "LOWBATT1", "LOWBATT2", "LOG", "OFF",       \
  "ERROR", "FIXED", "TIME"

typedef enum {
	GPS_LOCKED1,	// The GPS was locked, the GPS has been switched off
	GPS_LOCKED2,	// The GPS is locked, the GPS has been kept switched on
	GPS_LOSS,		// The GPS was switched on but did not acquire a fix
	GPS_LOWBATT1,	// The GPS wasn't switched on because the battery is low
	GPS_LOWBATT2,	// The GPS was switched on but switched off due low battery
	GPS_LOG,		// The position has been taken from the log (switch on)
	GPS_OFF,		// There was no prior acquisition by GPS
	GPS_ERROR,		// The GPS has a communication error
    GPS_FIXED,      // Fixed location data used from configuration location
    GPS_TIME        // Time stamp of RTC on first getting GPS time
} gpsState_t;

#define GPS_STATE_MAX   GPS_TIME

typedef struct {
	// Voltage and current measurement
	uint16_t adc_vsol;		// Current solar voltage in mV
	uint16_t adc_vbat;		// Current battery voltage in mV
	uint16_t pac_vsol;
	uint16_t pac_vbat;
	int16_t pac_pbat;
	int16_t pac_psol;

	uint16_t light_intensity;

	// GPS
	gpsState_t gps_state;	// GPS state
	uint8_t gps_sats;		// Satellites used for solution
	uint8_t gps_ttff;		// Time to first fix in seconds
	uint8_t gps_pdop;		// Position DOP in 0.05 per arbitrary unit
	uint16_t gps_alt;		// Altitude in meter
	int32_t gps_lat;		// Latitude in 10^(-7)� per unit
	int32_t gps_lon;		// Longitude in 10^(-7)� per unit

	// BME280 (on board i1 + off board e1 & e2)
	uint32_t sen_i1_press;		// Airpressure in Pa*10 (in 0.1Pa)
	uint32_t sen_e1_press;		// Airpressure in Pa*10 (in 0.1Pa)
	uint32_t sen_e2_press;		// Airpressure in Pa*10 (in 0.1Pa)

	int16_t sen_i1_temp;		// Temperature in 0.01�C per unit
	int16_t sen_e1_temp;		// Temperature in 0.01�C per unit
	int16_t sen_e2_temp;		// Temperature in 0.01�C per unit

	uint8_t sen_i1_hum;			// Rel. humidity in %
	uint8_t sen_e1_hum;			// Rel. humidity in %
	uint8_t sen_e2_hum;			// Rel. humidity in %

	uint8_t dummy2;

	int16_t stm32_temp;
	int16_t si446x_temp;

	uint16_t reset;
	uint32_t id;			// Serial ID
	uint32_t gps_time;		// GPS time

	uint32_t sys_time;		// System time (in seconds)
	uint32_t sys_error;			// System error flags
                      /*
                       * Set system errors.
                       *
                       * Bit usage:
                       * -  0:1  I2C status
                       * -  2:2  GPS status
                       * -  3:4  pac1720 status
                       * -  5:7  OV5640 status
                       * -  8:9  BMEi1 status (0 = OK, 1 = Fail, 2 = Not fitted)
                       * - 10:11 BMEe1 status (0 = OK, 1 = Fail, 2 = Not fitted)
                       * - 12:13 BMEe2 status (0 = OK, 1 = Fail, 2 = Not fitted)
                       */

    uint8_t  gpio;    // GPIO states
} dataPoint_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern thread_t *collector_thd;

#ifdef __cplusplus
extern "C" {
#endif
dataPoint_t*    getLastDataPoint(void);
void            getSensors(dataPoint_t* tp);
void            setSystemStatus(dataPoint_t* tp);
void            init_data_collector(void);
const char*     get_gps_state_name(uint8_t index);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Has GPS achieved lock (even if now switched off).
 *
 * @param[in] pointer to data point
 *
 * @returns result of check
 * @retval  true if lock has been achieved
 * @retval  false if lock has not yet been achieved
 *
 * @api
 */
#define hasGPSacquiredLock(dp) (dp->gps_state == GPS_LOCKED1                 \
                                || dp->gps_state == GPS_LOCKED2)

/**
 * @brief   Is position valid.
 *
 * @param[in] pointer to data point
 *
 * @returns result of check
 * @retval  true if position data is from satellite or fixed.
 * @retval  false if position not valid.
 *
 * @api
 */
#define isPositionValid(dp)   (dp->gps_state == GPS_LOCKED1                  \
                              || dp->gps_state == GPS_LOCKED2                \
                              || dp->gps_state == GPS_FIXED                  \
                              || dp->gps_state == GPS_LOG)

/**
 * @brief   Is position current.
 *
 * @param[in] pointer to data point
 *
 * @returns result of check
 * @retval  true if position data is from satellite or fixed.
 * @retval  false if position not valid.
 *
 * @api
 */
#define isPositionCurrent(dp)   (dp->gps_state == GPS_LOCKED1                \
                              || dp->gps_state == GPS_LOCKED2                \
                              || dp->gps_state == GPS_FIXED)

/**
 * @brief   Is position from a satellite.
 * @notes   This is an alias for hasGPSacquiredLock
 *
 * @param[in] pointer to data point
 *
 * @returns result of check
 * @retval  true if position data is from satellite
 * @retval  false if position not satellite
 *
 * @api
 */
#define isPositionFromSV(dp) hasGPSacquiredLock(dp)

/**
 * @brief   Is GPS able to operate on battery.
 *
 * @param[in] pointer to data point
 *
 * @returns result of check
 * @retval  true if battery operable
 * @retval  false if not battery operable
 *
 * @api
 */
#define isGPSbatteryOperable(dp) (dp->gps_state == GPS_LOCKED1               \
                                 || dp->gps_state == GPS_LOCKED2             \
                                 || dp->gps_state == GPS_LOSS)

#endif /* __COLLECTOR_H__ */

