#include "ch.h"
#include "hal.h"

#include "tracking.h"
#include "debug.h"
#include "config.h"
#include "ublox.h"
#include "bme280.h"
#include "padc.h"
#include "pac1720.h"
#include "ov5640.h"
#include "radio.h"
#include "flash.h"
#include "watchdog.h"
#include "pi2c.h"
#include "si446x.h"
#include "log.h"

static trackPoint_t trackPoints[2];
static trackPoint_t* lastTrackPoint;
static bool threadStarted = false;
static bool tracking_useGPS = false;

/**
  * Returns most recent track point witch is complete.
  */
trackPoint_t* getLastTrackPoint(void)
{
	return lastTrackPoint;
}

void waitForNewTrackPoint(void)
{
	uint32_t old_id = getLastTrackPoint()->id;
	while(old_id == getLastTrackPoint()->id)
		chThdSleep(TIME_S2I(1));
}


static void aquirePosition(trackPoint_t* tp, trackPoint_t* ltp, sysinterval_t timeout)
{
	sysinterval_t start = chVTGetSystemTime();

	gpsFix_t gpsFix;
	memset(&gpsFix, 0, sizeof(gpsFix_t));

	// Switch on GPS if enough power is available and GPS is needed by any position thread
	uint16_t batt = stm32_get_vbat();
	if(!tracking_useGPS) { // No position thread running
		tp->gps_lock = GPS_OFF;
	} else if(batt < conf_sram.gps_on_vbat) {
		tp->gps_lock = GPS_LOWBATT1;
	} else {

		// Switch on GPS
		bool status = GPS_Init();

		if(status) {
			// Search for lock as long enough power is available
			do {
				batt = stm32_get_vbat();
				gps_get_fix(&gpsFix);
			} while(!isGPSLocked(&gpsFix) && batt >= conf_sram.gps_off_vbat && chVTGetSystemTime() <= start + timeout); // Do as long no GPS lock and within timeout, timeout=cycle-1sec (-3sec in order to keep synchronization)

			if(batt < conf_sram.gps_off_vbat) { // GPS was switched on but prematurely switched off because the battery is low on power, switch off GPS

				GPS_Deinit();
				TRACE_WARN("TRAC > GPS sampling finished GPS LOW BATT");
				tp->gps_lock = GPS_LOWBATT2;

			} else if(!isGPSLocked(&gpsFix)) { // GPS was switched on but it failed to get a lock, keep GPS switched on

				TRACE_WARN("TRAC > GPS sampling finished GPS LOSS");
				tp->gps_lock = GPS_LOSS;

			} else { // GPS locked successfully, switch off GPS (unless cycle is less than 60 seconds)

				// Switch off GPS (if cycle time is more than 60 seconds)
				if(timeout < TIME_S2I(60)) {
					TRACE_INFO("TRAC > Keep GPS switched on because cycle < 60sec");
					tp->gps_lock = GPS_LOCKED2;
				} else if(conf_sram.gps_onper_vbat != 0 && batt >= conf_sram.gps_onper_vbat) {
					TRACE_INFO("TRAC > Keep GPS switched on because VBAT >= %dmV", conf_sram.gps_onper_vbat);
					tp->gps_lock = GPS_LOCKED2;
				} else {
					TRACE_INFO("TRAC > Switch off GPS");
					GPS_Deinit();
					tp->gps_lock = GPS_LOCKED1;
				}

				// Debug
				TRACE_INFO("TRAC > GPS sampling finished GPS LOCK");

				// Calibrate RTC
				setTime(&gpsFix.time);

				// Take time from GPS
				tp->gps_time = date2UnixTimestamp(&gpsFix.time);

				// Set new GPS fix
				tp->gps_lat = gpsFix.lat;
				tp->gps_lon = gpsFix.lon;
				tp->gps_alt = gpsFix.alt;

				tp->gps_sats = gpsFix.num_svs;
				tp->gps_pdop = (gpsFix.pdop+3)/5;
			}

		} else { // GPS communication error

			GPS_Deinit();
			tp->gps_lock = GPS_ERROR;

		}
	}

	tp->gps_ttff = TIME_I2S(chVTGetSystemTime() - start); // Time to first fix

	if(tp->gps_lock != GPS_LOCKED1 && tp->gps_lock != GPS_LOCKED2) { // We have no valid GPS fix
		// Take time from internal RTC
		ptime_t time;
		getTime(&time);
		tp->gps_time = date2UnixTimestamp(&time);

		// Take GPS fix from old lock
		tp->gps_lat = ltp->gps_lat;
		tp->gps_lon = ltp->gps_lon;
		tp->gps_alt = ltp->gps_alt;
	}
}

static void measureVoltage(trackPoint_t* tp)
{
	tp->adc_vbat = stm32_get_vbat();
	tp->adc_vsol = stm32_get_vsol();

	pac1720_get_avg(&tp->pac_vbat, &tp->pac_vsol, &tp->pac_pbat, &tp->pac_psol);
}

static uint8_t bme280_error;

static void getSensors(trackPoint_t* tp)
{
	// Measure BME280
	bme280_error = 0;
	bme280_t handle;

	// Internal BME280
	if(BME280_isAvailable(BME280_I1)) {
		BME280_Init(&handle, BME280_I1);
		tp->sen_i1_press = BME280_getPressure(&handle, 32);
		tp->sen_i1_hum = BME280_getHumidity(&handle);
		tp->sen_i1_temp = BME280_getTemperature(&handle);
	} else { // No internal BME280 found
		TRACE_ERROR("TRAC > Internal BME280 I1 not found");
		tp->sen_i1_press = 0;
		tp->sen_i1_hum = 0;
		tp->sen_i1_temp = 0;
		bme280_error |= 0x1;
	}

	// External BME280 Sensor 1
	if(BME280_isAvailable(BME280_E1)) {
		BME280_Init(&handle, BME280_E1);
		tp->sen_e1_press = BME280_getPressure(&handle, 32);
		tp->sen_e1_hum = BME280_getHumidity(&handle);
		tp->sen_e1_temp = BME280_getTemperature(&handle);
	} else { // No internal BME280 found
		TRACE_ERROR("TRAC > External BME280 E1 not found");
		tp->sen_e1_press = 0;
		tp->sen_e1_hum = 0;
		tp->sen_e1_temp = 0;
		bme280_error |= 0x2;
	}

	// External BME280 Sensor 2
	if(BME280_isAvailable(BME280_E2)) {
		BME280_Init(&handle, BME280_E2);
		tp->sen_e2_press = BME280_getPressure(&handle, 32);
		tp->sen_e2_hum = BME280_getHumidity(&handle);
		tp->sen_e2_temp = BME280_getTemperature(&handle);
	} else { // No internal BME280 found
		TRACE_ERROR("TRAC > External BME280 E2 not found");
		tp->sen_e2_press = 0;
		tp->sen_e2_hum = 0;
		tp->sen_e2_temp = 0;
		bme280_error |= 0x4;
	}

	// Measure various temperature sensors
	tp->stm32_temp = stm32_get_temp();
	tp->si446x_temp = Si446x_getLastTemperature();

	// Measure light intensity from OV5640
	tp->light_intensity = OV5640_getLastLightIntensity() & 0xFFFF;
}

static void setSystemStatus(trackPoint_t* tp) {
	// Set system errors
	tp->sys_error = 0;

	tp->sys_error |= (I2C_hasError()     & 0x1)  << 0;
	tp->sys_error |= (tp->gps_lock == GPS_ERROR) << 2;
	tp->sys_error |= (pac1720_hasError() & 0x3)  << 3;
	tp->sys_error |= (OV5640_hasError()  & 0x7)  << 5;

	tp->sys_error |= (bme280_error & 0x7)        << 8;

	// Set system time
	tp->sys_time = TIME_I2S(chVTGetSystemTime());
}

/**
  * Tracking Module (Thread)
  */
THD_FUNCTION(trackingThread, arg) {
	(void)arg;

	uint32_t id = 0;
	lastTrackPoint = &trackPoints[0];

	// Read time from RTC
	ptime_t time;
	getTime(&time);
	lastTrackPoint->gps_time = date2UnixTimestamp(&time);

	// Get last tracking point from memory
	TRACE_INFO("TRAC > Read last track point from flash memory");
	trackPoint_t* lastLogPoint = getNewestLogEntry();

	if(lastLogPoint != NULL) { // If there has been stored a trackpoint, then get the last know GPS fix
		trackPoints[0].reset     = lastLogPoint->reset+1;
		trackPoints[1].reset     = lastLogPoint->reset+1;
		lastTrackPoint->gps_lat  = lastLogPoint->gps_lat;
		lastTrackPoint->gps_lon  = lastLogPoint->gps_lon;
		lastTrackPoint->gps_alt  = lastLogPoint->gps_alt;
		lastTrackPoint->gps_sats = lastLogPoint->gps_sats;
		lastTrackPoint->gps_ttff = lastLogPoint->gps_ttff;

		TRACE_INFO(
			"TRAC > Last track point (from memory)\r\n"
			"%s Reset %d ID %d\r\n"
			"%s Latitude: %d.%07ddeg\r\n"
			"%s Longitude: %d.%07ddeg\r\n"
			"%s Altitude: %d Meter",
			TRACE_TAB, lastLogPoint->reset, lastLogPoint->id,
			TRACE_TAB, lastTrackPoint->gps_lat/10000000, (lastTrackPoint->gps_lat > 0 ? 1:-1)*lastTrackPoint->gps_lat%10000000,
			TRACE_TAB, lastTrackPoint->gps_lon/10000000, (lastTrackPoint->gps_lon > 0 ? 1:-1)*lastTrackPoint->gps_lon%10000000,
			TRACE_TAB, lastTrackPoint->gps_alt
		);
	} else {
		TRACE_INFO("TRAC > No track point found in flash memory");
	}

	lastTrackPoint->gps_lock = GPS_LOG; // Mark trackPoint as LOG packet

	// Measure telemetry
	measureVoltage(lastTrackPoint);
	getSensors(lastTrackPoint);
	setSystemStatus(lastTrackPoint);

	// Write Trackpoint to Flash memory
	writeLogTrackPoint(lastTrackPoint);

	// Wait for position threads to start
	chThdSleep(TIME_MS2I(500));

	sysinterval_t cycle_time = chVTGetSystemTime();
	while(true)
	{
		TRACE_INFO("TRAC > Do module TRACKING MANAGER cycle");

		trackPoint_t* tp  = &trackPoints[(id+1) % 2]; // Current track point (the one which is processed now)
		trackPoint_t* ltp = &trackPoints[ id    % 2]; // Last track point

		// Determine cycle time
		sysinterval_t track_cycle_time = TIME_S2I(600);
		if(conf_sram.pos_pri.thread_conf.active && conf_sram.pos_sec.thread_conf.active) { // Both position threads are active
			track_cycle_time = conf_sram.pos_pri.thread_conf.cycle < conf_sram.pos_sec.thread_conf.cycle ? conf_sram.pos_pri.thread_conf.cycle : conf_sram.pos_sec.thread_conf.cycle; // Choose the smallest cycle
		} else if(conf_sram.pos_pri.thread_conf.active) { // Only primary position thread is active
			track_cycle_time = conf_sram.pos_pri.thread_conf.cycle;
		} else if(conf_sram.pos_sec.thread_conf.active) { // Only secondary position thread is active
			track_cycle_time = conf_sram.pos_pri.thread_conf.cycle;
		} else { // There must be an error
			TRACE_ERROR("TRAC > Tracking manager started but no position thread is active");
		}

		// Get GPS position
		aquirePosition(tp, ltp, track_cycle_time - TIME_S2I(3));

		tp->id = ++id; // Serial ID

		// Measure telemetry
		measureVoltage(tp);
		getSensors(tp);
		setSystemStatus(tp);

		// Trace data
		unixTimestamp2Date(&time, tp->gps_time);
		TRACE_INFO(	"TRAC > New tracking point available (ID=%d)\r\n"
					"%s Time %04d-%02d-%02d %02d:%02d:%02d\r\n"
					"%s Pos  %d.%05d %d.%05d Alt %dm\r\n"
					"%s Sats %d TTFF %dsec\r\n"
					"%s ADC Vbat=%d.%03dV Vsol=%d.%03dV Pbat=%dmW\r\n"
					"%s AIR p=%6d.%01dPa T=%2d.%02ddegC phi=%2d.%01d%%",
					tp->id,
					TRACE_TAB, time.year, time.month, time.day, time.hour, time.minute, time.day,
					TRACE_TAB, tp->gps_lat/10000000, (tp->gps_lat > 0 ? 1:-1)*(tp->gps_lat/100)%100000, tp->gps_lon/10000000, (tp->gps_lon > 0 ? 1:-1)*(tp->gps_lon/100)%100000, tp->gps_alt,
					TRACE_TAB, tp->gps_sats, tp->gps_ttff,
					TRACE_TAB, tp->adc_vbat/1000, (tp->adc_vbat%1000), tp->adc_vsol/1000, (tp->adc_vsol%1000), tp->pac_pbat,
					TRACE_TAB, tp->sen_i1_press/10, tp->sen_i1_press%10, tp->sen_i1_temp/100, tp->sen_i1_temp%100, tp->sen_i1_hum/10, tp->sen_i1_hum%10
		);

		// Write Trackpoint to Flash memory
		writeLogTrackPoint(tp);

		// Switch last track point
		lastTrackPoint = tp;

		// Wait until cycle
		cycle_time = chThdSleepUntilWindowed(cycle_time, cycle_time + track_cycle_time);
	}
}

void init_tracking_manager(bool useGPS)
{
	if(useGPS)
		tracking_useGPS = true;

	if(!threadStarted)
	{
		threadStarted = true;

		TRACE_INFO("TRAC > Startup tracking thread");
		thread_t *th = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(2*1024), "TRA", NORMALPRIO+1, trackingThread, NULL);
		if(!th) {
			// Print startup error, do not start watchdog for this thread
			TRACE_ERROR("TRAC > Could not startup thread (not enough memory available)");
		} else {
			chThdSleep(TIME_MS2I(300)); // Wait a little bit until tracking manager has initialized first dataset
		}
	}
}

