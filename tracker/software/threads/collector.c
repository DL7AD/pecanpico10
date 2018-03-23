#include "ch.h"
#include "hal.h"

#include "collector.h"
#include "debug.h"
#include "config.h"
#include "ublox.h"
#include "bme280.h"
#include "padc.h"
#include "pac1720.h"
#include "ov5640.h"
#include "radio.h"
#include "watchdog.h"
#include "pi2c.h"
#include "si446x.h"
#include "pflash.h"

static dataPoint_t dataPoints[2];
static dataPoint_t* lastDataPoint;
static bool threadStarted = false;

/**
  * Returns most recent data point witch is complete.
  */
dataPoint_t* getLastDataPoint(void)
{
	return lastDataPoint;
}

void waitForNewDataPoint(void)
{
	uint32_t old_id = getLastDataPoint()->id;
	while(old_id == getLastDataPoint()->id)
		chThdSleep(TIME_S2I(1));
}


static void aquirePosition(dataPoint_t* tp, dataPoint_t* ltp, sysinterval_t timeout)
{
	sysinterval_t start = chVTGetSystemTime();

	gpsFix_t gpsFix;
	memset(&gpsFix, 0, sizeof(gpsFix_t));

	// Switch on GPS if enough power is available and GPS is needed by any position thread
	uint16_t batt = stm32_get_vbat();
	if(batt < conf_sram.gps_on_vbat) {
		tp->gps_state = GPS_LOWBATT1;
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
				TRACE_WARN("COLL > GPS sampling finished GPS LOW BATT");
				tp->gps_state = GPS_LOWBATT2;

			} else if(!isGPSLocked(&gpsFix)) { // GPS was switched on but it failed to get a lock, keep GPS switched on

				TRACE_WARN("COLL > GPS sampling finished GPS LOSS");
				tp->gps_state = GPS_LOSS;

			} else { // GPS locked successfully, switch off GPS (unless cycle is less than 60 seconds)

				// Switch off GPS (if cycle time is more than 60 seconds)
				if(timeout < TIME_S2I(60)) {
					TRACE_INFO("COLL > Keep GPS switched on because cycle < 60sec");
					tp->gps_state = GPS_LOCKED2;
				} else if(conf_sram.gps_onper_vbat != 0 && batt >= conf_sram.gps_onper_vbat) {
					TRACE_INFO("COLL > Keep GPS switched on because VBAT >= %dmV", conf_sram.gps_onper_vbat);
					tp->gps_state = GPS_LOCKED2;
				} else {
					TRACE_INFO("COLL > Switch off GPS");
					GPS_Deinit();
					tp->gps_state = GPS_LOCKED1;
				}

				// Debug
				TRACE_INFO("COLL > GPS sampling finished GPS LOCK");

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
			tp->gps_state = GPS_ERROR;

		}
	}

	tp->gps_ttff = TIME_I2S(chVTGetSystemTime() - start); // Time to first fix

	if(tp->gps_state != GPS_LOCKED1 && tp->gps_state != GPS_LOCKED2) { // We have no valid GPS fix
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

static void measureVoltage(dataPoint_t* tp)
{
	tp->adc_vbat = stm32_get_vbat();
	tp->adc_vsol = stm32_get_vsol();

	pac1720_get_avg(&tp->pac_vbat, &tp->pac_vsol, &tp->pac_pbat, &tp->pac_psol);
}

static uint8_t bme280_error;

static void getSensors(dataPoint_t* tp)
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
		TRACE_ERROR("COLL > Internal BME280 I1 not found");
		tp->sen_i1_press = 0;
		tp->sen_i1_hum = 0;
		tp->sen_i1_temp = 0;
		bme280_error |= 0x1;
	}

#if     ENABLE_EXTERNAL_I2C == TRUE
	// External BME280 Sensor 1
	if(BME280_isAvailable(BME280_E1)) {
		BME280_Init(&handle, BME280_E1);
		tp->sen_e1_press = BME280_getPressure(&handle, 32);
		tp->sen_e1_hum = BME280_getHumidity(&handle);
		tp->sen_e1_temp = BME280_getTemperature(&handle);
	} else { // No internal BME280 found
		TRACE_ERROR("COLL > External BME280 E1 not found");
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
		TRACE_ERROR("COLL > External BME280 E2 not found");
		tp->sen_e2_press = 0;
		tp->sen_e2_hum = 0;
		tp->sen_e2_temp = 0;
		bme280_error |= 0x4;
	}
#endif
	// Measure various temperature sensors
	tp->stm32_temp = stm32_get_temp();
	tp->si446x_temp = Si446x_getLastTemperature();

	// Measure light intensity from OV5640
	tp->light_intensity = OV5640_getLastLightIntensity() & 0xFFFF;
}

static void setSystemStatus(dataPoint_t* tp) {
	// Set system errors
	tp->sys_error = 0;

	tp->sys_error |= (I2C_hasError()     & 0x1)   << 0;
	tp->sys_error |= (tp->gps_state == GPS_ERROR) << 2;
	tp->sys_error |= (pac1720_hasError() & 0x3)   << 3;
	tp->sys_error |= (OV5640_hasError()  & 0x7)   << 5;

	tp->sys_error |= (bme280_error & 0x7)         << 8;

	// Set system time
	tp->sys_time = TIME_I2S(chVTGetSystemTime());
}

/**
  * Data Collector (Thread)
  */
THD_FUNCTION(collectorThread, arg) {
	(void)arg;

	uint32_t id = 0;
	lastDataPoint = &dataPoints[0];

	// Read time from RTC
	ptime_t time;
	getTime(&time);
	lastDataPoint->gps_time = date2UnixTimestamp(&time);

	// Get last data point from memory
	TRACE_INFO("COLL > Read last data point from flash memory");
	dataPoint_t* lastLogPoint = flash_getNewestLogEntry();

	if(lastLogPoint != NULL) { // If there has been stored a data point, then get the last know GPS fix
		dataPoints[0].reset     = lastLogPoint->reset+1;
		dataPoints[1].reset     = lastLogPoint->reset+1;
		lastDataPoint->gps_lat  = lastLogPoint->gps_lat;
		lastDataPoint->gps_lon  = lastLogPoint->gps_lon;
		lastDataPoint->gps_alt  = lastLogPoint->gps_alt;
		lastDataPoint->gps_sats = lastLogPoint->gps_sats;
		lastDataPoint->gps_ttff = lastLogPoint->gps_ttff;

		TRACE_INFO(
			"COLL > Last data point (from memory)\r\n"
			"%s Reset %d ID %d\r\n"
			"%s Latitude: %d.%07ddeg\r\n"
			"%s Longitude: %d.%07ddeg\r\n"
			"%s Altitude: %d Meter",
			TRACE_TAB, lastLogPoint->reset, lastLogPoint->id,
			TRACE_TAB, lastDataPoint->gps_lat/10000000, (lastDataPoint->gps_lat > 0 ? 1:-1)*lastDataPoint->gps_lat%10000000,
			TRACE_TAB, lastDataPoint->gps_lon/10000000, (lastDataPoint->gps_lon > 0 ? 1:-1)*lastDataPoint->gps_lon%10000000,
			TRACE_TAB, lastDataPoint->gps_alt
		);
	} else {
		TRACE_INFO("COLL > No data point found in flash memory");
	}

	lastDataPoint->gps_state = GPS_LOG; // Mark dataPoint as LOG packet

	// Measure telemetry
	measureVoltage(lastDataPoint);
	getSensors(lastDataPoint);
	setSystemStatus(lastDataPoint);

	// Write data point to Flash memory
	flash_writeLogDataPoint(lastDataPoint);

	// Wait for position threads to start
	chThdSleep(TIME_MS2I(500));

	sysinterval_t cycle_time = chVTGetSystemTime();
	while(true)
	{
		TRACE_INFO("COLL > Do module DATA COLLECTOR cycle");

		dataPoint_t* tp  = &dataPoints[(id+1) % 2]; // Current data point (the one which is processed now)
		dataPoint_t* ltp = &dataPoints[ id    % 2]; // Last data point

		// Determine cycle time
		sysinterval_t data_cycle_time = TIME_S2I(600);
		if(conf_sram.pos_pri.thread_conf.active && conf_sram.pos_sec.thread_conf.active) { // Both position threads are active
			data_cycle_time = conf_sram.pos_pri.thread_conf.cycle < conf_sram.pos_sec.thread_conf.cycle ? conf_sram.pos_pri.thread_conf.cycle : conf_sram.pos_sec.thread_conf.cycle; // Choose the smallest cycle
		} else if(conf_sram.pos_pri.thread_conf.active) { // Only primary position thread is active
			data_cycle_time = conf_sram.pos_pri.thread_conf.cycle;
		} else if(conf_sram.pos_sec.thread_conf.active) { // Only secondary position thread is active
			data_cycle_time = conf_sram.pos_pri.thread_conf.cycle;
		} else { // There must be an error
			TRACE_ERROR("COLL > Data collector started but no position thread is active");
		}

		// Get GPS position
		aquirePosition(tp, ltp, data_cycle_time - TIME_S2I(3));

		tp->id = ++id; // Serial ID

		// Measure telemetry
		measureVoltage(tp);
		getSensors(tp);
		setSystemStatus(tp);

		// Trace data
		unixTimestamp2Date(&time, tp->gps_time);
		TRACE_INFO(	"COLL > New data point available (ID=%d)\r\n"
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

		// Write data point to Flash memory
		flash_writeLogDataPoint(tp);

		// Switch last data point
		lastDataPoint = tp;

		// Wait until cycle
		cycle_time = chThdSleepUntilWindowed(cycle_time, cycle_time + data_cycle_time);
	}
}

void init_data_collector(void)
{
	if(!threadStarted)
	{
		threadStarted = true;

		TRACE_INFO("COLL > Startup data collector thread");
		thread_t *th = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(2*1024), "TRA", NORMALPRIO+1, collectorThread, NULL);
		if(!th) {
			// Print startup error, do not start watchdog for this thread
			TRACE_ERROR("COLL > Could not startup thread (not enough memory available)");
		} else {
			chThdSleep(TIME_MS2I(300)); // Wait a little bit until data collector has initialized first dataset
		}
	}
}

