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
#include "pkttypes.h"

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static dataPoint_t dataPoints[2];
static dataPoint_t* lastDataPoint;
static bool threadStarted = false;
static uint8_t useGPS = 0;
static uint8_t useTEL = 0;

/**
  * Returns most recent data point which is complete.
  */
dataPoint_t* getLastDataPoint(void) {
	return lastDataPoint;
}

/*
 *
 */
void waitForNewDataPoint(void) {
	uint32_t old_id = getLastDataPoint()->id;
	while(old_id == getLastDataPoint()->id)
		chThdSleep(TIME_S2I(1));
}

/*
 *
 */
static void aquirePosition(dataPoint_t* tp, dataPoint_t* ltp,
                           sysinterval_t timeout) {
  sysinterval_t start = chVTGetSystemTime();

  gpsFix_t gpsFix;
  memset(&gpsFix, 0, sizeof(gpsFix_t));

  // Switch on GPS if enough power is available and GPS is needed by any position thread
  uint16_t batt = stm32_get_vbat();
  if(batt < conf_sram.gps_on_vbat) {
    tp->gps_state = GPS_LOWBATT1;
  } else {

    /* Switch on GPS
     * If BMEi1 is OK then use air pressure to decide if airborne mode is set.
     */
    bool dynamic = conf_sram.gps_airborne != 0;
    TRACE_INFO("COLL > GPS %s in dynamic mode", dynamic ? "is" : "is not");

    bool status = GPS_Init();

    if(status) {
      // Search for lock as long as enough power is available
      do {
        batt = stm32_get_vbat();
        gps_set_model(dynamic);
        gps_get_fix(&gpsFix);
      } while(!isGPSLocked(&gpsFix)
          && batt >= conf_sram.gps_off_vbat
          && chVTIsSystemTimeWithin(start, start + timeout)); // Do as long no GPS lock and within timeout, timeout=cycle-1sec (-3sec in order to keep synchronization)

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
    if(time.year != RTC_BASE_YEAR) {
      /* RTC has been set so OK to use data. */
      tp->gps_time = date2UnixTimestamp(&time);

      // Take GPS fix from old lock
      tp->gps_lat = ltp->gps_lat;
      tp->gps_lon = ltp->gps_lon;
      tp->gps_alt = ltp->gps_alt;
    } /* Else don't set tp data since RTC is not valid. */
  }
}

/*
 *
 */
static void measureVoltage(dataPoint_t* tp)
{
	tp->adc_vbat = stm32_get_vbat();
	tp->adc_vsol = stm32_get_vsol();

	pac1720_get_avg(&tp->pac_vbat, &tp->pac_vsol, &tp->pac_pbat, &tp->pac_psol);
}

static uint8_t bme280_error;

/*
 *
 */
void getSensors(dataPoint_t* tp) {
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
	} else { // No external BME280 found
		TRACE_WARN("COLL > External BME280 E1 not found");
		tp->sen_e1_press = 0;
		tp->sen_e1_hum = 0;
		tp->sen_e1_temp = 0;
		bme280_error |= 0x4;
	}

	// External BME280 Sensor 2
	if(BME280_isAvailable(BME280_E2)) {
		BME280_Init(&handle, BME280_E2);
		tp->sen_e2_press = BME280_getPressure(&handle, 32);
		tp->sen_e2_hum = BME280_getHumidity(&handle);
		tp->sen_e2_temp = BME280_getTemperature(&handle);
	} else { // No external BME280 found
		TRACE_WARN("COLL > External BME280 E2 not found");
		tp->sen_e2_press = 0;
		tp->sen_e2_hum = 0;
		tp->sen_e2_temp = 0;
		bme280_error |= 0x10;
	}
#else
	/* Set status to "not fitted" for E1 & E2. */
	bme280_error |= 0x28;
#endif
	// Measure various temperature sensors
	tp->stm32_temp = stm32_get_temp();
	tp->si446x_temp = Si446x_getLastTemperature(PKT_RADIO_1);

	// Measure light intensity from OV5640
	tp->light_intensity = OV5640_getLastLightIntensity() & 0xFFFF;
}

/*
 * Get the state of GPIOs available on edge connector.
 */
static void getGPIO(dataPoint_t* tp) {
#if     ENABLE_EXTERNAL_I2C == TRUE
  tp->gpio = palReadLine(LINE_GPIO_PIN);
#else
  tp->gpio = palReadLine(LINE_GPIO_PIN)
      | palReadLine(LINE_IO_TXD) << 1
      | palReadLine(LINE_IO_RXD) << 2;
#endif
}

/*
 *
 */
void setSystemStatus(dataPoint_t* tp) {

  /*
   * Set system errors.
   *
   * Bit usage:
   * -  0:1  I2C status
   * -  2:2  GPS status
   * -  3:4  pac1720 status
   * -  5:7  OV5640 status
   * -  8:9  BMEi1 status (0 = OK, 1 = Fail, 2 = Not fitted)
   * -  9:10 BMEe1 status (0 = OK, 1 = Fail, 2 = Not fitted)
   * - 10:11 BMEe2 status (0 = OK, 1 = Fail, 2 = Not fitted)
   */
	tp->sys_error = 0;

	tp->sys_error |= (I2C_hasError()     & 0x1)   << 0;
	tp->sys_error |= (tp->gps_state == GPS_ERROR) << 2;
	tp->sys_error |= (pac1720_hasError() & 0x3)   << 3;
	tp->sys_error |= (OV5640_hasError()  & 0x7)   << 5;

	tp->sys_error |= (bme280_error & BME_ALL_STATUS_MASK)
	    << BME_ALL_STATUS_SHIFT;

	// Set system time
	tp->sys_time = TIME_I2S(chVTGetSystemTime());
}

/**
  * Data Collector (Thread)
  */
THD_FUNCTION(collectorThread, arg) {
  uint8_t *useGPS = arg;

  uint32_t id = 0;

  // Read time from RTC
  ptime_t time;
  getTime(&time);
  dataPoints[0].gps_time = date2UnixTimestamp(&time);
  dataPoints[1].gps_time = date2UnixTimestamp(&time);

  lastDataPoint = &dataPoints[0];

  // Get last data point from memory
  TRACE_INFO("COLL > Read last data point from flash memory");
  dataPoint_t* lastLogPoint = flash_getNewestLogEntry();

  if(lastLogPoint != NULL) { // If there is stored data point, then get it.
    dataPoints[0].reset     = lastLogPoint->reset+1;
    dataPoints[1].reset     = lastLogPoint->reset+1;
    unixTimestamp2Date(&time, lastDataPoint->gps_time);
    lastDataPoint->gps_lat  = lastLogPoint->gps_lat;
    lastDataPoint->gps_lon  = lastLogPoint->gps_lon;
    lastDataPoint->gps_alt  = lastLogPoint->gps_alt;
    lastDataPoint->gps_sats = lastLogPoint->gps_sats;
    lastDataPoint->gps_ttff = lastLogPoint->gps_ttff;

    TRACE_INFO(
        "COLL > Last data point (from memory)\r\n"
        "%s Reset %d ID %d\r\n"
        "%s Time %04d-%02d-%02d %02d:%02d:%02d\r\n"
        "%s Latitude: %d.%07ddeg\r\n"
        "%s Longitude: %d.%07ddeg\r\n"
        "%s Altitude: %d Meter",
        TRACE_TAB, lastLogPoint->reset, lastLogPoint->id,
        TRACE_TAB, time.year, time.month, time.day, time.hour,
        time.minute, time.day,
        TRACE_TAB, lastDataPoint->gps_lat/10000000,
        (lastDataPoint->gps_lat > 0
            ? 1:-1)*lastDataPoint->gps_lat%10000000,
            TRACE_TAB, lastDataPoint->gps_lon/10000000,
            (lastDataPoint->gps_lon > 0
                ? 1:-1)*lastDataPoint->gps_lon%10000000,
                TRACE_TAB, lastDataPoint->gps_alt
    );
    lastDataPoint->gps_state = GPS_LOG; // Mark dataPoint as LOG packet
  } else {
    TRACE_INFO("COLL > No data point found in flash memory");
    lastDataPoint->gps_state = GPS_OFF; // Mark dataPoint unset
  }

  // Measure telemetry
  measureVoltage(lastDataPoint);
  getSensors(lastDataPoint);
  getGPIO(lastDataPoint);
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

    // Determine cycle time and if GPS should be used.
    sysinterval_t data_cycle_time = TIME_S2I(600);
    if(conf_sram.pos_pri.thread_conf.active && conf_sram.pos_sec.thread_conf.active) { // Both position threads are active
      data_cycle_time = conf_sram.pos_pri.thread_conf.cycle < conf_sram.pos_sec.thread_conf.cycle ? conf_sram.pos_pri.thread_conf.cycle : conf_sram.pos_sec.thread_conf.cycle; // Choose the smallest cycle
      (*useGPS)++;
    } else if(conf_sram.pos_pri.thread_conf.active) { // Only primary position thread is active
      data_cycle_time = conf_sram.pos_pri.thread_conf.cycle;
      (*useGPS)++;
    } else if(conf_sram.pos_sec.thread_conf.active) { // Only secondary position thread is active
      data_cycle_time = conf_sram.pos_sec.thread_conf.cycle;
      (*useGPS)++;
    } else if(conf_sram.aprs.thread_conf.active && conf_sram.aprs.tx.beacon) { // APRS beacon is active
      data_cycle_time = conf_sram.aprs.tx.cycle;
      if(conf_sram.aprs.tx.gps) {
        (*useGPS)++;
      }
    } else { // There must be an error
      TRACE_ERROR("COLL > Data collector started but no position thread is active");
    }

    // Gather telemetry and system status data
    measureVoltage(tp);
    getSensors(tp);
    getGPIO(tp);
    setSystemStatus(tp);

    /*
     *  Enable GPS position acquisition if requested.
     *  a) If the RTC was not set then enable GPS temporarily to set it.
     *  b) If a thread is using GPS for position.
     */
    unixTimestamp2Date(&time, ltp->gps_time);
    if(*useGPS == 0 && time.year == RTC_BASE_YEAR) {
      TRACE_INFO("COLL > Acquire time using GPS");
      aquirePosition(tp, ltp, data_cycle_time - TIME_S2I(3));
      /* GPS done. */
      if(!(tp->gps_state == GPS_LOCKED1
          || tp->gps_state == GPS_LOCKED2)) {
        /* Acquisition failed. Wait and then try again. */
        TRACE_INFO("COLL > Time acquisition from GPS failed");
        chThdSleep(TIME_S2I(60));
        continue;
      }
      TRACE_INFO("COLL > Time acquired from GPS");
      /* Switch GPS off. */
      GPS_Deinit();
    }

    if(*useGPS > 0) {
      TRACE_INFO("COLL > Acquire position using GPS");
      aquirePosition(tp, ltp, data_cycle_time - TIME_S2I(3));
    } else {
      /*
       * No threads using GPS.
       * RTC valid so set tp & ltp from fixed location data.
       */
      TRACE_INFO("COLL > Using fixed location");
      unixTimestamp2Date(&time, tp->gps_time);
      tp->gps_alt = conf_sram.aprs.tx.alt;
      tp->gps_lat = conf_sram.aprs.tx.lat;
      tp->gps_lon = conf_sram.aprs.tx.lon;
      tp->gps_state = GPS_FIXED;
/*
      ltp->gps_time = tp->gps_time;
      ltp->gps_alt = tp->gps_alt;
      ltp->gps_lat = tp->gps_lat;
      ltp->gps_lon = tp->gps_lon;
      ltp->gps_state = GPS_FIXED;*/
    }

    tp->id = ++id; // Serial ID

    // Trace data
    unixTimestamp2Date(&time, tp->gps_time);
    TRACE_INFO(	"COLL > New data point available (ID=%d)\r\n"
        "%s Time %04d-%02d-%02d %02d:%02d:%02d\r\n"
        "%s Pos  %d.%05d %d.%05d Alt %dm\r\n"
        "%s Sats %d TTFF %dsec\r\n"
        "%s ADC Vbat=%d.%03dV Vsol=%d.%03dV Pbat=%dmW\r\n"
        "%s AIR p=%d.%01dPa T=%d.%02ddegC phi=%d.%01d%%\r\n"
#if     ENABLE_EXTERNAL_I2C == TRUE
        "%s IOP IO1=%d",

#else
        "%s IOP IO1=%d IO2=%d IO3=%d",
#endif
        tp->id,
        TRACE_TAB, time.year, time.month, time.day, time.hour, time.minute, time.day,
        TRACE_TAB, tp->gps_lat/10000000, (tp->gps_lat > 0 ? 1:-1)*(tp->gps_lat/100)%100000, tp->gps_lon/10000000, (tp->gps_lon > 0 ? 1:-1)*(tp->gps_lon/100)%100000, tp->gps_alt,
        TRACE_TAB, tp->gps_sats, tp->gps_ttff,
        TRACE_TAB, tp->adc_vbat/1000, (tp->adc_vbat%1000), tp->adc_vsol/1000, (tp->adc_vsol%1000), tp->pac_pbat,
        TRACE_TAB, tp->sen_i1_press/10, tp->sen_i1_press%10, tp->sen_i1_temp/100, tp->sen_i1_temp%100, tp->sen_i1_hum/10, tp->sen_i1_hum%10,
#if     ENABLE_EXTERNAL_I2C == TRUE
        TRACE_TAB, tp->gpio & 1
#else
        TRACE_TAB, tp->gpio & 1, (tp->gpio >> 1) & 1, (tp->gpio >> 2) & 1
#endif
    );

    // Write data point to Flash memory
    flash_writeLogDataPoint(tp);

    // Switch last data point
    lastDataPoint = tp;

    // Wait until cycle
    cycle_time = chThdSleepUntilWindowed(cycle_time, cycle_time + data_cycle_time);
  }
}

/**
  * Telemetry config (Thread)
  */
THD_FUNCTION(configThread, arg) {
  uint8_t *useTEL = arg;
  while(true) chThdSleep(TIME_S2I(1));
}

/**
 *
 */
void init_data_collector() {
  if(!threadStarted) {

    threadStarted = true;

    TRACE_INFO("COLL > Startup data collector thread");
    thread_t *th = chThdCreateFromHeap(NULL,
                                       THD_WORKING_AREA_SIZE(10*1024),
                                       "COL", LOWPRIO,
                                       collectorThread, &useGPS);
    if(!th) {
      // Print startup error, do not start watchdog for this thread
      TRACE_ERROR("COLL > Could not start"
          " thread (not enough memory available)");
    } else {
      chThdSleep(TIME_MS2I(300)); // Wait a little bit until data collector has initialized first dataset
    }

    TRACE_INFO("CFG > Startup telemetry config thread");
    th = chThdCreateFromHeap(NULL,
                                       THD_WORKING_AREA_SIZE(10*1024),
                                       "CFG", LOWPRIO,
                                       configThread, &useTEL);
    if(!th) {
      // Print startup error, do not start watchdog for this thread
      TRACE_ERROR("CFG > Could not start"
          " thread (not enough memory available)");
    } else {
      chThdSleep(TIME_MS2I(300));
    }
  }
}

