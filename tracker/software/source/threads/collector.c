/**
 * @file        collector.c
 * @brief       Telemetry data collector.
 *
 * @addtogroup  telemetry
 * @{
 */

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
static uint8_t bme280_error;

/**
 * Array for looking up model name
 */
static const char *state[] = {GPS_STATE_NAMES};

/* Remembers power state. */
static bool stay_on = false;

/*===========================================================================*/
/* Module external variables.                                                */
/*===========================================================================*/

thread_t *collector_thd;

/**
 * Get pointer to state name as string
 */
const char *get_gps_state_name(uint8_t index) {
  if(index > GPS_STATE_MAX)
    return "INVALID";
  return state[index];
}

/**
  * Returns most recent data point which is complete.
  */
dataPoint_t* getLastDataPoint(void) {
	return lastDataPoint;
}

/**
 *
 */
/*void waitForNewDataPoint(void) {
	uint32_t old_id = getLastDataPoint()->id;
	while(old_id == getLastDataPoint()->id)
		chThdSleep(TIME_S2I(1));
}*/

/**
 * @brief   Determine best fallback data when GPS not operable.
 * @notes   If the last point is valid that prior data is carried forward.
 * @post    The provided data point (record) is updated.
 *
 * @param[in]   tp      pointer to current @p datapoint structure
 * @param[in]   ltp     pointer to prior @p datapoint structure
 * @param[in]   state   state to set in current datapoint
 *
 * @notapi
 */
static void getPositionFallback(dataPoint_t* tp,
                                dataPoint_t* ltp,
                                gpsState_t state) {
  tp->gps_state = state;
  tp->gps_time = 0;
  tp->gps_lat = 0;
  tp->gps_lon = 0;
  tp->gps_alt = 0;
  tp->gps_sats = 0;
  tp->gps_ttff = 0;
  tp->gps_pdop = 0;
  if(isPositionFromSV(ltp)) {
    tp->gps_lat = ltp->gps_lat;
    tp->gps_lon = ltp->gps_lon;
    tp->gps_alt = ltp->gps_alt;
    tp->gps_time = ltp->gps_time;
    tp->gps_sats = ltp->gps_sats;
    tp->gps_ttff = ltp->gps_ttff;
    tp->gps_pdop = ltp->gps_pdop;
  }
  ptime_t time;
  getTime(&time);
  if(time.year != RTC_BASE_YEAR)
    /* The RTC has been set so use RTC time. */
    tp->gps_time = date2UnixTimestamp(&time);
}

/**
 * @brief   Acquire GPS position and time data.
 * @notes	The GPS is switched on only if a service requires it.
 * @notes	The GPS model used can be controlled by barometric pressure.
 * @notes	The model for high/low pressure is set in config.c.
 * @notes	Model switching enables more reliable lock at low altitude.
 * @notes	The switch to airborne model is then made at high altitude.
 *
 * @post    The provided data point (record) is updated.
 *
 * @param[in]   tp  	pointer to current @p datapoint structure
 * @param[in]	ltp		pointer to prior @p datapoint structure
 * @param[in]   timeout time limit to wait for acquisition
 *
 * @return      state of GPS acquisition
 * @retval      true    if a new position lock was attained.
 * @retval      false   if no new lock attained and stored data used
 * @notapi
 */
static bool aquirePosition(dataPoint_t* tp, dataPoint_t* ltp,
                           bcn_app_conf_t *config,
                           sysinterval_t timeout) {
  systime_t start = chVTGetSystemTime();

  gpsFix_t gpsFix = {0};

  /*
   * Is there enough battery voltage for GPS?
   */
  uint16_t batt = stm32_get_vbat();
  if(batt < conf_sram.gps_on_vbat) {
    getPositionFallback(tp, ltp, GPS_LOWBATT1);
    /* In case GPS was already on then power it off. */
    stay_on = false;
    GPS_Deinit();
    return false;
  }

  /* Try to switch on GPS. If there is an error switch off. */
  if(!GPS_Init()) {
    getPositionFallback(tp, ltp, GPS_ERROR);
    stay_on = false;
    GPS_Deinit();
    return false;
  }
  /* If a Pa pressure is set then GPS model depends on BME reading.
   * If BME is OK then stationary model will be used until Pa < airborne.
   * Then airborne model will be set.
   * If the BME is not OK then airborne model will be used immediately.
   */
  bool dynamic = conf_sram.gps_pressure != 0;
  TRACE_INFO("COLL > GPS %s in dynamic mode switching at %dPa", dynamic
             ? "is" : "is not", conf_sram.gps_pressure);
  /*
   *  Search for GPS lock within the timeout period and while battery is good.
   *  Search timeout=cycle-1sec (-3sec in order to keep synchronization)
   */
  uint32_t x = 0;
  gps_set_model(dynamic);
  do {
    batt = stm32_get_vbat();
    if(++x % 30) gps_set_model(dynamic); // Set model periodically
    chThdSleepMilliseconds(100);
    gps_get_fix(&gpsFix);
  } while(!isGPSLocked(&gpsFix)
      && batt >= conf_sram.gps_off_vbat
      && chVTIsSystemTimeWithin(start, chTimeAddX(start, timeout)));

  if(batt < conf_sram.gps_off_vbat) {
    /*
     * GPS was switched on but battery fell below threshold during acquisition.
     * Switch off GPS and set fallback position data.
     */

    TRACE_WARN("COLL > GPS acquisition stopped due low battery");
    getPositionFallback(tp, ltp, GPS_LOWBATT2);
    stay_on = false;
    GPS_Deinit();
    return false;
  }

  /*
   * Remember conditions to keep power on.
   * Keep GPS switched on if...
   * - battery threshold met and not a run once request.
   * - not a fixed beacon (just wants RTC to be set).
   * This is an OR of the multiple users of the GPS.
   */
  stay_on |= !config->run_once
      && (conf_sram.gps_onper_vbat != 0
          && batt >= conf_sram.gps_onper_vbat)
          && !config->beacon.fixed;

  if(!isGPSLocked(&gpsFix)) {
    /* GPS was switched on but failed to lock within a timeout period. */
    TRACE_WARN("COLL > GPS sampling finished GPS LOSS");
    getPositionFallback(tp, ltp, GPS_LOSS);

    if(stay_on) {
        TRACE_INFO("COLL > Keep GPS switched on. VBAT >= %dmV",
                   conf_sram.gps_onper_vbat);
      } else {
        GPS_Deinit();
        TRACE_INFO("COLL > Switching off GPS");
      }
    return false;
  }

  /*
   * GPS locked successfully.
   * Output SV info.
   * Switch off GPS (unless cycle is less than 60 seconds).
   */
  TRACE_INFO("GPS  > Lock acquired. Model in use is %s",
             gps_get_model_name(gpsFix.model));

  gps_svinfo_t svinfo;
  if(gps_get_sv_info(&svinfo, sizeof(svinfo))) {
    TRACE_INFO("GPS  > Space Vehicle info iTOW=%d numCh=%02d globalFlags=%d",
               svinfo.iTOW, svinfo.numCh, svinfo.globalFlags);

    uint8_t i;
    for(i = 0; i < svinfo.numCh; i++) {
      gps_svchn_t *sat = &svinfo.svinfo[i];
      TRACE_INFO("GPS  > Satellite info chn=%03d svid=%03d flags=0x%02x"
          " quality=%02d cno=%03d elev=%03d azim=%06d, prRes=%06d",
           sat->chn, sat->svid, sat->flags, sat->flags,
           sat->quality, sat->cno, sat->elev, sat->azim, sat->prRes);
    }
  } else {
    TRACE_ERROR("GPS  > Error getting Space Vehicle info");
  }

  /* Enable power saving mode. */
  gps_switch_power_save_mode(true);

  /* Leave GPS on if cycle time is less than 60 seconds. */
  if(timeout < TIME_S2I(60)) {
    TRACE_INFO("COLL > Keep GPS switched on. Cycle < 60sec");
    tp->gps_state = GPS_LOCKED2;
    /* Leave GPS on if power conditions good. */
  } else if(stay_on) {
    TRACE_INFO("COLL > Keep GPS switched on. VBAT >= %dmV",
               conf_sram.gps_onper_vbat);
    tp->gps_state = GPS_LOCKED2;
  } else {
    TRACE_INFO("COLL > Switching off GPS");
    GPS_Deinit();
    tp->gps_state = GPS_LOCKED1;
  }

  // Debug
  TRACE_INFO("COLL > GPS sampling finished GPS LOCK");

  /* Set RTC if not already. */
  ptime_t time;
  getTime(&time);
  if(time.year == RTC_BASE_YEAR) {
    /* Snapshot the RTC time. Old time entries can be adjusted using this data. */
    ltp->gps_state = GPS_TIME;
    ltp->gps_time = date2UnixTimestamp(&time);
  }
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
  tp->gps_ttff = TIME_I2S(chVTGetSystemTime() - start); // Time to first fix
  return true;
}

/**
 * @brief   Get voltage status and save in datapoint.
 * @notes	The battery and solar voltages are read and stored.
 *
 * @post    The provided data point (record) is updated.
 *
 * @param[in]   tp   pointer to a @p datapoint structure
 *
 * @notapi
 */
static void measureVoltage(dataPoint_t* tp)
{
	tp->adc_vbat = stm32_get_vbat();
	tp->adc_vsol = stm32_get_vsol();

	pac1720_get_avg(&tp->pac_vbat, &tp->pac_vsol, &tp->pac_pbat, &tp->pac_psol);
}

/**
 * @brief   Get sensor status and save in datapoint.
 * @notes	The active/installed sensor(s) are read and stored.
 *
 * @post    The provided data point (record) is updated.
 *
 * @param[in]   tp   pointer to a @p datapoint structure
 *
 * @api
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
		TRACE_ERROR("COLL > Internal BME280 I1 not operational");
		tp->sen_i1_press = 0;
		tp->sen_i1_hum = 0;
		tp->sen_i1_temp = 0;
		bme280_error |= 0x1;
	}

#if     ENABLE_EXTERNAL_I2C == TRUE
#if     BME280_E1_IS_FITTED == TRUE
	// External BME280 Sensor 1
	if(BME280_isAvailable(BME280_E1)) {
		BME280_Init(&handle, BME280_E1);
		tp->sen_e1_press = BME280_getPressure(&handle, 32);
		tp->sen_e1_hum = BME280_getHumidity(&handle);
		tp->sen_e1_temp = BME280_getTemperature(&handle);
	} else { // No external BME280 found
		TRACE_ERROR("COLL > External BME280 E1 not operational");
		tp->sen_e1_press = 0;
		tp->sen_e1_hum = 0;
		tp->sen_e1_temp = 0;
		bme280_error |= 0x4;
	}
#else /* BME280_E1_IS_FITTED != TRUE */
	bme280_error |= 0x8;
#endif /* BME280_E1_IS_FITTED == TRUE */

#if     BME280_E2_IS_FITTED == TRUE
	// External BME280 Sensor 2
	if(BME280_isAvailable(BME280_E2)) {
		BME280_Init(&handle, BME280_E2);
		tp->sen_e2_press = BME280_getPressure(&handle, 32);
		tp->sen_e2_hum = BME280_getHumidity(&handle);
		tp->sen_e2_temp = BME280_getTemperature(&handle);
	} else { // No external BME280 found
		TRACE_ERROR("COLL > External BME280 E2 not operational");
		tp->sen_e2_press = 0;
		tp->sen_e2_hum = 0;
		tp->sen_e2_temp = 0;
		bme280_error |= 0x10;
	}
#else /* BME280_E2_IS_FITTED != TRUE */
	bme280_error |= 0x20;
#endif /* BME280_E2_IS_FITTED == TRUE */

#else /*  ENABLE_EXTERNAL_I2C != TRUE */
	/* Set status to "not fitted" for E1 & E2. */
	bme280_error |= 0x28;
#endif /*  ENABLE_EXTERNAL_I2C == TRUE */
	// Measure various temperature sensors
	/* TODO: Add LLD API to radio. */
	tp->stm32_temp = stm32_get_temp();
	tp->si446x_temp = Si446x_getLastTemperature(PKT_RADIO_1);

	// Measure light intensity from OV5640
	tp->light_intensity = OV5640_getLastLightIntensity() & 0xFFFF;
}

/**
 * @brief   Get GPIO port status and save in datapoint.
 * @notes	The input state or current output state is read.
 * @notes	The GPIO mode is determined by the port user.
 *
 * @post    The provided data point (record) is updated.
 *
 * @param[in]   tp   pointer to a @p datapoint structure
 * @param[out]  tp   gpio field is updated with current GPIO states.
 *
 * @notapi
 */
static void getGPIO(dataPoint_t* tp) {
  tp->gpio = pktReadIOlines();
}

/**
 * @brief   Update/set system status data.
 * @notes	The system hardware health/status is read.
 * @notes	This covers the main hardware units...
 * @notes	I2C, GPS, Power, Camera & Environment.
 *
 * @post    The provided data point (record) is updated.
 *
 * @param[in]   tp   pointer to a @p datapoint structure
 *
 * @api
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
   * -  10:11 BMEe1 status (0 = OK, 1 = Fail, 2 = Not fitted)
   * -  12:13 BMEe2 status (0 = OK, 1 = Fail, 2 = Not fitted)
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

/*===========================================================================*/
/* Data collector thread.                                                    */
/*===========================================================================*/

/**
  *
  */
THD_FUNCTION(collectorThread, arg) {
  thread_t *caller = (thread_t *)arg;

  uint32_t id = 0;

  // Read time from RTC
  ptime_t time;
  getTime(&time);
  dataPoints[0].gps_time = date2UnixTimestamp(&time);
  dataPoints[1].gps_time = date2UnixTimestamp(&time);

  lastDataPoint = &dataPoints[0];
  //dataPoint_t *newDataPoint = lastDataPoint++;

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
    /* State indicates that no valid stored position is available. */
    lastDataPoint->gps_lat = 0;
    lastDataPoint->gps_lon = 0;
    lastDataPoint->gps_alt = 0;
    lastDataPoint->gps_ttff = 0;
    lastDataPoint->gps_pdop = 0;
    lastDataPoint->gps_sats = 0;
    lastDataPoint->gps_state = GPS_OFF;

    // Measure telemetry
    measureVoltage(lastDataPoint);
    getSensors(lastDataPoint);
    getGPIO(lastDataPoint);
    setSystemStatus(lastDataPoint);

    // Write data point to Flash memory
    flash_writeLogDataPoint(lastDataPoint);
  }
  /* Now check if the controller has been reset (RTC not set). */
  getTime(&time);
  /* Let initializer know if this is a normal or cold start (power loss). */
  (void)chMsgSend(caller, time.year == RTC_BASE_YEAR ? MSG_RESET : MSG_OK);

  /*
   * Done with initialization now.
   * lastDataPoint becomes the first history entry for the loop.
   */
  while(true) { /* Primary loop. */
    /* Wait for a request from a client. */
    caller = chMsgWait();
    /* Fetch the message. */
    bcn_app_conf_t *config;
    config = (bcn_app_conf_t *)chMsgGet(caller);

    TRACE_INFO("COLL > Respond to request for DATA COLLECTOR cycle");

    dataPoint_t* tp  = &dataPoints[(id+1) % 2]; // Current data point (the one which is processed now)
    dataPoint_t* ltp = &dataPoints[ id    % 2]; // Last data point

    /* Gather telemetry and system status data. */
    measureVoltage(tp);
    getSensors(tp);
    getGPIO(tp);
    setSystemStatus(tp);

    /* Default maximum GPS wait time for acquisition. */
    sysinterval_t gps_wait_fix = TIME_S2I(60);

    /* If there is wait time specified get it. */
    if(config->gps_wait != 0)
      gps_wait_fix = config->gps_wait;

    /*
     * When cycle is less than minimum then use cycle but limit to 1 second
     */
    if(config->beacon.cycle < gps_wait_fix) {
      gps_wait_fix = (config->beacon.cycle == 0) ? TIME_S2I(1)
                                                  : config->beacon.cycle;
    }
    getTime(&time);
    if(time.year == RTC_BASE_YEAR) {
      /*
       * The RTC is not set.
       * Note: This test is only good until RTC ticks over the base year.
       * There may be a fixed location beacon requiring time only.
       * Alternatively this can be a normal request for a full fix.
       * Enable the GPS and attempt a lock which results in setting the RTC.
       * Allow up to half the fix timeout for time acquisition.
      */
      sysinterval_t gps_wait_time = gps_wait_fix / 2;
      TRACE_INFO("COLL > Attempt time acquisition using GPS "
                        "for up to %d.%d seconds",
                 chTimeI2MS(gps_wait_time) / 1000,
                 chTimeI2MS(gps_wait_time) % 1000);

      if(aquirePosition(tp, ltp, config, gps_wait_time)) {
        /* Acquisition succeeded. */
        if(ltp->gps_state == GPS_TIME) {
          /* Write the time stamp where RTC was calibrated. */
          ltp->gps_sats = 0;
          ltp->gps_ttff = 0;
          ltp->gps_pdop = 0;
          /* Use the new ID for the GPS_TIME entry. */
          ltp->id = tp->id;
          flash_writeLogDataPoint(ltp);
          /* Increment to next ID for new datapoint. */
          tp->id++;
        }
        TRACE_INFO("COLL > RTC update acquired from GPS");
      } else {
        /* Time is stale record. */
        TRACE_INFO("COLL > RTC update not acquired from GPS");
      }
      /* Let the acquisition run and set the datapoint. */
      gps_wait_fix /= 2;
    }

    if(config->beacon.fixed) {
      /*
       * Use fixed position data.
       * Update set fixed position.
       * Set GPS time from RTC.
       */
      TRACE_INFO("COLL > Using fixed location for %s", config->call);
      tp->gps_alt = config->beacon.alt;
      tp->gps_lat = config->beacon.lat;
      tp->gps_lon = config->beacon.lon;
      tp->gps_sats = 0;
      tp->gps_ttff = 0;
      tp->gps_pdop = 0;
      tp->gps_state = GPS_FIXED;
      getTime(&time);
      tp->gps_time = date2UnixTimestamp(&time);
    } else {

      /*
       *  Try GPS lock to get data.
       *  If lock not attained fallback data is set.
       *  Timeout will be remainder of time if RTC set was done.
       */
      TRACE_INFO("COLL > Acquire position using GPS for up to %d.%d seconds",
                                   chTimeI2MS(gps_wait_fix) / 1000,
                                   chTimeI2MS(gps_wait_fix) % 1000);
      if(aquirePosition(tp, ltp, config, gps_wait_fix)) {
        if(ltp->gps_state == GPS_TIME) {
          /* Write the time stamp where RTC was calibrated. */
          ltp->gps_sats = 0;
          ltp->gps_ttff = 0;
          ltp->gps_pdop = 0;
          flash_writeLogDataPoint(ltp);
          TRACE_INFO("COLL > RTC update acquired from GPS");
        }
        TRACE_INFO("COLL > Acquired fresh GPS data");
      } else {
        /* Historical data has been carried forward. */
        TRACE_INFO("COLL > Unable to acquire fresh GPS data");
      }
    }

    tp->id = ++id; // Serial ID
    extern uint8_t gps_model;
    // Trace data
    unixTimestamp2Date(&time, tp->gps_time);
    TRACE_INFO( "COLL > GPS status: state=%s model=%s",
                get_gps_state_name(tp->gps_state),
                gps_get_model_name(gps_model));
    TRACE_INFO( "COLL > New data point (ID=%d)\r\n"
        "%s Time %04d-%02d-%02d %02d:%02d:%02d\r\n"
        "%s Pos  %d.%05d %d.%05d Alt %dm\r\n"
        "%s Sats %d TTFF %dsec\r\n"
        "%s ADC  Vbat=%d.%03dV Vsol=%d.%03dV Pbat=%dmW\r\n"
        "%s AIR  p=%d.%01dPa T=%d.%02ddegC phi=%d.%01d%%\r\n"
        "%s IOP  IO1=%d IO2=%d IO3=%d IO4=%d\r\n"
        "%s STN  %s",
        tp->id,
        TRACE_TAB, time.year, time.month, time.day, time.hour, time.minute, time.day,
        TRACE_TAB, tp->gps_lat/10000000, (tp->gps_lat > 0 ? 1:-1)*(tp->gps_lat/100)%100000, tp->gps_lon/10000000, (tp->gps_lon > 0 ? 1:-1)*(tp->gps_lon/100)%100000, tp->gps_alt,
        TRACE_TAB, tp->gps_sats, tp->gps_ttff,
        TRACE_TAB, tp->adc_vbat/1000, (tp->adc_vbat%1000), tp->adc_vsol/1000, (tp->adc_vsol%1000), tp->pac_pbat / 10,
        TRACE_TAB, tp->sen_i1_press/10, tp->sen_i1_press%10, tp->sen_i1_temp/100, tp->sen_i1_temp%100, tp->sen_i1_hum/10, tp->sen_i1_hum%10,
        TRACE_TAB, tp->gpio & 1, (tp->gpio >> 1) & 1, (tp->gpio >> 2) & 1, (tp->gpio >> 3) & 1,
        TRACE_TAB, config->call
    );

    // Write data point to Flash memory
    flash_writeLogDataPoint(tp);

    // Switch last data point
    lastDataPoint = tp;
    /* Reply to the calling thread. */
    chMsgRelease(caller, (msg_t)tp);
  }
}

/**
 *
 */
void init_data_collector() {
  if(!threadStarted) {

    threadStarted = true;
    TRACE_INFO("COLL > Startup data collector thread");
    thread_t *th = chThdCreateFromHeap(NULL,
                                       THD_WORKING_AREA_SIZE(2 * 1024),
                                       "COL", LOWPRIO,
                                       collectorThread, chThdGetSelfX());
    collector_thd = th;
    if(!th) {
      // Print startup error, do not start watchdog for this thread
      TRACE_ERROR("COLL > Could not start"
          " thread (not enough memory available)");
    } else {
      /* Wait for collector to start. */
      thread_t *tp = chMsgWait();
      msg_t msg = chMsgGet(tp);
      if(msg == MSG_RESET) {
        TRACE_INFO("COLL > Executed cold start");
      }
      else {
        TRACE_INFO("COLL > Executed warm start");
      }
      chMsgRelease(tp, MSG_OK);
    }
  }
}

/** @} */
