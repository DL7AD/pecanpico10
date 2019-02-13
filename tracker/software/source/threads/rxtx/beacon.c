#include "ch.h"
#include "hal.h"

#include "beacon.h"
#include "debug.h"
#include "threads.h"
#include "config.h"
#include "radio.h"
#include "aprs.h"
#include "sleep.h"
#include "chprintf.h"
#include <string.h>
#include <math.h>
#include "watchdog.h"
#include "collector.h"

/*
 *
 */
THD_FUNCTION(bcnThread, arg) {
  bcn_app_conf_t *s_conf = (bcn_app_conf_t *)arg;

  /* Copy the conf so we can adjust local copy on first pass. */
  bcn_app_conf_t conf = *s_conf;

  /*
   * Start data collector (if not running yet).
   * TODO: Register with the collector (add to list of using apps).
   * Unregister when terminating this thread.
   * Use an object like event listener as collector registration object?
   * Use thread_t *chRegFindThreadByName(const char *name) to find COLL
   */
  init_data_collector();

  // Start position thread
  TRACE_INFO("BCN  > Startup beacon thread");

  /* Now wait for our delay before starting (handles TIME_IMMEDIATE). */

  chThdSleepUntil(chVTGetSystemTime() + conf.beacon.init_delay);

  /* Set telemetry configuration transmission timing.
   * Each beacon keeps its own timing.
   * Set zero to produce telemetry on first pass.
   */
  systime_t next_conf_transmission = 0;

  /**
   * Altitude controlled beaconing.
   *
   * This feature is primarily for short latex based flights.
   * In such case high rate beaconing is preferable on descent to improve
   *  the chances of beacon reception and location accuracy for recovery.
   * A second beacon should be assigned for such purpose.
   *
   * Ascent based beaconing is also implemented for completeness.
   *
   * Altitude control is done with an ARM and RUN field in config.
   * If not explicitly set in config these values will both be zero and
   *  normal beacon operation takes place.
   *
   * There are two altitude controlled cases.
   * In both cases an ARM altitude must be reached to enable a beacon.
   * The altitude control commences after the normal startup delay has expired.
   * 1. Ascending activated
   *    If RUN >= ARM then the beacon is of ascending type.
   *    i.e. the beacon runs after first being enabled and then exceeding the
   *    activate altitude.
   *
   * 2. Descending activated
   *    If RUN < ARM then the beacon is of descending type.
   *    i.e. the beacon will run after first being enabled and then falling
   *    below the RUN altitude level.
   *
   * Once activated an altitude controlled beacon runs on the cycle time
   *  and ignores the ARM and RUN settings from then on.
   *
   * Normally ARM would be zero in the case of an ascending beacon
   *  although it need not be if an altitude delayed start is required.
   */

  if (conf.run_alt != 0) {
    /* This is an altitude controlled beacon.
       Wait for enable altitude. */
    msg_t dpmsg;
    dataPoint_t *dataPoint;
    TRACE_DEBUG("POS  > Altitude controlled beacon waiting for enable at %dM",
                conf.arm_alt);
    do {
      dpmsg = chMsgSend(collector_thd, (msg_t)&conf);
      dataPoint = (dataPoint_t *)dpmsg;
    } while (!hasGPSacquiredLock(dataPoint)
        || dataPoint->gps_alt < conf.arm_alt);
    TRACE_DEBUG("POS  > Altitude controlled %dM beacon enabled at %dM",
                conf.arm_alt, dataPoint->gps_alt);
    if (conf.run_alt >= conf.arm_alt) {
      /* This is an ascending beacon. */
      do {
        dpmsg = chMsgSend(collector_thd, (msg_t)&conf);
        dataPoint = (dataPoint_t *)dpmsg;
      } while (!hasGPSacquiredLock(dataPoint)
          || dataPoint->gps_alt < conf.run_alt);
      TRACE_DEBUG("POS  > Ascending %dM beacon activated at %dM",
                  conf.run_alt, dataPoint->gps_alt);
    } else {
      /* This is a descending beacon. */
      do {
        dpmsg = chMsgSend(collector_thd, (msg_t)&conf);
        dataPoint = (dataPoint_t *)dpmsg;
      } while (!hasGPSacquiredLock(dataPoint)
          || dataPoint->gps_alt > conf.run_alt);
      TRACE_DEBUG("POS  > Descending %dM beacon activated at %dM",
                  conf.run_alt, dataPoint->gps_alt);
    }
  }

  /*
   * Force fast timeout on first attempt from normal BCN app.
   * ?APRSP command can set its own interval to override the timing.
   */
  if (conf.run_once != true)
    conf.gps_wait = TIME_MS2I(100);
  while (true) {
    /*
     * Get system time at start of loop.
     * Any consumed time in collector does not subtract from cycle time.
     */
    systime_t time = chVTGetSystemTime();
    char code_s[100];
    pktDisplayFrequencyCode(conf.radio_conf.freq, code_s, sizeof(code_s));
    TRACE_DEBUG("POS  > Do module BEACON cycle for %s on %s%s",
                conf.call, code_s, conf.run_once ? " (?aprsp response)" : "");

    /* Pass pointer to beacon config to the collector thread. */
    msg_t dpmsg = chMsgSend(collector_thd, (msg_t)&conf);
    dataPoint_t *dataPoint = (dataPoint_t *)dpmsg;

    /* Continue here when collector responds. */
    if (!p_sleep(&conf.beacon.sleep_conf)) {

      /* Telemetry encoding parameter transmissions. */
      if (conf_sram.tel_enc_cycle != 0 &&  time > next_conf_transmission) {
        TRACE_MON("BCN  > Transmit telemetry configuration");

        /* Encode and transmit telemetry configuration data. */
        uint8_t type = 0;
        do {
          packet_t packet = aprs_encode_telemetry_configuration(
              conf.call,
              conf.path,
              conf.call,
              type);
          if (packet == NULL) {
            TRACE_WARN("BCN  > No free packet objects for"
                " telemetry config transmission %d", type);
          } else {
            if(!pktTransmitOnRadio(packet,
                                   conf.radio_conf.freq,
                                   0,
                                   0,
                                   conf.radio_conf.pwr,
                                   conf.radio_conf.mod,
                                   conf.radio_conf.cca)) {
              /* Packet is released in transmitOnRadio. */
              TRACE_ERROR("BCN  > Failed to transmit telemetry config");
            }
          }
          chThdSleep(TIME_S2I(5));
        } while (++type < APRS_NUM_TELEM_GROUPS);
        /* Set timing for next telemetry configuration transmission. */
        next_conf_transmission = chTimeAddX(time, conf_sram.tel_enc_cycle);
      }

      TRACE_MON("BCN  > Transmit position and telemetry");

      /* Encode/Transmit position packet. */
      packet_t packet = aprs_encode_position_and_telemetry(conf.call,
                                                           conf.path,
                                                           conf.symbol,
                                                           dataPoint, true);
      if (packet == NULL) {
        TRACE_ERROR("BCN  > No free packet objects"
            " for position transmission");
      } else {
        if(!pktTransmitOnRadio(packet,
                               conf.radio_conf.freq,
                               0,
                               0,
                               conf.radio_conf.pwr,
                               conf.radio_conf.mod,
                               conf.radio_conf.cca)) {
          TRACE_ERROR("BCN  > failed to transmit beacon data");
        }
      }

      /* If this is a run once (?APRSP) terminate thread now. */
      if (conf.run_once) {
        chHeapFree(s_conf);
        pktThdTerminateSelf();
      }

      chThdSleep(TIME_S2I(5));
      /* Send direct list. */
      TRACE_MON("BCN  > Transmit recently heard direct");
      /*
       * Encode/Transmit APRSD packet.
       * This is a tracker originated message (not a reply to a request).
       * The message will be addressed to the base station if set.
       * Else send it to device identity.
       */
      char *call = conf_sram.base.enabled
          ? conf_sram.base.call : conf.call;
      char *path = conf_sram.base.enabled
          ? conf_sram.base.path : conf.path;
      /*
       * Send message from this device.
       * Use call sign and path as specified in base config.
       * There is no acknowledgement requested.
       */
      packet = aprs_compose_aprsd_message(conf.call, path, call);
      if (packet == NULL) {
        TRACE_ERROR("BCN  > No free packet objects "
            "or badly formed APRSD message");
      } else {
        if(!pktTransmitOnRadio(packet,
                               conf.radio_conf.freq,
                               0,
                               0,
                               conf.radio_conf.pwr,
                               conf.radio_conf.mod,
                               conf.radio_conf.cca
        )) {
          TRACE_ERROR("BCN  > Failed to transmit APRSD data");
        }
        chThdSleep(TIME_S2I(5));
      }
    } /* psleep */
    time = waitForTrigger(time, conf.beacon.cycle);
    /* Reset conf to external configuration. */
    conf = *s_conf;
  }
}

/**
 *
 */
thread_t * start_beacon_thread(bcn_app_conf_t *conf, const char *name) {
  //extern memory_heap_t *ccm_heap;
  thread_t *th = chThdCreateFromHeap(ccm_heap,
                                     THD_WORKING_AREA_SIZE(PKT_APRS_BEACON_WA_SIZE),
                                     name, LOWPRIO, bcnThread, conf);
  if(!th) {
    // Print startup error, do not start watchdog for this thread
    TRACE_ERROR("BCN  > Could not start thread (insufficient memory)");
  }
  return th;
}

