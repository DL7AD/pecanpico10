/*
    Aerospace Decoder - Copyright (C) 2018-2019 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file        tcxo.c
 * @brief       TCXO manager.
 *
 * @addtogroup  devices
 * @{
 */

#include "tcxo.h"
#include "pktconf.h"
#include "portab.h"
#include "collector.h"


/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

BSEMAPHORE_DECL(tcxo_busy, false);

xtal_osc_t  tcxo_period;
xtal_osc_t  tcxo_active = PKT_TCXO_DEFAULT_CLOCK;
volatile tcxo_state_t  tcxo_state;
cnt_t       samples;

/*===========================================================================*/
/* Module local structures.                                                  */
/*===========================================================================*/

static void pktCBWidthTCXO(ICUDriver *icup);
static void pktCBPeriodTCXO(ICUDriver *icup);
static void pktCBOverflowTCXO(ICUDriver *icup);

ICUConfig tcxo_cfg = {
  ICU_INPUT_ACTIVE_HIGH,
  PKT_TCXO_TIMER_CLOCK,    /**< ICU clock frequency. */
  pktCBWidthTCXO,          /**< ICU width callback. */
  pktCBPeriodTCXO,         /**< ICU period callback. */
  pktCBOverflowTCXO,       /**< ICU overflow callback. */
  PKT_TCXO_TIMER_CHANNEL,  /**< Timer channel. */
  0                        /**< DIER bits. */
};

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 *
 */
static void pktCBWidthTCXO(ICUDriver *icup) {
  (void)icup;
  return;
}

/**
 *
 */
static void pktCBPeriodTCXO(ICUDriver *icup) {
  if(tcxo_state == TCXO_CAPTURE) {
    chSysLockFromISR();
    tcxo_period += icuGetPeriodX(icup);
    icuDisableNotificationsI(icup);
    tcxo_state = TCXO_COMPLETE;
    chSysUnlockFromISR();
    return;
  } else if(tcxo_state == TCXO_READY) {
    tcxo_period = 0;
    tcxo_state = TCXO_CAPTURE;
    return;
  }
}

/**
 *
 */
static void pktCBOverflowTCXO(ICUDriver *icup) {
  (void)icup;
  if(tcxo_state == TCXO_CAPTURE)
    tcxo_period += TCXO_OVERFLOW_COUNT;
}

/**
 * @brief  Run a measurement of the TCXO
 * @param[in] timeout Maximum interval to wait for measurement to complete
 *
 * @return Measurement result
 * @retval Frequency count for TCXO.
 * @retval Zero if count not able to be done.
 *
 * @notapi
 */
static xtal_osc_t pktMeasureTCXO(sysinterval_t timeout) {
  chDbgCheck((timeout != TIME_INFINITE) && (timeout != TIME_IMMEDIATE));
#if 0
  if(timeout == TIME_INFINITE || timeout == TIME_IMMEDIATE)
    return 0;
#endif
  /* Get exclusive access to TCXO measurement system. */
  msg_t msg = chBSemWait(&tcxo_busy);
  if(msg == MSG_RESET)
    return 0;

  /* Check if GPS has a lock so TP is pulsing. */
  gps_navinfo_t nav = {0};
  if (!gps_get_nav_status(&nav, sizeof(nav))) {
    chBSemSignal(&tcxo_busy);
    return 0;
  }
  /* Check if fix is OK. Note use the fixOK flag versus the fix type. */
  if ((nav.flags & 1) == 0) {
    chBSemSignal(&tcxo_busy);
    return 0;
  }

  /* Start ICU and start capture. */
  icuStart(&PKT_TCXO_TIMER, &tcxo_cfg);
  icuStartCapture(&PKT_TCXO_TIMER);
  icuEnableNotifications(&PKT_TCXO_TIMER);
  tcxo_state = TCXO_READY;
  systime_t start = chVTGetSystemTimeX();
  systime_t end = chTimeAddX(start, timeout);
  tcxo_state_t s = tcxo_state;
  while((s = tcxo_state) != TCXO_COMPLETE
      && chVTIsSystemTimeWithinX(start, end)) {
    chThdSleep(TIME_MS2I(100));
  }
  icuDisableNotifications(&PKT_TCXO_TIMER);
  icuStopCapture(&PKT_TCXO_TIMER);
  icuStop(&PKT_TCXO_TIMER);
  tcxo_state = TCXO_READY;
  xtal_osc_t f_tcxo = (float32_t)STM32_HSECLK
      * ((float32_t)tcxo_period / (float32_t)PKT_TCXO_TIMER_CLOCK);
  chBSemSignal(&tcxo_busy);
  return s == TCXO_COMPLETE ? f_tcxo : 0;
}

/**
 * @brief Run TCXO frequency check and update radios upon change.
 *
 * @notapi
 */
THD_FUNCTION(tcxo_thd, arg) {
  (void)arg;

  while (true) {
    uint32_t f = pktMeasureTCXO(TIME_S2I(8));
    if (f != 0) {
      if (!pktPPMcheckTCXO(f)) {
        TRACE_WARN("TCXO > Measured frequency %d Hz is outside PPM bounds", f);
        chThdSleep(TIME_S2I(60));
        continue;
      }
      if (f != tcxo_active) {
        tcxo_active = f;
        TRACE_DEBUG("TCXO > Update to %d Hz", f);
        /* Advise radio managers of update. */
        uint8_t radios = pktGetNumRadios();
        const radio_config_t *list = pktGetRadioList();
        for(uint8_t i = 0; i < radios; i++) {
          radio_unit_t radio = list->unit;
          if (!pktIsServiceAvailable(radio))
            /* Service is not open for this radio. */
            continue;
          /* Update xtal frequency in service object and send update to radio. */
          packet_svc_t *handler = pktGetServiceObject(radio);
          handler->xtal = tcxo_active;
          msg_t result;
          msg_t msg = pktQueueRadioCommand(radio,
                                           PKT_RADIO_TCXO_UPDATE,
                                           NULL,
                                           TIME_MS2I(100),
                                           &result,
                                           NULL);
          if (msg != MSG_OK) {
            TRACE_ERROR("TCXO > Could not post TCXO update to radio %d",
                        radio);
          }
          if (result != MSG_OK) {
            TRACE_ERROR("TCXO > TCXO update to radio %d failed with result %d",
                        radio, result);
          }
          /* Next radio data record. */
          list++;
        } /* End for radios*/
      } else {
        TRACE_DEBUG("TCXO > Unchanged at %d Hz", f);
      }
    } else
      TRACE_WARN("TCXO > No update available");
    chThdSleep(TIME_S2I(60));
  }
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 *
 */
void pktInitTCXO()  {
  TRACE_INFO("TCXO > Init TCXO");
  pktSetGPIOlineMode(LINE_GPS_TIMEPULSE, PAL_MODE_INPUT | PAL_MODE_ALTERNATE(9));
  icuObjectInit(&PKT_TCXO_TIMER);
  /* TODO: Get last known good from flash. */
  tcxo_active = PKT_TCXO_DEFAULT_CLOCK;
  TRACE_INFO("TCXO > Start TCXO continuous measurement");
  if (chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "TCXO", LOWPRIO,
                      tcxo_thd, NULL) == NULL) {
    TRACE_ERROR("TCXO > Start TCXO thread failed");
}
  chThdSleep(TIME_MS2I(10));
}

/**
 * @brief Get TCXO frequency
 *
 * @return  The default if TCXO has not yet been measured else current measurement.
 */
inline xtal_osc_t pktGetCurrentTCXO()  {
  if(pktPPMcheckTCXO(tcxo_active))
    return tcxo_active;
  return PKT_TCXO_DEFAULT_CLOCK;
}

/**
 * @brief Check if TCXO has been updated
 *
 * @return  the new TCXO frequency or 0 if unchanged.
 *
 */
xtal_osc_t pktCheckUpdatedTCXO(xtal_osc_t current)  {
  if(current != tcxo_active)
    return pktGetCurrentTCXO();
  return 0;
}

/**
 * @brief Verify that the measure frequency is not out of PPM bounds.
 *
 * @return true if within bounds, false if not.
 */
bool pktPPMcheckTCXO(xtal_osc_t f) {
  return (f >= PKT_TCXO_CLOCK_MIN && PKT_TCXO_CLOCK_MAX >= f);
}

/** @} */
