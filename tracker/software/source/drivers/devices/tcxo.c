/*
 * tcxo.c
 *
 *  Created on: 20 Oct 2018
 *      Author: bob
 */

#include "tcxo.h"
#include "pktconf.h"
#include "portab.h"
#include "collector.h"


/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

BSEMAPHORE_DECL(tcxo_busy, false);

xtal_osc_t tcxo_period;
xtal_osc_t tcxo_active = PKT_TCXO_DEFAULT_CLOCK;
volatile tcxo_state_t  tcxo_state;

/*===========================================================================*/
/* Module local structures.                                                  */
/*===========================================================================*/

static void pktCBPeriodTCXO(ICUDriver *icup);
static void pktCBOverflowTCXO(ICUDriver *icup);

ICUConfig tcxo_cfg = {
  ICU_INPUT_ACTIVE_HIGH,
  PKT_TCXO_TIMER_CLOCK,    /**< ICU clock frequency. */
  NULL,                    /**< ICU width callback. */
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
static bool pktPPMcheckTCXO(xtal_osc_t f) {
  return (f >= PKT_TCXO_CLOCK_MIN && PKT_TCXO_CLOCK_MAX >= f);
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
 *
 */
static xtal_osc_t pktMeasureTCXO(sysinterval_t timeout) {
  if(timeout == TIME_INFINITE)
    return 0;
  /* TODO: The correct test is if GPS is active and locked. */
  if(!hasGPSacquiredLock(getLastDataPoint()))
    return 0;
  msg_t msg = chBSemWait(&tcxo_busy);
  if(msg == MSG_RESET)
    return 0;

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
 *
 */
THD_FUNCTION(tcxo_thd, arg) {
  (void)arg;

  while(true) {
    uint32_t f = pktMeasureTCXO(TIME_S2I(5));
    if(f != 0) {
      if(!pktPPMcheckTCXO(f)) {
        TRACE_WARN("TCXO > Measured frequency %d Hz is outside PPM bounds", f);
      }
      if(f != tcxo_active) {
        tcxo_active = f;
        TRACE_DEBUG("TCXO > Update to %d Hz", f);
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
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "TCXO", LOWPRIO,
                      tcxo_thd, NULL);
  chThdSleep(TIME_MS2I(10));
}

/**
 *
 */
inline xtal_osc_t pktGetCurrentTCXO()  {
  if(pktPPMcheckTCXO(tcxo_active))
    return tcxo_active;
  return PKT_TCXO_DEFAULT_CLOCK;
}

/**
 *
 */
xtal_osc_t pktCheckUpdatedTCXO(xtal_osc_t current)  {
  if(current != tcxo_active)
    return pktGetCurrentTCXO();
  return 0;
}

