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
/* Module local variables.                                                   */
/*===========================================================================*/

BSEMAPHORE_DECL(tcxo_busy, false);

uint32_t tcxo_period;
uint32_t tcxo_active;
volatile tcxo_state_t  tcxo_state;

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 *
 */

THD_FUNCTION(tcxo_thd, arg)
{
    (void)arg;

    while(true)
    {
      uint32_t f = pktMeasureTCXO(TIME_S2I(30));
      if(f != 0) {
        if(f != tcxo_active) {
          tcxo_active = f;
          TRACE_INFO("TCXO > Update to %d Hz", f);
        } else {
          TRACE_INFO("TCXO > Unchanged at %d Hz", f);
        }
      } else
        TRACE_WARN("TCXO > No update available");
        chThdSleep(TIME_S2I(60));
    }
}

/**
 *
 */
void pktInitTCXO()  {
  TRACE_INFO("TCXO > Init TCXO");
  pktSetGPIOlineMode(LINE_GPS_TIMEPULSE, PAL_MODE_INPUT | PAL_MODE_ALTERNATE(9));
  icuObjectInit(&PKT_TCXO_TIMER);
  /* TODO: Get last known good from flash. */
  tcxo_active = STM32_HSECLK + PKT_TCXO_DEFAULT_ERROR;
  TRACE_INFO("TCXO > Start TCXO continuous measurement");
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "TCXO", LOWPRIO,
                      tcxo_thd, NULL);
  chThdSleep(TIME_MS2I(10));
}

/**
 *
 */
uint32_t pktGetCurrentTCXO()  {
  return tcxo_active;
}

/**
 *
 */
uint32_t pktMeasureTCXO(sysinterval_t timeout) {
  if(timeout == TIME_INFINITE)
    return 0;
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
  uint32_t f_tcxo = (float32_t)STM32_HSECLK
      * ((float32_t)tcxo_period / (float32_t)PKT_TCXO_TIMER_CLOCK);
  chBSemSignal(&tcxo_busy);
  return s == TCXO_COMPLETE ? f_tcxo : 0;
}

/**
 *
 */
void pktCBPeriodTCXO(ICUDriver *icup) {
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
void pktCBOverflowTCXO(ICUDriver *icup) {
  (void)icup;
  if(tcxo_state == TCXO_CAPTURE)
    tcxo_period += TCXO_OVERFLOW_COUNT;
}
