/*
 * tcxo.h
 *
 *  Created on: 20 Oct 2018
 *      Author: bob
 */

#ifndef SOURCE_DRIVERS_DEVICES_TCXO_H_
#define SOURCE_DRIVERS_DEVICES_TCXO_H_

#include "pktconf.h"

#define TCXO_OVERFLOW_COUNT 0x10000

typedef enum {
  TCXO_READY = 0,
  TCXO_CAPTURE,
  TCXO_COMPLETE
} tcxo_state_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void      pktInitTCXO(void);
  uint32_t  pktMeasureTCXO(sysinterval_t timeout);
  uint32_t  pktGetCurrentTCXO(void);
  void      pktCBPeriodTCXO(ICUDriver *icup);
  void      pktCBOverflowTCXO(ICUDriver *icup);
#ifdef __cplusplus
}
#endif

#endif /* SOURCE_DRIVERS_DEVICES_TCXO_H_ */
