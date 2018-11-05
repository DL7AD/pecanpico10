/*
 * tcxo.h
 *
 *  Created on: 20 Oct 2018
 *      Author: bob
 */

#ifndef SOURCE_DRIVERS_DEVICES_TCXO_H_
#define SOURCE_DRIVERS_DEVICES_TCXO_H_

#include "pktconf.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/* The wrap around count for the TCXO ICU timer. */
#define TCXO_OVERFLOW_COUNT 0x10000

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef struct {
  xtal_osc_t prior;
  xtal_osc_t update;
} tcxo_query_t;

typedef enum {
  TCXO_READY = 0,
  TCXO_CAPTURE,
  TCXO_COMPLETE
} tcxo_state_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void          pktInitTCXO(void);
  xtal_osc_t    pktGetCurrentTCXO(void);
  xtal_osc_t    pktCheckUpdatedTCXO(xtal_osc_t current);
#ifdef __cplusplus
}
#endif

#endif /* SOURCE_DRIVERS_DEVICES_TCXO_H_ */
