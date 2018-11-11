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
#define TCXO_OVERFLOW_COUNT     0x10000

#define PKT_TCXO_MAX_PPM_ERROR      40       /**< Max ppm drift from default */
#define PKT_TCXO_DEFAULT_CLOCK      (PKT_TCXO_CLOCK + PKT_TCXO_DEFAULT_ERROR_HZ)
#define PKT_TCXO_MAX_ERROR_HZ       ((PKT_TCXO_DEFAULT_CLOCK / 1000000)      \
                                        * PKT_TCXO_MAX_PPM_ERROR)
#define PKT_TCXO_CLOCK_MAX          (PKT_TCXO_DEFAULT_CLOCK                  \
                                     + PKT_TCXO_MAX_ERROR_HZ)
#define PKT_TCXO_CLOCK_MIN          (PKT_TCXO_DEFAULT_CLOCK                  \
                                     - PKT_TCXO_MAX_ERROR_HZ)

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
