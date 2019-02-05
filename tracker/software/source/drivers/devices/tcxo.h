/*
    Aerospace Decoder - Copyright (C) 2018-2019 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    tcxo.h
 * @brief   TCXO manager.
 *
 * @addtogroup devices
 * @{
 */

#ifndef PKTTCXO_H
#define PKTTCXO_H

#include "pktconf.h"

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/* The wrap around count for the TCXO ICU timer. */
#define TCXO_OVERFLOW_COUNT         0x10000

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
  bool          pktPPMcheckTCXO(xtal_osc_t f);
#ifdef __cplusplus
}
#endif

#endif /* PKTTCXO_H */

/** @} */
