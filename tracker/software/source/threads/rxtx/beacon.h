#ifndef __BEACON_H__
#define __BEACON_H__

#include "types.h"

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define PKT_APRS_BEACON_WA_SIZE (4 * 1024)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum bcnType {
  PKT_BCN_STD = 0,
  PKT_BCN_DESCEND,
  PKT_BCN_ASCEND,
  PKT_BCN_ALTITUDE
} bc_type_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  thread_t * start_beacon_thread(bcn_app_conf_t *conf, const char *name);
#ifdef __cplusplus
}
#endif

#endif /* __BEACON_H__ */
