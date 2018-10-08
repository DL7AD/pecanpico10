#ifndef __THREADS_H__
#define __THREADS_H__

#include "ch.h"

void start_essential_threads(void);
void start_user_threads(void);
void pktThdTerminateSelf(void);
void pktIdleThread(void);

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/* Thread names.
 * Note: Keep the names unique as they are used to lookup threads by name.
 */
#define PKT_POS_PRI_THD_NAME    "POS1"
#define PKT_POS_SEC_THD_NAME    "POS2"
#define PKT_IMG_PRI_THD_NAME    "IMG1"
#define PKT_IMG_SEC_THD_NAME    "IMG2"
#define PKT_DIG_BCN_THD_NAME    "BCN"
#define PKT_COM_BCN_THD_NAME    "APRSP"
#define PKT_RCV_PRI_THD_NAME    "APRS"

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

extern systime_t watchdog_tracking; // Last update time for module TRACKING

#endif /* __THREADS_H__ */
