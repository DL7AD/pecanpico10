#ifndef __BEACON_H__
#define __BEACON_H__

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif
  void start_beacon_thread(const thd_aprs_conf_t *conf);
#ifdef __cplusplus
}
#endif

#endif /* __BEACON_H__ */
