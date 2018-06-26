#ifndef __BEACON_H__
#define __BEACON_H__

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif
  thread_t * start_beacon_thread(bcn_app_conf_t *conf, const char *name);
#ifdef __cplusplus
}
#endif

#endif /* __BEACON_H__ */
