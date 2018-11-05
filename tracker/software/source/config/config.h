#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifndef TRACE_SHOW_TIME
#define TRACE_SHOW_TIME	            TRUE		/* Enables time tracing on debugging port */
#endif
#ifndef TRACE_SHOW_FILE
#define TRACE_SHOW_FILE				TRUE		/* Enables file and line tracing on debugging port */
#endif
#ifndef TRACE_SHOW_THREAD
#define TRACE_SHOW_THREAD           TRUE        /* Enables thread tracing on debugging port */
#endif

#include "types.h"

#define CONFIG_MAGIC_DEFAULT        0x41583235
#define CONFIG_MAGIC_UPDATED        0x46583235

extern conf_t conf_sram;
extern const conf_t conf_flash_default;

#endif /* __CONFIG_H__ */
