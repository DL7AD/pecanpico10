#ifndef __CONFIG_H__
#define __CONFIG_H__

#define TRACE_TIME					TRUE		/* Enables time tracing on debugging port */
#define TRACE_FILE					TRUE		/* Enables file and line tracing on debugging port */

#include "types.h"

#define CONFIG_MAGIC_DEFAULT        0x41583235
#define CONFIG_MAGIC_UPDATED        0x46583235

extern conf_t conf_sram;
extern const conf_t conf_flash_default;

#endif /* __CONFIG_H__ */
