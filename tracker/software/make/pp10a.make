##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = -std=c11
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra moptions here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = yes
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x1000
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x5000
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16 -fsingle-precision-constant
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ch

# Imported source files and paths
CHIBIOS = ChibiOS
CONFDIR := ${CURDIR}/cfg/pp10a
BUILDDIR := ${CURDIR}/build/pp10a
DEPDIR := ${CURDIR}/.dep/pp10a
AUTOBUILD_ROOT := ${CURDIR}/source/

# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# BOARD files.
include $(CONFDIR)/board/board.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# Auto-build files in AUTOBUILD_ROOT recursively.
#include $(CHIBIOS)/tools/mk/autobuild.mk
# Other files (optional).
include $(CHIBIOS)/test/lib/test.mk
include $(CHIBIOS)/test/rt/rt_test.mk
include $(CHIBIOS)/test/oslib/oslib_test.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk
include $(CHIBIOS)/os/various/shell/shell.mk
#include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk

# Define linker script file here
LDSCRIPT= $(CONFDIR)/STM32F413xH.ld

$(info $$ALLCSRC is [${ALLCSRC}])
$(info $$CONFDIR is [${CONFDIR}])
$(info $$ALLINC is [${ALLINC}])

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(TESTSRC) \
       $(CHIBIOS)/os/various/fatfs_bindings/fatfs_diskio.c \
       $(CHIBIOS)/ext/fatfs/src/ff.c \
       source/threads/rxtx/beacon.c \
       source/threads/collector.c \
       source/threads/rxtx/position.c \
       source/threads/rxtx/image.c \
       source/threads/rxtx/log.c \
       source/threads/rxtx/radio.c \
       source/protocols/ssdv/ssdv.c \
       source/protocols/ssdv/rs8.c \
       source/pkt/protocols/aprs2/ax25_pad.c \
       source/pkt/protocols/aprs2/dedupe.c \
       source/pkt/protocols/aprs2/fcs_calc.c \
       source/protocols/packet/aprs.c \
       source/pkt/protocols/aprs2/digipeater.c \
       source/drivers/wrapper/pi2c.c \
       source/drivers/wrapper/ei2c.c \
       source/drivers/wrapper/padc.c \
       source/drivers/wrapper/ptime.c \
       source/drivers/wrapper/pflash.c \
       source/drivers/ublox.c \
       source/pkt/devices/si446x.c \
       source/drivers/bme280.c \
       source/drivers/pac1720.c \
       source/drivers/ov5640.c \
       source/drivers/sd.c \
       source/drivers/flash/flash.c \
       source/drivers/flash/helper.c \
       source/drivers/flash/ihex.c \
       source/drivers/usb/debug.c \
       source/config/sleep.c \
       source/threads/threads.c \
       source/math/base91.c \
       source/math/geofence.c \
       source/config/config.c \
       source/threads/watchdog.c \
       source/drivers/usb/usbcfg.c \
       source/drivers/usb/commands.c \
       source/drivers/usb/usb.c \
       source/pkt/sys/regex/crx.c \
       \
       $(CONFDIR)/portab.c \
       source/pkt/pktconf.c \
       source/pkt/protocols/crc_calc.c \
       source/pkt/channels/rxafsk.c \
       source/pkt/managers/pktradio.c \
       source/pkt/channels/rxpwm.c \
       source/pkt/filters/dsp.c \
       source/pkt/filters/firfilter_q31.c \
       source/pkt/decoders/corr_q31.c \
       source/pkt/protocols/rxhdlc.c \
       source/pkt/managers/pktservice.c \
       source/pkt/devices/dbguart.c \
       source/pkt/sys/ihex_out.c \
       source/pkt/diagnostics/ax25_dump.c \
       source/pkt/diagnostics/pktevt.c \
       source/pkt/protocols/txhdlc.c \
       \
       main.c \

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC)

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

INCDIR = $(CONFDIR) $(ALLINC) $(TESTINC)
$(info $$INCDIR is [${INCDIR}])
#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -D_GNU_SOURCE -DARM_MATH_CM4 -DSHELL_CMD_TEST_ENABLED=0 \
		-DSHELL_CMD_EXIT_ENABLED=1 -DUSB_TRACE_LEVEL=5 \
		-DSHELL_CMD_MEM_ENABLED=0
		# -DDISABLE_HW_WATCHDOG=1

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR = source/threads/ source/drivers/ source/drivers/wrapper/ source/pkt/protocols/aprs2 \
          source/protocols/ssdv source/math/ source/drivers/flash/ source/drivers/usb/ \
          source/protocols/packet source/drivers/fatfs/ source/threads/rxtx/ \
          source/pkt source/pkt/channels source/pkt/managers source/pkt/devices source/pkt/protocols \
          source/pkt/diagnostics source/pkt/filters source/pkt/decoders source/pkt/sys CMSIS/include \
          source/pkt/sys/regex/ $(CHIBIOS)/ext/fatfs/src source/config

# List the user directory to look for the libraries here
ULIBDIR = CMSIS/Lib/GCC

# List all user libraries here
ULIBS = -lm CMSIS/Lib/GCC/libarm_cortexM4l_math.a

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk

burn:
	st-flash write build/$(PROJECT).bin 0x08000000

