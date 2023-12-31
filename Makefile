##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

CC = /opt/arm/bin/arm-none-eabi-gcc -Isrc
LD = /opt/arm/bin/arm-none-eabi-gcc -Isrc 
AS = /opt/arm/bin/arm-none-eabi-gcc -Isrc -x assembler-with-cpp -ggdb
AR = /opt/arm/bin/arm-none-eabi-ar
OD = /opt/arm/bin/arm-none-eabi-objdump
CP = /opt/arm/bin/arm-none-eabi-objcopy
SZ = /opt/arm/bin/arm-none-eabi-size

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -fno-inline-small-functions -ggdb -fomit-frame-pointer -falign-functions=16 --specs=nano.specs -fstack-usage
  #USE_OPT = -fno-inline-small-functions -ggdb -fomit-frame-pointer -falign-functions=16 --specs=nano.specs -fstack-usage
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
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

ifeq ($(VERSION),)
  VERSION="$(shell git describe --tags)"
endif

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x800
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x200
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Dvice node to flash
DEVICE = /dev/cu.usbmodem401
#DEVICE = /dev/ttyACM0

# Define project name here
PROJECT = ch

# Imported source files and paths
#CHIBIOS = ../ChibiOS-RT
CHIBIOS = ChibiOS
PROJ = .

# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f0xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F0xx/platform.mk
#include $(CHIBIOS)/os/hal/boards/ST_STM32F072B_DISCOVERY/board.mk
include NANOVNA_STM32_F072/board.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v6m.mk
# Other files (optional).
#include $(CHIBIOS)/test/rt/test.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk
#include $(CHIBIOS)/os/various/shell/shell.mk

# Define linker script file here
#LDSCRIPT= $(STARTUPLD)/STM32F072xB.ld
LDSCRIPT= STM32F072xB.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(STREAMSSRC) \
       src/usbcfg.c \
       src/strconvert.c \
       src/cobs.c \
       src/si5351.c \
       src/cmdhandler.c \
       src/tlv320aic3204.c \
       src/dsp.c \
       src/main.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

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
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC)  \
         $(STREAMSINC) \
         ./src
         

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m0

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   ?= $(TRGT)gcc
CPPC ?= $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   ?= $(TRGT)gcc
#LD   = $(TRGT)g++
CP   ?= $(TRGT)objcopy
AS   ?= $(TRGT)gcc -x assembler-with-cpp -ggdb
AR   ?= $(TRGT)ar
OD   ?= $(TRGT)objdump
SZ   ?= $(TRGT)size
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
UDEFS = -DSHELL_CMD_TEST_ENABLED=FALSE -DSHELL_CMD_MEM_ENABLED=FALSE -DARM_MATH_CM0 -DVERSION=\"$(VERSION)\"

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR = 

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk

flash: build/ch.bin
	dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D build/ch.bin

dfu:
	-printf "reset dfu\r" >$(DEVICE) && sleep 1

