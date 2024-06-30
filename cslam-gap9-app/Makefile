# User Test
#------------------------------------
APP              = slamtest
# App sources
APP_SRCS         = main.c
APP_SRCS         += dma/dma_transfers.c
APP_SRCS    += $(wildcard spi/*.c)
APP_SRCS         += icp/icp.c
APP_SRCS    += $(wildcard slam/*.c)
APP_SRCS    += $(wildcard slam/rcm/*.c)
APP_SRCS    += $(wildcard point-cloud/*.c)
APP_SRCS    += gpio/gpio_config.c
# App includes
APP_INC	         = slam
APP_INC	         += slam/rcm
APP_INC	         += dma
APP_INC	         += icp
APP_INC	         += spi
APP_INC	         += point-cloud
APP_INC	         += gpio
# Compiler flags
APP_CFLAGS       = -O3
# Linker flags
APP_LDFLAGS      = -lm -Wl,--print-memory-usage

# Custom linker
APP_LINK_SCRIPT  =

include $(RULES_DIR)/pmsis_rules.mk
