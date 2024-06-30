CRAZYFLIE_BASE := ../crazyflie-firmware

EXTRA_CFLAGS += -I$(PWD)/../uwb-software-library/inc
EXTRA_CFLAGS += -I$(PWD)/../uwb-software-library/decawave_api
EXTRA_CFLAGS += -I$(PWD)/src
EXTRA_CFLAGS += -I$(PWD)/src/tof-matrix
EXTRA_CFLAGS += -I$(PWD)/src/tof-matrix/drivers
EXTRA_CFLAGS += -I$(PWD)/src/exploration
EXTRA_CFLAGS += -I$(PWD)/src/spi_comm
EXTRA_CFLAGS += -Wno-error
EXTRA_CFLAGS += -Wno-conversion

OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk

