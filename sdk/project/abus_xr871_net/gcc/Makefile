# ABUS PPIC90520 XR871 bridge firmware, milestone 2 (WiFi + SPI link)
include localconfig.mk
ROOT_PATH := ../../..
include $(ROOT_PATH)/gcc.mk
PROJECT := abus_xr871_net
INCLUDE_PATHS += -I$(ROOT_PATH)/project/$(PROJECT)
CC_SYMBOLS += -DBR_WIFI
DIRS_IGNORE := ../gcc% ../image% $(ROOT_PATH)/project/common/board/%
DIRS_ALL := $(shell find -L .. $(ROOT_PATH)/project/common -type d)
DIRS := $(filter-out $(DIRS_IGNORE),$(DIRS_ALL))
DIRS += $(ROOT_PATH)/project/common/board/$(__PRJ_CONFIG_BOARD)
SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))
OBJS := $(addsuffix .o,$(SRCS))
include $(PRJ_MAKE_RULES)
