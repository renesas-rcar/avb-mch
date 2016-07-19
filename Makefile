#
# Makefile for the Renesas device drivers.
#

ifndef CONFIG_AVB_MCH
CONFIG_MCH_CORE ?= m
endif

mch_core-objs := mch_core_main.o
obj-$(CONFIG_MCH_CORE) += mch_core.o

ifndef CONFIG_AVB_MCH
SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) $@

endif
