# SPDX-License-Identifier: GPL-2.0
# Out-of-tree build wrapper, used by the LineageOS inline kernel build
# (TARGET_KERNEL_EXT_MODULES) and by any "make -C <kernel> M=<this dir>" build.
KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS="$(OUT_DIR)/../sapphire-modules/xiaomi/opensource/miev/Module.symvers"

all: modules

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)
