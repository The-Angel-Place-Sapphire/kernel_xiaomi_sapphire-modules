# SPDX-License-Identifier: GPL-2.0-only

KBUILD_OPTIONS+= DISPLAY_ROOT=$(KERNEL_SRC)/$(M)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)

clean:
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions

# --- sapphire: LineageOS inline kernel build (TARGET_KERNEL_EXT_MODULES) ---
KBUILD_OPTIONS += CONFIG_DRM_MSM=m
KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS="$(OUT_DIR)/../sapphire-modules/qcom/opensource/mmrm-driver/Module.symvers $(OUT_DIR)/../sapphire-modules/xiaomi/opensource/miev/Module.symvers"
