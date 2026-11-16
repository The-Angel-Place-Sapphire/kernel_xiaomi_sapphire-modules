#By default build for CLD
RMNET_WLAN_SELECT := CONFIG_RMNET_WLAN=m
KBUILD_OPTIONS += $(RMNET_WLAN_SELECT)
KBUILD_OPTIONS += $(KBUILD_EXTRA) # Extra config if any
KBUILD_EXTRA_SYMBOLS := $(M)/../../$(DATARMNET_CORE_PATH)/Module.symvers

M ?= $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS) $(KBUILD_EXTRA_SYMBOLS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean

# --- sapphire: LineageOS inline kernel build (TARGET_KERNEL_EXT_MODULES) ---
KBUILD_OPTIONS += KCFLAGS="-Wno-error=misleading-indentation -I$(KERNEL_SRC)/../sapphire-modules/qcom/opensource/datarmnet/core"
KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS="$(OUT_DIR)/../sapphire-modules/qcom/opensource/datarmnet/core/Module.symvers"
KBUILD_EXTRA_SYMBOLS :=
