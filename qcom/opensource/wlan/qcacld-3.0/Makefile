# --- sapphire: LineageOS inline kernel build (TARGET_KERNEL_EXT_MODULES) ---
WLAN_ROOT := $(abspath $(KERNEL_SRC)/$(M))

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

# The Make variable $(M) must point to the directory that contains the module
# source code (which includes this Makefile). It can either be an absolute or a
# relative path. If it is a relative path, then it must be relative to the
# kernel source directory (KERNEL_SRC). An absolute path can be obtained very
# easily through $(shell pwd). Generating a path relative to KERNEL_SRC is
# difficult and we accept some outside help by letting the caller override the
# variable $(M). Allowing a relative path for $(M) enables us to have the build
# system put output/object files (.o, .ko.) into a directory different from the
# module source directory.
M ?= $(shell pwd)

ifeq ($(WLAN_ROOT),)
# WLAN_ROOT must contain an absolute path (i.e. not a relative path)
KBUILD_OPTIONS := WLAN_ROOT=$(shell cd $(KERNEL_SRC); readlink -e $(M))

# MODNAME should be qca_cld3_wlan for helium based platform
ifeq (qca_cld3, $(WLAN_CHIPSET))
KBUILD_OPTIONS += MODNAME?=$(WLAN_CHIPSET)_wlan
else
KBUILD_OPTIONS += MODNAME?=wlan
endif

#By default build for CLD
WLAN_SELECT := CONFIG_QCA_CLD_WLAN=m
KBUILD_OPTIONS += CONFIG_QCA_WIFI_ISOC=0
KBUILD_OPTIONS += CONFIG_QCA_WIFI_2_0=1
KBUILD_OPTIONS += $(WLAN_SELECT)
KBUILD_OPTIONS += $(KBUILD_EXTRA) # Extra config if any
endif

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean $(KBUILD_OPTIONS)

# --- sapphire: LineageOS inline kernel build (TARGET_KERNEL_EXT_MODULES) ---
KBUILD_OPTIONS += KCFLAGS=-DCFG80211_SINGLE_NETDEV_MULTI_LINK_SUPPORT
KBUILD_OPTIONS += CONFIG_CNSS_GENL=m
KBUILD_OPTIONS += WLAN_CTRL_NAME=wlan
KBUILD_OPTIONS += WLAN_ROOT=$(WLAN_ROOT)
KBUILD_OPTIONS += WLAN_COMMON_ROOT=cmn
KBUILD_OPTIONS += WLAN_COMMON_INC=$(WLAN_ROOT)/../qca-wifi-host-cmn
KBUILD_OPTIONS += WLAN_FW_API=$(WLAN_ROOT)/../fw-api
KBUILD_OPTIONS += WLAN_PROFILE=adrastea
KBUILD_OPTIONS += MODNAME=wlan
KBUILD_OPTIONS += DEVNAME=wlan
KBUILD_OPTIONS += CONFIG_QCA_CLD_WLAN=m
KBUILD_OPTIONS += CONFIG_ICNSS2=m
KBUILD_OPTIONS += CONFIG_CNSS_OUT_OF_TREE=y
KBUILD_OPTIONS += WLAN_PLATFORM_INC=$(WLAN_ROOT)/../platform/inc
KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS="$(OUT_DIR)/../sapphire-modules/qcom/opensource/wlan/platform/Module.symvers $(OUT_DIR)/../sapphire-modules/qcom/opensource/datarmnet-ext/wlan/Module.symvers"
