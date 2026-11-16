ifeq ($(KP_MODULE_ROOT),)
KP_MODULE_ROOT=$(KERNEL_SRC)/$(M)
endif

KBUILD_OPTIONS+=KBUILD_DTC_INCLUDE=$(KP_MODULE_ROOT)

all: modules # dtbs

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)

# --- sapphire: LineageOS inline kernel build (TARGET_KERNEL_EXT_MODULES) ---
KBUILD_OPTIONS += CONFIG_LOCALVERSION='"-perf"'
KBUILD_OPTIONS += DATAIPA_ROOT=$(KERNEL_SRC)/../sapphire-modules/qcom/opensource/dataipa
