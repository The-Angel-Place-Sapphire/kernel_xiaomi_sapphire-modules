M=$(PWD)
SSG_MODULE_ROOT=$(KERNEL_SRC)/$(M)
INC=-I/$(M)/linux/*
KBUILD_OPTIONS+=SSG_MODULE_ROOT=$(SSG_MODULE_ROOT)

all: modules

clean:
	rm -f *.cmd *.d *.mod *.o *.ko *.mod.c *.mod.o Module.symvers modules.order

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(INC) $@ $(KBUILD_OPTIONS)

# --- sapphire: LineageOS inline kernel build (TARGET_KERNEL_EXT_MODULES) ---
KBUILD_OPTIONS += CONFIG_DSQB=y
