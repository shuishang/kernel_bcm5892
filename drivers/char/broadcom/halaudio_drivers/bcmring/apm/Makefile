EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../../drivers/char/broadcom/halaudio_drivers/bcmring/apm

.PHONY: all

else

obj-$(CONFIG_BCM_HALAUDIO_APM_BCMRING) += halaudio_apm.o

halaudio_apm-objs := apm_drv.o apm_headset.o

endif
