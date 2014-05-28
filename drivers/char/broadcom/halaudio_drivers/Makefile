EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../../drivers/char/broadcom/halaudio_drivers/

.PHONY: all

else

obj-m                                        += bcmring/
obj-$(CONFIG_BCM_HALAUDIO_ALSA)              += alsa/
obj-$(CONFIG_BCM_HALAUDIO_EXTERNAL_AK4642)   += ak4642_i2s/
obj-$(CONFIG_BCM_HALAUDIO_USB)               += usb/

endif
