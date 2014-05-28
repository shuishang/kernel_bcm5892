EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../../drivers/char/broadcom/halaudio_drivers/bcmring/

.PHONY: all

else

obj-$(CONFIG_BCM_HALAUDIO_APM_BCMRING) += apm/
obj-$(CONFIG_BCM_HALAUDIO_I2S_BCMRING) += i2s/
obj-$(CONFIG_BCM_HALAUDIO_PCM_BCMRING) += pcm/

endif
