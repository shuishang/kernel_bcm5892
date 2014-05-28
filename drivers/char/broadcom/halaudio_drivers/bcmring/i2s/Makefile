EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../../drivers/char/broadcom/halaudio_drivers/bcmring/i2s

.PHONY: all

else

obj-$(CONFIG_BCM_HALAUDIO_I2S_BCMRING) += halaudio_i2s.o 

endif
