EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../../drivers/char/broadcom/bcm_alsa/

.PHONY: all

else

obj-$(CONFIG_BCM_ALSA) += bcm_alsa.o

bcm_alsa-objs := \
	bcm_alsa_dev_multi.o \
	bcm_alsa_pcm_multi.o \
	bcm_alsa_ctl_multi.o

endif
