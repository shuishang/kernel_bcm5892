#
# Makefile which builds Broadcom drivers
#

EXTRA_CFLAGS := -Werror

#obj-y += spi_load.o
obj-$(CONFIG_BCM_HAVE_PROC_GPIO) += proc_gpio.o

obj-$(CONFIG_BCM_PERFCNT_SUPPORT) += perfcnt.o
obj-$(CONFIG_BCM_PERFTEST_SUPPORT) += mymemcpy.o _mymemcpy.o perftest.o
obj-$(CONFIG_BCM_BUS_ARBITER) += hpm.o
obj-$(CONFIG_BCM_KNLLOG_SUPPORT) += knllog.o dump-mem.o

obj-$(CONFIG_BCM_POWER_MANAGER)         += PowerManager.o
obj-$(CONFIG_BCM_IDLE_PROFILER_SUPPORT)                         += idle_profiler.o
obj-$(CONFIG_BCM_FRAME_PROFILER_SUPPORT)                         += frame_profiler.o
obj-$(CONFIG_BCM_EPTDRIVER_SUPPORT)             += sock.o
obj-$(CONFIG_BCM_PMU)                     += pmu_chip.o
obj-$(CONFIG_BCM_PMU_BCM59001)            += pmu_bcm59001/
obj-$(CONFIG_BCM_PMU_BCM59035_B0)         += pmu_bcm59035b0/
obj-$(CONFIG_BCM_PMU_PCF506XX)            += pmu_pcf506xx/
obj-$(CONFIG_BCM_HALAUDIO)                += halaudio/ halaudio_drivers/
obj-$(CONFIG_BCM_ALSA)                    += bcm_alsa/
obj-$(CONFIG_BCM_AMXR)                    += amxr_drv.o
obj-$(CONFIG_BCM_DEBUG_PAUSE)             += debug_pause.o
obj-$(CONFIG_BCM_OTP)                     += otp.o
obj-$(CONFIG_BCM_GPIO_DRIVER)		  += user-gpio.o
obj-$(CONFIG_BCM_4325_DRIVER)		  += bcm4325.o
obj-$(CONFIG_BCM_GVR)            += gvr/

obj-$(CONFIG_BCM_GIST)                  += gist.o
#obj-$(CONFIG_BCM_GIST)                  += gist_test.o

obj-$(CONFIG_BCM_SIGMORPH)              += sigmorph.o
#obj-$(CONFIG_BCM_SIGMORPH)              += sigmorph-test.o

obj-$(CONFIG_BCM_CSX)                   += csx/
obj-$(CONFIG_BCM_CSX_GIST)              += csx/
obj-$(CONFIG_BCM_CSX_HALAUDIO)          += csx/
obj-$(CONFIG_BCM_CSX_SIGMORPH)          += csx/

obj-$(CONFIG_BCM_BSC)                   += bsc.o
#obj-y += trample-test.o

obj-$(CONFIG_BCM_VCP)                   += vcp/

obj-$(CONFIG_BCM_SIGNAL_TEST) += signal-test/

