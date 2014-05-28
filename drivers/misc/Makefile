#
# Makefile for misc devices that really don't fit anywhere else.
#
ifneq (,$(CONFIG_BCM_VCHIQ_VC03))
obj-$(CONFIG_BCM_VCHIQ_VC03)		+= vmcs_services/
obj-$(CONFIG_BCM_VCHIQ_VC03)		+= vmcs_modules/
endif
ifneq (,$(CONFIG_BCM_VCHIQ_VC04))
obj-$(CONFIG_BCM_VCHIQ_VC04)		+= vc04_services/
#obj-$(CONFIG_BCM_VCHIQ_VC04)		+= vc04_modules/
endif
obj-$(CONFIG_BCMBLT_RFKILL)	+= bcmblt-rfkill.o
obj-$(CONFIG_BTHID)		+= bthid.o

obj-$(CONFIG_IBM_ASM)		+= ibmasm/
obj-$(CONFIG_HDPU_FEATURES)	+= hdpuftrs/
obj-$(CONFIG_ATMEL_PWM)		+= atmel_pwm.o
obj-$(CONFIG_ATMEL_SSC)		+= atmel-ssc.o
obj-$(CONFIG_ATMEL_TCLIB)	+= atmel_tclib.o
obj-$(CONFIG_ICS932S401)	+= ics932s401.o
obj-$(CONFIG_LKDTM)		+= lkdtm.o
obj-$(CONFIG_TIFM_CORE)       	+= tifm_core.o
obj-$(CONFIG_TIFM_7XX1)       	+= tifm_7xx1.o
obj-$(CONFIG_PHANTOM)		+= phantom.o
obj-$(CONFIG_SGI_IOC4)		+= ioc4.o
obj-$(CONFIG_ENCLOSURE_SERVICES) += enclosure.o
obj-$(CONFIG_KGDB_TESTS)	+= kgdbts.o
obj-$(CONFIG_SGI_XP)		+= sgi-xp/
obj-$(CONFIG_SGI_GRU)		+= sgi-gru/
obj-$(CONFIG_HP_ILO)		+= hpilo.o
obj-$(CONFIG_ISL29003)		+= isl29003.o
obj-$(CONFIG_EP93XX_PWM)	+= ep93xx_pwm.o
obj-$(CONFIG_C2PORT)		+= c2port/
obj-y				+= eeprom/
obj-y				+= cb710/
