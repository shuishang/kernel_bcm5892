#
# Makefile which builds BCM116x drivers
#

#
# IMPORTANT: The order of these is important.
#
#	PMU requires that i2c is initialized first. Since the is2 init is done
#	using subsys_init (see include/linux/init.h) it will be initialized
#	before anything using module_init.
#
#	Withint this directory, the order that objects are added to obj-y
#	will determine the order that the init functions are called.
#
EXTRA_CFLAGS := -Werror
EXTRA_CFLAGS += -I$(LINUX_DIR)/include/linux/broadcom/omx
EXTRA_CFLAGS += -I$(LINUX_DIR)/include/linux/broadcom/vc03

obj-y               += dma.o
obj-y                                  += gpio.o
obj-$(CONFIG_BCM_BATTERY_MANAGER)       += pm_battmgr.o

obj-$(CONFIG_BCM_POWER_MANAGER)		+= pm_platform.o

#
# Common BCM91161 platform support functions
#
obj-$(CONFIG_BCM_POWER_MANAGER)		+= pm_bcm91161_common.o

obj-$(CONFIG_BCM116X_KEYPAD) 		+= keypad.o
obj-$(CONFIG_BCM116X_LCD) 		+= lcd.o
obj-$(CONFIG_BCM116X_LCD_SPLASH)	+= splash.o
obj-$(CONFIG_BCM116X_AUXADC) 		+= auxadc.o

obj-$(CONFIG_BCM_LCD_BACKLIGHT)         += lcd_backlight.o
obj-$(CONFIG_BCM_LCD_BTG222432)         += lcd_BTG222432.o lcd_common.o

#
# If the kernel make is called from the top level make, then CONVBMP will
# already be defined. If we call the kernel make directly, then it won't. So
# we define it for this case
#

CONVBMP ?= $(TOPDIR)/../bin/convbmp

%.c: %.bmp
	@echo Converting $< ...
	@$(CONVBMP) $(CONVBMP_FLAGS) -o $@ $<
