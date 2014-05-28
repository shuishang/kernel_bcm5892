power_supply-objs := power_supply_core.o

ifeq ($(CONFIG_SYSFS),y)
power_supply-objs += power_supply_sysfs.o
endif

ifeq ($(CONFIG_LEDS_TRIGGERS),y)
power_supply-objs += power_supply_leds.o
endif

ifeq ($(CONFIG_POWER_SUPPLY_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

obj-$(CONFIG_POWER_SUPPLY)	+= power_supply.o

obj-$(CONFIG_PDA_POWER)		+= pda_power.o
obj-$(CONFIG_APM_POWER)		+= apm_power.o
obj-$(CONFIG_WM831X_POWER)	+= wm831x_power.o
obj-$(CONFIG_WM8350_POWER)	+= wm8350_power.o

obj-$(CONFIG_BATTERY_DS2760)	+= ds2760_battery.o
obj-$(CONFIG_BATTERY_DS2782)	+= ds2782_battery.o
obj-$(CONFIG_BATTERY_PMU)	+= pmu_battery.o
obj-$(CONFIG_BATTERY_OLPC)	+= olpc_battery.o
obj-$(CONFIG_BATTERY_TOSA)	+= tosa_battery.o
obj-$(CONFIG_BATTERY_WM97XX)	+= wm97xx_battery.o
obj-$(CONFIG_BATTERY_BQ27x00)	+= bq27x00_battery.o
obj-$(CONFIG_BATTERY_DA9030)	+= da9030_battery.o
obj-$(CONFIG_BATTERY_MAX17040)	+= max17040_battery.o
obj-$(CONFIG_CHARGER_PCF50633)	+= pcf50633-charger.o
obj-$(CONFIG_BCM_FAKE_BATTERY_DRIVER) += fake_battery.o

# One of these may be loaded after boot, named as 'cmp_battery.ko'
ifeq ($(CONFIG_BCM_CMP_BATTERY_DRIVER),m)
obj-m += cmp_battery.o
endif

ifeq ($(CONFIG_BCM_CMP_BATTERY_LTC4006),m)
obj-m += cmp_battery.o
cmp_battery-objs += cmp_battery_ltc4006.o
endif

ifeq ($(CONFIG_BCM_CMP_BATTERY_RT9505),m)
obj-m += cmp_battery.o
cmp_battery-objs += cmp_battery_rt9505.o
endif

obj-$(CONFIG_BCM_POWER_MGMT_INIT)	+= pm_init.o
