# Backlight & LCD drivers

obj-$(CONFIG_LCD_CLASS_DEVICE)     += lcd.o
obj-$(CONFIG_LCD_CORGI)		   += corgi_lcd.o
obj-$(CONFIG_LCD_HP700)		   += jornada720_lcd.o
obj-$(CONFIG_LCD_LMS283GF05)	   += lms283gf05.o
obj-$(CONFIG_LCD_LTV350QV)	   += ltv350qv.o
obj-$(CONFIG_LCD_ILI9320)	   += ili9320.o
obj-$(CONFIG_LCD_PLATFORM)	   += platform_lcd.o
obj-$(CONFIG_LCD_VGG2432A4)	   += vgg2432a4.o
obj-$(CONFIG_LCD_TDO24M)	   += tdo24m.o
obj-$(CONFIG_LCD_TOSA)		   += tosa_lcd.o

obj-$(CONFIG_BACKLIGHT_CLASS_DEVICE) += backlight.o
obj-$(CONFIG_BACKLIGHT_ATMEL_PWM)    += atmel-pwm-bl.o
obj-$(CONFIG_BACKLIGHT_GENERIC)	+= generic_bl.o
obj-$(CONFIG_BACKLIGHT_HP700)	+= jornada720_bl.o
obj-$(CONFIG_BACKLIGHT_HP680)	+= hp680_bl.o
obj-$(CONFIG_BACKLIGHT_LOCOMO)	+= locomolcd.o
CFLAGS_vc_backlight.o		+= -I"${BRCM_VIDEOCORE_SERVICES_DIR}"
CFLAGS_vc_backlight.o		+= -I"${BRCM_VIDEOCORE_FIRMWARE_DIR}"
CFLAGS_vc_backlight.o		+= -I"${BRCM_VIDEOCORE_FIRMWARE_DIR}/interface/vchi/os/linux/"
CFLAGS_vc_backlight.o		+= -I"${BRCM_VIDEOCORE_FIRMWARE_DIR}/interface/vmcs_host/linux/"
CFLAGS_vc_backlight.o		+= -I"${BRCM_VIDEOCORE_FIRMWARE_DIR}/interface/vcos/linux_kernel/"
obj-$(CONFIG_BACKLIGHT_VC)      += vc_backlight.o
obj-$(CONFIG_BACKLIGHT_OMAP1)	+= omap1_bl.o
obj-$(CONFIG_BACKLIGHT_PROGEAR) += progear_bl.o
obj-$(CONFIG_BACKLIGHT_CARILLO_RANCH) += cr_bllcd.o
obj-$(CONFIG_BACKLIGHT_PWM)	+= pwm_bl.o
obj-$(CONFIG_BACKLIGHT_DA903X)	+= da903x_bl.o
obj-$(CONFIG_BACKLIGHT_MBP_NVIDIA) += mbp_nvidia_bl.o
obj-$(CONFIG_BACKLIGHT_TOSA)	+= tosa_bl.o
obj-$(CONFIG_BACKLIGHT_SAHARA)	+= kb3886_bl.o
obj-$(CONFIG_BACKLIGHT_LCDFB) += lcdfb_backlight.o
obj-$(CONFIG_BACKLIGHT_WM831X)	+= wm831x_bl.o
obj-$(CONFIG_BACKLIGHT_ADX)    += adx_bl.o
obj-$(CONFIG_BACKLIGHT_ADP5520)	+= adp5520_bl.o

