#
# USB peripheral controller drivers
#
#ifeq ($(CONFIG_USB_GADGET_DEBUG),y)
#	EXTRA_CFLAGS		+= -DDEBUG
#endif


#ifeq ($(CONFIG_USB_GADGET_DWC_OTG),y)
EXTRA_CFLAGS                    += -DDEBUG -O0
#endif

obj-$(CONFIG_USB_BCM589X_OTG)   := bcm589x_dummy.o
#bcm589x_otg-objs           += bcm589x_otg_driver.o bcm589x_otg_attr.o
#bcm589x_otg-objs           += bcm589x_otg_cil.o bcm589x_otg_cil_intr.o
#bcm589x_otg-objs           += bcm589x_otg_pcd.o bcm589x_otg_pcd_intr.o
#bcm589x_otg-objs += bcm589x_otg_hcd.o bcm589x_otg_hcd_intr.o bcm589x_otg_hcd_queue.o



#
# USB gadget drivers
#
g_bcm589x_otg_zero-objs		:= zero.o usbstring.o config.o epautoconf.o
g_bcm589x_otg_serial-objs	:= serial.o usbstring.o config.o epautoconf.o
g_bcm589x_otg_midi-objs		:= gmidi.o usbstring.o config.o epautoconf.o
g_bcm589x_otg_file_storage-objs	:= file_storage.o usbstring.o config.o \
					epautoconf.o

#ifeq ($(CONFIG_USB_ETH_RNDIS),y)
#	g_ether-objs		+= rndis.o
#endif
 
obj-$(CONFIG_USB_BCM589X_ZERO)		+= g_bcm589x_otg_zero.o
obj-$(CONFIG_USB_BCM589X_FILE_STORAGE)	+= g_bcm589x_otg_file_storage.o
obj-$(CONFIG_USB_BCM589X_G_SERIAL)	+= g_bcm589x_otg_serial.o
obj-$(CONFIG_USB_BCM589X_MIDI_GADGET)	+= g_bcm589x_otg_midi.o

