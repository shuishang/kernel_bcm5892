/*
*  Copyright 2009 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
*  http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*
*/
/*
 * bcm589x_dummy.c -- Dummy file created for avoiding modpost error messages
 * ERROR: "usb_otg_gadget_register_driver" 
 *	[drivers/usb/bcm589x-otg/g_bcm589x_otg_zero.ko] undefined!
 * ERROR: "usb_otg_gadget_unregister_driver" 
	[drivers/usb/bcm589x-otg/g_bcm589x_otg_zero.ko] undefined!
 * ...
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <mach/irqs.h>
#include <linux/version.h>

#include <linux/usb/ch9.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
#include <linux/usb_gadget.h>
#else
#include <linux/usb/gadget.h>
#endif

int usb_otg_gadget_unregister_driver(struct usb_gadget_driver *_driver)
{
	return 0;
}
EXPORT_SYMBOL(usb_otg_gadget_unregister_driver);

int usb_otg_gadget_register_driver(struct usb_gadget_driver *_driver)
{
	return 0;
}
EXPORT_SYMBOL(usb_otg_gadget_register_driver);

