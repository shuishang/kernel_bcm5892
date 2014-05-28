/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/


#include <linux/version.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <mach/pmb.h>
#include <mach/bcm5892_sw.h>
#include <mach/hardware.h>
#include <mach/regaccess.h>
#include <asm/io.h>
#include <asm/cacheflush.h>

/* From dmu_cpuapi.h */
#define REFCLK		0x0
#define PLLCLK_TAP1	0x1
#define BBLCLK		0x2
#define SPLCLK		0x3
#define CPUCLK		DMU_F_dmu_cpuclk_sel_MASK	
#define CPUCLK_SEL	DMU_F_dmu_cpuclk_sel_R

#define REFCLK_SPEED	(24*1000*1000)
#define BBLCLK_SPEED	(32768)
#define SPLCLK_SPEED	(1)

#define SLEEP_CLK_SPEED	BBLCLK_SPEED

/* VIC0 is the secure VIC. Allow any secure interrupt to wake us up. */
#define VIC0_MASK	(~0)

#define VIC1_USB_OTG	(1 << 26)
#define VIC1_USB_HOST1	(1 << 27)
#define VIC1_USB_UDC	VIC1_USB_HOST1
#define VIC1_USB_HOST2	(1 << 28)
#define VIC1_EMAC	(1 << 29)
#define VIC1_SEC_PEND	(1 << 0)

#define VIC1_MASK	(VIC1_USB_UDC | VIC1_SEC_PEND)

#define VIC2_UART0	(1 << 6)
#define VIC2_UART1	(1 << 7)
#define VIC2_UART2	(1 << 8)
#define VIC2_UART3	(1 << 9)

#define VIC2_MASK	(VIC2_UART0 | VIC2_UART1 | VIC2_UART2 | VIC2_UART3)

#define PREFIX	"BCM5892 PM: "



/*
 * return true if the UDC is a wakeup source, false otherwise.
 */
int bcm5892_pm_udc_wakeup(void)
{
	return VIC1_MASK & VIC1_USB_UDC;
}
EXPORT_SYMBOL(bcm5892_pm_udc_wakeup);

/*
 * return true if the OTG is a wakeup source, false otherwise.
 */
int bcm5892_pm_otg_wakeup(void)
{
	return VIC1_MASK & VIC1_USB_OTG;
}
EXPORT_SYMBOL(bcm5892_pm_otg_wakeup);


