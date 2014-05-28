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

/*
 * Description: This serves as a generic panel driver for the BCMRING LCD
 * controller. User should implement their own panel driver if specific panel
 * operations are required for enabling/disabling the panel
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/broadcom/bcmring_display.h>
#include <mach/csp/cap.h>

static atomic_t driver_is_initialized;

int display_panel_init(struct lcd_panel_operations *panel_ops)
{
   if (!atomic_read(&driver_is_initialized))
      return -ENODEV;

	if (!panel_ops)
		return -EINVAL;

	panel_ops->lcd_panel_enable = NULL;
	panel_ops->lcd_panel_disable = NULL;

	return 0;
}
EXPORT_SYMBOL(display_panel_init);

int display_panel_term(struct lcd_panel_operations *panel_ops)
{
   if (!atomic_read(&driver_is_initialized))
      return -ENODEV;

	if (!panel_ops)
		return -EINVAL;

	panel_ops->lcd_panel_enable = NULL;
	panel_ops->lcd_panel_disable = NULL;

	return 0;
}
EXPORT_SYMBOL(display_panel_term);

static __init int display_init(void)
{
   atomic_set(&driver_is_initialized, 0);

   if (cap_isPresent(CAP_CLCD, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "Display: Not supported\n");
      return -ENODEV;
   }

   atomic_set(&driver_is_initialized, 1);
	return 0;   
}
module_init(display_init);

static __exit void display_exit(void)
{
   atomic_set(&driver_is_initialized, 0);
}
module_exit(display_exit);

MODULE_DESCRIPTION("BCMRING Display Driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");
