/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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
 * Description: Public header of the BCMRING framebuffer driver
 */

#ifndef _BCMRING_FB_H_
#define _BCMRING_FB_H_

#include <linux/ioctl.h>
#include <linux/fb.h>
#include <linux/broadcom/bcmring_lcd.h>

#define BCM_FB_MAGIC               'B'
#define BCMRING_FB_PALETTE_SIZE    16

/*
 * BCMRING framebuffer data structure
 */
struct bcmring_fb {
   /* Linux framebuffer info */
   struct fb_info fb_info;
   /* LCD panel info */
   struct lcd_panel_param panel;
   /* LCD buffer info */
   struct lcd_buf_param buf;
   /* color palette */
   uint32_t palette[BCMRING_FB_PALETTE_SIZE];
};

#endif /* _BCMRING_FB_H_ */
