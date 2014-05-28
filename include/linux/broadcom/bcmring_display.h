/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
 * Description: Header of the BCMRING Display (panel) driver.
 */ 

#ifndef _BCMRING_DISPLAY_H
#define _BCMRING_DISPLAY_H

#ifdef __KERNEL__

#include <linux/broadcom/bcmring_lcd.h>

/*
 *	Call to initialize the panel and register the callbacks
 */
extern int display_panel_init(struct lcd_panel_operations *panel_ops);

/*
 *	Call to terminate the panel and unregister the callbacks
 */
extern int display_panel_term(struct lcd_panel_operations *panel_ops);

#endif /* __KERNEL__ */

#endif /* _BCMRING_DISPLAY_H */
