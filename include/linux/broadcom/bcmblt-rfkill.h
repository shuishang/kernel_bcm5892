/*****************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
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
 * Broadcom Bluetooth rfkill power control via GPIO
 *
 */

#ifndef _LINUX_BCMBLT_RFKILL_H
#define _LINUX_BCMBLT_RFKILL_H

#include <linux/rfkill.h>

struct bcmblt_rfkill_platform_data {
	int gpio;

	struct rfkill *rfkill;  /* for driver only */
};

#endif
