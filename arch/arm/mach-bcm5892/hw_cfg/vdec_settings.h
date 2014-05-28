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
 * VDEC configurations
 */

#ifndef VDEC_SETTINGS_H
#define VDEC_SETTINGS_H

#include <linux/broadcom/vdec_cfg.h>

/* Total 16M, 2M is for mmdma, so 14M for this */

#define HW_CFG_VDEC \
{ \
   .memsize = (14 * 1024 * 1024), \
}

#endif /* VDEC_SETTINGS_H */
