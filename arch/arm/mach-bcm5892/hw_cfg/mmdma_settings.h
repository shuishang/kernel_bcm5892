/*****************************************************************************
* Copyright 2006 - 2010 Broadcom Corporation.  All rights reserved.
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

#ifndef _MMDMA_SETTINGS_H_
#define _MMDMA_SETTINGS_H_

/*
 * Memory in chucks to be allocated in MMDMA for Multimedia usage
 *
 * NOTE:
 * 1. Size needs to be multiple of page size (4096 bytes in our case)
 * 1. Sizes should be listed in ascending order for most effective usage
 */

#define HW_CFG_MMDMA_MEM_TABLE \
{ \
   256 * 4096, /* Gstreamer video input buffer, 1M */ \
   256 * 4096, /* Gstreamer scratch buffer, 1M */ \
}

#endif
