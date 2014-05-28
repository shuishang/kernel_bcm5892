/*****************************************************************************
*  Copyright 2001 - 2009 Broadcom Corporation.  All rights reserved.
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
*****************************************************************************/

/*****************************************************************************
*
*  PURPOSE:
*  A simple API for getting the kernel PHYS_OFFSET value. This is used by
*  modules that are loading other processor images and need to ensure that they
*  are not trampling the kernel. If the module is inserted into an old kernel
*  that does not have this function then the insmod will fail as we are not
*  guaranteeing backward compatibility between modules and kernel versions.
*
*  NOTES:
*
*****************************************************************************/
#ifndef _GETPHYSOFFSET_H_
#define _GETPHYSOFFSET_H_

const void * getphysoffset(void);

#endif
