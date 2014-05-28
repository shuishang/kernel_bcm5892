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

#ifndef _BOOTMEMHEAP_H_
#define _BOOTMEMHEAP_H_

void * bootmemheap_alloc(char *memid_str, size_t size);

extern size_t (*bootmemheap_calc_fb_mem)( void );
extern size_t (*bootmemheap_calc_vdec_mem)( void );
extern size_t (*bootmemheap_calc_mmdma)( void );
#endif
