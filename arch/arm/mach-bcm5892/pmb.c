/*****************************************************************************
* Copyright 2009 Broadcom Corporation.  All rights reserved.
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/bcm5892_reg.h>
#include <asm/cacheflush.h>
#include <mach/hardware.h>



/*
 * Wrapper function to call pmb_call_secure_api().
 * This function prepares the mailbox argument, 
 * translates the ARM mode stacks addresses from virtual to physical, 
 * disable the caches and then call the pmb cpuapi.
 */
uint32_t call_secure_api_internal(uint32_t mb_id, uint32_t mb_argc, ...)
{
	printk("call_secure_api_internal: %d\n", mb_id);
 
  	return 0;
}

uint32_t (*call_secure_api)(uint32_t mb_id, uint32_t mb_argc, ...) = &call_secure_api_internal;



EXPORT_SYMBOL(call_secure_api);

