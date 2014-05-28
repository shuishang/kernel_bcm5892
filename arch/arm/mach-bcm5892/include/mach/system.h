/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
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


#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

extern void bcm5892_cpu_reset(void);

static inline void arch_idle(void)
{
}

static inline void arch_reset(char mode, const char *cmd)
{
	printk(KERN_CRIT "Resetting CPU..\n");
	bcm5892_cpu_reset();
}

unsigned int get_arm_revid(void);

#endif
