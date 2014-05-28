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


/* Low-level IRQ helper macros for BCM5892 platform */

#include <mach/hardware.h>
#include <mach/irqs.h>

	
 		.macro	disable_fiq
		.endm

	        .macro  get_irqnr_preamble, base, tmp
                .endm
	
		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
		ldr	\base, =IO_ADDRESS(VIC0_REG_BASE_ADDR)
		ldr	\irqstat, [\base, #VIC_IRQSTATUS]
		teq	\irqstat, #0
		beq	1001f
		mov	\irqnr, #SVIC_IRQ_END		
		clz	\tmp, \irqstat
		sub	\irqnr, \irqnr, \tmp
		b	1003f
1001:		ldr	\base,=IO_ADDRESS(VIC1_REG_BASE_ADDR)
		ldr	\irqstat, [\base, #VIC_IRQSTATUS]
		teq	\irqstat, #0
		beq	1002f
		mov	\irqnr, #OVIC0_IRQ_END		
		clz	\tmp, \irqstat
		sub	\irqnr, \irqnr, \tmp
		b	1003f
1002: 		ldr	\base,=IO_ADDRESS(VIC2_REG_BASE_ADDR)
		ldr	\irqstat, [\base, #VIC_IRQSTATUS]
		teq	\irqstat, #0
		beq	1003f
		mov	\irqnr, #OVIC1_IRQ_END		
		clz	\tmp, \irqstat
		sub	\irqnr, \irqnr, \tmp
1003:
		.endm

                .macro  arch_ret_to_user, tmp1, tmp2
	        .endm
	
