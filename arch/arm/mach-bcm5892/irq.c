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

#include <linux/init.h>
#include <linux/stddef.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/interrupt.h>

#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/pmb.h>
#include <asm/cacheflush.h>


/*
 * Interrupt Controller initialization
 * 
 * BCM5892 has 3 vectored interrupt controllers, viz. vic0,vic1 and vic2.
 * vic0 is a secur interrupt controller and vic1 and vic2 are open interrupt controllers.
 */
void bcm5892_clear_intr(unsigned int);

static void svic_mask_irq(unsigned int irq)
{
	irq -= SVIC_IRQ_START;
	writel (1 <<irq, IO_ADDRESS(VIC0_REG_BASE_ADDR + VIC_INTENCLEAR));
}

static void svic_unmask_irq(unsigned int irq)
{
	irq -= SVIC_IRQ_START;
	writel (1 <<irq, IO_ADDRESS(VIC0_REG_BASE_ADDR + VIC_INTENABLE));
}

static void ovic0_mask_irq(unsigned int irq)
{
	irq -= OVIC0_IRQ_START;
	writel (1 <<irq, IO_ADDRESS(VIC1_REG_BASE_ADDR + VIC_INTENCLEAR));
}

static void ovic0_unmask_irq(unsigned int irq)
{
	irq -= OVIC0_IRQ_START;
	writel (1 <<irq, IO_ADDRESS(VIC1_REG_BASE_ADDR + VIC_INTENABLE));
}

static void ovic1_mask_irq(unsigned int irq)
{
	irq -= OVIC1_IRQ_START;
	writel (1 <<irq, IO_ADDRESS(VIC2_REG_BASE_ADDR + VIC_INTENCLEAR));
}

static void ovic1_unmask_irq(unsigned int irq)
{
	irq -= OVIC1_IRQ_START;
	writel (1 <<irq, IO_ADDRESS(VIC2_REG_BASE_ADDR + VIC_INTENABLE));
}

static struct irq_chip bcm5892_sec_vic_chip= {
  	.ack  =  svic_mask_irq,
	.mask =  svic_mask_irq,
	.unmask= svic_unmask_irq,
};

static struct irq_chip bcm5892_open_vic0_chip= {
  	.ack  =  ovic0_mask_irq,
	.mask =  ovic0_mask_irq,
	.unmask= ovic0_unmask_irq,
};

static struct irq_chip bcm5892_open_vic1_chip= {
  	.ack  =  ovic1_mask_irq,
	.mask =  ovic1_mask_irq,
	.unmask= ovic1_unmask_irq,
};

#if 0
#define RTC_INT_MASK		0x00000003

extern void do_rtc_pending(uint32_t);
/* This is the common secure irq handler */
irqreturn_t pend_sec_irq_handler(int irq, void *dev_id) 
{
	uint32_t status;
	int ret;

        dmac_flush_range(&status,(&status+4));
	ret = call_secure_api(CLS_SECURE_INT_ID, 1, virt_to_phys(&status));
        dmac_flush_range(&status,(&status+4));

	if (ret) {
		printk(KERN_DEBUG"%s(): secure mode operation failed\n", 
		       __func__);
		return IRQ_NONE;
	}
	
	/* do any tasks for each secure device */
	if (status & RTC_INT_MASK) 
		do_rtc_pending(status);

	return IRQ_HANDLED;
}
#endif

void __init bcm5892_init_irq(void)
{
	volatile unsigned int i = 0;

	writel (0x0, IO_ADDRESS(VIC0_REG_BASE_ADDR + VIC_INTSELECT));
	writel (0x0, IO_ADDRESS(VIC1_REG_BASE_ADDR + VIC_INTSELECT));
	writel (0x0, IO_ADDRESS(VIC2_REG_BASE_ADDR + VIC_INTSELECT));

	/*
	 * Disable  all interrupts
	 */
	writel (0xffffffff,
	       IO_ADDRESS(VIC0_REG_BASE_ADDR + VIC_INTENCLEAR));
	writel (0xffffffff,
	       IO_ADDRESS(VIC1_REG_BASE_ADDR + VIC_INTENCLEAR));
	writel (0xffffffff,
	       IO_ADDRESS(VIC2_REG_BASE_ADDR + VIC_INTENCLEAR));

	for (i = SVIC_IRQ_START; i <= SVIC_IRQ_END; i++) {
		set_irq_chip (i,&bcm5892_sec_vic_chip);
		set_irq_handler (i,handle_level_irq);
		set_irq_flags (i,IRQF_VALID | IRQF_PROBE);
	}	

	for (i = OVIC0_IRQ_START; i <= OVIC0_IRQ_END; i++) {
		set_irq_chip (i, &bcm5892_open_vic0_chip);
		set_irq_handler (i, handle_level_irq);
		set_irq_flags (i, IRQF_VALID | IRQF_PROBE);
	}	

	for (i = OVIC1_IRQ_START; i <= OVIC1_IRQ_END; i++) {
		set_irq_chip (i, &bcm5892_open_vic1_chip);
		set_irq_handler (i, handle_level_irq);
		set_irq_flags (i, IRQF_VALID | IRQF_PROBE);
	}	


	/* 
	 * RNG interrupt is cleared, noisy it is.
	 */
	writel (0x0, IO_ADDRESS(RNG_R_RNG_CTRL_MEMADDR)); 
	writel (0x1, IO_ADDRESS(RNG_R_RNG_INT_MASK_MEMADDR));
	writel (0x1, IO_ADDRESS(RNG_R_RNG_CTRL_MEMADDR)); 
}
