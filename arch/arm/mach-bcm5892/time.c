/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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


#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/broadcom/timer.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/time.h>
#include <asm/leds.h>
#include <asm/io.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/in.h>
#include <linux/ctype.h>
#include <linux/net.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include <linux/inet.h>

/*
 * TICKS_PER_uSEC controls how often interrupts happen.
 * CLOCK_TICK_RATE controls the actual speed of 'time'.
 */

#if defined(CONFIG_ARCH_BCM5892)

#define TIMER_CONTROL_OFFSET		TIM0_R_TIM0_Timer1Control_SEL
#define TIMER_VALUE_OFFSET		TIM0_R_TIM0_Timer1Value_SEL
#define TIMER_LOAD_OFFSET		TIM0_R_TIM0_Timer1Load_SEL
#define TIMER_INTCLR_OFFSET		TIM0_R_TIM0_Timer1IntClr_SEL
#define TIMER_CTRL_ONESHOTMODE		(1 << 0)
#define TIMER_CTRL_DIV1			(0 << 2)
#define TIMER_CTRL_PREBY16		(1 << 2)
#define TIMER_CTRL_PREBY256		(2 << 2)
#define TIMER_CTRL_CLK2			(1 << 9)


/*
 * Indexes into timers_map[]
 */
#define CLOCKEVENT_TIMER		2
#define CLOCKSOURCE_TIMER		3

#endif

#ifndef SYS_TIMER_FREQ_IN_KHZ
#define SYS_TIMER_FREQ_IN_KHZ		(TICKS_PER_uSEC * 1000)
#endif

#define TIMER_INTERVAL	(TICKS_PER_uSEC * mSEC_10)
#if (TIMER_INTERVAL >= 0x100000)
#define TIMER_RELOAD	(TIMER_INTERVAL >> 8)
#define TIMER_DIVISOR	(TIMER_CTRL_PREBY256)
#elif (TIMER_INTERVAL >= 0x10000)
#define TIMER_RELOAD	(TIMER_INTERVAL >> 4)		/* Divide by 16 */
#define TIMER_DIVISOR	(TIMER_CTRL_PREBY16)
#else
#define TIMER_RELOAD	(TIMER_INTERVAL)
#define TIMER_DIVISOR	(TIMER_CTRL_DIV1)
#endif

typedef struct {
    uint8_t	vic;		/* VIC controller */
    uint32_t	base;		/* timer base address */
    uint32_t	vec;		/* interrupt vector */
} timer_map_t;

/* Mapping from logical timer to physical timer */
static const timer_map_t timers_map[] = {
#if defined(CONFIG_ARCH_BCM5892)
    /* VIC0 timers */
    {0, TIM0_REG_BASE_ADDR + 0x00, IRQ_OTIMER0},	/* open or secure */
    {0, TIM0_REG_BASE_ADDR + 0x20, IRQ_OTIMER1},	/* open or secure */
    {0, TIM1_REG_BASE_ADDR + 0x00, IRQ_OTIMER2},	/* open only */
    {0, TIM1_REG_BASE_ADDR + 0x20, IRQ_OTIMER3},	/* open only */
    /* VIC1 timers */
    {1, TIM2_REG_BASE_ADDR + 0x00, IRQ_OTIMER4},	/* open only */
    {1, TIM2_REG_BASE_ADDR + 0x20, IRQ_OTIMER5},	/* open only */
    {1, TIM3_REG_BASE_ADDR + 0x00, IRQ_OTIMER6},	/* open only */
    {1, TIM3_REG_BASE_ADDR + 0x20, IRQ_OTIMER7},	/* open only */
#endif
#if defined(CONFIG_ARCH_BCM476X)
    /* VIC0 timers */
    {0,0,0},
    {0,0,BCM4760_INTR_TIM0_CNTR1},
    {0,1,0},
    /* VIC1 timers */
    {1,1,1},
    {1,0,2},
    {1,1,2},
    {1,0,3},
    {1,1,3},
#endif
};

static unsigned int tick_count = 0;

static int __init hw_clocksource_init(void);
static int hw_timer0_set_next_event(unsigned long evt,struct clock_event_device *unused);
static void hw_timer0_set_mode(enum clock_event_mode mode, struct clock_event_device *evt);


/* Returns registers' base address of the time. */
uint32_t hw_timer_base(int32_t timer_id)
{
	return timers_map[timer_id].base;
}

/* Returns interrupt vector of the timer. */
uint32_t hw_timer_vec(int32_t timer_id)
{
	return timers_map[timer_id].vec;
}

void hw_timer_enable(int timer_id)
{
	uint32_t timer_base, timer_ctrl;

	timer_base = hw_timer_base(timer_id);
	timer_ctrl = readl(IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
	timer_ctrl |= TIMER_CTRL_EN;
	writel(timer_ctrl, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
}

void hw_timer_disable(int timer_id)
{
	uint32_t timer_base, timer_ctrl;

	timer_base = hw_timer_base(timer_id);
	timer_ctrl = readl(IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
	timer_ctrl &= ~TIMER_CTRL_EN;
	writel(timer_ctrl, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
	/* Clear pending interrupt */
	writel(1, IO_ADDRESS(timer_base + TIMER_INTCLR_OFFSET));
}

/* Returns timer's counter */
uint32_t hw_timer_get_counter(int timer_id)
{
	uint32_t counter_addr;

	counter_addr = hw_timer_base(timer_id) + TIMER_VALUE_OFFSET;
	return readl(IO_ADDRESS(counter_addr));
}

static cycle_t hw_get_cycles(void)
{
	return ~hw_timer_get_counter(CLOCKSOURCE_TIMER);
}

#ifdef CONFIG_PM
static void hw_clocksource_resume(void)
{
	uint32_t timer_base;
	uint32_t config;

	timer_base = hw_timer_base(CLOCKSOURCE_TIMER);
	/* Disable the timer */
	writel(0, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));

	/* Enable timer */
	writel(0xFFFFFFFFU, IO_ADDRESS(timer_base + TIMER_VALUE_OFFSET));
	writel(0xFFFFFFFFU, IO_ADDRESS(timer_base + TIMER_LOAD_OFFSET));

	/* use same divisor as clockevent driver */
	config = TIMER_CTRL_PERIODIC | TIMER_CTRL_32BIT | TIMER_DIVISOR;
	writel(config, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
	writel(config | TIMER_CTRL_CLK2, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
	/* Enable timer */
	writel(config | TIMER_CTRL_CLK2 | TIMER_CTRL_EN, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
}
#endif

static struct clocksource clocksource_ddi0271 = {
	.name       = "hw_source_timer",
	.rating     = 250,
	.read       = (cycle_t (*)(void))hw_get_cycles,
#ifdef CONFIG_PM
	.resume     = (void (*)(void))hw_clocksource_resume,
#endif
	.mask       = CLOCKSOURCE_MASK(32),
	.shift      = 20,
	.flags      = CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init hw_clocksource_init(void)
{
	clocksource_ddi0271.mult =
		clocksource_hz2mult(CLOCK_TICK_RATE, clocksource_ddi0271.shift);
#ifdef CONFIG_PM
	hw_clocksource_resume();
#endif
	clocksource_register(&clocksource_ddi0271);
	return 0;
}

static struct clock_event_device clockevent_ddi0271 = {
	.name       = "hw_event_timer0",
	.features   = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.shift      = 32,
	.set_mode   = hw_timer0_set_mode,
	.set_next_event = hw_timer0_set_next_event,
	.rating     = 300,
};

static int32_t hw_set_next_event(int timer_id, uint32_t evt,struct clock_event_device *unused)
{
	uint32_t load_addr;

	load_addr = hw_timer_base(timer_id) + TIMER_LOAD_OFFSET;
	writel(evt, IO_ADDRESS(load_addr));
	return 0;
}

static int hw_timer0_set_next_event(unsigned long evt, struct clock_event_device *unused)
{
	return hw_set_next_event(CLOCKEVENT_TIMER, (uint32_t)evt, unused);
}

static void hw_timer_set_mode(int timer_id, enum clock_event_mode mode, struct clock_event_device *evt)
{
	unsigned long flags;
	uint32_t timer_base;

	timer_base = hw_timer_base(timer_id);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		printk(KERN_ERR "timer%d: periodic %d\n", timer_id, TIMER_RELOAD);
		local_irq_save(flags);
		writel(TIMER_RELOAD, IO_ADDRESS(timer_base + TIMER_LOAD_OFFSET));
		writel(TIMER_DIVISOR | TIMER_CTRL_CLK2 | TIMER_CTRL_PERIODIC | TIMER_CTRL_EN | TIMER_CTRL_IE, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
		local_irq_restore(flags);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
/*		printk(KERN_ERR "timer%d: oneshot\n", timer_id); */
		local_irq_save(flags);
		writel(TIMER_CTRL_PREBY16 | TIMER_CTRL_CLK2 | TIMER_CTRL_ONESHOTMODE | TIMER_CTRL_EN | TIMER_CTRL_IE, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
		local_irq_restore(flags);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
/*		printk(KERN_ERR "timer%d: shutdown\n", timer_id); */
		break;
	case CLOCK_EVT_MODE_RESUME:
/*		printk(KERN_ERR "timer%d: resume\n", timer_id); */
#if 0
		local_irq_save(flags);
		writel(TIMER_RELOAD, IO_ADDRESS(timer_base + TIMER_LOAD_OFFSET));
		writel(TIMER_DIVISOR | TIMER_CTRL_CLK2 | TIMER_CTRL_PERIODIC | TIMER_CTRL_EN | TIMER_CTRL_IE, IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET));
		local_irq_restore(flags);
#endif
		break;
	case CLOCK_EVT_MODE_UNUSED:
/*		printk(KERN_ERR "hw_timer_set_mode: mode %d is not supported for DDI0271\n", mode); */
		break;
	}
}

static void hw_timer0_set_mode(enum clock_event_mode mode, struct clock_event_device *evt)
{
	hw_timer_set_mode(CLOCKEVENT_TIMER, mode, evt);
}

static int hw_clockevent_init(void)
{
	clockevent_ddi0271.irq = hw_timer_vec(CLOCKEVENT_TIMER);
	clockevent_ddi0271.mult = div_sc(CLOCK_TICK_RATE, NSEC_PER_SEC, clockevent_ddi0271.shift);
	clockevent_ddi0271.max_delta_ns = clockevent_delta2ns(0xFFFFFFFF, &clockevent_ddi0271);
	clockevent_ddi0271.min_delta_ns = clockevent_delta2ns(0x0000000F, &clockevent_ddi0271);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,27)
    clockevent_ddi0271.cpumask = cpumask_of(0);
#else
	clockevent_ddi0271.cpumask = cpumask_of_cpu(0);
#endif
	clockevents_register_device(&clockevent_ddi0271);
	return 0;
}

/*
 * IRQ handler for the timer
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t hw_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t hw_timer_interrupt(int irq, void *dev_id)
#endif
{
	struct clock_event_device *evt = &clockevent_ddi0271;
	int timer_id = (int) dev_id;
	uint32_t base_addr = hw_timer_base(timer_id);

	tick_count++;

	/*
	 * clear the interrupt
	 */

	writel(1, IO_ADDRESS(base_addr + TIMER_INTCLR_OFFSET));

	if (evt->event_handler)
		evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct irqaction hw_timer0_irq =
{
	.name		= "DDI0271 Timer Tick",
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	.flags		= SA_INTERRUPT | SA_TIMER,
#else
	.flags		= IRQF_DISABLED | IRQF_TIMER,
#endif
	.handler	= hw_timer_interrupt,
	.dev_id		= (void *) CLOCKEVENT_TIMER
};

/* Called from arch.c to initialize the timer driver */
void __init sys_timer_init(void)
{
	int n;
	uint32_t vec;

	vec = hw_timer_vec(CLOCKEVENT_TIMER);
	if (vec)
		setup_irq(vec, &hw_timer0_irq);
	for (n = 0; n < sizeof(timers_map)/sizeof(timers_map[0]); n++) {
        	hw_timer_disable(n);
	}
	hw_clocksource_init();
	hw_clockevent_init();
}

void sys_timer_resume(void)
{
	hw_clocksource_init();
	hw_clockevent_init();
}

/*
 * Return the current time in seconds.
 */

timer_tick_count_t timer_get_tick_count(void)
{
	return ((tick_count * SYS_TIMER_FREQ_IN_KHZ));
}
EXPORT_SYMBOL(timer_get_tick_count);

timer_tick_rate_t timer_get_tick_rate(void)
{
    return SYS_TIMER_FREQ_IN_KHZ;
}
EXPORT_SYMBOL(timer_get_tick_rate);

timer_msec_t timer_ticks_to_msec(timer_tick_count_t ticks)
{
	timer_tick_rate_t tickrate;
	timer_msec_t msec;

	tickrate = timer_get_tick_rate();

	msec = (ticks * 1000)/tickrate;
	return msec;
}
EXPORT_SYMBOL(timer_ticks_to_msec);

timer_msec_t timer_get_msec(void)
{
	return timer_ticks_to_msec(timer_get_tick_count());
}
EXPORT_SYMBOL(timer_get_msec);

EXPORT_SYMBOL(hw_timer_base);
EXPORT_SYMBOL(hw_timer_enable);
EXPORT_SYMBOL(hw_timer_disable);
EXPORT_SYMBOL(hw_timer_get_counter);
