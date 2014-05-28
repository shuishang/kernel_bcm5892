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


/*  BCM5892 platform header file. */

#ifndef __ASM_ARCH_PLATFORM_H
#define __ASM_ARCH_PLATFORM_H

/* Most of the register definitions are here */
#include "bcm5892_reg.h"

/*
* Memory definitions
*/
#define BCM5892_DRAM_BASE	0x40000000
/*#define BCM5892_DRAM_SIZE	SZ_512M */
#define BCM5892_DRAM_SIZE	SZ_32M
#define BCM5892_SCRATCH_BASE	START_SCRATCH
#define BCM5892_SCRATCH_SIZE	SZ_128K
#define BCM5892_SRAM_BASE	0x38000000
#define BCM5892_PERIPH_BASE	0x00080000
#define BCM5892_DMA_BASE	0x00083000

/* 
*  NOR Flash
*/
#define BCM28XX_NOR_BASE	START_NOR_FLASH0   /* NOR flash address base */ 

/*
 * NAND Flash Controller
 */
/*#define BCM5892_NAND_BASE       0x????????   NAND flash controller base */

/*
 * Processor Mode Block
 */
#define BCM5892_PMB_BASE	START_SYS_CFG

/* 
 * Interrupt Controller 
 */
#define VIC_IRQSTATUS		0x00
#define VIC_INTSELECT		0x00c
#define VIC_INTENABLE		0x010
#define VIC_INTENCLEAR		0x014

/* 
* System Timer 
*/
#define TICKS_PER_uSEC		(24/4)	/* REFCLK / 4 */
/*
 *#define TICKS_PER_uSEC	10	on SRAM with FPGA
 *#define TICKS_PER_uSEC	5	on DDR with FPGA
 */

/* 
 * Watchdog offsets 
 */
#define WDOG_LOAD	0x00
#define WDOG_VAL	0x04
#define WDOG_CTL	0x08
#define WDOG_INTCLR	0x0c
#define WDOG_MIS	0x14

/*
 * These are useconds NOT ticks.
 *
 */
#define mSEC_1			1000
#define mSEC_5			(mSEC_1 * 5)
#define mSEC_10			(mSEC_1 * 10)
#define mSEC_25			(mSEC_1 * 25)
#define SEC_1			(mSEC_1 * 1000)

/*
 * Timer Control Register Bits
 */
#define TIMER_CTRL_16BIT	(0 << 1)	/* 16-bit counter mode */
#define TIMER_CTRL_32BIT	(1 << 1)	/* 32-bit counter mode */
#define TIMER_CTRL_IE		(1 << 5)	/* Interrupt enable */
#define TIMER_CTRL_PERIODIC	(1 << 6)	/* Periodic mode */
#define TIMER_CTRL_EN		(1 << 7)	/* Timer enable */		

/* 
 * UART REF clock - fixed at 24Mhz in BCM5892
 */
#define REFCLK_FREQ		24000000	
#define BCM5892_WDOG_CLK      (REFCLK_FREQ/4)

/* BBL RTC */
#define RTC_PER_125ms		0x00000001
#define RTC_PER_250ms		0x00000002
#define RTC_PER_500ms		0x00000004
#define RTC_PER_1SEC		0x00000008
#define RTC_PER_2SECS		0x00000010
#define RTC_PER_4SECS		0x00000020
#define RTC_PER_8SECS		0x00000040
#define RTC_PER_16SECS		0x00000080
#define RTC_PER_32SECS		0x00000100
#define RTC_PER_64SECS		0x00000200
#define RTC_PER_128SECS		0x00000400
#define RTC_PER_256SECS		0x00000800

/* D1W reg offsets */
#define D1W_DIN	0x0
#define D1W_DOUT	0x4
#define D1W_ADDR	0x8
#define D1W_CTL	0xC

#define DMU_SWRST	0

#endif /* __ASM_ARCH_PLATFORM_H */  

