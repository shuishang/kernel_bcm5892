/*****************************************************************************
* Copyright 2003 - 2008 Broadcom Corporation.  All rights reserved.
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
*
*****************************************************************************
*
*  sysctl.h
*
*  PURPOSE:
*
*     This file contains Broadcom specific definitions for the sysctl call
*
*  NOTES:
*
*****************************************************************************/


#if !defined( BCM_SYSCTL_H )
#define BCM_SYSCTL_H

/* ---- Include Files ---------------------------------------------------- */


/* ---- Constants and Types ---------------------------------------------- */

/*
 * The following need to be unique (see include/linux/sysctl.h)
 */

#define CTL_BCM_LCD                    100
#define CTL_BCM_PMU                    101
#define CTL_BCM_EPT                    102
#define CTL_BCM_PM                     104
#define CTL_BCM_GPIO                   105
#define CTL_BCM_DMA                    106
#define CTL_BCM_VC                     108 /* VC02 */
#define CTL_BCM_USB                    109
#define CTL_BCM_MAX3301                110 /* USB Transceiver */
#define CTL_BCM_USB_I2C                111
#define CTL_BCM_REBOOT                 112
#define CTL_BCM_KNLLOG                 113
#define CTL_BCM_ICP                    114
#define CTL_BCM_I2S                    115
#define CTL_BCM_INTCODEC               116
#define CTL_BCM_IODUMP                 117
#define CTL_BCM_I2C                    118
#define CTL_BCM_MIXER                  119
#define CTL_BCM_HALAUDIO               120
#define CTL_BCM_TSC                    121 /* Touchscreen Controller */
#define CTL_BCM_FB                     200
#define CTL_BCM_TIMER                  201
#define CTL_BCM_PERFCNT                202
#define CTL_BCM_HPM							203
#define CTL_BCM_PERFTEST               204
#define CTL_BCM_ETH                    205
#define CTL_BCM_CSX_HAL_IOD            206
#define CTL_BCM_DDRPHASE               207
#define CTL_BCM_SPREADSPECTRUM         208
#define CTL_BCM_PM_ARAM			         209
#define CTL_BCM_CPU                    210

#define BCM_SYSCTL_LCD_PERF            1   /* int: 0 = off, 1 = on     */
#define BCM_SYSCTL_LCD_PERF_FREQ       2   /* int: jiffies per update  */
#define BCM_SYSCTL_LCD_PERF_PEAKBUSY   3   /* int: peak busy in percent */
#define BCM_SYSCTL_LCD_PERF_USRBUSY    4   /* int: user busy in cpu usage percentage */
#define BCM_SYSCTL_LCD_PERF_SYSBUSY    5   /* int: system busy in cpu usage percentage */
#define BCM_SYSCTL_LCD_PERF_NICEBUSY   6   /* int: nice busy in cpu usage percentage */
#define BCM_SYSCTL_LCD_BUS_ENABLE      7   /* int: bus usage 0 = off, 1 = on */
#define BCM_SYSCTL_LCD_BUS_PEAKBUSY    8   /* int: bus peak usage in percent */

#define BCM_SYSCTL_PMU_LEVEL           1   /* int: debug message level          */
#define BCM_SYSCTL_PMU_IRQS            2   /* int: # interrupts processed       */
#define BCM_SYSCTL_PMU_BATTLEVEL       3   /* int: battery level (0 to 5)       */
#define BCM_SYSCTL_PMU_CHGPLUGGEDIN    4   /* int: charger insertion status     */
#define BCM_SYSCTL_PMU_CHGSTATE        5   /* int: 1 when battery fully charged */
#define BCM_SYSCTL_PMU_PWRON           6   /* int: power-on condition           */
#define BCM_SYSCTL_PMU_PWRSTATE        7   /* int: 0 = off, 1 = on              */
#define BCM_SYSCTL_PMU_HSSTATE         8   /* int: headset state                */

#define BCM_SYSCTL_GPIO_LEVEL          1   /* int: debug message level          */
#define BCM_SYSCTL_GPIO_ISR            2   /* int: global GPIO ISR registered   */
#define BCM_SYSCTL_GPIO_IRQS           3   /* int: number of global GPIO IRQs   */
#define BCM_SYSCTL_GPIO_REGISTERED0    4   /* int: mask of GPIO line registered  (0 to 31) */
#define BCM_SYSCTL_GPIO_REGISTERED1    5   /* int: mask of GPIO line registered (32 to 64) */
#define BCM_SYSCTL_GPIO_BASE           6   /* int: base number for GPIO info    */

#define BCM_SYSCTL_DMA_LEVEL           1
#define BCM_SYSCTL_DMA_ISR             2
#define BCM_SYSCTL_DMA_IRQS            3
#define BCM_SYSCTL_DMA_0               4
#define BCM_SYSCTL_DMA_1               5
#define BCM_SYSCTL_DMA_2               6
#define BCM_SYSCTL_DMA_3               7

#define BCM_SYSCTL_DMA_LOCK            8
#define BCM_SYSCTL_DMA_NAME            9
#define BCM_SYSCTL_DMA_CALLED          10
#define BCM_SYSCTL_DMA_ERRORS          11

#define BCM_SYSCTL_REBOOT_WARM               1
extern int gArchWarmReboot;

#define BCM_SYSCTL_CPU_POLL                  1
extern int gArchCpuPoll;

#define BCM_SYSCTL_DDRPHASE_PANIC            1
#define BCM_SYSCTL_DDRPHASE_ALIGN_AVG        2
#define BCM_SYSCTL_DDRPHASE_ALIGN_MAX        3
#define BCM_SYSCTL_DDRPHASE_ALIGN_MIN        4
#define BCM_SYSCTL_DDRPHASE_DRIFT_AVG        5
#define BCM_SYSCTL_DDRPHASE_DRIFT_MAX        6
#define BCM_SYSCTL_DDRPHASE_DRIFT_MIN        7
#define BCM_SYSCTL_DDRPHASE_VALUE            8
#define BCM_SYSCTL_DDRPHASE_SETTIME          9

#endif  /* BCM_SYSCTL_H */
