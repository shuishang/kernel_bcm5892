/*****************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
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

#ifndef __ASM_BRCM_BT_LPM_H
#define __ASM_BRCM_BT_LPM_H

#include <linux/serial_core.h>
#include <linux/broadcom/bcmblt_rfkill_settings.h>

/* may need some clock locking mechanism hook to inhibit or allow sleep */
struct bcm_bt_lpm_platform_data {
	unsigned int gpio_bt_wake;    /* HOST -> BCM chip wakeup gpio */
	unsigned int gpio_host_wake;  /* BCM chip -> HOST wakeup gpio */
};

extern const struct bcm_bt_lpm_platform_data brcm_bt_lpm_data;
extern int serial11211_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg);
extern int brcm_init_bt_wake( const struct bcm_bt_lpm_platform_data * gpio_data );

#ifndef TIO_ASSERT_BT_WAKE
#define TIO_ASSERT_BT_WAKE      0x8003
#endif
#ifndef TIO_DEASSERT_BT_WAKE
#define TIO_DEASSERT_BT_WAKE    0x8004
#endif
#ifndef TIO_GET_BT_WAKE_STATE
#define TIO_GET_BT_WAKE_STATE   0x8005
#endif

#ifndef GPIO_BT_WAKE
#define GPIO_BT_WAKE 14
#endif
#ifndef GPIO_HOST_WAKE
#define GPIO_HOST_WAKE 18
#endif
#define GPIO_BT_CLK32K_EN 31
#define GPIO_BT_RESET 26
#define GPIO_BT_VREG_CTL 28

/* this define electrical level of GPIO for assert/de-asserted stated. sleep logic has by default negative
   logic */

#ifndef BT_WAKE_ASSERT
#define BT_WAKE_ASSERT 0
#endif
#ifndef BT_WAKE_DEASSERT
#define BT_WAKE_DEASSERT !(BT_WAKE_ASSERT)
#endif


#endif
