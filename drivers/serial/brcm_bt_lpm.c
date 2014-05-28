/*
 *  linux/drivers/serial/brcm_bt_lpm.c
 *
 *  Driver for brcm bt wake handling
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * A note about mapbase / membase
 *
 *  mapbase is the physical address of the IO port.
 *  membase is an 'ioremapped' cookie.
 */

/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
* 
* 	@file	linux/drivers/serial/brcm_bt_lpm.c
*
* Unless you and Broadcom execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
* 
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*******************************************************************************/

#if defined(CONFIG_SERIAL_8250_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <mach/csp/gpiomux.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include "8250.h"

#include <mach/gpio.h>

#include "brcm_bt_lpm.h"

const struct bcm_bt_lpm_platform_data brcm_bt_lpm_data = {
	.gpio_bt_wake   = GPIO_BT_WAKE,
	.gpio_host_wake = GPIO_HOST_WAKE,
};

static struct bcm_bt_lpm_platform_data bcm_bt_lpm_data;

static int brcm_assert_bt_wake(struct uart_port *port)
{
/*	 struct tty_port *port = &state->port; */

	/* TODO: make gpio number depending on uart number in case multiple chips are connected to multiple ports! */
	gpio_set_value( bcm_bt_lpm_data.gpio_bt_wake, BT_WAKE_ASSERT );
	pr_info("bt_wake assert: gpio: %d, %d\n", bcm_bt_lpm_data.gpio_bt_wake, BT_WAKE_ASSERT );
	printk(KERN_ERR "bt_wake assert: gpio: %d, %d\n", bcm_bt_lpm_data.gpio_bt_wake, BT_WAKE_ASSERT );
	return 0;
}


static int brcm_deassert_bt_wake(struct uart_port *port)
{
/*	 struct tty_port *port = &state->port; */

	/* TODO: make gpio number depending on uart number in case multiple chips are connected to multiple ports! */
	gpio_set_value( bcm_bt_lpm_data.gpio_bt_wake, BT_WAKE_DEASSERT );
	pr_info("bt_wake DEassert: gpio: %d, %d\n", bcm_bt_lpm_data.gpio_bt_wake, BT_WAKE_DEASSERT );
	printk(KERN_ERR "bt_wake DEassert: gpio: %d, %d\n", bcm_bt_lpm_data.gpio_bt_wake, BT_WAKE_DEASSERT );
	return 0;
}

static int brcm_get_bt_wake_state(struct uart_port *port, unsigned long __user *retinfo)
{
	/*struct tty_port *port = &state->port;*/
	unsigned long tmp;

	/* mutex_lock(&port->mutex); */
	tmp = gpio_get_value( bcm_bt_lpm_data.gpio_bt_wake );
	/* mutex_unlock(&port->mutex); */

	pr_info("brcm_get_bt_wake_state(bt_wake:%d) state:%ld\n",  bcm_bt_lpm_data.gpio_bt_wake, tmp );

	if (copy_to_user(retinfo, &tmp, sizeof(*retinfo)))
		return -EFAULT;
	return 0;
}


int serial11211_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	int ret = -ENOIOCTLCMD;

	pr_debug("serial11211_ioctl(cmd: x%x)\n", cmd );
	switch (cmd) {
	case TIO_ASSERT_BT_WAKE:
		ret = brcm_assert_bt_wake( port );
	printk(KERN_ERR "serial11211_ioctl(cmd: x%x) waking\n", cmd  );
		break;

	case TIO_DEASSERT_BT_WAKE:
	printk(KERN_ERR "serial11211_ioctl(cmd: x%x) sleeping\n", cmd  );
		ret = brcm_deassert_bt_wake( port );
		break;

	case TIO_GET_BT_WAKE_STATE:
		ret = brcm_get_bt_wake_state( port, uarg );
		break;
	}

	return ret;
}


int brcm_init_bt_wake( const struct bcm_bt_lpm_platform_data * gpio_data )
{
	pr_info("brcm_init_bt_wake( gpio_bt_wake: %d )",gpio_data->gpio_bt_wake );

	bcm_bt_lpm_data.gpio_bt_wake = gpio_data->gpio_bt_wake;
	bcm_bt_lpm_data.gpio_host_wake = gpio_data->gpio_host_wake;

   gpiomux_request(bcm_bt_lpm_data.gpio_bt_wake, chipcHw_GPIO_FUNCTION_GPIO,
		 "BT Power Mgmt");
   gpiomux_request(bcm_bt_lpm_data.gpio_host_wake, chipcHw_GPIO_FUNCTION_GPIO,
		 "BT Host Power Mgmt");

	 /* depending on when init is called or if high/low electrical level is resulting in less power 
	   conusmption you may want to change this. it should be noted however that if BT_WAKE is de-
	   asserted after uart open, this might result in bt chip lock-up due enterning LPM! */
	gpio_direction_output( gpio_data->gpio_bt_wake, BT_WAKE_ASSERT );
	
	/* TODO: Need to set the gpio pin direction for BT_HOST_WAKED_ASSERT
	   when the support for BT_HOST_WAKED_ASSERT is added */

	return 0;
}
