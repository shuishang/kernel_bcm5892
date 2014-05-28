/*****************************************************************************
* Copyright 2001 - 2009 Broadcom Corporation.  All rights reserved.
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
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/amba/bus.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <mach/dmu.h>
#include <mach/irqs.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#define WATCHDOG_DEFAULT_TIME	10

static int tmr_margin   = WATCHDOG_DEFAULT_TIME;
static int nowayout	= WATCHDOG_NOWAYOUT;
static int noboot	= 0;

module_param(tmr_margin, int, 0);
MODULE_PARM_DESC(tmr_margin, "BCM5892 timer margin in seconds. default=" __MODULE_STRING(TIMER_MARGIN) ")");

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

module_param(noboot, int, 0);
MODULE_PARM_DESC(noboot, "Watchdog action, set to 1 to ignore reboots, 0 to reboot (default=" __MODULE_STRING(noboot) ")");


static DECLARE_MUTEX(open_lock);

typedef enum close_state {
	CLOSE_STATE_NOT,
	CLOSE_STATE_ALLOW=0x4021
} close_state_t;


static unsigned int wdt_count;
static close_state_t allow_close;


/*
 *	bcm5892_wdt_keepalive - reload the timer
 *
 */
static void bcm5892_wdt_keepalive(void)
{
	writel(wdt_count, IO_ADDRESS(WDT_REG_BASE_ADDR + WDOG_LOAD));
}

/*
 *	This is the interrupt handler. 
 */
static irqreturn_t bcm5892_wdt_int(int irq, void *arg)
{
	/* Clear the interrupt on the watchdog */
	writel(0x1, IO_ADDRESS(WDT_REG_BASE_ADDR + WDOG_INTCLR));
	bcm5892_wdt_keepalive();
	
	return IRQ_HANDLED;
}

static void bcm5892_wdt_stop(void)
{
	/* Set INTEN low to disable the counter */
	writel(0x0, IO_ADDRESS(WDT_REG_BASE_ADDR + WDOG_CTL));
}

static void bcm5892_wdt_start(void)
{
	/* Stop the wdt and reload */
	bcm5892_wdt_stop();
	bcm5892_wdt_keepalive();

	/* Enable it here */
	writel ((1 << 0) | (1 << 1), IO_ADDRESS(WDT_REG_BASE_ADDR + WDOG_CTL));

}

static int bcm5892_wdt_set_heartbeat(int timeout)
{
	if(timeout < 0x1)
		return -EINVAL;
 
	/* Calculate the load value */
	wdt_count = (timeout * BCM5892_WDOG_CLK) - 1;

	return 0;
}

/*
 *	/dev/watchdog handling
 */
static int bcm5892_wdt_open(struct inode *inode, struct file *file)
{

	if(down_trylock(&open_lock))
		return -EBUSY;

	if (nowayout) {
		__module_get(THIS_MODULE);
	} else {
		allow_close = CLOSE_STATE_ALLOW;
	}

	/*
	 *	Activate timer
	 */
	bcm5892_wdt_start();

	return nonseekable_open(inode, file);
}

static int bcm5892_wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *	Shut off the timer.
	 * 	Lock it in if it's a module and we set nowayout
	 */
	if (allow_close == CLOSE_STATE_ALLOW) {
		bcm5892_wdt_stop();
	} else {
		printk("Unexpected close, not stopping watchdog!\n");
		bcm5892_wdt_keepalive();
	}

	allow_close = CLOSE_STATE_NOT;
	up(&open_lock);
	return 0;

}

static ssize_t bcm5892_wdt_write(struct file *file, const char *data, 
				 size_t len, loff_t *ppos)
{
	/*
	 *	Refresh the timer.
	 */
	if(len) {
		if (!nowayout) {
			size_t i;

			/* In case it was set long ago */
			allow_close = CLOSE_STATE_NOT;

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					allow_close = CLOSE_STATE_ALLOW;
			}
		}

		bcm5892_wdt_keepalive();
	}
	return len;

}

static struct watchdog_info ident = {
	.options		= WDIOF_SETTIMEOUT |
				WDIOF_KEEPALIVEPING |
				WDIOF_MAGICCLOSE,
	.identity		= "BCM5892 Hardware Watchdog",
};

static int bcm5892_wdt_ioctl(struct inode *inode, struct file *file,
			     unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_margin;

	switch (cmd) {
		default:
			return -ENOIOCTLCMD;

		case WDIOC_GETSUPPORT:
			return copy_to_user(argp, &ident,
				sizeof(ident)) ? -EFAULT : 0;

		case WDIOC_GETSTATUS:
		case WDIOC_GETBOOTSTATUS:
			return put_user(0, p);

		case WDIOC_KEEPALIVE:
			bcm5892_wdt_keepalive();
			return 0;

		case WDIOC_SETTIMEOUT:
			if (get_user(new_margin, p))
				return -EFAULT;

			if (bcm5892_wdt_set_heartbeat(new_margin))
				return -EINVAL;

			bcm5892_wdt_keepalive();
			return put_user(tmr_margin, p);

		case WDIOC_GETTIMEOUT:
			return put_user(tmr_margin, p);
	}

	return 0;
}

/*
 *	Kernel Interfaces
 */
static struct file_operations bcm5892_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= bcm5892_wdt_write,
	.ioctl		= bcm5892_wdt_ioctl,
	.open		= bcm5892_wdt_open,
	.release	= bcm5892_wdt_release,
};

static struct miscdevice bcm5892_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &bcm5892_wdt_fops,
};

static int bcm5892_wdt_probe(struct amba_device *pdev, void *id)
{
	struct device *dev = &pdev->dev;
	int ret;
	int started = 0;
	
	ret = request_irq(IRQ_OWDOG_TIMOUT, bcm5892_wdt_int, IRQF_DISABLED,"Watchdog", dev);
	if (ret) {
		printk("cannot register IRQ for watchdog\n");
		return 0;
	}

	cls_dmu_block_enable (DMU_WDT_PWR_ENABLE);

	/* This call checks if timeout is valid */
	started = bcm5892_wdt_set_heartbeat(WATCHDOG_DEFAULT_TIME);

	if (started == 0) {
	  printk("tmr_margin, default %d used\n",
		 WATCHDOG_DEFAULT_TIME);
	} else {
	  printk("default timer value is out of range, cannot start\n");
	}

	ret = misc_register(&bcm5892_wdt_miscdev);
	if (ret) {
		  printk("cannot register miscdev \n");
		  return ret;
	}
	
	if(started == 0) {
		bcm5892_wdt_start();
	}

	return 0;

}

static int bcm5892_wdt_remove(struct amba_device *pdev)
{
	struct device *dev = &pdev->dev;

	free_irq(IRQ_OWDOG_TIMOUT,dev);
	misc_deregister(&bcm5892_wdt_miscdev);

	return 0;
}

static struct amba_id wdog_ids[] __initdata = {
	{
		.id	= 0x00041805,
		.mask	= 0x000fffff,
	},
	{ 0, 0 },
};

static struct amba_driver bcm5892_wdt_driver = {
	.drv = {
		.name	= "watchdog",
	},
	.id_table	= wdog_ids,
	.probe		= bcm5892_wdt_probe,
	.remove		= bcm5892_wdt_remove,
};

static char banner[] __initdata = KERN_INFO "BCM5892 Watchdog Timer\n";

static int __init bcm5892_wdt_init(void)
{
	int ret;

	printk(banner);
	ret =  amba_driver_register(&bcm5892_wdt_driver);
	if (ret) 
		printk("amba_driver_register failed\n");

	return ret;
}

static void __exit bcm5892_wdt_exit(void)
{
	amba_driver_unregister(&bcm5892_wdt_driver);
}

module_init(bcm5892_wdt_init);
module_exit(bcm5892_wdt_exit);

MODULE_AUTHOR("Broadcom")
MODULE_DESCRIPTION("BCM5892 Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
