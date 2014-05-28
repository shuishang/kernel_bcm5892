/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*****************************************************************************/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <csp/wdogHw.h>
#include <mach/csp/chipcHw_inline.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>

#include <linux/device.h>
#include <linux/amba/bus.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#define PFX "watchdog: "

#define WATCHDOG_MAX_TIMEOUT 60		/* 60 sec default timeout */

static uint32_t tmr_margin   = WATCHDOG_MAX_TIMEOUT;

module_param(tmr_margin, int, 0);
MODULE_PARM_DESC(tmr_margin, "WDT timer margin in seconds. default=" __MODULE_STRING(WATCHDOG_MAX_TIMEOUT) ")");

static int nowayout	= WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int boot_status;

static DECLARE_MUTEX(open_lock);

typedef enum close_state {
	CLOSE_STATE_NOT,
	CLOSE_STATE_ALLOW=0x4021
} close_state_t;

static close_state_t allow_close;
static int timer_activated = 0;

static void watchdog_start(void)
{
   wdogHw_startWatchdog();
}

static void watchdog_stop(void)
{
	if (allow_close == CLOSE_STATE_ALLOW) 
   {
      wdogHw_stopWatchdog();
   	printk(KERN_INFO PFX "Stopped watchdog timer.\n");
   }
   else
   {
   	printk(KERN_INFO PFX "Did NOT stop watchdog timer.\n");
   }
}

/*
 *	watchdog_keepalive - reload the timer
 *
 */
static void watchdog_keepalive(void)
{
   wdogHw_restartWatchdog();
}

/****************************************************************************
*
*  watchdog_interrupt
*
***************************************************************************/
static irqreturn_t watchdog_interrupt(int irq, void *data)
{
	static int first = 1;

	if (first) {
		/* "Quiet" mode, so just print message once */
		printk(KERN_INFO PFX "watchdog interrupt - we're all going to die!\n");
		first = 0;
	}

	return( IRQ_HANDLED );
}

/*
 *	/dev/watchdog handling
 *	Allow only one person to hold it open
*/
static int watchdog_open(struct inode *inode, struct file *file)
{
   if(down_trylock(&open_lock))
		return -EBUSY;

	if (nowayout)
   {
		__module_get(THIS_MODULE);
	}
   else
   {
      allow_close = CLOSE_STATE_ALLOW;
	}

   /* Activate timer */
	timer_activated = 1;
	watchdog_start();
	watchdog_keepalive();

	printk(KERN_INFO "Started watchdog timer.\n");

	return nonseekable_open(inode, file);
}

static int watchdog_release(struct inode *inode, struct file *file)
{
	/* 
    * Shut off the timer.
	 *	Lock it in if it's a module and we set nowayout 
    */
	if (allow_close == CLOSE_STATE_ALLOW) 
   {
		watchdog_stop();		/* Turn the WDT off */
	} 
   else
   {
		printk(KERN_INFO "not stopping watchdog!\n");
		watchdog_keepalive();
	}

	allow_close = CLOSE_STATE_NOT;
	up(&open_lock);
	return 0;
}

static ssize_t watchdog_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
   /* Refresh the timer. */
	if(len) {
      if (!nowayout)
      {
         if (data[0] == 'N')
         {
            /* Lock the watchdog driver and do not allow it to be shutdown */
            allow_close = CLOSE_STATE_NOT;
            nowayout = 1;
         }
      }
		watchdog_keepalive();
	}
	return len;
}

static struct watchdog_info ident = {
	.options		= WDIOF_CARDRESET | WDIOF_SETTIMEOUT |	WDIOF_KEEPALIVEPING |
				WDIOF_MAGICCLOSE,
   .firmware_version = 0,
	.identity		= "Hardware Watchdog",
};

static int watchdog_ioctl(struct inode *inode, struct file *file,
			     unsigned int cmd, unsigned long arg)
{
	int ret = -ENOIOCTLCMD;
	int time, options;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	switch (cmd) {
		default:
      break;
		case WDIOC_GETSUPPORT:
			ret = copy_to_user(argp, &ident,
				sizeof(ident)) ? -EFAULT : 0;
		break;

		case WDIOC_GETSTATUS:
		ret = put_user(0, p);
		break;

		case WDIOC_GETBOOTSTATUS:
			ret = put_user(boot_status, p);
		break;

   case WDIOC_SETOPTIONS:
		ret = get_user(options, p);
		if (ret)
         break;

		if (options & WDIOS_DISABLECARD) {
			watchdog_stop();
			ret = 0;
		}

		if (options & WDIOS_ENABLECARD) {
			watchdog_start();
			ret = 0;
		}
      break;

		case WDIOC_SETTIMEOUT:
   		ret = get_user(time, p);
   		if (ret)
   			break;

   		if (time <= 0 || time > WATCHDOG_MAX_TIMEOUT)
         {
   			ret = -EINVAL;
   			break;
   	   }

			tmr_margin = wdogHw_setWatchdogInterval(time*1000)/1000;
         if (tmr_margin != time)
         {
   			ret = -EINVAL;
   			break;
   		}
		   /*fall through*/
		case WDIOC_GETTIMEOUT:
			ret = put_user(tmr_margin, p);
         break;

		case WDIOC_KEEPALIVE:
         watchdog_keepalive();
         ret = 0;
         break;
	}
	return ret;
}

/*
 *	Notifier for system down
 */

static int watchdog_notify_sys(struct notifier_block *this, unsigned long code, void *unused)
{
   if (code == SYS_DOWN || code == SYS_HALT)
	   watchdog_stop();		/* Turn the WDT off */

	return NOTIFY_DONE;
}

/*
 *	Kernel Interfaces
 */
static struct file_operations watchdog_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= watchdog_write,
	.ioctl		= watchdog_ioctl,
	.open		= watchdog_open,
	.release	= watchdog_release,
};

static struct miscdevice watchdog_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &watchdog_fops,
};

static struct notifier_block watchdog_notifier = {
	.notifier_call = watchdog_notify_sys,
};

static const __devinitconst char banner[] =
	KERN_INFO PFX "BROADCOM WDT driver\n";


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
static int __devinit bcmring_wdt_probe(struct amba_device *pdev, struct amba_id *id)
#else
static int __devinit bcmring_wdt_probe(struct amba_device *pdev, void *id)
#endif
{
	struct device *dev = &pdev->dev;
	int ret;

	printk(banner);

   /* Setup interrupt handler for 1/2 done watchdog interrupt */
	ret = request_irq(IRQ_WATCHDOG, watchdog_interrupt, 0,"wdog", dev);
	if (ret) 
   {
      printk( KERN_ERR "Failed to get wdog IRQ\n" );
		return( ret );
	}

   /* Determine if the watchdog reset occured or if it was a normal power-up */
	boot_status = 0;
   if (chipcHw_getStickyBits() & chipcHw_REG_STICKY_WDOG_RESET)
   {
      /* Watchdog reset occured */
	   boot_status = 1;
      chipcHw_clearStickyBits(chipcHw_REG_STICKY_WDOG_RESET);
   }

	/* This call checks if timeout is valid */
   wdogHw_setWatchdogInterval(tmr_margin*1000);

	ret = register_reboot_notifier(&watchdog_notifier);
	if (ret) {
		printk(KERN_ERR PFX "cannot register reboot notifier (err=%d)\n",
			ret);
      return ret;
	}

	ret = misc_register(&watchdog_miscdev);
	if (ret) {
		printk(KERN_ERR PFX "cannot register miscdev on minor=%d (err=%d)\n",
			WATCHDOG_MINOR, ret);
   	unregister_reboot_notifier(&watchdog_notifier);
	}

	return ret;
}

static int bcmring_wdt_remove(struct amba_device *pdev)
{
	struct device *dev = &pdev->dev;

	misc_deregister(&watchdog_miscdev);
	unregister_reboot_notifier(&watchdog_notifier);

   free_irq( IRQ_WATCHDOG, dev );

	return 0;
}

#ifdef CONFIG_PM
static int bcmring_wdt_suspend(struct amba_device *pdev, pm_message_t msg)
{
   if (allow_close == CLOSE_STATE_ALLOW) 
   {
		watchdog_stop();		/* Turn the WDT off */
      allow_close = CLOSE_STATE_NOT;
	} 

	return 0;
}

static int bcmring_wdt_resume(struct amba_device *pdev)
{
   if (!nowayout && timer_activated)
   {
      /* re-activate timer */
	   watchdog_start();
		watchdog_keepalive();
      allow_close = CLOSE_STATE_ALLOW;
	}

	return 0;
}
#else
#define bcmring_wdt_suspend    NULL
#define bcmring_wdt_resume     NULL
#endif

static struct amba_id bcmring_id_table[] __initdata = {
	{
		.id	= 0x00041805,
		.mask	= 0x000fffff,
	},
	{ 0, 0 },
};

static struct amba_driver bcmring_driver = {
	.drv = {
		.name	= "bcmring-wdt",
	},
	.probe		= bcmring_wdt_probe,
	.remove		= bcmring_wdt_remove,
   .suspend    = bcmring_wdt_suspend,
   .resume     = bcmring_wdt_resume,
	.id_table	= bcmring_id_table,
};

static int __init watchdog_init(void)
{
	return amba_driver_register(&bcmring_driver);
}
module_init(watchdog_init);

static void __exit watchdog_exit(void)
{
	amba_driver_unregister(&bcmring_driver);
}
module_exit(watchdog_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BROADCOM WDT");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
