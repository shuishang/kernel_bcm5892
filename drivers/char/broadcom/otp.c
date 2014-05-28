/*****************************************************************************
* Copyright 2005 - 2009 Broadcom Corporation.  All rights reserved.
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
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/ioctl.h>

#include <asm/uaccess.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/otp.h>

#include <mach/csp/chipcHw_inline.h>
#include <mach/csp/otpHw_inline.h>
#include <mach/csp/otpHw_reg.h>

#undef OTP_DBG
#define OTP_DBG 0

#if (OTP_DBG == 1)
#define OTP_DEBUG(fmt, args...) printk(KERN_NOTICE "OTP: " fmt, ## args)
#else
#define OTP_DEBUG(fmt, args...) 
#endif

static char banner[] __initdata = KERN_INFO "Broadcom OTP Driver\n";

static struct semaphore lock;

static int otp_read_row(struct otp_data *data)
{
	if (data->row > 143)
		return -EINVAL;

	down(&lock);

   if (data->enable_ecc)
		data->data = otpHw_readData(data->row);
	else
		data->data = otpHw_readRawData(data->row);

	up(&lock);
	
	return 0;
}

static int otp_open(struct inode *inode, struct file *filp)
{
	OTP_DEBUG("Device opened\n");
	return 0;
}

static int otp_release(struct inode *inode, struct file *filp)
{
	OTP_DEBUG("Device closed\n");
	return 0;
}

static int otp_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
      unsigned long arg)
{
	switch (cmd) {
		case OTP_IOCTL_READ_ROW:
		{
			int ret;
			struct otp_data data;

			ret = copy_from_user(&data, (struct otp_data *)arg, sizeof(data));
			if (ret)
				return -EFAULT;
			
			ret = otp_read_row(&data);
			if (ret < 0)
				return ret;

			ret = copy_to_user((void *)arg, &data, sizeof(data));
			if (ret)
				return -EFAULT;
		}
		break;
		
		default:
		return -EINVAL;
	}
	return 0;
}

struct file_operations otp_fops =
{
	owner: THIS_MODULE,
	open: otp_open,
	release: otp_release,
	ioctl: otp_ioctl
};

static int otp_probe(struct platform_device *pdev)
{
	int rc;

	printk(banner);

	init_MUTEX(&lock); /* unlocked */

	rc = register_chrdev(BCM_OTP_MAJOR, "otp", &otp_fops);
	if (rc < 0) {
		printk(KERN_WARNING "OTP: register_chrdev failed for major %d\n",
				BCM_OTP_MAJOR);
		return rc;
	}

	chipcHw_busInterfaceClockEnable(chipcHw_REG_BUS_CLOCK_OTP);
	otpHw_init();

   return 0;
}

static int otp_remove(struct platform_device *pdev)
{
	unregister_chrdev(BCM_OTP_MAJOR, "otp");
	chipcHw_busInterfaceClockDisable(chipcHw_REG_BUS_CLOCK_OTP);

	return 0;
}

static struct platform_driver otp_driver = {
	.driver = {
		.name = "bcmring-otp",
		.owner = THIS_MODULE,
	},
	.probe = otp_probe,
	.remove = otp_remove,
};

static int __init otp_init(void)
{
	return platform_driver_register(&otp_driver);
}

static void __exit otp_exit(void)
{
	platform_driver_unregister(&otp_driver);
}

module_init(otp_init);
module_exit(otp_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("OTP Driver");
MODULE_LICENSE("GPL");
