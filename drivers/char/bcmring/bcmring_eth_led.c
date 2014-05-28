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
*  Description: BCMRING Ethernet LED control driver.
*  
*  
*/ 

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <csp/stdio.h>
#include <csp/string.h>
#include <csp/module.h>
#include <csp/delay.h>

#include <csp/ethHw.h>
#include <mach/csp/ethHw_reg.h>
#include <mach/csp/chipcHw_inline.h>
#include <mach/hardware.h>
#include <asm/io.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/bcmring/gpio_defs.h>
#include <mach/csp/gpiomux.h>


#define ETH_LED_CTL_REG_BASE	0x30400078

#define LED_CONFIGURATION       0x00
#define FUNC_0_PORT_LED_CTL     0x08
#define FUNC_1_PORT_LED_CTL     0x18
#define PORT_LED_FUNC_MAP       0x28
#define LED_EN_MAP              0x38
#define LED_MODE_MAP0           0x48
#define LED_MODE_MAP1           0x58
#define LED_OUTPUT_EN           0x68
static int result;
static int ethledctl_major = BCM_ETH_LED_MAJOR;
static unsigned long phys_mem=0;
ssize_t ethled_read(struct file *filp, char *phy_data, size_t count, loff_t *f_pos);
int ethled_open (struct inode *inode, struct file *filp);
int ethled_release (struct inode *inode, struct file *filp);
static int ethled_register_cdev(void);
extern int ethHw_port_phy_get( int unit, int port, uint32_t flags, uint32_t phy_reg_addr, uint32_t *phy_data );
int ethled_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long int arg);

static struct file_operations ethLEDctl_fops={
	.open = ethled_open,
	.ioctl = ethled_ioctl,
	.release = ethled_release,
	.read =	ethled_read,
};


int ethled_open (struct inode *inode, struct file *filp)
{
	int num = MINOR (inode->i_rdev);
	phys_mem = (unsigned long)ioremap( ETH_LED_CTL_REG_BASE, 1024);
	printk( "call open -> minor : %d\n", num);
	printk( "phys_mem = 0x%x", (unsigned int)phys_mem);

	return 0;

}

static int __init eth_api_start(void)
{
	printk("[RZ]Eth Status Get module start \n");
	result = ethled_register_cdev();

	printk("[RZ]Request GPIO group \n");
	gpiomux_requestGroup( gpiomux_group_gphyled, "bcmring_eth_led");
	return result;
}

static void __exit eth_api_exit (void)
{
	printk(KERN_INFO "Goodbye world 1. \n");
	unregister_chrdev(ethledctl_major,"bcmring_eth_led");
}

static int ethled_register_cdev(void)
{
	static int result;

	printk(" Call bcmring_eth_led register cdev \n");

	result = register_chrdev( ethledctl_major, "bcmring_eth_led", &ethLEDctl_fops);

	printk(" Call bcmring_eth_led major = %d \n", ethledctl_major);

	if (result < 0) {
		printk("bcmring_eth_led register error, error number is %d \n",result);
		return result;
	}
	return 0;

}


int ethled_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long int arg)
{
	unsigned short value = (unsigned short)arg;
	switch(cmd) {
		case 1:
			printk("SWG: LED_CONFIGURATION Write = 0x%04x\n",value);
			(* (volatile unsigned short *) (phys_mem+LED_CONFIGURATION)) = value;
			break;
		case 2:
			printk("SWG: FUNC_0_PORT_LED_CTL Write = 0x%04x\n",value);
			(* (volatile unsigned short *) (phys_mem+FUNC_0_PORT_LED_CTL)) = value;
			break;
		case 3:
			printk("SWG: FUNC_1_PORT_LED_CTL Write = 0x%04x\n",value);
			(* (volatile unsigned short *) (phys_mem+FUNC_1_PORT_LED_CTL)) = value;
			break;
		case 4:
			printk("SWG: PORT_LED_FUNC_MAP Write = 0x%04x\n",value);
			(* (volatile unsigned short *) (phys_mem+PORT_LED_FUNC_MAP)) = value;
			break;
		case 5:
			printk("SWG: LED_EN_MAP Write = 0x%04x\n",value);
			(* (volatile unsigned short *) (phys_mem+LED_EN_MAP)) = value;
			break;
		case 6:
			printk("SWG: LED_MODE_MAP0 Write = 0x%04x\n",value);
			(* (volatile unsigned short *) (phys_mem+LED_MODE_MAP0)) = value;
			break;
		case 7:
			printk("SWG: LED_MODE_MAP1 Write = 0x%04x\n",value);
			(* (volatile unsigned short *) (phys_mem+LED_MODE_MAP1)) = value;
			break;
		case 8:
			printk("SWG: LED_OUTPUT_EN Write = 0x%04x\n",value);
			(* (volatile unsigned short *) (phys_mem+LED_OUTPUT_EN)) = value;
			break;
		case 9:
			printk("SWG: ETH LED CTL register init to defalut value\n");
            (* (volatile unsigned short *) (phys_mem+LED_CONFIGURATION)) = 0x0003;
            (* (volatile unsigned short *) (phys_mem+FUNC_0_PORT_LED_CTL)) = 0x0124;
            (* (volatile unsigned short *) (phys_mem+FUNC_1_PORT_LED_CTL)) = 0x0324;
            (* (volatile unsigned short *) (phys_mem+PORT_LED_FUNC_MAP)) = 0x01ff;
            (* (volatile unsigned short *) (phys_mem+LED_EN_MAP)) = 0x001f;
            (* (volatile unsigned short *) (phys_mem+LED_MODE_MAP0)) = 0x01ff;
            (* (volatile unsigned short *) (phys_mem+LED_MODE_MAP1)) = 0x01ff;
            (* (volatile unsigned short *) (phys_mem+LED_OUTPUT_EN)) = 0x00;
			break;
		default :
			printk("Cmd not valid ( valid num = 1 ~ 9 ) \n");
			return 1;
			break;
	}
	return 0;

}

ssize_t ethled_read(struct file *filp, char *phy_data, size_t count, loff_t *f_pos)
{
	int	unit;
	uint32_t	flags;
	int	port;
	unit=0;
	port=1;
	flags=0;
/*	printk(KERN_INFO "Eth Api read.\n"); */
	printk("SWG: 1. LED_CONFIGURATION    = 0x%04x\n",(* (volatile unsigned short *)(phys_mem+LED_CONFIGURATION)));
	printk("SWG: 2. FUNC_0_PORT_LED_CTL  = 0x%04x\n",(* (volatile unsigned short *)(phys_mem+FUNC_0_PORT_LED_CTL)));
	printk("SWG: 3. FUNC_1_PORT_LED_CTL  = 0x%04x\n",(* (volatile unsigned short *)(phys_mem+FUNC_1_PORT_LED_CTL)));
	printk("SWG: 4. PORT_LED_FUNC_MAP    = 0x%04x\n",(* (volatile unsigned short *)(phys_mem+PORT_LED_FUNC_MAP)));
	printk("SWG: 5. LED_EN_MAP           = 0x%04x\n",(* (volatile unsigned short *)(phys_mem+LED_EN_MAP)));
	printk("SWG: 6. LED_MODE_MAP0        = 0x%04x\n",(* (volatile unsigned short *)(phys_mem+LED_MODE_MAP0)));
	printk("SWG: 7. LED_MODE_MAP1        = 0x%04x\n",(* (volatile unsigned short *)(phys_mem+LED_MODE_MAP1)));
	printk("SWG: 8. LED_OUTPUT_EN        = 0x%04x\n",(* (volatile unsigned short *)(phys_mem+LED_OUTPUT_EN)));
	return 0;
}


int ethled_release (struct inode *inode, struct file *filp)
{
/*	printk("call release \n"); */
	return 0;
}

module_init(eth_api_start);
module_exit(eth_api_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Ethernet LED control driver");
MODULE_LICENSE("GPL");
