/*
 * linux/arch/arm/mach-za9l/tti_proc.c
 *
 * Copyright (C) 2010 strong - lion, Inc.
 *
 * Author: lxyvslyr <lxyvslyr@yahoo.com.cn.>
 * Date  : $Date: 2010/12/23 06:25:59 $
 *
 * $Revision: 1.1.1.1 $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the  
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */ 
 
#include <linux/module.h> 
#include <linux/kernel.h> 
#include <linux/init.h>
#include <linux/unistd.h>
#include <linux/proc_fs.h>
#include <linux/errno.h>
#include <linux/compile.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/strong_lion_def.h> 

#define PM_PROC_BUF_LEN	16

static char pm_proc_buf[PM_PROC_BUF_LEN];

struct proc_dir_entry *stronglion_root;

struct proc_dir_entry *pm_root;
struct proc_dir_entry *ver_root;

static struct proc_dir_entry *sleep_level;

struct proc_misc_members pms;

int DC_status = NO_PROVIED;

EXPORT_SYMBOL(DC_status);

static int proc_read_sleep_level(char *page,char **start,off_t off, 
			int count,int *eof,void *data)
{
	int len;

	len = sprintf(page, "%d\n",pms.sys_sleep_level);
	
	*eof = 1;
//	*start = page;
	
	return len;  
}

static int proc_write_sleep_level(struct file *file,const char *buffer, unsigned long count, void *data)
{
 //   printk("here0\n");

	if (count > 1)
		return -EINVAL;

	if(copy_from_user(sleep_level->data,buffer,count))
		return -EFAULT;
	
 //   printk("here1\n")	
	if ( memcmp(sleep_level->data,"1",count) == 0 )
	{
		pms.sys_sleep_level = 1;
		printk("sleep_level : 1\n");
	}	
	else if ( memcmp(sleep_level->data,"2",count) == 0  )
	{
		pms.sys_sleep_level = 2;
		printk("sleep_level : 2\n");
	}	
	else
	{
		printk("invalid value of sleep level\n");
		return -EINVAL;
	}	
    
    	return count;
}


static void hareware_init(void)
{

}

static void wakeup_hareware_init(void)
{

}

static int pm_misc_probe(struct platform_device *pdev)
{
	printk("pm_misc_probe\n");

	hareware_init();	

	return 0;
}

#ifdef CONFIG_PM
static int pm_misc_suspend(struct platform_device *dev,pm_message_t state)
{
	printk("pm_misc_suspend\n");		

	return 0;
}

static int pm_misc_resume(struct platform_device *dev)
{
	printk("pm_misc_resume\n");

	wakeup_hareware_init();		

	return 0;
}

#else
#define pm_misc_suspend NULL
#define pm_misc_resume NULL
#endif

static struct platform_driver pm_misc_driver = {
	.probe		= pm_misc_probe,
	.suspend	 	= pm_misc_suspend,
	.resume	 	= pm_misc_resume,
	.driver	 	= {
		.name	 = "bcm5892-pm-misc",
	},
};

static int __init init_stronglion_proc(void)
{
	int ret = 0;        

	ret = platform_driver_register(&pm_misc_driver);
	if (ret !=0)
	{
		printk("platform register failed!,power manager may dont work\n");
	}

	printk("Creat strong lion proc root\n");
	stronglion_root = proc_mkdir("strong_lion", 0);
	if (stronglion_root == NULL)
	{
		ret = -ENOMEM;
		goto out;
	} 


	pm_root = proc_mkdir("power_manager",stronglion_root);
	if (pm_root == NULL)
	{
		ret = -ENOMEM;	
		goto bad_out0;
	}		

	ver_root = proc_mkdir("version",stronglion_root);
	if (ver_root == NULL)
	{
		ret = -ENOMEM;	
		goto bad_out1;
	}		

	sleep_level = create_proc_entry("sleep_level",0666,stronglion_root);	
	if (sleep_level == NULL)	
	{		
		ret = -ENOMEM;
		goto bad_out2;
	}	

	sleep_level->data = pm_proc_buf;	
	sleep_level->read_proc = &proc_read_sleep_level;	
	sleep_level->write_proc = &proc_write_sleep_level;
	pms.sys_sleep_level = 1;

	goto out;

bad_out2:
	remove_proc_entry("version",stronglion_root);

bad_out1:
	remove_proc_entry("power_manager",stronglion_root);
	
bad_out0:
	remove_proc_entry("strong_lion",NULL);
	
out:    
	return ret;
}

static void __exit cleanup_stronglion_proc(void)
{
	remove_proc_entry("sleep_level",stronglion_root);

	remove_proc_entry("version",stronglion_root);

	remove_proc_entry("power_manager",stronglion_root);

	remove_proc_entry("strong_lion",NULL); 
   
	printk("clean the strong_lion proc\n");

	platform_driver_unregister(&pm_misc_driver);
}

EXPORT_SYMBOL(stronglion_root);
EXPORT_SYMBOL(pm_root);

module_init(init_stronglion_proc);
module_exit(cleanup_stronglion_proc);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LJJ<lxyvslyr@yahoo.com.cn>");
MODULE_DESCRIPTION("The driver of strong lion proc device");

