/*****************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
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
 * Power Management Initialization
 *
 * Use this driver to turn off unnecessary blocks on the host device at startup
 * to reduce power consumption.
 *
 */

/* ---- Include Files ---------------------------------------------------- */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>

#include <linux/broadcom/pm_init.h>

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
const struct POWER_MGMT_INIT_t *gp_power_mgmt_init_t = NULL;

static struct platform_device  *gp_power_mgmt_device = NULL;

/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

static int power_mgmt_set_gpio(int gpio_pin, char *block_name, int value)
{
   int rc;
   
   if ((rc = gpio_request(gpio_pin, block_name)) != 0)
   {
      printk("%s gpio_request(pin %d) failed, rc = %d\n", __FUNCTION__,
             gpio_pin, rc);      
      return rc;
   }     

   if ((rc = gpio_direction_output(gpio_pin, value)) != 0)
   {
      printk("%s gpio_direction_output(%d, %d) error %d\n", __FUNCTION__,
             gpio_pin, value, rc);
      return rc;
   }   
   return rc;
}

static int power_mgmt_probe(struct platform_device *p_platform_device)
{
   int i;
   char buffer[80];
   
   if (p_platform_device->dev.platform_data == NULL)
   {
      printk(KERN_ERR "%s power_mgmt_probe() p_platform_device->dev.platform_data == NULL\n",
             PM_INIT_DRIVER_NAME);      
      return -1;
   }   
   
   /* Get the information compiled in for this platform. */   
   gp_power_mgmt_init_t = (struct POWER_MGMT_INIT_t *)p_platform_device->dev.platform_data;

   if (gp_power_mgmt_init_t != NULL && gp_power_mgmt_init_t->pm_init_cnt > 0)
   {      
      for (i = 0; i < gp_power_mgmt_init_t->pm_init_cnt; i++)
      {
         buffer[0] = 0;
         sprintf(buffer, "PM:%s off", gp_power_mgmt_init_t->p_power_blocks[i].p_block_name);
         printk("%s turned %s off\n", 
                PM_INIT_DRIVER_NAME, 
                gp_power_mgmt_init_t->p_power_blocks[i].p_block_name); 
         power_mgmt_set_gpio(gp_power_mgmt_init_t->p_power_blocks[i].gpio_pin, 
                          gp_power_mgmt_init_t->p_power_blocks[i].p_block_name, 
                          gp_power_mgmt_init_t->p_power_blocks[i].disable_val);       
      }
   }
   else
   {
      printk("%s not shutting power off to any blocks\n", PM_INIT_DRIVER_NAME);
   }
   
   return 0;
}

static int __devexit power_mgmt_remove(struct platform_device *p_platform_device)
{
   int i;
   printk("entering %s\n", __FUNCTION__); 
   
   if (gp_power_mgmt_init_t != NULL)
   {
      for (i = 0; i < gp_power_mgmt_init_t->pm_init_cnt; i++)
      {
         gpio_free(gp_power_mgmt_init_t->p_power_blocks[i].gpio_pin);
      }   
   }   
   return 0;
}

static struct platform_driver power_mgmt_driver = {
   .driver.name  = PM_INIT_DRIVER_NAME,
   .driver.owner = THIS_MODULE,
   .probe        = power_mgmt_probe,
   .remove       = __devexit_p(power_mgmt_remove),
};

static int __init power_mgmt_init(void)
{
   int ret;

	gp_power_mgmt_device = platform_device_alloc("pm-init", -1);
	if (!gp_power_mgmt_device) 
   {
		printk("%s: platform_device_alloc failed\n", __FUNCTION__);
		return -ENOMEM;
	}
      
   ret = platform_driver_register(&power_mgmt_driver);

	ret = platform_device_add(gp_power_mgmt_device);
	if (ret) {
		printk("%s: platform_device_add failed %d\n", __FUNCTION__,
		       ret);
		goto err_platform_device_add;
	}

	return 0;

   err_platform_device_add:
	platform_driver_unregister(&power_mgmt_driver);
   
   return ret;
}

static void __exit power_mgmt_exit(void)
{
   platform_driver_unregister(&power_mgmt_driver);
}

module_init(power_mgmt_init);
module_exit(power_mgmt_exit);

MODULE_DESCRIPTION("Power Management driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");
