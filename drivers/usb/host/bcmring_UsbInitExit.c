/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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
 * This file contains BCMRING USB init/exit code that is to be shared between
 * both the EHCI/OHCI drivers
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include <mach/csp/chipcHw_inline.h>
#include <mach/csp/chipcHw_def.h>

#include <linux/broadcom/bcmring/gpio_defs.h>
#include <mach/csp/gpiomux.h>
#include <linux/broadcom/gpio.h>
#include <linux/broadcom/bcmring_usb.h>

#include <csp/usbDevHw.h>
#include <csp/usbHostHw.h>
#include <csp/usbPhyHw.h>
#include <mach/csp/usbDevHw_def.h>

/* #define DEBUG */
#ifdef DEBUG
   #define USB_DEBUG(fmt, args...)     printk(fmt, ## args)
#else
   #define USB_DEBUG(fmt...)
#endif

#define MAX_NUM_USB_PORT     2
#define USB_CLOCK_RATE_HZ    30000000

#define MAX_PROC_BUF_SIZE     256
#define PROC_PARENT_DIR       "usb_port"
#define PROC_ENTRY_INFO       "info"

struct proc_dir {
   struct proc_dir_entry *parent_dir;
};

struct usb_cfg {
   struct clk *clk; /* core and bus interface clock */
   unsigned int is_gadget; /* whether port1 is configured as device or host */
   unsigned int host_cnt; /* number of host ports in use */
   unsigned int port_cnt[MAX_NUM_USB_PORT]; /* keep track of port enables */
   unsigned int tot_cnt; /* keep track of total enables */
   struct notifier_block pm_notifier; /* power management callbacks */
};

static struct proc_dir gProc;
static struct usb_cfg gUsbCfg;

static int pm_callback(struct notifier_block *nfb, unsigned long action,
      void *ignored);

static int
proc_info_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   unsigned int len = 0;
   struct usb_cfg *cfg = &gUsbCfg;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "USB Port config info:\n");
   len += sprintf(buffer + len, "is_gadget=%u host_cnt=%u port_cnt[0]=%u port_cnt[1]=%u tot_cnt=%u\n",
         cfg->is_gadget, cfg->host_cnt, cfg->port_cnt[0], cfg->port_cnt[1], cfg->tot_cnt);

   return len;
}

static int proc_init(void)
{
   struct proc_dir_entry *proc_info;

   gProc.parent_dir = proc_mkdir(PROC_PARENT_DIR, NULL);

   proc_info = create_proc_entry(PROC_ENTRY_INFO, 0644, gProc.parent_dir);
   if (proc_info == NULL) {
      return -ENOMEM;
   }
   proc_info->read_proc = proc_info_read;
   proc_info->write_proc = NULL;
   proc_info->data = NULL;

   return 0;
}

static void proc_term(void)
{
   remove_proc_entry(PROC_ENTRY_INFO, gProc.parent_dir);
   remove_proc_entry(PROC_PARENT_DIR, NULL);
}

/*
 * This function can be invoked by both OHCI and EHCI drivers. Logic in this
 * function makes sure it will be executed once and only once. "host_cnt"
 * denotes the number of ports that are used as USB host
 */
int bcmring_usb_init(unsigned int port_index, unsigned int is_gadget)
{
   int rval = -EFAULT;
   gpiomux_rc_e gpio_status;
   struct usb_cfg *cfg = &gUsbCfg;
   
   if ((port_index >= MAX_NUM_USB_PORT) ||
       (is_gadget && port_index != 1)) {
      return -EINVAL;
   }

   if (is_gadget && cfg->port_cnt[1] && !cfg->is_gadget) {
      printk(KERN_ERR "USB: Error - Port 1 has been configured as host\n");
      return -EINVAL;
   }

   if (!is_gadget && cfg->port_cnt[1] && cfg->is_gadget && port_index == 1) {
      printk(KERN_ERR "USB: Error - Port 1 has been configured as gadget\n");
      return -EINVAL;
   }

   /* first time calling this function */
   if (cfg->tot_cnt == 0) {
      /* power down USB PHY, this is not port specific */
      chipcHw_miscControlDisable(chipcHw_REG_MISC_CTRL_USB_POWEROFF);

      /* acquire and enable clocks */
      cfg->clk = clk_get(NULL, "USB");
      if (IS_ERR(cfg->clk)) {
         printk(KERN_ERR "USB: Unable to find USB clock\n");
         return PTR_ERR(cfg->clk);
      }
      rval = clk_set_rate(cfg->clk, USB_CLOCK_RATE_HZ);
      if (rval < 0) {
         printk(KERN_ERR "USB: Unable to set USB clock frequency\n");
         goto err_clk_put;
      }
      rval = clk_enable(cfg->clk);
      if (rval < 0) {
         printk(KERN_ERR "USB: Unable to enable USB clock\n");
         goto err_clk_put;
      }

      /*
       * Turn on UTMI clocks and AFEs for both ports. Need to be done here
       * before the USB block is initialized
       */
      usbPhyHw_ClkOn(0);
      usbPhyHw_ClkOn(1);
      usbPhyHw_PwrOn(0);
      usbPhyHw_PwrOn(1);


      /* initialize the USB block */
      usbPhyHw_BlkInit();

      /*
       * GNATs 8272 work-around: Set PHY MDIO register 9 bit 9 (afe_rxlogicr).
       * This will keep the CDR always running and prevent potential lockup in
       * the RX logic. This lockup has only been noticed with the USB Device
       * Controller (UDC), but the PHY MDIO register operations are not port
       * specific, so it is done here
       *
       * Note: the USB_DEBUG() read of register 0x16 is for determining if
       * read operations are working properly, and should return the value
       * 0xdead. The read of this register should have no impact on the write
       * to register 9
       */     
      usbPhyHw_MdioInit();
      USB_DEBUG( "***> usb_udc_init: mdio16=0x%04x\n", usbPhyHw_MdioRead( 0x16 ) );
      usbPhyHw_MdioWrite( 9, (1 << 9) );
      USB_DEBUG( "***> usb_udc_init: mdio9=0x%04x\n", usbPhyHw_MdioRead( 9 ) );

      /* power up the USB PHY */
      chipcHw_powerUpUsbPhy();
   }

   /* first time enabling this port */
   if (cfg->port_cnt[port_index] == 0) {
      if (port_index == 0) { /* port 0 config */
         gpio_status = gpiomux_requestGroup(gpiomux_group_usb0_pwronflt,
               "USB0 Host");
         if (gpio_status != gpiomux_rc_SUCCESS) {
            printk(KERN_ERR "USB: Unable to request usb0_pwrflt group\n");
            rval = -EFAULT;
            goto err_free_gpio;
         }
      } else { /* port 1 config */
         if (is_gadget) { /* port 1 as gadget */
            chipcHw_miscControlDisable(chipcHw_REG_MISC_CTRL_USB_MODE_HOST);
            usbDevHw_OpsFinis();
            usbDevHw_DeviceBusDisconnect();
            cfg->is_gadget = 1;
         } else { /* port 1 as host */
            gpio_status = gpiomux_requestGroup(gpiomux_group_usb1_pwronflt,
               "USB1 Host");
            if (gpio_status != gpiomux_rc_SUCCESS) {
               printk( KERN_ERR "USB: Unable to request usb1_pwrflt group\n");
               rval = -EFAULT;
               goto err_free_gpio;
            }
            /* set as host port */
            chipcHw_setUsbHost();
            cfg->is_gadget = 0;
         }
      }

      usbPhyHw_OpsInit(port_index);
      usbPhyHw_Start(port_index);

      printk(KERN_INFO "USB: Port %u has been brought up as %s\n",
         port_index, is_gadget ? "Gadget" : "Host");
   }

   /* register the PM notifier */
   if (cfg->tot_cnt == 0) {
      cfg->pm_notifier.notifier_call = &pm_callback;
      cfg->pm_notifier.priority = 99;
      rval = register_pm_notifier(&cfg->pm_notifier);
      if (rval < 0) {
         printk(KERN_ERR "USB: register_pm_notifier failed\n");
         goto err_phy_down;
      }

      rval = proc_init();
      if (rval < 0) {
         printk(KERN_ERR "USB: procfs init failed\n");
         goto err_unreg_pm;
      }
   }

   if (is_gadget == 0 && cfg->port_cnt[port_index] == 0)
      cfg->host_cnt++;
   cfg->tot_cnt++;
   cfg->port_cnt[port_index]++;

   return 0;

err_unreg_pm:
   if (cfg->tot_cnt == 0) {
      unregister_pm_notifier(&cfg->pm_notifier);
   }
   
err_phy_down:
   usbPhyHw_OpsFinis(port_index);

err_free_gpio:
   if (cfg->port_cnt[port_index] == 0) {
      if (port_index == 0) { /* port 0 config */
         gpiomux_freeGroup(gpiomux_group_usb0_pwronflt);
      } else if (is_gadget == 0) {
         gpiomux_freeGroup(gpiomux_group_usb1_pwronflt);
      }
   }
   
   if (cfg->tot_cnt == 0)
      clk_disable(cfg->clk);

err_clk_put:
   if (cfg->tot_cnt == 0) {
      clk_put(cfg->clk);
      cfg->clk = NULL;
   }
   return rval;
}
EXPORT_SYMBOL(bcmring_usb_init);

int bcmring_usb_term(unsigned int port_index)
{
   struct usb_cfg *cfg = &gUsbCfg;

   if (port_index >= MAX_NUM_USB_PORT ||
       cfg->port_cnt[port_index] == 0 ||
       cfg->tot_cnt == 0)
      return -EINVAL;

   cfg->port_cnt[port_index]--;
   cfg->tot_cnt--;

   /* disable port */
   if (cfg->port_cnt[port_index] == 0) {
      usbPhyHw_OpsFinis(port_index);

      if (port_index == 0) {
         gpiomux_freeGroup(gpiomux_group_usb0_pwronflt);
      } else if (!cfg->is_gadget) {
         gpiomux_freeGroup(gpiomux_group_usb1_pwronflt);
      }
   }

   /* disable the enture USB block */
   if (cfg->tot_cnt == 0) {
      proc_term();

      /* unregister PM callbacks */
      unregister_pm_notifier(&cfg->pm_notifier);

      /* power down the USB PHY */
      chipcHw_powerDownUsbPhy();

      /* disable the USB clocks */
      clk_disable(cfg->clk);
      clk_put(cfg->clk);
      cfg->clk = NULL;

      cfg->host_cnt = 0;
   }

   return 0;
}
EXPORT_SYMBOL(bcmring_usb_term);

int bcmring_usb_suspend(unsigned int port_index)
{
   struct usb_cfg *cfg = &gUsbCfg;

   if (port_index >= MAX_NUM_USB_PORT ||
       cfg->port_cnt[port_index] == 0 ||
       cfg->tot_cnt == 0)
      return -EINVAL;

   cfg->port_cnt[port_index]--;
   cfg->tot_cnt--;

   /* disconnect the port */
   if (cfg->port_cnt[port_index] == 0) {
      /* don't do anything for USB gadget */
      if (!(port_index == 1 && cfg->is_gadget))
         usbPhyHw_OpsFinis(port_index);
   }

   if (cfg->tot_cnt == 0) {
      /* power down the USB PHY */
      chipcHw_powerDownUsbPhy();
   
      /* disable the USB clock */
      clk_disable(cfg->clk);
   }

   return 0;
}
EXPORT_SYMBOL(bcmring_usb_suspend);

int bcmring_usb_resume(unsigned int port_index)
{
   int rval = -EFAULT;
   struct usb_cfg *cfg = &gUsbCfg;

   if (port_index >= MAX_NUM_USB_PORT)
      return -EINVAL;

   if (cfg->tot_cnt == 0) {
      rval = clk_enable(cfg->clk);
      if (rval < 0) {
         printk(KERN_ERR "USB: Unable to enable USB clock\n");
         return rval;
      }

      /* power up the USB PHY */
      chipcHw_powerUpUsbPhy();
   }

   if (cfg->port_cnt[port_index] == 0) {
      /* don't do anything for USB gadget */
      if (!(port_index == 1 && cfg->is_gadget)) {
         usbPhyHw_OpsInit(port_index);
         usbPhyHw_Start(port_index);
      }
   }

   cfg->port_cnt[port_index]++;
   cfg->tot_cnt++;

   return 0;
}
EXPORT_SYMBOL(bcmring_usb_resume);

static int pm_callback(struct notifier_block *nfb, unsigned long action,
      void *ignored)
{
   struct usb_cfg *cfg = &gUsbCfg;

   switch (action) {
      case PM_HIBERNATION_PREPARE:
      case PM_SUSPEND_PREPARE:
         chipcHw_setGpioPinFunction(GPIO36_USB0_PWRON,
               chipcHw_GPIO_FUNCTION_GPIO);
         
         gpio_direction_output(GPIO36_USB0_PWRON, 0);
         gpio_set_value(GPIO36_USB0_PWRON, 0);
         if (cfg->host_cnt == 2) {
            chipcHw_setGpioPinFunction(GPIO59_KEY_OUT7_ETM14_USB1_PWRON,
                  chipcHw_GPIO_FUNCTION_GPIO);
            gpio_direction_output(GPIO59_KEY_OUT7_ETM14_USB1_PWRON, 0);
            gpio_set_value(GPIO59_KEY_OUT7_ETM14_USB1_PWRON, 0);
         }
         return NOTIFY_OK;

      case PM_POST_HIBERNATION:
      case PM_POST_SUSPEND:
         chipcHw_setGpioPinFunction(GPIO36_USB0_PWRON,
               chipcHw_GPIO_FUNCTION_MISC);
         if (cfg->host_cnt == 2) {
            chipcHw_setGpioPinFunction(GPIO59_KEY_OUT7_ETM14_USB1_PWRON,
                  chipcHw_GPIO_FUNCTION_MISC);
         }
         return NOTIFY_OK;
   }
   
   return NOTIFY_DONE;
}

static int __init usb_init(void)
{
   return 0;
}

static void __exit usb_exit(void)
{
   return;
}

module_init(usb_init);
module_exit(usb_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCMRING USB Port Driver");
MODULE_LICENSE("GPL");
