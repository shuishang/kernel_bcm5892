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

/*
 * This serves as the SDHCI platform driver (for BCMRING family chips) that
 * talks to the lower level SDHCI driver
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <cfg_global.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/gpio.h>

#include "sdhci.h"

#include <mach/csp/cap.h>
#include <linux/broadcom/bcmring/gpio_defs.h>
#include <linux/broadcom/sdio_platform.h>
#include <mach/csp/gpiomux.h>
#include <csp/sdioHw.h>

#define DRIVER_NAME "sdhci_platform"

#define SDIO_CLK_FREQ_HZ 50000000

struct sdhci_platform_data {
   struct sdhci_host *host;
   struct clk *clk;
   unsigned host_num;
};

#ifdef CONFIG_BCM_SDIOWL
#include <linux/sysdev.h>
static struct sdio_wifi_gpio_cfg *wifi_gpio;

void bcm_sdiowl_reset_b(int onoff)
{
	if (wifi_gpio && wifi_gpio->reset >= 0)
		gpio_set_value(wifi_gpio->reset, onoff);
	else
		printk(KERN_WARNING "SDIO: WLAN gpio reset pin not defined\n");
	/* Insert delay - required for chip to wake up or enter reset */
	msleep(200);
}
EXPORT_SYMBOL(bcm_sdiowl_reset_b);
static struct sdhci_host *host_sdio;
extern int sdio_reset_comm(struct mmc_card *card);
int bcm_sdiowl_rescan(void);
int bcm_sdiowl_rescan(void)
{
	struct mmc_host *mmc;
	struct mmc_card *card;
	int err = 0;
	if(host_sdio) {
		mmc = host_sdio->mmc;
		card = mmc->card;
		err = sdio_reset_comm(card);
	}
	return err;
}
EXPORT_SYMBOL(bcm_sdiowl_rescan);


int bcm_sdiowl_get_hwakeup_pin(void)
{ 
	if (wifi_gpio && wifi_gpio->host_wake >= 0)
		return wifi_gpio->host_wake;
}
EXPORT_SYMBOL(bcm_sdiowl_get_hwakeup_pin);


int bcm_sdiowl_is_hwakeup_pin_high(void)
{
	int WL_HOST_WAKE;
	WL_HOST_WAKE = bcm_sdiowl_get_hwakeup_pin();
	return gpio_get_value(WL_HOST_WAKE);
}
EXPORT_SYMBOL(bcm_sdiowl_is_hwakeup_pin_high);

static int hwakeup_pin = -1;

#ifdef CONFIG_PM
static irqreturn_t hostwakeup_isr(int irq, void *dev_id)
{
   /* Nothing to do.  Should not be called actually... */
   printk ("%s: %d\n", __FUNCTION__, __LINE__); //Ljin
   (void)(irq);
   (void)(dev_id);
   return IRQ_HANDLED;
}

static int wifi_hostwake_suspend(struct sys_device *dev, pm_message_t state)
{
   /* If Host wake requested. request GPIO as IRQ */
   if (hwakeup_pin != -1)
   {
      int ret;
      ret = request_irq(hwakeup_pin, hostwakeup_isr,
                       IRQF_DISABLED | IRQF_TRIGGER_HIGH, "hostwakeup", NULL);
      printk ("%s : Install hostwake ISR IRQ%d\n",__FUNCTION__,hwakeup_pin);
      if (ret) {
         /* Failed to install hostwake ISR */
         printk ("%s : Fail to install hostwake ISR\n",__FUNCTION__);
         return -EINVAL;
      }
   }
   return 0;
}

static int wifi_hostwake_resume(struct sys_device *dev)
{
   /* If Host wake requested. release GPIO as IRQ */
   if (hwakeup_pin != -1)
   {
      printk ("%s: Freeing IRQ %d\n",__FUNCTION__,hwakeup_pin);
      free_irq(hwakeup_pin, NULL);
   }
   return 0;
}
#else
#define wifi_hostwake_suspend NULL
#define wifi_hostwake_resume NULL
#endif

static struct sysdev_class wifi_hostwake_sysclass = {
   .name    = "wifi_hostwake",
   .suspend = wifi_hostwake_suspend,
   .resume  = wifi_hostwake_resume,
};

static struct sys_device wifi_hostwake_sysdev = {
   .id  = 0,
   .cls = &wifi_hostwake_sysclass,
};

int bcm_sdiowl_register_hwakeup_pin(int gpio);
int bcm_sdiowl_register_hwakeup_pin(int gpio)
{
   hwakeup_pin = gpio;
   return 0;
}
EXPORT_SYMBOL(bcm_sdiowl_register_hwakeup_pin);

void bcm_sdiowl_unregister_hwakeup_pin(void);
void bcm_sdiowl_unregister_hwakeup_pin(void)
{
   hwakeup_pin = -1;
}
EXPORT_SYMBOL(bcm_sdiowl_unregister_hwakeup_pin);

#endif

/*
 * Get the base clock
 */
unsigned long sdhci_platform_get_clk(struct sdhci_host *host)
{
   struct sdhci_platform_data *data = sdhci_priv(host);

   return clk_get_rate(data->clk);
}

static struct sdhci_ops sdhci_platform_ops = {
   .enable_dma = NULL,
   .get_max_clk = sdhci_platform_get_clk,
};

#ifdef CONFIG_BCM_SDIOWL
static int wifi_gpio_request(struct sdio_wifi_gpio_cfg *gpio)
{
   int ret;

   if (gpio->reg >= 0) {
      ret = gpio_request(gpio->reg, "WL_REG_ON");
      if (ret < 0)
      {
         printk(KERN_ERR "%s(): Unable to request GPIO pin %d\n",
               __FUNCTION__, gpio->reg);
         return -EBUSY;
      }
      /* ensure GPIO pins set to output mode */
      gpio_direction_output(gpio->reg, 1);
      /* disable by default */
      gpio_set_value(gpio->reg, 1);
   }

   if (gpio->reset >= 0) {
      ret = gpio_request(gpio->reset, "WL_RST");
      if (ret < 0)
      {
         printk(KERN_ERR "%s(): Unable to request GPIO pin %d\n",
               __FUNCTION__, gpio->reset);
         goto err_reg;
      }
      /* ensure GPIO pins set to output mode */
      gpio_direction_output(gpio->reset, 1);
      /* disable by default */
      gpio_set_value(gpio->reset, 1);
   }

   if (gpio->shutdown >= 0) {
      ret = gpio_request(gpio->shutdown, "WL_WAKE");
      if (ret < 0)
      {
         printk(KERN_ERR "%s(): Unable to request GPIO pin %d\n",
               __FUNCTION__, gpio->shutdown);
         goto err_reset;
      }
      gpio_direction_output(gpio->shutdown, 1);
      /* disable by default */
      gpio_set_value(gpio->shutdown, 1);
   }

   if (gpio->host_wake >= 0) {
      ret = gpio_request(gpio->host_wake, "WL_HOST_WAKE");
      if (ret < 0)
      {
         printk(KERN_ERR "%s(): Unable to request GPIO pin %d\n",
               __FUNCTION__, gpio->host_wake);
         goto err_shutdown;
      }
      gpio_direction_input(gpio->host_wake);
   }


   return 0;

err_shutdown:
   if (gpio->shutdown >= 0)
      gpiomux_free(gpio->shutdown);

err_reset:
   if (gpio->reset >= 0)
      gpiomux_free(gpio->reset);
	  
err_reg:
   if (gpio->reg >= 0)
      gpiomux_free(gpio->reg);

   return ret;
}
#endif

#ifdef CONFIG_BCM_SDIOWL
static void wifi_gpio_free(struct sdio_wifi_gpio_cfg *gpio)
{
   if (gpio->reg >= 0)
      gpiomux_free(gpio->reg);

   if (gpio->reset >= 0)
      gpiomux_free(gpio->reset);

   if (gpio->shutdown >= 0)
      gpiomux_free(gpio->shutdown);

   if (gpio->host_wake >= 0)
      gpiomux_free(gpio->host_wake);
	  
}
#endif

static int __init sdhci_platform_probe(struct platform_device *pdev)
{
   struct resource *res;
   struct sdhci_host *host;
   struct sdio_platform_cfg *cfg;
   struct sdhci_platform_data *data;
   struct clk *clk;
#define STR_SIZE 16
   char name_str[STR_SIZE];
   int ret = 0;
   int irq;

   if (cap_isPresent(CAP_SDIO, pdev->id) != CAP_PRESENT) {
      printk(KERN_WARNING "SDIO%d is NOT supported\n", pdev->id);
      return -ENODEV;
   }

   res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   irq = platform_get_irq(pdev, 0);
   if (res == NULL || irq < 0) {
      printk(KERN_ERR "SDIO%d: Unable to get platform resource or IRQ\n",
            pdev->id);
      return -ENXIO;
   }

   res = request_mem_region(res->start, res->end - res->start + 1,
         pdev->name);
   if (res == NULL) {
      printk(KERN_ERR "SDIO%d: request_mem_region failed\n", pdev->id);
      return -EBUSY;
   }

   cfg = pdev->dev.platform_data;
   if (cfg == NULL) {
      printk(KERN_ERR "SDIO%d: Unable to get platform data\n",
            pdev->id);
      ret = -ENODEV;
      goto err_free_mem_region;
   }

   /* allocate SDHCI host + platform data memory */
   host = sdhci_alloc_host(&pdev->dev, sizeof(struct sdhci_platform_data));
   if (IS_ERR(host)) {
      ret = PTR_ERR(host);
      printk(KERN_ERR "SDIO%d: Unable to allocate SDHCI host\n",
            pdev->id);
      goto err_free_mem_region;
   }

   /* set up data structure */
   data = sdhci_priv(host);
   data->host = host;
   data->host_num = pdev->id;
   host->hw_name = "BCM-SDIO";
   host->quirks = 0;
   host->ops = &sdhci_platform_ops;
   host->irq = irq;

   /* get SDIO clock */
   snprintf(name_str, STR_SIZE, "%s%d", "SDIO", data->host_num);
   clk = clk_get(NULL, name_str);
   if (IS_ERR(clk)) {
      printk(KERN_ERR "SDIO%d: Unable to get clock\n", pdev->id);
      ret = PTR_ERR(clk);
      goto err_free_host;
   }

   /* set to desired clock rate and enable the SDIO clock */
   ret = clk_set_rate(clk, SDIO_CLK_FREQ_HZ);
   if (ret < 0) {
      printk(KERN_ERR "SDIO%d: Unable to set to clock rate=%u\n",
            pdev->id, SDIO_CLK_FREQ_HZ);
      goto err_clk_put;
   }
   ret = clk_enable(clk);
   if (ret < 0) {
      printk(KERN_ERR "SDIO%d: Unable to enable clock\n",
            pdev->id);
      goto err_clk_put;
   }
   data->clk = clk;

   /* set GPIO MUX for SDIO, 4-bit mode by default */
   if (pdev->id == 0) { /* SDIO0 */
      if (gpiomux_requestGroup(gpiomux_group_sdio0_4, "SDIO0 SD/MMC") !=
            gpiomux_rc_SUCCESS) {
         printk(KERN_ERR "SDIO%d: Unable reserve SDIO0_4 GPIO group\n",
               pdev->id);
         ret = -EBUSY;
         goto err_clk_disable;
      }
   } else if (pdev->id == 1) { /* SDIO1 */
      if (gpiomux_requestGroup(gpiomux_group_sdio1_4, "SDIO1 SD/MMC") !=
         gpiomux_rc_SUCCESS) {
         printk(KERN_ERR "SDIO%d: Unable reserve SDIO1_4 GPIO group\n",
               pdev->id);
         ret = -EBUSY;
         goto err_clk_disable;
      }
   }
   
#ifdef CONFIG_BCM_SDIOWL
   if (cfg->devtype == SDIO_DEV_TYPE_WIFI) {
      wifi_gpio = &cfg->dev_option.wifi_gpio;
      /* request WiFi GPIOs */
      ret = wifi_gpio_request(wifi_gpio);
      if (ret < 0) {
         printk(KERN_ERR "SDIO%d: Unable reserve WiFi GPIOs\n",
               pdev->id);
         goto err_free_gpio;
      }
      host_sdio = host;

      /* create sysdev device to access sysdev suspend/resume with interrupts disabled */
      sysdev_class_register(&wifi_hostwake_sysclass);
      sysdev_register(&wifi_hostwake_sysdev);

      /* reset the wireless chip to make sure it is in a clean state */
      bcm_sdiowl_reset_b(0);
      msleep(100);
      bcm_sdiowl_reset_b(1);
      msleep(100);
   }
#endif

   /* now, initialize the SDIO block */
   SdioHw_Init(pdev->id, 0, cfg->data_pullup);

   /* map registers */
   host->ioaddr = ioremap_nocache(res->start, (res->end - res->start) + 1);
   if (!host->ioaddr) {
      printk(KERN_ERR "SDIO%d: Unable to iomap SDIO registers\n",
               pdev->id);
      ret = -ENXIO;
      goto err_sdio_exit;
   }

   platform_set_drvdata(pdev, data);
   ret = sdhci_add_host(host);
   if (ret) {
      printk(KERN_ERR "SDIO%d: Failed to add SDHCI host\n",
               pdev->id);
      goto err_iounmap;
   }
   
   return 0;

err_iounmap:
   iounmap(host->ioaddr);

err_sdio_exit:
   SdioHw_Exit(pdev->id);


#ifdef CONFIG_BCM_SDIOWL
   if (cfg->devtype == SDIO_DEV_TYPE_WIFI)
   {
      wifi_gpio_free(wifi_gpio);
      sysdev_unregister(&wifi_hostwake_sysdev);
      sysdev_class_unregister(&wifi_hostwake_sysclass);
   }

   wifi_gpio = NULL;

err_free_gpio:
#endif
   if (pdev->id == 0)
      gpiomux_freeGroup(gpiomux_group_sdio0_4);
   else if (pdev->id == 1)
      gpiomux_freeGroup(gpiomux_group_sdio1_4);

err_clk_disable:
   clk_disable(clk);

err_clk_put:
   clk_put(clk);

err_free_host:
   sdhci_free_host(host);

err_free_mem_region:
   release_mem_region(res->start, res->end - res->start + 1);
   return ret;
}

static int __devexit sdhci_platform_remove(struct platform_device *pdev)
{
   int dead = 0;
   struct sdhci_platform_data *data = platform_get_drvdata(pdev);
#ifdef CONFIG_BCM_SDIOWL
   struct sdio_platform_cfg *cfg = pdev->dev.platform_data;
#endif
   u32 scratch = readl(data->host->ioaddr + SDHCI_INT_STATUS);
   struct sdhci_host *host = data->host;

   if (scratch == (u32)-1)
      dead = 1;

   sdhci_remove_host(host, 0);
   platform_set_drvdata(pdev, NULL);
   iounmap(host->ioaddr);
   if (pdev->id == 0) {
      SdioHw_Exit(0);
      gpiomux_freeGroup(gpiomux_group_sdio0_4);
   } else if(pdev->id == 1) {
      SdioHw_Exit(1);
      gpiomux_freeGroup(gpiomux_group_sdio1_4);
   }
#ifdef CONFIG_BCM_SDIOWL
   if (cfg->devtype == SDIO_DEV_TYPE_WIFI)
   {
      wifi_gpio_free(&cfg->dev_option.wifi_gpio);
      sysdev_unregister(&wifi_hostwake_sysdev);
      sysdev_class_unregister(&wifi_hostwake_sysclass);
   }

   wifi_gpio = NULL;
#endif
   clk_disable(data->clk);
   clk_put(data->clk);
   sdhci_free_host(host);
   release_mem_region(pdev->resource[0].start,
         pdev->resource[0].end - pdev->resource[0].start + 1);
   return 0;
}

#ifdef CONFIG_PM

static int sdhci_platform_suspend(struct platform_device *pdev,
      pm_message_t state)
{
   int ret;
   struct sdhci_platform_data *data = platform_get_drvdata(pdev);
   struct sdio_platform_cfg *cfg = pdev->dev.platform_data;
 
   if (!data || !data->clk)
      return -EINVAL;

   if (cfg->devtype == SDIO_DEV_TYPE_WIFI) /* SDIO1/wifi -- this is required for system wake up upon wifi packets */
   {
      clk_disable(data->clk);
      return 0;
   }

   ret = sdhci_suspend_host(data->host, state);
   if (ret < 0)
      return ret;

   clk_disable(data->clk);

   return 0;
}

static int sdhci_platform_resume(struct platform_device *pdev)
{
   int ret;
   struct sdhci_platform_data *data = platform_get_drvdata(pdev);
   struct sdio_platform_cfg *cfg = pdev->dev.platform_data;
   

   if (!data || !data->clk)
      return -EINVAL;

   ret = clk_enable(data->clk);
   if (cfg->devtype == SDIO_DEV_TYPE_WIFI) /* SDIO1/wifi -- this is required for system wake up upon wifi packets */
      return ret;
   
   if (ret < 0)
      return ret;

   return sdhci_resume_host(data->host);
}

#else /* CONFIG_PM */

#define sdhci_platform_suspend NULL
#define sdhci_platform_resume NULL

#endif /* CONFIG_PM */

static struct platform_driver sdhci_platform_driver = {
   .probe = sdhci_platform_probe,
   .remove = __devexit_p(sdhci_platform_remove),
   .suspend = sdhci_platform_suspend,
   .resume = sdhci_platform_resume,
   .driver = {
      .name = "bcm-sdio",
      .owner = THIS_MODULE,
   },
};

static int __init sdhci_platform_init(void)
{
   int ret;

   ret = platform_driver_register(&sdhci_platform_driver);
   if (ret) {
      printk(KERN_ERR DRIVER_NAME
            ": Unable to register the SDHCI Platform driver\n");
   return ret;
   }

   return 0;
}

static void __exit sdhci_platform_exit(void)
{
   platform_driver_unregister(&sdhci_platform_driver);
}

module_init(sdhci_platform_init);
module_exit(sdhci_platform_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("SDHCI Platform driver");
MODULE_LICENSE("GPL");
