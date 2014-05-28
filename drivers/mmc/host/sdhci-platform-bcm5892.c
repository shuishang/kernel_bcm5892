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
 * Copied platform_bcmring.c till a generic platform file evolves.
 */
/*
 * This serves as the SDHCI platform driver (for 5892 family chips) that
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

#include <linux/broadcom/sdio_platform.h>

#define DRIVER_NAME "sdhci_platform"

#define SDIO_CLK_FREQ_HZ 50000000

struct sdhci_platform_data {
   struct sdhci_host *host;
   struct clk *clk;
   unsigned host_num;
};
#ifdef CONFIG_BCM_SDIOWL

void bcm_sdiowl_reset_b(int onoff)
{
	printk("WL: bcm_sdiowl_reset_b not implemented\n");
}
EXPORT_SYMBOL(bcm_sdiowl_reset_b);
int bcm_sdiowl_rescan(void)
{
	printk("WL: bcm_sdiowl_rescan not implemented\n");
	return 0;
}
EXPORT_SYMBOL(bcm_sdiowl_rescan);
#endif


/*
 * Get the base clock
 */
unsigned long sdhci_platform_get_clk(struct sdhci_host *host)
{
   struct sdhci_platform_data *data = sdhci_priv(host);

   return SDIO_CLK_FREQ_HZ;
}

static struct sdhci_ops sdhci_platform_ops = {
   .enable_dma = NULL,
   .get_max_clk = sdhci_platform_get_clk,
};



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

   /* map registers */ /* Fixme -- use the mapped address from arch.c */
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

#if 0	
err_clk_disable:
   clk_disable(clk);

err_clk_put:
   clk_put(clk);
#endif

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
#if 0 
   clk_disable(data->clk);
   clk_put(data->clk);
#endif
   sdhci_free_host(host);
   release_mem_region(pdev->resource[0].start,
         pdev->resource[0].end - pdev->resource[0].start + 1);
   return 0;
}

#ifdef CONFIG_PM

static int sdhci_platform_suspend(struct platform_device *pdev,
      pm_message_t state)
{
   int ret=0;
   struct sdhci_platform_data *data = platform_get_drvdata(pdev);

#if 0
   if (!data || !data->clk) {
	printk("%s: %d\n", __FILE__, __LINE__);
      return -EINVAL;
   }

   ret = sdhci_suspend_host(data->host, state);
   if (ret < 0) {
	printk("%s: %d\n", __FILE__, __LINE__);
      return ret;
   }

#endif
#if 0
   clk_disable(data->clk);
#endif

   return 0;
}

static int sdhci_platform_resume(struct platform_device *pdev)
{
   int ret =0;
   struct sdhci_platform_data *data = platform_get_drvdata(pdev);

#if 0
   if (!data || !data->clk)
      return -EINVAL;

   ret = clk_enable(data->clk);

   if (ret < 0)
      return ret;

   return sdhci_resume_host(data->host);
#endif
   return 0;
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
 	printk( "SDIO: sdhci_platform_done\n");

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
