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

/*
 * This is the BCMRING LED Matrix driver
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/leds.h>
#include <linux/broadcom/bcmring/gpio_defs.h>
#include <mach/csp/gpiomux.h>
#include <mach/csp/ledmHw_reg.h>
#include <mach/csp/cap.h>
#include <csp/ledmHw.h>

#include <linux/broadcom/ledm.h>

#define MAX_LED_NAME_LEN 10

struct ledm {
   struct led_classdev cdev;
   char name[MAX_LED_NAME_LEN];
   unsigned int row_index;
   unsigned int col_index;
};

static struct clk *led_clk;
static char banner[] __initdata = KERN_INFO "Broadcom LED Matrix Driver\n";

/*
 * Set the clock divisor
 */
static int led_clk_set(LEDM_CLK_DIV divisor)
{
   switch (divisor) {
      case LEDM_CLK_DIV_2:
         LEDMHW_SCANCLK_Set(LEDMHW_SERIAL_83_0_MHZ);
         break;

      case LEDM_CLK_DIV_4:
         LEDMHW_SCANCLK_Set(LEDMHW_SERIAL_41_5_MHZ);
         break;
         
      case LEDM_CLK_DIV_8:
         LEDMHW_SCANCLK_Set(LEDMHW_SERIAL_20_75_MHZ);
         break;

      case LEDM_CLK_DIV_16:
         LEDMHW_SCANCLK_Set(LEDMHW_SERIAL_10_375_MHZ);
         break;

      default:
         return -1;
   }

   return 0;
}

static void ledm_brightness_set(struct led_classdev *cdev,
      enum led_brightness brightness)
{
   unsigned on_off;
   struct ledm *led;

   if (brightness)
      on_off = 1;
   else
      on_off = 0;
   
   led = container_of(cdev, struct ledm, cdev);

   ledmHw_WriteCoords(led->row_index + 1, led->col_index + 1, on_off);
}

static int ledm_probe(struct platform_device *pdev)
{
   LEDM_CFG *cfg;
   struct led_platform_data *pdata;
   struct ledm *ledm_array;
   unsigned int i, j, led_index, serial_mode;
   int rc = 0;
   gpiomux_rc_e gpio_rc;

   if (cap_isPresent(CAP_LEDM, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "LEDM: Not supported\n");
      return -ENODEV;
   }

   /* make sure the LED platform device is there */
   cfg = pdev->dev.platform_data;
   if (cfg == NULL || cfg->row_cnt < 1 || cfg->col_cnt < 1) {
      printk(KERN_ERR "LEDM: Platform data invalid\n");
      return -ENODEV;
   }
   pdata = cfg->leds;
   if (pdata == NULL || pdata->num_leds < 1) {
      printk(KERN_ERR "LEDM: led_platform_data invalid\n");
      return -ENODEV;
   }

   /* allocate memories for the LEDM array */
   ledm_array = kcalloc(pdata->num_leds, sizeof(*ledm_array), GFP_KERNEL);
   if (ledm_array == NULL)
      return -ENOMEM;

   if (cfg->mode == LEDM_MODE_SERIAL) {
      /* serial mode just need 2 GPIOMUX pins */
      gpio_rc = gpiomux_request(GPIO16_MTX_Y0_MTX_SCAN_DA_ETM26,
            chipcHw_GPIO_FUNCTION_LEDMTXS, "LEDM");
      if (gpio_rc != gpiomux_rc_SUCCESS) {
         printk(KERN_ERR "LEDM: Unable to request GPIO pin %u\n",
               GPIO16_MTX_Y0_MTX_SCAN_DA_ETM26);
         rc = -EFAULT;
         goto err_kfree;
      }
      gpio_rc = gpiomux_request(GPIO17_MTX_Y1_MTX_SCAN_CLK_ETM27,
         chipcHw_GPIO_FUNCTION_LEDMTXS, "LEDM");
      if (gpio_rc != gpiomux_rc_SUCCESS) {
         printk(KERN_ERR "LEDM: Unable to request GPIO pin %u\n",
               GPIO17_MTX_Y1_MTX_SCAN_CLK_ETM27);
         rc= -EFAULT;
         goto err_free_gpio;
      }
   
      serial_mode = 1;
   } else {
      /* parallel mode */

      #define GPIO_OFFSET_ROW    GPIO16_MTX_Y0_MTX_SCAN_DA_ETM26
      #define GPIO_OFFSET_COL    GPIO00_SCL_MTX_X0_EPHYLED_DATA

      /* reserve GPIOMUX pins according to row/col map */
      for (i = 0; i < cfg->row_cnt; i++) {
         gpio_rc = gpiomux_request(cfg->row_map[i] + GPIO_OFFSET_ROW,
               chipcHw_GPIO_FUNCTION_LEDMTXP, "LEDM");
         if (gpio_rc != gpiomux_rc_SUCCESS) {
            printk(KERN_ERR "LEDM: Unable to request GPIO pin %u\n",
                  cfg->row_map[i] + GPIO_OFFSET_ROW);
            rc = -EFAULT;
            goto err_free_gpio;
         }
      }

      for (i = 0; i < cfg->col_cnt; i++) {
         gpio_rc = gpiomux_request(cfg->col_map[i] + GPIO_OFFSET_COL,
               chipcHw_GPIO_FUNCTION_LEDMTXP, "LEDM");
         if (gpio_rc != gpiomux_rc_SUCCESS) {
            printk(KERN_ERR "LEDM: Unable to request GPIO pin %u\n",
                  cfg->col_map[i] + GPIO_OFFSET_COL);
            rc = -EFAULT;
            goto err_free_gpio;
         }
      }
      serial_mode = 0;
   }

   /* enable the bus interface clock */
   chipcHw_busInterfaceClockEnable(chipcHw_REG_BUS_CLOCK_LED);

   /* enable the input clock via the clock framework */
   led_clk = clk_get(NULL, "LED_PWM");
   if (IS_ERR(led_clk)) {
      printk(KERN_ERR "LEDM: Unable to find LED_PWM clock\n");
      goto err_free_gpio;
   }

   rc = clk_enable(led_clk);
   if (rc < 0) {
      printk(KERN_ERR "LEDM: Unable to enable LED_PWM clock\n");
      goto err_free_gpio;
   }

   led_clk_set(cfg->clk_div);
   LEDMHW_RESOLUTION_Set(cfg->res_div);
   LEDMHW_OnTime_Set(cfg->on_time);
   LEDMHW_GuardTime_Set(cfg->guard_time);

   /* initialize values to 0 */
   LEDMHW_NextPat_0Set(0);
   LEDMHW_NextPat_1Set(0);
   LEDMHW_NextPat_2Set(0);
   LEDMHW_NextPat_3Set(0);
   LEDMHW_NextPat_4Set(0);
   LEDMHW_NextPat_5Set(0);

   if (cfg->mode == LEDM_MODE_SERIAL) {
      rc = ledmHw_Init(serial_mode, cfg->row_cnt, cfg->col_cnt,
            cfg->row_on_val, cfg->col_on_val);
   } else {
      rc = ledmHw_Init(serial_mode, LEDM_MAX_ROWS, LEDM_MAX_COLS,
            cfg->row_on_val, cfg->col_on_val);
   }

   if (rc != 0) {
      printk(KERN_ERR "LEDML: ledmHw_Init failed\n");
      rc = -EFAULT;
      goto err_free_clk;
   }

   /* go through each LED and register to Linux LED class */
   led_index = 0;
   for (i = 0; i < cfg->row_cnt; i++) {
      for (j = 0; j < cfg->col_cnt; j++) {
         struct ledm *led = ledm_array + led_index;
         snprintf(led->name, sizeof(led->name), "LED%u", led_index);
         
         if (cfg->mode == LEDM_MODE_SERIAL) {
            led->row_index = i;
            led->col_index = j;
         } else {
            led->row_index = cfg->row_map[i];
            led->col_index = cfg->col_map[j];
         }

         led->cdev.name = led->name;
         led->cdev.brightness = LED_OFF;
         led->cdev.brightness_set = ledm_brightness_set;
         
         if (led_classdev_register(&pdev->dev, &led->cdev) < 0) {
            if (cfg->mode == LEDM_MODE_SERIAL) {
               printk(KERN_ERR "LEDM: led_classdev_register failed for LED "
                     "row=%u col=%u\n", i, j);
            } else {
               printk(KERN_ERR "LEDM: led_classdev_register failed for LED "
                     "row=%u col=%u\n", cfg->row_map[i], cfg->col_map[j]);
            }
            rc = -EFAULT;
            goto err_unregister;
         }
         led_index++;
      }
   }

   printk(KERN_INFO "LEDM: Driver initialized\n");
   printk(KERN_INFO "LEDM: mode=%s clk_div=%u res_div=%u on_time=%u "
         "guard_time=%u\n", cfg->mode == LEDM_MODE_SERIAL ? "serial" : "parallel",
         cfg->clk_div, cfg->res_div, cfg->on_time, cfg->guard_time);
   printk(KERN_INFO "LEDM: rows=%u cols=%u num_leds=%u\n",
         cfg->row_cnt, cfg->col_cnt, pdata->num_leds);

   platform_set_drvdata(pdev, ledm_array);
   return 0;

err_unregister:
   for (i = 0; i < pdata->num_leds; i++)
      led_classdev_unregister(&ledm_array[i].cdev);
err_free_clk:
   ledmHw_Exit();
   clk_disable(led_clk);
   clk_put(led_clk);
err_free_gpio:
   if (cfg->mode == LEDM_MODE_SERIAL) {
      gpiomux_free(GPIO16_MTX_Y0_MTX_SCAN_DA_ETM26);
      gpiomux_free(GPIO17_MTX_Y1_MTX_SCAN_CLK_ETM27);
   } else {
      for (i = 0; i < cfg->row_cnt; i++)
         gpiomux_free(cfg->row_map[i] + GPIO_OFFSET_ROW);

      for (i = 0; i < cfg->col_cnt; i++)
         gpiomux_free(cfg->col_map[i] + GPIO_OFFSET_COL);
   }
err_kfree:
   kfree(ledm_array);
   return rc;
}

static int ledm_remove(struct platform_device *pdev)
{
   LEDM_CFG *cfg;
   const struct led_platform_data *pdata;
   struct ledm *leds;
   unsigned i;

   cfg = pdev->dev.platform_data;
   pdata = cfg->leds;
   leds = platform_get_drvdata(pdev);

   for (i = 0; i < pdata->num_leds; i++) {
      struct ledm *led = leds + i;
      led_classdev_unregister(&led->cdev);
   }
   
   kfree(leds);
   platform_set_drvdata(pdev, NULL);

   ledmHw_Exit();
   clk_disable(led_clk);
   clk_put(led_clk);

   if (cfg->mode == LEDM_MODE_SERIAL) {
      gpiomux_free(GPIO16_MTX_Y0_MTX_SCAN_DA_ETM26);
      gpiomux_free(GPIO17_MTX_Y1_MTX_SCAN_CLK_ETM27);
   } else {
      for (i = 0; i < cfg->row_cnt; i++)
         gpiomux_free(cfg->row_map[i] + GPIO_OFFSET_ROW);

      for (i = 0; i < cfg->col_cnt; i++)
         gpiomux_free(cfg->col_map[i] + GPIO_OFFSET_COL);
   }

   return 0;
}

static struct platform_driver ledm_driver = {
   .driver = {
      .name = "bcmring-ledm",
      .owner = THIS_MODULE,
   },
   .probe = ledm_probe,
   .remove = ledm_remove,
};

static int __init ledm_init(void)
{
   printk(banner);
   return platform_driver_register(&ledm_driver);
}

static void __exit ledm_exit(void)
{
   platform_driver_unregister(&ledm_driver);
}

module_init(ledm_init);
module_exit(ledm_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("LED Matrix Driver");
MODULE_LICENSE("GPL");
