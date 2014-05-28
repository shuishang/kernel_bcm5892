/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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
#include <linux/err.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/semaphore.h>
#include <mach/csp/chipcHw_inline.h>
#include <linux/broadcom/gpio.h>
#include <linux/broadcom/lcd_backlight.h>

#include <mach/csp/gpiomux.h>
#include <csp/pwmHw.h>

struct lcd_backlight_param gParam;

static atomic_t gCurrLevel;
static struct semaphore gLock;
static struct clk *clk;

void lcd_backlight_init(struct lcd_backlight_param *param)
{
   int rval;
   atomic_set(&gCurrLevel, LCD_BACKLIGHT_FULL_ON);
   init_MUTEX(&gLock); /* unlocked */

   memcpy(&gParam, param, sizeof(gParam));

   /* enable the bus interface clock */
   chipcHw_busInterfaceClockEnable(chipcHw_REG_BUS_CLOCK_LED);

   /* enable the input clock via the clock framework */
   clk = clk_get(NULL, "LED_PWM");
   if (IS_ERR(clk)) {
      printk(KERN_ERR "Backlight: Unable to find LED_PWM clock\n");
      return;
   }

   rval = clk_enable(clk);
   if (rval < 0) {
      printk(KERN_ERR "Backlight: Unable to enable LED_PWM clock\n");
      return;
   }

   /* reserve and set MUX to GPIO, and set it as output */
   gpiomux_request(param->gpio, chipcHw_GPIO_FUNCTION_GPIO,
         "LCD Backlight");
   gpio_direction_output(param->gpio, 1);

   /* set to a fixed PWM frequency */
   pwmHw_FreqSet(param->pwm_freq);

   /* now enable the backlight */
   lcd_backlight_enable(atomic_read(&gCurrLevel));
}

void lcd_backlight_deinit(void)
{
   lcd_backlight_enable(LCD_BACKLIGHT_OFF);
   pwmHw_DisableModulation();
   gpiomux_free(gParam.gpio);

   clk_disable(clk);
   clk_put(clk);
}

void lcd_backlight_enable(LCD_BACKLIGHT_LEVEL level)
{
   gpiomux_rc_e status;

   if (down_interruptible(&gLock) != 0) {
      printk(KERN_WARNING "Backlight: lock interrupted by signals, "
            "try again\n");
      return;
   }

   if (level == LCD_BACKLIGHT_OFF || level > LCD_BACKLIGHT_MAX_LEVEL) {
      /*
       * Disable PWM and use GPIO to fully turn on/off the LCD backlight
       * to save power
       */
      gpiomux_free(gParam.gpio);
      /* reserve and set MUX to GPIO, and set it as output */
      status = gpiomux_request(gParam.gpio,
            chipcHw_GPIO_FUNCTION_GPIO, "LCD Backlight");
      if (status != gpiomux_rc_SUCCESS) {
         printk(KERN_ERR "Backlight: gpiomux_request failed with err=%d\n",
               status);
         up(&gLock);
         return;
      }
      gpio_direction_output(gParam.gpio, 1);
      gpio_set_value(gParam.gpio, level);

      /* now disable PWM */
      pwmHw_DisableModulation();
   } else {
        /* use PWM to control backlight */
      pwmHw_DutyCycleSet(level);
      
      pwmHw_EnableModulation();

      gpiomux_free(gParam.gpio);
      /* reserve and set MUX to PWM */
      status = gpiomux_request(gParam.gpio, chipcHw_GPIO_FUNCTION_MISC,
            "LCD Backlight");
      if (status != gpiomux_rc_SUCCESS) {
         printk(KERN_ERR "Backlight: gpiomux_request failed with err=%d\n",
               status);
         up(&gLock);
         return;
      }
   }

   atomic_set(&gCurrLevel, level);
   up(&gLock);
}
EXPORT_SYMBOL(lcd_backlight_enable);

LCD_BACKLIGHT_LEVEL lcd_backlight_max_level(void)
{
   return LCD_BACKLIGHT_FULL_ON;
}

LCD_BACKLIGHT_LEVEL lcd_backlight_curr_level(void)
{
   return atomic_read(&gCurrLevel);
}
EXPORT_SYMBOL(lcd_backlight_curr_level);