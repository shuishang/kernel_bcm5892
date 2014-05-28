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




/*
*
*****************************************************************************
*
*  lcd_backlight.c
*
*  PURPOSE:
*
*   This implements the 116x specific LCD backlight driver.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/mutex.h>

#include <linux/broadcom/lcd_backlight.h>
#include <linux/broadcom/gpio.h>
#include <linux/broadcom/hw_cfg.h>

#include <asm/atomic.h>
#include <mach/reg_lcd.h>
#include <mach/reg_sys.h>
#include <linux/broadcom/regaccess.h>

#ifdef CONFIG_BCM_SLEEP_MODE
   #include <linux/broadcom/cpu_sleep.h>
#endif


/*
 * ---- Public Variables ------------------------------------------------- 
 * ---- Private Constants and Types -------------------------------------- 
 * ---- Private Variables ------------------------------------------------ 
 */

static LCD_BACKLIGHT_LEVEL  gCurrLevel = LCD_BACKLIGHT_FULL_ON / 2;

/*
 * ---- Private Function Prototypes -------------------------------------- 
 * ---- Functions -------------------------------------------------------- 
 */

/****************************************************************************
*
*  lcd_backlight_init
*
*
***************************************************************************/
void lcd_backlight_init( void )
{
   gpio_request( HW_GPIO_LCD_MAIN_BACKLIGHT_ENABLE, "LCD Backlight" );
   gpio_direction_output( HW_GPIO_LCD_MAIN_BACKLIGHT_ENABLE, gCurrLevel );
   lcd_backlight_enable( gCurrLevel );
}

/****************************************************************************
*
*  lcd_backlight_deinit
*
*
***************************************************************************/
void lcd_backlight_deinit( void )
{
    lcd_backlight_enable( LCD_BACKLIGHT_OFF );
    gpio_free( HW_GPIO_LCD_MAIN_BACKLIGHT_ENABLE );
}

/****************************************************************************
*
*  lcd_enable_backlight
*
*
***************************************************************************/
void lcd_backlight_enable( LCD_BACKLIGHT_LEVEL level )
{
#ifdef CONFIG_BCM_SLEEP_MODE
   static DEFINE_MUTEX(modSem);
   static int modulated_backlight = 0;
#endif

   if (( level == LCD_BACKLIGHT_OFF ) || ( level >= LCD_BACKLIGHT_MAX_LEVEL ))
   {
      /* Disable backlight controller (PWM not required) */

      REG_LCD_BLCR = 0;
      regaccess_and_bits( &REG_SYS_IOCR0, ~REG_SYS_IOCR0_BKLIGHT );
      gpio_set_value( HW_GPIO_LCD_MAIN_BACKLIGHT_ENABLE, level );

      #ifdef CONFIG_BCM_SLEEP_MODE
      {
         /* If backlight was previously modulated, we no longer need to override sleep mode */
         mutex_lock(&modSem);
         if (modulated_backlight)
         {
            modulated_backlight = 0;
            atomic_dec(&cpu_sleep_override);
         }
         mutex_unlock(&modSem);
      }
      #endif
   }
   else
   {
      #ifdef CONFIG_BCM_SLEEP_MODE
      {
         /* Modulated backlight requires 13MHz PLL so we need to override sleep mode */
         mutex_lock(&modSem);
         if (!modulated_backlight)
         {
            modulated_backlight = 1;
            atomic_inc(&cpu_sleep_override);
         }
         mutex_unlock(&modSem);
      }
      #endif


      /*
       * Enable modulated backlight controller 
       * Other values allow backlight dimmer control 
       */
      regaccess_or_bits( &REG_SYS_IOCR0, REG_SYS_IOCR0_BKLIGHT );
      REG_LCD_BLCR = (min((int)level, LCD_BACKLIGHT_MAX_LEVEL) << REG_LCD_BLCR_DUTYSHFT) |
                      REG_LCD_BLCR_FREQ_50KHZ | REG_LCD_BLCR_MOD_ON;
   }
   gCurrLevel = level;
}

/****************************************************************************
*
*   Returns the highest LCD Backlight level that this device supports.
*
***************************************************************************/

LCD_BACKLIGHT_LEVEL lcd_backlight_max_level( void )
{
    return LCD_BACKLIGHT_FULL_ON;
}

/****************************************************************************
*
*   Returns the current LCD Backlight level.
*
***************************************************************************/

LCD_BACKLIGHT_LEVEL lcd_backlight_curr_level( void )
{
    return gCurrLevel;
}

