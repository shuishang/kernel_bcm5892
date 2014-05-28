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
*
*****************************************************************************
*
*  pm_bcm91161_common.c
*
*  PURPOSE:
*
*     Some common functions amongst BCM91161 Platforms
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>

#include <linux/fs.h>

#include <mach/reg_sys.h>
#include <mach/reg_clkpwr.h>
#include <linux/broadcom/gpio.h>
#include <linux/broadcom/regaccess.h>

#include <linux/broadcom/halaudio_settings.h>
#include <linux/broadcom/wlan_settings.h>
#include <cfg_global.h>
#include <linux/broadcom/PowerManager.h>
#include <linux/broadcom/pm_platforms.h>
#include <linux/delay.h>


/* ---- Private Function Prototypes -------------------------------------- */

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*   pm_bcm91161_platform_extra_init - common platform initialization routine
*
***************************************************************************/
void pm_bcm91161_platform_extra_init( void )
{
   /* Initialize UARTS (enable A and B) */
   REG_CLKPWR_CLK_UARTA_ENABLE = 1;
   REG_CLKPWR_CLK_UARTB_ENABLE = 1;

   /*
    * Legacy BT code enabled uart C, and never shut it off again, 
    * presumably since even though the core was told to sleep, 
    * it may have needed to talk later in a real application. 
    * Thus there should be no penalty leaving it on in general. 
    * Another way to to this might be to have platform specific 
    * code set or clear this bit, or perhaps another way is to 
    * have a CONFIG option drive setting this via a uartclk 
    * driver... 
    */
   REG_CLKPWR_CLK_UARTC_ENABLE = 1;
}

/****************************************************************************
*
*   pm_bcm91161_wifi_extra_init - common wifi initialization routine for most platforms
*
***************************************************************************/
void pm_bcm91161_wifi_extra_init( void )
{
   /* Configure SDIO pins as GPIO pins and drive the low to prevent leakage */
   regaccess_and_bits( &REG_SYS_IOCR0, ~REG_SYS_IOCR0_SPI_GPIO_MASK );
   regaccess_or_bits( &REG_SYS_IOCR0, (2 << 3) );

   gpio_direction_output( HW_GPIO_SDIOH_PIN_1, 0 );
   gpio_direction_output( HW_GPIO_SDIOH_PIN_2, 0 );
   gpio_direction_output( HW_GPIO_SDIOH_PIN_3, 0 );
   gpio_direction_output( HW_GPIO_SDIOH_PIN_4, 0 );
   gpio_direction_output( HW_GPIO_SDIOH_PIN_5, 0 );
   gpio_direction_output( HW_GPIO_SDIOH_PIN_6, 0 );

   REG_CLKPWR_CLK_SDIO0_ENABLE = 0;

#if defined( HW_GPIO_802_11_PWR_PIN )
   /* Set GPIO pin type */
   gpio_direction_output( HW_GPIO_802_11_PWR_PIN, 0 );
#endif
}


/****************************************************************************
*
*   pm_bcm91161_wifi_extra_action - common wifi action routine for most platforms
*
***************************************************************************/
void pm_bcm91161_wifi_extra_action(PM_CompPowerLevel powerLevel)
{
   if (powerLevel == PM_COMP_PWR_OFF)
   {
      /* Disable clock to SDIO host controller */
      REG_CLKPWR_CLK_SDIO0_ENABLE = 0;

      /* Configure SDIO pins as GPIO pins and drive the low to prevent leakage */
      regaccess_and_bits( &REG_SYS_IOCR0, ~REG_SYS_IOCR0_SPI_GPIO_MASK );
      regaccess_or_bits( &REG_SYS_IOCR0, (2 << 3) );
      gpio_direction_output( HW_GPIO_SDIOH_PIN_1, 0 );
      gpio_direction_output( HW_GPIO_SDIOH_PIN_2, 0 );
      gpio_direction_output( HW_GPIO_SDIOH_PIN_3, 0 );
      gpio_direction_output( HW_GPIO_SDIOH_PIN_4, 0 );
      gpio_direction_output( HW_GPIO_SDIOH_PIN_5, 0 );
      gpio_direction_output( HW_GPIO_SDIOH_PIN_6, 0 );

#if defined( HW_GPIO_802_11_PWR_PIN )
      /* Set GPIO */
      gpio_set_value( HW_GPIO_802_11_PWR_PIN, 0 );
#endif

   }
   else
   {
      /* Configure SDIO pins as SDIO pins */
      regaccess_and_bits( &REG_SYS_IOCR0, ~REG_SYS_IOCR0_SPI_GPIO_MASK );

      /* Enable clock to SDIO host controller */
      REG_CLKPWR_CLK_SDIO0_DIV = 1;
      REG_CLKPWR_CLK_SDIO0_ENABLE = 1;

#if defined( HW_GPIO_802_11_PWR_PIN )
      udelay (1000);
      /* Set GPIO */
      gpio_set_value( HW_GPIO_802_11_PWR_PIN, 1 );
      udelay (1000);
#endif
   }
}


/****************************************************************************
*
*   pm_bcm91161_usb_extra_init - common usb initialization routine for most platforms
*
***************************************************************************/
void pm_bcm91161_usb_extra_init( void )
{
   REG_CLKPWR_USBPLL_ENABLE = 0;
   REG_CLKPWR_USBPLL_OEN = 0;
   regaccess_or_bits( &REG_SYS_IOCR0, REG_SYS_IOCR0_GPIO_USB );
   regaccess_or_bits( &REG_SYS_IOCR2, REG_SYS_IOCR2_USB_SUSPEND );
#if defined( HW_GPIO_USB_RST_PIN )
   gpio_direction_output( HW_GPIO_USB_RST_PIN, 1 );
#endif
}


/****************************************************************************
*
*   pm_bcm91161_usb_extra_action - common usb action routine for most platforms
*
***************************************************************************/
void pm_bcm91161_usb_extra_action(PM_CompPowerLevel powerLevel)
{
   if (powerLevel == PM_COMP_PWR_OFF)
   {
      REG_CLKPWR_USBPLL_ENABLE = 0;
      REG_CLKPWR_USBPLL_OEN = 0;
   }
   else
   {
      REG_CLKPWR_USBPLL_ENABLE = 1;
      REG_CLKPWR_USBPLL_OEN = 1;
   }
}


/****************************************************************************
*
*   pm_bcm91161_audio_extra_init - common audio initialization routine for most platforms
*
***************************************************************************/
void pm_bcm91161_audio_extra_init( void )
{
   /* set the (active low) power down pin on the AK4642 external codec to 0 */
#if defined( HW_GPIO_CODEC_PDN_PIN )
   gpio_direction_output( HW_GPIO_CODEC_PDN_PIN, 0 );
#endif
}



