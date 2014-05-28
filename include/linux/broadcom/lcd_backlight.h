/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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
*  lcd_backlight.h
*
*  PURPOSE:
*
*     This file defines the platform-independent kernel API for the
*     LCD backlight.
*
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_LCD_BACKLIGHT_H )
#define LINUX_LCD_BACKLIGHT_H

#if defined( __KERNEL__ )

/*
 * ---- Include Files ---------------------------------------------------- 
 * ---- Constants and Types ---------------------------------------------- 
 */


typedef enum LCD_BACKLIGHT_LEVEL
{
   LCD_BACKLIGHT_OFF       = 0,     /* Controlled by GPIO */
   LCD_BACKLIGHT_MIN_LEVEL = 1,     /* Controlled by PWM */
   LCD_BACKLIGHT_MAX_LEVEL = 31,    /* Controlled by PWM */
   LCD_BACKLIGHT_FULL_ON   = 32,    /* Controlled by GPIO */

} LCD_BACKLIGHT_LEVEL;

struct lcd_backlight_param
{
   /* GPIO pins to be reserved for backlight control */
   unsigned int gpio;

   /* PWM frequencies */
   unsigned int pwm_freq;
};

/*
 * ---- Variable Externs ------------------------------------------------- 
 * ---- Function Prototypes ---------------------------------------------- 
 */

void lcd_backlight_init( struct lcd_backlight_param *param );
void lcd_backlight_deinit( void );
void lcd_backlight_enable( LCD_BACKLIGHT_LEVEL level );
LCD_BACKLIGHT_LEVEL lcd_backlight_max_level( void );
LCD_BACKLIGHT_LEVEL lcd_backlight_curr_level( void );

#endif   /* __KERNEL__ */
#endif  /* LINUX_LCD_BACKLIGHT_H */
