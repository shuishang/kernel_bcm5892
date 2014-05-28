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

#ifndef KEYMAP_H
#define KEYMAP_H

/*
 * GPIOs used as keys
 */
typedef struct
{
   unsigned int gpio;
   unsigned int keycode;
   unsigned int invert;
} GPIOMAP;

#define KEYMAP_FAKE_GPIO 0xFA7EFA7E

/*
 * Keypad mapping
 */
typedef struct
{
   unsigned int scancode;
   unsigned int keycode;
} KEYMAP;

typedef struct
{
   GPIOMAP *gpiomap;
   unsigned int gpiomap_cnt;

   KEYMAP *keymap;
   unsigned int keymap_cnt;

   /* key sets for powering off (reboot) Linux */
   unsigned int *pwroff;
   unsigned int pwroff_cnt;
} KEYPAD_DATA;

#endif
