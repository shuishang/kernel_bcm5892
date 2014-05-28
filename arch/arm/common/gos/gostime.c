/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
/**
*
*  @file    gostime.c
*
*  @brief   This file implements the Generic OS (gos) system time abstraction.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/jiffies.h>                   /* For jiffies, etc */
#include <linux/broadcom/gos/gos.h>              /* GOS API */

#include "gos_priv.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  Retrieve system time
*
*  @return system time in ms since system startup.
*/
unsigned int gosElapsedTime_ms( void )
{
   return jiffies_to_msecs(jiffies);
}
EXPORT_SYMBOL( gosElapsedTime_ms );

/***************************************************************************/
/**
*  Retrieve the system time in seconds and usecs elapsed since system startup.
*
*  @return  0 always
*
*  @remarks
*/
int gosElapsedTime_msec_usec(uint32_t *seconds, uint32_t *useconds)
{
   struct timeval tv;

   jiffies_to_timeval( jiffies, &tv );

   *seconds  = tv.tv_sec;
   *useconds = tv.tv_usec;

   return 0;
}
EXPORT_SYMBOL( gosElapsedTime_msec_usec );
