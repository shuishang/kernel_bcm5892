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
*  @file    gos.c
*
*  @brief   This file implements the Generic OS (gos) abstraction layer.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/signal.h>
#include <linux/sched.h>                     /* For schedule_timeout */
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
*  Blocks and sleeps for the specified period in milli-seconds
*
*  @return 
*     0        Success
*     -ve      On general failure
*/
int gosSleepMs( 
   unsigned int msec                /**< (i) Period in ms to sleep */
)
{
   int timeout;

   set_current_state( TASK_INTERRUPTIBLE );
   timeout = schedule_timeout( msecs_to_jiffies( msec ) );
   return ( timeout ? -EINTR : 0 );
}
EXPORT_SYMBOL( gosSleepMs );
