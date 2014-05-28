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
*  @file    gosirq.c
*
*  @brief   This file implements the Generic OS (gos) interrupt abstraction.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/broadcom/gos/gos.h>              /* GOS API */

#include "gos_priv.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  Nestable interrupts disable
*
*  @return Interrupt state used to restore interrupts with
*/
GOS_ISR_STATE gosInterruptsSave( void )
{
   unsigned long flags;
   local_irq_save( flags );
   return flags;
}
EXPORT_SYMBOL( gosInterruptsSave );

/***************************************************************************/
/**
*  Nestable interrupts enable
*
*  @return Nothing
*/
void gosInterruptsRestore( 
   GOS_ISR_STATE state              /**< (i) Interrupt state to restore */
)
{
   local_irq_restore( state );
}
EXPORT_SYMBOL( gosInterruptsRestore );
