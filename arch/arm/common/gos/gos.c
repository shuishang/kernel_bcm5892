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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>                   /* For /proc/gos */
#include <linux/init.h>

#include <linux/broadcom/gos/gos.h>              /* GOS API */

#include "gos_priv.h"                        /* Private definitions */

/* ---- Public Variables ------------------------------------------------- */
struct proc_dir_entry            *gGosProcDir;

/* ---- Private Constants and Types -------------------------------------- */

/* Procfs file name */
#define GOS_PROCDIR_NAME         "gos"

/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  procfs initialization 
*
*  @return
*     0              Success
*     -ve            Error code
*/
static int __init gosProcInit( void )
{
   gGosProcDir = proc_mkdir( GOS_PROCDIR_NAME, NULL );
   return 0;
}

/***************************************************************************/
/**
*  Contructor routine
*
*  @return
*     0              Success
*     -ve            Error code
*/
static int __init gos_init( void )
{
   int err;

   err = gosProcInit();
   if ( err )
   {
      return err;
   }

   err = gosSemInit();
   if ( err )
   {
      return err;
   }

   err = gosMemInit();
   if ( err )
   {
      return err;
   }

   err = gosThreadInit();
   if ( err )
   {
      return err;
   }

   return 0;
}

/* 
 * Some drivers need gos primitives in their startup code so we need to make 
 * sure GOS is initialized before the regular driver init calls. 
 */
early_initcall(gos_init);

