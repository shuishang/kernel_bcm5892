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
#include <linux/mm.h>                        /* For memory alloc */
#include <linux/proc_fs.h>                   /* For procfs */
#include <linux/vmalloc.h>                   /* For memory alloc */
#include <linux/broadcom/gos/gos.h>              /* GOS API */
#include <asm/atomic.h>                      /* Atomic operations */

#include "gos_priv.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

/* Procfs file name */
#define MEM_PROC_NAME         "mem"

/* Threshold for choosing between kmalloc and vmalloc */
#define MEM_KMALLOC_THRESH    PAGE_SIZE

/* ---- Private Variables ------------------------------------------------ */
/* Allocated memory buffers statistics */
static atomic_t   gStatKmallocd = ATOMIC_INIT( 0 );
static atomic_t   gStatVmallocd = ATOMIC_INIT( 0 );

/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  Allocate memory
*
*  @return
*     ptr     Valid pointer to allocated memory
*     NULL    Insufficient memory
*
*  @remark
*     This routine may block and may not return physically contiguous
*     memory.
*/
void *gosMemAlloc(
   unsigned int bytes               /**< (i) Size in bytes */
)
{
   void *memp;

   if ( bytes < MEM_KMALLOC_THRESH )
   {
      memp = kmalloc( bytes, GFP_KERNEL );
      if ( memp )
      {
         atomic_inc( &gStatKmallocd );
      }
   }
   else
   {
      memp = vmalloc( bytes );
      if ( memp )
      {
         atomic_inc( &gStatVmallocd );
      }
   }

   return memp;
}
EXPORT_SYMBOL( gosMemAlloc );

/***************************************************************************/
/**
*  Allocate memory, specifying virtual or non-virtual memory
*
*  @return
*     ptr     Valid pointer to allocated memory
*     NULL    Insufficient memory
*
*  @remark
*     This routine may block and may not return physically contiguous
*     memory.
*/
void *gosMemAllocType(
   unsigned int bytes,              /**< (i) Size in bytes */
   GOS_MEM_TYPE type                /**< (i) Type of memory */
)
{
   void *memp;

   if( type == GOS_MEM_TYPE_VIRTUAL )
   {
      memp = vmalloc( bytes );
      if ( memp )
      {
         atomic_inc( &gStatVmallocd );
      }
   }
   else if( type == GOS_MEM_TYPE_NOT_VIRTUAL )
   {
      memp = kmalloc( bytes, GFP_KERNEL );
      if ( memp )
      {
         atomic_inc( &gStatKmallocd );
      }
   }
   else
   {
      memp = gosMemAlloc( bytes );
   }
   return memp;
}
EXPORT_SYMBOL( gosMemAllocType );

/***************************************************************************/
/**
*  Free previously allocated memory
*
*  @return
*     0        Success
*     -ve      Failed to free memory
*/
int gosMemFree(
   void          *memp              /**< (i) Memory to free */
)
{
   if ( memp == NULL )
   {
      return -EINVAL;
   }

   if ( is_vmalloc_addr( memp ))
   {
      vfree( memp );
      atomic_dec( &gStatVmallocd );
   }
   else
   {
      kfree( memp );
      atomic_dec( &gStatKmallocd );
   }

   return 0;
}
EXPORT_SYMBOL( gosMemFree );

/***************************************************************************/
/**
*  Procfs read callback function
*
*  @return  Number of characters to print
*/
static int gosMemReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int len = 0;

   len += sprintf( buf+len, "kmalloc_bufs=%i vmalloc_bufs=%i\n",
         atomic_read( &gStatKmallocd ), atomic_read( &gStatVmallocd ));

   *eof = 1;
   return len;
}

/***************************************************************************/
/**
*  Contructor routine
*
*  @return
*     0              Success
*     -ve            Error code
*/
int __init gosMemInit( void )
{
   create_proc_read_entry( MEM_PROC_NAME, 0, gGosProcDir, gosMemReadProc, NULL );
   return 0;
}

