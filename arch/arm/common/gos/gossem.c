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
*  @file    gossem.c
*
*  @brief   This file implements the Generic OS (gos) semaphore abstraction.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/types.h>                     /* For stdint types: uint8_t, etc. */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>                   /* For procfs */
#include <linux/init.h>
#include <linux/string.h>
#include <linux/semaphore.h>                 /* For down_interruptible, up, etc. */
#include <linux/jiffies.h>                   /* For jiffies */
#include <linux/list.h>                      /* Linked list */

#include <linux/broadcom/gos/gos.h>              /* GOS API */
#include <asm/atomic.h>                      /* Atomic operations */

#include "gos_priv.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

/* Semaphore node */
struct gos_sema_node
{
   struct list_head           lnode;         /* List node */
   struct semaphore           semaphore;     /* Linux semaphore */
   char                       name[32];      /* Name string */
   atomic_t                   inuse;         /* For debugging and error checking */
};

/* information instructure */
struct gos_sem_info
{
   struct list_head           list;          /* List of semaphores */
   struct semaphore           lock;          /* Protect semaphore list */
   int                        total;         /* Number of semaphores */
};

/* Procfs file name */
#define SEM_PROC_NAME         "sem"

/* ---- Private Variables ------------------------------------------------ */

static struct gos_sem_info    gSem;

/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  Allocates a counting semaphore object
*
*  @return
*     0        Success, semp contains valid semaphore object
*     -ve      On general failure
*/
int gosSemAlloc(
   const char *name,                /**< (i) Name to assign to the semaphore */
   int         initval,             /**< (i) Initial value of the semaphore */
   GOS_SEM    *semp                 /**< (o) Place to return new semaphore */
)
{
   struct gos_sema_node *semnodep;

   /* FIXME: use gosMemAlloc()? */
   semnodep = kmalloc( sizeof(*semnodep), GFP_KERNEL );
   if ( semnodep == NULL )
   {
      return -ENOMEM;
   }

   memset( semnodep, 0, sizeof(*semnodep) );
   strncpy( semnodep->name, name, sizeof(semnodep->name)-1 );
   sema_init( &semnodep->semaphore, initval );
   atomic_set( &semnodep->inuse, 0 );

   down( &gSem.lock );
   list_add_tail( &semnodep->lnode, &gSem.list );
   gSem.total++;
   up( &gSem.lock );

   *semp = semnodep;

   return 0;
}
EXPORT_SYMBOL( gosSemAlloc );

/***************************************************************************/
/**
*  Frees a previously created counting semaphore object
*
*  @return
*     0        Success
*     -ve      On general failure
*/
int gosSemFree(
   GOS_SEM     sem                  /**< (i) Semaphore to free */
)
{
   struct gos_sema_node *semnodep = sem;

   if ( semnodep == NULL )
   {
      return -EINVAL;
   }

   if ( atomic_read( &semnodep->inuse ))
   {
      return -EBUSY;
   }

   down( &gSem.lock );
   list_del( &semnodep->lnode );
   gSem.total--;
   up( &gSem.lock );

   /* FIXME: use gosMemFree() */
   kfree( semnodep );

   return 0;
}
EXPORT_SYMBOL( gosSemFree );

/***************************************************************************/
/**
*  Increments the count of the specified semaphore object.
*
*  @return
*     0        Success
*     -ve      On general failure
*/
int gosSemGive(
   GOS_SEM     sem                  /**< (i) Semaphore to give */
)
{
   struct gos_sema_node *semnodep = sem;

   up( &semnodep->semaphore );
   atomic_inc( &semnodep->inuse );

   return 0;
}
EXPORT_SYMBOL( gosSemGive );

/***************************************************************************/
/**
*  Decrements the count of the specified semaphore object. The calling
*  thread may block.
*
*  @return
*     0        Success
*     -ve      On general failure
*/
int gosSemTake(
   GOS_SEM     sem                  /**< (i) Semaphore to take */
)
{
   struct gos_sema_node *semnodep = sem;
   int err;

   err = 0;
   atomic_dec( &semnodep->inuse );
   down( &semnodep->semaphore );

   return err;
}
EXPORT_SYMBOL( gosSemTake );

/***************************************************************************/
/**
*  Decrements the count of the specified semaphore object. The calling
*  thread may block with a timeout.
*
*  @return
*     0        Success
*     -ve      On general failure
*/
int gosSemTimedTake(
   GOS_SEM     sem,          /**< (i) Semaphore to take */
   int         timeout       /**< (i) Timeout (in ms) for semaphore take */
)
{
   struct gos_sema_node *semnodep = sem;
   int err;
   long time_out_jiffies;

   err = 0;

   time_out_jiffies = msecs_to_jiffies(timeout);

   atomic_dec( &semnodep->inuse );
   err = down_timeout( &semnodep->semaphore, time_out_jiffies );
   if ( err != 0 )
   {
      atomic_inc( &semnodep->inuse );
      /* timeout errno will be -ETIME */
   }

   return err;
}
EXPORT_SYMBOL( gosSemTimedTake );

/***************************************************************************/
/**
*  Procfs read callback function
*
*  @return  Number of characters to print
*/
static int gosSemReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   struct gos_sema_node *semnodep;
   int len;

   len = 0;

   len += sprintf( buf+len, "Semaphores: total=%i\n", gSem.total );
   list_for_each_entry( semnodep, &gSem.list, lnode )
   {
      len += sprintf( buf+len, " %s: inuse=%i\n",
            semnodep->name, atomic_read( &semnodep->inuse ));
   }

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
int __init gosSemInit( void )
{
   INIT_LIST_HEAD( &gSem.list );
   init_MUTEX( &gSem.lock );
   create_proc_read_entry( SEM_PROC_NAME, 0, gGosProcDir, gosSemReadProc, NULL );
   return 0;
}

