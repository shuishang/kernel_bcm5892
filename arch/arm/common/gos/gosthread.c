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
*  @file    gosthread.c
*
*  @brief   This file implements the Generic OS (gos) system thread abstraction.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>                       /* For IS_ERR and PTR_ERR */
#include <linux/kthread.h>                   /* For kthread */
#include <linux/sched.h>                     /* For scheduler */
#include <linux/signal.h>                    /* For signal */
#include <linux/semaphore.h>                 /* For semaphore */
#include <linux/proc_fs.h>                   /* For procfs */
#include <linux/jiffies.h>                   /* For jiffies, etc */
#include <linux/broadcom/gos/gos.h>              /* GOS API */

#include "gos_priv.h" 

#define THREAD_PRIO_STEP (MAX_RT_PRIO / GOS_THRD_PRIO_HIGH)
#define MAX_NAME_LENGTH 32

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

/* Thread node */
struct gos_thread_node
{
   struct list_head           lnode;         /* List node */
   struct task_struct         *threadp;        /* Linux kthread */
   char                       name[MAX_NAME_LENGTH];      /* Name string */
   int                        policy;        /* Sched policy */
   int                        priority;      /* Priority */
};

/* information instructure */
struct gos_thread_info
{
   struct list_head           list;          /* List of threads */
   struct semaphore           lock;          /* Protect semaphore list */
   int                        total;         /* Number of threads */
};

/* General thread parameter */
struct gos_thread_parm
{
   struct gos_thread_node *pThreadNode;
   void *pThreadParm;
   int (*funcp)( void *arg );
};

/* Procfs file name */
#define THREAD_PROC_NAME         "thread"

/* ---- Private Variables ------------------------------------------------ */

static struct gos_thread_info    gThreadSem;

/* ---- Private Function Prototypes -------------------------------------- */
static int gosThreadRoutine( void *arg );

/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  Create a processing thread.
*
*  @return 
*     0        Success
*     -ve      On general failure
*/
int gosThreadCreate(
   const char     *name,            /**< (i) Name to assign to the new thread */
   GOS_THRD_PRIO   priority,        /**< (i) Thread priority */
   int (*funcp)( void *arg ),      /**< (i) Thread processing function */
   void           *arg,             /**< (i) Argument to pass to processing thread */
   GOS_THRD_ID    *threadidp        /**< (o) Place to store the ID of the new thread */
)
{
   struct gos_thread_node *threadnodep;
   struct task_struct *kthreadp;
   struct sched_param param;
   int policy;
   struct gos_thread_parm  * pParm;

   if(( priority > GOS_THRD_PRIO_HIGH ) && ( GOS_THRD_PRIO_LOW > GOS_THRD_PRIO_HIGH ))
   {
      return -EINVAL;
   }

   threadnodep = kmalloc( sizeof(*threadnodep), GFP_KERNEL );
   if ( threadnodep == NULL )
   {
      return -ENOMEM;
   }

   pParm = kmalloc(sizeof(struct gos_thread_parm), GFP_KERNEL);
   if ( pParm == NULL )
   {
      kfree(threadnodep);
      return -ENOMEM;
   }

   memset( pParm, 0, sizeof(struct gos_thread_parm) );
   pParm->pThreadNode = threadnodep;
   pParm->pThreadParm = arg;
   pParm->funcp = funcp;

   kthreadp = kthread_create( gosThreadRoutine, pParm, name );
   if( IS_ERR( kthreadp ) )
   {
      kfree( threadnodep );
      kfree( pParm );
      return PTR_ERR( kthreadp );
   }

   if( priority == GOS_THRD_PRIO_LOW )
   {
      param.sched_priority = 0;
      policy = SCHED_NORMAL;
   }
   else
   {
      param.sched_priority = THREAD_PRIO_STEP * priority - 1;
      policy = SCHED_FIFO;
   }

   if( sched_setscheduler( kthreadp, policy, &param ) )
   {
      kthread_stop( kthreadp );
      kfree( threadnodep );
      kfree( pParm );
      return -EINVAL;
   }

   threadnodep->threadp = kthreadp;

   strncpy(threadnodep->name, name, MAX_NAME_LENGTH);
   threadnodep->name[MAX_NAME_LENGTH - 1] = '\0';
   threadnodep->policy = policy;
   threadnodep->priority = param.sched_priority;

   down( &gThreadSem.lock );
   list_add_tail( &threadnodep->lnode, &gThreadSem.list );
   gThreadSem.total++;
   up( &gThreadSem.lock );

   /* Run the thread */
   wake_up_process( threadnodep->threadp );

   *threadidp = threadnodep;
   return 0;
}
EXPORT_SYMBOL( gosThreadCreate );

/***************************************************************************/
/**
*  Destroy a processing thread. This function may block until the 
*  child thread exits.
*
*  @return 
*     0        Success
*     -ve      On general failure
*/
int gosThreadDestroy(
   GOS_THRD_ID    threadid          /**< (i) Thread to destroy */
)
{
   struct gos_thread_node *threadnodep = threadid;
   int ret;
   int found = false;
   struct task_struct *pTask;
   struct list_head *item;

   if ( threadnodep == NULL )
   {
      return -EINVAL;
   }

   down( &gThreadSem.lock );
   list_for_each( item, &gThreadSem.list )
   {
      /* This thread node found */
      if( item == &(threadnodep->lnode) )
      {
         found = true;
      }
   }
   if( found == true )
   {
      pTask = threadnodep->threadp;
      up( &gThreadSem.lock );
   }
   else
   {
      up( &gThreadSem.lock );
      /* Thread already exits */
      return 0;
   }

   ret = send_sig( SIGTERM, pTask, 0 );

   /* 1) After signal mask changed in thread, kthread_stop never return when signal
      received by thread 
      2) General GOS thread body will do the cleanup
      The following is commented out
   */
#if 0
   if( ret == 0 )
   {
      ret = kthread_stop( pTask );

      down( &gThreadSem.lock );
      found = false;
      list_for_each( item, &gThreadSem.list )
      {
         /* This thread node found */
         if( item == &(threadnodep->lnode) )
         {
            found = true;
         }
      }
      if( found == true )
      {
         list_del( &threadnodep->lnode );
         gThreadSem.total--;
         kfree( threadnodep );
         up( &gThreadSem.lock );
      }
      else
      {
         up( &gThreadSem.lock );
         /* Thread already exits */
         return 0;
      }
   }
#endif

   return ret;
}

/***************************************************************************/
/**
*  General GOS thread body
*
*     0        Success
*     -ve      On general failure
*/
static int gosThreadRoutine( void *arg )
{
   struct gos_thread_parm parm;

   if( arg == NULL )
   {
      return -EINVAL;
   }

   parm = *((struct gos_thread_parm *)arg);

   kfree( arg );

   if( parm.pThreadNode == NULL )
   {
      return -EINVAL;
   }

   if( parm.funcp == NULL )
   {
      return -EINVAL;
   }

   allow_signal(SIGTERM);
   parm.funcp( parm.pThreadParm );

   down( &gThreadSem.lock );
   list_del( &parm.pThreadNode->lnode );
   gThreadSem.total--;
   kfree( parm.pThreadNode );
   up( &gThreadSem.lock );

   return 0;
}
EXPORT_SYMBOL( gosThreadDestroy );

/***************************************************************************/
/**
*  Procfs read callback function
*
*  @return  Number of characters to print
*/
static int gosThreadReadProc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   struct gos_thread_node *threadnodep;
   int len;

   len = 0;

   len += sprintf( buf+len, "Threads: total=%i\n", gThreadSem.total );
   list_for_each_entry( threadnodep, &gThreadSem.list, lnode )
   {
      len += sprintf( buf+len, " %s: policy=%i priority=%i\n", 
            threadnodep->name, threadnodep->policy, threadnodep->priority );
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
int __init gosThreadInit( void )
{
   INIT_LIST_HEAD( &gThreadSem.list );
   init_MUTEX( &gThreadSem.lock );
   create_proc_read_entry( THREAD_PROC_NAME, 0, gGosProcDir, gosThreadReadProc, NULL );
   return 0;
}

