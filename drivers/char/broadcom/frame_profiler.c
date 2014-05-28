/*****************************************************************************
* Copyright 2003 - 2011 Broadcom Corporation.  All rights reserved.
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
*****************************************************************************

*****************************************************************************
*
*  @file    frame_profiler
*
*  @brief   Keeps a history and statistics of the processing time of frames.
*
****************************************************************************/

/* ----------------------------------------------------------------------- */
/*                            Include Files                                */
/* ----------------------------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/broadcom/knllog.h>
#include <linux/broadcom/timer.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/broadcom/gos/gos.h>
#include <linux/broadcom/frame_profiler.h>

EXPORT_SYMBOL(frame_profiler_init);
EXPORT_SYMBOL(FP_startProfiling);
EXPORT_SYMBOL(FP_stopProfiling);

/* ----------------------------------------------------------------------- */
/*                     Private Constants and Types                         */
/* ----------------------------------------------------------------------- */

#define HISTORY_SIZE  100
#define AVG_RANGE  HISTORY_SIZE/2

typedef struct                      /* Read-only */
{
   unsigned int currTime;
   unsigned int maxTime;
   unsigned int avgPercentTime;
   unsigned int maxAvgPercentTime;
   unsigned int avgWeightedTime;
   unsigned int maxAvgWeightedTime;

} STATS;

typedef struct                      /* Read and Write */
{
   unsigned int enable;      /* Frame processing time monitor on of off. Initialized at startup, read-only by kernel and write/read by user via proc entry */
   unsigned int reset;       /* Forced reset of stats, Intiailized at startup, read-only by kernel and write/read by user via proc entry */
   unsigned int alphaFactor; /* Weightage of current value to the average (in %). Initialized at startup, readonly by kernel, write/read by user via proc entry */
   unsigned int position;
   unsigned int avgSum;

} STATE;

typedef struct                     /* Read-only */
{
   unsigned int history[HISTORY_SIZE];
   timer_tick_count_t start[HISTORY_SIZE];
   timer_tick_count_t end[HISTORY_SIZE];

} DATA;

/* ----------------------------------------------------------------------- */
/*                          Private Variables                              */
/* ----------------------------------------------------------------------- */

static STATE gProfilerState;
static STATS gProfilerStats;
static DATA gProfilerData;
static GOS_SEM gSemThread;
static GOS_SEM gSemRead;

static struct task_struct  *gProfilerTask = NULL;
static struct ctl_table_header *gSysCtlHeader;
static struct proc_dir_entry *gProcDir;

/* ----------------------------------------------------------------------- */
/*                  Private Function Prototypes                            */
/* ----------------------------------------------------------------------- */

static int profilerThread( void *data );
unsigned int microSec( timer_tick_count_t ticks );

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_do_profiler_intvec_clear( ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos );
#else
static int proc_do_profiler_intvec_clear( ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
#endif

static struct ctl_table gSysCtlChild[] = {
   {
      .ctl_name   = 1,
      .procname   = "enable",
      .data       = &gProfilerState.enable,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 2,
      .procname   = "reset",
      .data       = &gProfilerState.reset,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_do_profiler_intvec_clear
   },
   {
      .ctl_name   = 3,
      .procname   = "alpha-factor",
      .data       = &gProfilerState.alphaFactor,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name   = 4,
      .procname   = "state",
      .data       = &gProfilerState,
      .maxlen     = 3*sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};

static ctl_table gSysCtl[] = {
   {
      .ctl_name   = 1,
      .procname   = "frameProfiler",
      .mode       = 0555,
      .child      = gSysCtlChild
   },
   {}
};

/* ----------------------------------------------------------------------- */
/*                              Functions                                  */
/* ----------------------------------------------------------------------- */

/****************************************************************************
*
*  readProcHistory
*
****************************************************************************/

static int readProcHistory( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int len = 0;
   int i;
   (void) start; (void) offset; (void) count;      /* avoid compiler warning */

   gosSemTake( gSemRead );
   len += sprintf( buf+len, "\n********************* Summary of History ************************\n\n" );
   len += sprintf( buf+len, "Processing Time             Start Time               End Time    \n\n" );

   if ( gProfilerState.position == 0 )
   {
      for ( i = 0; i < HISTORY_SIZE/2; i++ )
      {
         len += sprintf( buf+len, "%15u %22u %22u \n", gProfilerData.history[i], microSec(gProfilerData.start[HISTORY_SIZE/2-1-i]), microSec(gProfilerData.end[HISTORY_SIZE/2-1-i]) );
      }
   }
   else
   {
      for ( i = 0; i < HISTORY_SIZE/2; i++ )
      {
         len += sprintf( buf+len, "%15u %22u %22u \n", gProfilerData.history[i], microSec(gProfilerData.start[HISTORY_SIZE-1-i]), microSec(gProfilerData.end[HISTORY_SIZE-1-i]) );
      }
   }
   gosSemGive( gSemRead );

   *eof = 1;
   return len+1;
}

/****************************************************************************
*
*  readProcStats
*
****************************************************************************/

static int readProcStats( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int len = 0;
   (void) start; (void) offset; (void) count;      /* avoid compiler warning */

   len += sprintf( buf+len, "\n********************* Summary of Statistics *********************\n\n" );
   len += sprintf( buf+len, "Statistic                            Current         Maximum \n\n" );
   len += sprintf( buf+len, "%-27s: %15u %15u \n", "Processing Time (us)", gProfilerStats.currTime, gProfilerStats.maxTime );
   len += sprintf( buf+len, "%-27s: %15u %15u \n", "Average Percent Time (us)", gProfilerStats.avgPercentTime, gProfilerStats.maxAvgPercentTime );
   len += sprintf( buf+len, "%-27s: %15u %15u \n", "Average Weighted Time (us)", gProfilerStats.avgWeightedTime, gProfilerStats.maxAvgWeightedTime );
   
   *eof = 1;
   return len+1;
}

/****************************************************************************
*
*  proc_do_profiler_intvec_clear
*  
****************************************************************************/

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_do_profiler_intvec_clear( ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int proc_do_profiler_intvec_clear( ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#endif
{
   int rc = 0;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec( table, write, buffer, lenp, ppos );
#else
      rc = proc_dointvec( table, write, filp, buffer, lenp, ppos );
#endif

   if ( !table || !table->data )
      return -EINVAL;

   if ( write && gProfilerState.reset )
   {
      memset( &gProfilerStats, 0, sizeof( STATS ));
      memset( &gProfilerData, 0, sizeof( DATA ));
      gProfilerState.alphaFactor = ( 1<< 4 );   /* 1/16 in Q8 number */
      gProfilerState.avgSum = 0;
      gProfilerState.reset = 0;
   }

   return rc;
}

/****************************************************************************
 *
 * frame_profiler_init
 * 
 ***************************************************************************/

int frame_profiler_init( void )
{
   gProcDir = proc_mkdir( "frameProfiler", NULL );

   if ( gProcDir == NULL )
   {
      printk( KERN_ERR "%s: Failed to create directory for proc entries\n", __FUNCTION__ );
      return -1;
   }

   create_proc_read_entry( "history", 0, gProcDir, readProcHistory, NULL );
   create_proc_read_entry( "stats", 0, gProcDir, readProcStats, NULL );

   gSysCtlHeader = register_sysctl_table( gSysCtl );

   memset( &gProfilerState, 0, sizeof( STATE ));
   memset( &gProfilerStats, 0, sizeof( STATS ));
   memset( &gProfilerData, 0, sizeof( DATA ));
   gProfilerState.enable = 1;
   gProfilerState.alphaFactor = ( 1<< 4 );   /* 1/16 in Q8 number */
   gosSemAlloc( "sem_Thread", 1, &gSemThread );
   gosSemAlloc( "sem_Read", 1, &gSemRead );

   if (( gProfilerTask == NULL ) || IS_ERR( gProfilerTask ))
   {
      gProfilerTask = kthread_run( profilerThread, NULL, "profilerThread" );

      if ( IS_ERR( gProfilerTask ))
      {
         printk( KERN_ERR "Init: failed to start profiler thread: %ld\n", PTR_ERR( gProfilerTask ));
         return -1;
      }
   }
   printk( "calling frame_profiler_init\n" );
   return 0;
}

/****************************************************************************
* 
* frame_profiler_exit
*
 ***************************************************************************/

int frame_profiler_exit( void )
{
   remove_proc_entry( "stats", gProcDir );
   remove_proc_entry( "history", gProcDir );
   remove_proc_entry( "frameProfiler", NULL );

   if ( gSysCtlHeader )
   {
      unregister_sysctl_table( gSysCtlHeader );
   }

   kthread_stop( gProfilerTask );
   return 0;
}

/****************************************************************************
* 
* startProfiling
*
 ***************************************************************************/

void FP_startProfiling()
{
   static timer_tick_count_t temp[HISTORY_SIZE];
   static timer_tick_count_t *pstart = temp;

   *pstart = timer_get_tick_count();
   pstart++;

   if( pstart == ( temp + HISTORY_SIZE ))
   {
      pstart = temp;
      memcpy(&gProfilerData.start[HISTORY_SIZE/2], &temp[HISTORY_SIZE/2], HISTORY_SIZE/2*sizeof(unsigned int) );
   }

   if( pstart == ( temp + HISTORY_SIZE/2 ))
   {
      memcpy(&gProfilerData.start[0], &temp[0], HISTORY_SIZE/2*sizeof(unsigned int) );
   }
}

/****************************************************************************
* 
* stopProfiling
*
 ***************************************************************************/

void FP_stopProfiling()
{
   static timer_tick_count_t temp[HISTORY_SIZE];
   static timer_tick_count_t *pend = temp;

   *pend = timer_get_tick_count();
   pend++;

   if( pend == ( temp + HISTORY_SIZE ))
   {
      pend = temp;
      gosSemTake( gSemThread );
      memcpy(&gProfilerData.end[HISTORY_SIZE/2], &temp[HISTORY_SIZE/2], HISTORY_SIZE/2*sizeof(unsigned int) );
      gProfilerState.position = HISTORY_SIZE/2;
      wake_up_process( gProfilerTask );
   }

   if( pend == ( temp + HISTORY_SIZE/2 ))
   {
      gosSemTake( gSemThread );
      memcpy(&gProfilerData.end[0], &temp[0], HISTORY_SIZE/2*sizeof(unsigned int) );
      gProfilerState.position = 0;
      wake_up_process( gProfilerTask );
   }
}

/****************************************************************************
* 
* microSec
*
 ***************************************************************************/

unsigned int microSec( timer_tick_count_t ticks )
{
   return ( ticks / (timer_get_tick_rate() / 1000000) );
}

/****************************************************************************
* 
* percentAvg
*
 ***************************************************************************/

unsigned int percentAvg( int index )
{
   gProfilerState.avgSum += gProfilerData.history[1] - gProfilerData.history[AVG_RANGE];

   if ((index - AVG_RANGE + 1) < 0)
   {
      return (( gProfilerState.avgSum*10000 ) / ( microSec(gProfilerData.start[index] - gProfilerData.start[index - AVG_RANGE + 1 + HISTORY_SIZE] ) + 1));
   }
   else
   {
      return (( gProfilerState.avgSum*10000 ) / ( microSec(gProfilerData.start[index] - gProfilerData.start[index - AVG_RANGE + 1] ) + 1));
   }
}

/****************************************************************************
* 
* weightedAvg
*
 ***************************************************************************/

unsigned int weightedAvg( void )
{
   if ( !gProfilerStats.avgWeightedTime )
   {
      return gProfilerStats.currTime;
   }
   else
   {
      return ((gProfilerStats.currTime * gProfilerState.alphaFactor) + (gProfilerStats.avgWeightedTime * ((1 << 8) - gProfilerState.alphaFactor))) >> 8;
   }
}

/****************************************************************************
* 
* profilerThread
*
 ***************************************************************************/

int profilerThread(void *data)
{
   int index;
   int i;
   printk( "************* Starting Profiler Thread **************\n" );

   while( 1 )
   {
      set_current_state( TASK_INTERRUPTIBLE );
      schedule();

      if ( !gProfilerState.enable )
      {
         gosSemGive( gSemThread );
         continue;
      }
    
      gosSemTake( gSemRead );

      for( index = gProfilerState.position; index < (HISTORY_SIZE/2 + gProfilerState.position); index++ )
      {
         gProfilerStats.currTime = microSec( gProfilerData.end[index] - gProfilerData.start[index] );

         for( i = (HISTORY_SIZE-1); i > 0; i-- )
         {
            gProfilerData.history[i] = gProfilerData.history[i-1];
         }

         gProfilerData.history[0] = gProfilerStats.currTime;           

         if ( gProfilerStats.maxTime < gProfilerStats.currTime )
         {
            gProfilerStats.maxTime = gProfilerStats.currTime;
         }

         gProfilerStats.avgPercentTime = percentAvg(index);

         if ( gProfilerStats.maxAvgPercentTime < gProfilerStats.avgPercentTime )
         {
            gProfilerStats.maxAvgPercentTime = gProfilerStats.avgPercentTime;
         }

         gProfilerStats.avgWeightedTime = weightedAvg();
         
         if ( gProfilerStats.maxAvgWeightedTime < gProfilerStats.avgWeightedTime )
         {
            gProfilerStats.maxAvgWeightedTime = gProfilerStats.avgWeightedTime;
         }
      }
      gosSemGive( gSemRead );
      gosSemGive( gSemThread );
   }
   return 0;
}