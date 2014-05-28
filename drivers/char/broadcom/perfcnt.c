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
*  perfcnt.c
*
*  PURPOSE:
*  Kernel realtime event logger (when printk's just don't do the job...)
*
*  NOTES:
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/sysctl.h>
#include <linux/string.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/broadcom/perfcnt.h>
#include <linux/broadcom/timer.h>
#include <cfg_global.h>

/* ---- Constants and Types ---------------------------------------------- */

#ifdef CONFIG_OPROFILE
extern unsigned long oprofile_started;
#endif

/*
 * Timers are 30 times faster on fpga so scale them down accordingly
 * for bus busy and CPI calculations.
 */
#define TMR_DIV_FACTOR 1

/* The perfcnt object */
typedef struct
{
   PERFCNT_CNTRS cntrs;
   int perfstart;          /* flag to start profiling */
   int perfstop;           /* flag to stop profiling */
   int perfclear;          /* flag to clear stats */
   int perfread;           /* flag to read stats */
   int help;               /* help string */
   int verbose;            /* verbosity 0 = cpi, 1 = perfcnt summary, 2 = details */
   int perfpoll;           /* printk busbusy and cpi every 'poll' msec. 0 to disable. */
   int event0;             /* event type to capture in event counter 0 */
   int event1;             /* event type to capture in event counter 1 */
   int last_cr0;
   int last_cr1;
   int last_bus_idle;
   int last_tmr;
}
PERFCNT_OBJ;

/* storage for the perfcnt object */
static PERFCNT_OBJ perfcnt;

static long perfpollThreadPid = 0;
static struct completion perfpollExited;
DECLARE_WAIT_QUEUE_HEAD(perfpollWakeQ);
static int perfpollWake;

/* sysctl */
static  struct ctl_table_header    *gSysCtlHeader;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_do_perfcnt_intvec_event(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_perfstart(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_perfstop(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_perfclear(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_perfread(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_perfpoll(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_help(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos );
#else
static int proc_do_perfcnt_intvec_event(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_perfstart(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_perfstop(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_perfclear(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_perfread(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_perfpoll(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_do_perfcnt_intvec_help(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
#endif

static struct ctl_table gSysCtlPerfCnt[] = {
   {
      .ctl_name      = 1,
      .procname      = "event0",
      .data          = &perfcnt.event0,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perfcnt_intvec_event
   },
   {
      .ctl_name      = 2,
      .procname      = "event1",
      .data          = &perfcnt.event1,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perfcnt_intvec_event
   },
   {
      .ctl_name      = 4,
      .procname      = "start",
      .data          = &perfcnt.perfstart,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perfcnt_intvec_perfstart
   },
   {
      .ctl_name      = 5,
      .procname      = "stop",
      .data          = &perfcnt.perfstop,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perfcnt_intvec_perfstop
   },
   {
      .ctl_name      = 6,
      .procname      = "clear",
      .data          = &perfcnt.perfclear,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perfcnt_intvec_perfclear
   },
   {
      .ctl_name      = 7,
      .procname      = "read",
      .data          = &perfcnt.perfread,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perfcnt_intvec_perfread
   },
   {
      .ctl_name      = 8,
      .procname      = "help",
      .data          = &perfcnt.help,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perfcnt_intvec_help
   },
   {
      .ctl_name      = 9,
      .procname      = "verbose",
      .data          = &perfcnt.verbose,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {
      .ctl_name      = 10,
      .procname      = "perfpoll",
      .data          = &perfcnt.perfpoll,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_do_perfcnt_intvec_perfpoll
   },
   {}
};

static struct ctl_table gSysCtl[] = {
   {
      .ctl_name = CTL_BCM_PERFCNT,
      .procname = "perfcnt",
      .mode     = 0555,
      .child    = gSysCtlPerfCnt
   },
   {}
};
/****************************************************************************
*
*  perfpoll_thread
*
*   Worker thread to transfer data to the file or socket.
*
***************************************************************************/
static int perfpoll_thread(void *data)
{
   int justWoken;
   PERFCNT_CNTRS pollcntrs;
   uint32_t ctrl, evt0, evt1, delta_bus_idle, delta_cr0, delta_cr1, delta_tmr, tmr_val;

   daemonize("perfpoll");

   while (1)
   {
      if (0 == wait_event_interruptible(perfpollWakeQ, perfpollWake))
      {
         /* Waking here is a signal to start polling busy/cpi. */
         perfpollWake = 0;
         justWoken = 1;
         while (perfcnt.perfpoll != 0)
         {
            msleep(perfcnt.perfpoll);
            tmr_val = timer_get_tick_count();
            perfcnt_read(&pollcntrs);
            ctrl = arm_perf_read_pmnc();
            evt0 = ARM_PERF_GETEVT0(ctrl);
            evt1 = ARM_PERF_GETEVT1(ctrl);
            delta_bus_idle = pollcntrs.bus_idle - perfcnt.last_bus_idle;
            delta_tmr = tmr_val - perfcnt.last_tmr;
            delta_cr0 = pollcntrs.cr0 - perfcnt.last_cr0;
            delta_cr1 = pollcntrs.cr1 - perfcnt.last_cr1;
            if (perfcnt.verbose >= 3)
            {
               printk("bus_idle = 0x%x, delta_bus_idle=0x%x, delta_cr0=0x%x, delta_cr1=0x%x, delta_tmr=0x%x\n",
                      pollcntrs.bus_idle, delta_bus_idle, delta_cr0, delta_cr1, delta_tmr);
            }
            if ((evt0 == ARM_PERF_I_EXEC) && (delta_cr0/100))
            {
               if (perfcnt.verbose >= 3)
               {
                  printk("pollcntrs.cr0=0x%x perfcnt.last_cr0=0x%x\n", pollcntrs.cr0, perfcnt.last_cr0);
               }
               if (!justWoken)
               {
                  /* Note CCR cycle counter stops when ARM sleeps on "wait for interrupt" in linux idle loop so don't use it for cpi calculation. */
                  printk("CPI*100 = %u    ", 3*delta_tmr/TMR_DIV_FACTOR/(delta_cr0/100));
               }
               perfcnt.last_cr0 = pollcntrs.cr0;
            }
            else if ((evt1 == ARM_PERF_I_EXEC) && (delta_cr1/100))
            {
               if (perfcnt.verbose >= 3)
               {
                  printk("pollcntrs.cr1=0x%x perfcnt.last_cr1=0x%x\n", pollcntrs.cr1, perfcnt.last_cr1);
               }
               if (!justWoken)
               {
                  printk("CPI*100 = %u    ", 3*delta_tmr/TMR_DIV_FACTOR/(delta_cr1/100));
               }
               perfcnt.last_cr1 = pollcntrs.cr1;
            }
            if (!justWoken)
            {
               printk("BusBusy = %d%%\n", 100 - (delta_bus_idle/2)/(delta_tmr/TMR_DIV_FACTOR/100));
            }
            perfcnt.last_tmr = tmr_val;
            perfcnt.last_bus_idle = pollcntrs.bus_idle;
            justWoken = 0;
         }
      }
   }
   complete_and_exit(&perfpollExited, 0);
}

/****************************************************************************/
/**
*  @brief   Check for performance counters availability
*
*  @return 0 on available or -EBUSY for already used by another program
*/
/****************************************************************************/
int perfcnt_availability_check(void)
{
   int rc = 0;

#ifdef CONFIG_OPROFILE
   /* check if oprofile is running */
   if (oprofile_started == 1)
   {
	   printk("Perfcnt has detected that Oprofile is already running\n");
	   printk("Please shutdown Oprofile before using perfcnt.\n");
	   printk("Aborting...\n");
	   rc = -EBUSY;
   }
#endif

   return rc;
}

/****************************************************************************/
/**
*  @brief   Clear performance counters and start counting
*
*  @return
*/
/****************************************************************************/
int perfcnt_start(void)
{
   int rc = perfcnt_availability_check();
   if (rc == 0)
   {
       arm_perf_reset_bus_idle_count();
       arm_perf_start();
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Stop performance counters and print results
*
*  @return
*/
/****************************************************************************/
int perfcnt_stop(PERFCNT_CNTRS *cntrp)
{
	int rc = perfcnt_availability_check();
    if (rc == 0)
    {
	    arm_perf_stop();
        arm_perf_print_mon_cnts(printk, perfcnt.verbose);
    }
    return rc;
}
/****************************************************************************/
/**
*  @brief   Clear performance counters but do not stop or start counting
*
*  @return
*/
/****************************************************************************/
int perfcnt_clear(void)
{
	int rc = perfcnt_availability_check();
	if (rc == 0)
	{
	   arm_perf_reset_bus_idle_count();
	   arm_perf_clearCounts();
	   perfcnt_read(&perfcnt.cntrs);
	}
	return rc;
}
/****************************************************************************/
/**
*  @brief   Read performance counters but do not stop or reset values or PRINT!
*  printing in this function will break knllog since it does reads to capture events
*
*  @return  Fills in cntrp structure
*/
/****************************************************************************/
void perfcnt_read(PERFCNT_CNTRS *cntrp)
{
	arm_perf_getCounts( cntrp );
}
/****************************************************************************/
/**
*  @brief   Get string for event0
*
*  @return  string meaning of first event
*/
/****************************************************************************/
const char *perfcnt_get_evtstr0(void)
{
   return arm_perf_Evt2Str(perfcnt.event0);
}
/****************************************************************************/
/**
*  @brief   Get string for event1
*
*  @return  string meaning of second event
*/
/****************************************************************************/
const char *perfcnt_get_evtstr1(void)
{
   return arm_perf_Evt2Str(perfcnt.event1);
}

/****************************************************************************/
/**
*  @brief   Provide a help string
*
*  @return
*/
/****************************************************************************/
void perfcnt_help(void)
{
   arm_perf_printEvtList(printk);
}

/****************************************************************************/
/**
*  @brief   Process echo N >event[0|1]. Select events to capture on.
*
*  @return
*/
/****************************************************************************/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_do_perfcnt_intvec_event(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int proc_do_perfcnt_intvec_event(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#endif
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos);
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
#endif
      if (rc < 0)
      {
         return rc;
      }
      arm_perf_init_start( perfcnt.event0, perfcnt.event1 );
   }
   else
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos ); /* No special processing for read. */
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); /* No special processing for read. */
#endif
      return rc;
   }
   return rc;
}

/****************************************************************************/
/**
*  @brief   Process echo 1 >start - start performance counters
*
*  @return
*/
/****************************************************************************/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_do_perfcnt_intvec_perfstart(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int proc_do_perfcnt_intvec_perfstart(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#endif
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos);
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
#endif
      if (rc < 0)
      {
         return rc;
      }
      rc = perfcnt_start();
   }
   else
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos ); /* No special processing for read. */
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); /* No special processing for read. */
#endif
      return rc;
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Process echo 1 >stop - stop performance counters and print results
*
*  @return
*/
/****************************************************************************/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_do_perfcnt_intvec_perfstop(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int proc_do_perfcnt_intvec_perfstop(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#endif
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos);
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
#endif
      if (rc < 0)
      {
         return rc;
      }
      rc = perfcnt_stop(&perfcnt.cntrs);
   }
   else
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos ); /* No special processing for read. */
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); /* No special processing for read. */
#endif
      return rc;
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Process echo 1 >clear - clear performance counters and keep counting
*
*  @return
*/
/****************************************************************************/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_do_perfcnt_intvec_perfclear(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int proc_do_perfcnt_intvec_perfclear(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#endif
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos);
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
#endif
      if (rc < 0)
      {
         return rc;
      }
      rc = perfcnt_clear();
   }
   else
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos ); /* No special processing for read. */
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); /* No special processing for read. */
#endif
      return rc;
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Process echo 1 >read - read performance counters and keep counting
*
*  @return
*/
/****************************************************************************/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_do_perfcnt_intvec_perfread(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int proc_do_perfcnt_intvec_perfread(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#endif
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos);
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
#endif
      if (rc < 0)
      {
         return rc;
      }
      arm_perf_print_mon_cnts(printk, perfcnt.verbose);    /* read and print function */
   }
   else
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos ); /* No special processing for read. */
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); /* No special processing for read. */
#endif
      return rc;
   }
   return rc;
}
/*
 ***************************************************************************
 ***************************************************************************
 */
/**
*  @brief   Process echo N >poll - poll and printk bus busy and cpi every N msec, 0 to stop
*
*  @return
*/
/****************************************************************************/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_do_perfcnt_intvec_perfpoll(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int proc_do_perfcnt_intvec_perfpoll(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#endif
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos);
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
#endif
      if (rc < 0)
      {
         return rc;
      }
      if (perfcnt.perfpoll == 0) /* Thread polls this flag to shut down. */
      {
         printk("Stopping perfpoll\n");
         return 0;
      }
      else
      {
#ifdef CONFIG_OPROFILE
         /* check if oprofile is running */
    	 if (perfcnt_availability_check() != 0)
           return -EBUSY;
#endif
         /* wakeup kernel thread to perform dump. */
         perfpollWake = perfcnt.perfpoll; /* wait time in msec */
         wake_up_interruptible(&perfpollWakeQ);
      }
      return rc;
   }
   else
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos ); /* No special processing for read. */
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); /* No special processing for read. */
#endif
      return rc;
   }
   return rc;
}
/**
*  @brief   Process echo 1 >help - output help strings
*
*  @return
*/
/****************************************************************************/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int proc_do_perfcnt_intvec_help(ctl_table *table, int write,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#else
static int proc_do_perfcnt_intvec_help(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
#endif
{
   int rc = 0;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos);
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
#endif
      if (rc < 0)
      {
         return rc;
      }
      perfcnt_help();
   }
   else
   {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
      rc = proc_dointvec(table, write, buffer, lenp, ppos ); /* No special processing for read. */
#else
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); /* No special processing for read. */
#endif
      return rc;
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Initialize by setting up the sysctl and proc/perfcnt entries, allocating
*           default storage if any, and setting variables to defaults.
*
*  @return
*/
/****************************************************************************/
static int __init perfcnt_init(void)
{
   /* register sysctl table */
   printk("%s\n", __FUNCTION__);

#ifdef CONFIG_OPROFILE
   /* check if oprofile is running */
   if (perfcnt_availability_check() != 0)
      return -EBUSY;
#endif

   gSysCtlHeader = register_sysctl_table( gSysCtl );
   if ( gSysCtlHeader == NULL )
   {
      printk("%s: could not register sysctl table\n", __FUNCTION__);
   }

   memset(&perfcnt, 0, sizeof(PERFCNT_OBJ));   /* zero entries, idx, wrap, and enable */

   /* Create dumping to file or socket thread */
   init_completion(&perfpollExited);
   perfpollThreadPid = kernel_thread(perfpoll_thread, 0, 0);

   perfcnt.verbose = 1;
   perfcnt.event0 = ARM_PERF_I_EXEC;
   perfcnt.event1 = ARM_PERF_I_STALL;
   arm_perf_init_start(perfcnt.event0, perfcnt.event1);
   return 0;
}
subsys_initcall(perfcnt_init);
/****************************************************************************/
/**
*  @brief      Exit and cleanup (probably not done)
*
*  @return
*/
/****************************************************************************/
void perfcnt_exit(void)
{
   if ( gSysCtlHeader != NULL )
   {
      unregister_sysctl_table( gSysCtlHeader );
   }
}
