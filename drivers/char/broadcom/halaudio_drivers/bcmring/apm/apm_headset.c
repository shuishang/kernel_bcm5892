/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
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
*  @file    apm_headset.c
*
*  @brief   Implements the headset detection interface for the
*           BCMRING APM block.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/sched.h>                  /* needed for daemonize */
#include <linux/sysctl.h>                 /* sysctl interface */
#include <linux/timer.h>                  /* timer used to control test freq */
#include <linux/device.h>

#include <linux/broadcom/bcm_major.h>     /* For BCM_HEADSET_MAJOR */
#include <linux/broadcom/headset.h>       /* Headset API */

#include <mach/csp/apmHw_reg.h>
#include <csp/apmHw.h>

#include "apm_drv.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

#define HS_DEBUG( fmt,args... ) do { if (gHeadsetDbgPrint) printk( KERN_INFO "%s: " fmt, __FUNCTION__, ##args ); } while (0)
#define HS_ERROR( fmt,args... ) printk( KERN_ERR "%s: " fmt, __FUNCTION__, ##args )

#define HEADSET_STATE_DEFAULT_POLL_MS     200

/* Table entry structure used to pair string names to codes */
typedef struct TABLE_ENTRY
{
   int         code;
   const char *str;
}
TABLE_ENTRY;

/* Headset channel info structure */
struct headset_info
{
   APM_MICDET_CHAN   ch;            /* Channel index */
   atomic_t          avail;         /* Only allow single user because of select() */
   int               changed;       /* Flag to indicate state changed */
   headset_state     state;         /* Current state */
   headset_event     event;         /* Last event */
   atomic_t          timer_active;  /* Timer active flag */
   struct timer_list timer;         /* Polling timer */
   wait_queue_head_t waitq;         /* wait queue */
   int               pollms;        /* headset status polling period */
};

/* ---- Private Variables ------------------------------------------------ */
/* Debug print level */
static int gHeadsetDbgPrint = 0;

#if CONFIG_SYSFS
static struct class * headset_class;
static struct device * headset_dev;
#endif

/* Headset channel information */
static struct headset_info gHeadset[APM_MICDET_MAX_CHANS] =
{
   [APM_MICDET_A]    =
   {
      .ch            = APM_MICDET_A,
      .avail         = ATOMIC_INIT( 1 ),
      .pollms        = HEADSET_STATE_DEFAULT_POLL_MS,
   },
   [APM_MICDET_B]    =
   {
      .ch            = APM_MICDET_B,
      .avail         = ATOMIC_INIT( 1 ),
      .pollms        = HEADSET_STATE_DEFAULT_POLL_MS,
   },
};

static struct ctl_table gSysCtlHeadset[] =
{
   {
      .procname      = "dbgprint",
      .data          = &gHeadsetDbgPrint,
      .maxlen        = sizeof(int),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec,
   },
   {
      .procname      = "msec0",
      .data          = &gHeadset[0].pollms,
      .maxlen        = sizeof(int),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec,
   },
   {
      .procname      = "msec1",
      .data          = &gHeadset[1].pollms,
      .maxlen        = sizeof(int),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec,
   },
   {}
};
static struct ctl_table gSysCtl[] =
{
   {
      .procname      = "headset",
      .mode          = 0555,
      .child         = gSysCtlHeadset,
   },
   {}
};
static struct ctl_table_header *gSysCtlHeader;

static const TABLE_ENTRY gHeadsetStateNames[] =
{
   { HEADSET_UNPLUGGED, "unplugged" },
   { HEADSET_TOGGLE_A,  "plugged-toggle_a" },
   { HEADSET_TOGGLE_B,  "plugged-toggle_b" },
   {}    /* last entry */
};

static const TABLE_ENTRY gHeadsetEventNames[] =
{
   { HEADSET_REMOVED,   "removed" },
   { HEADSET_INSERTED,  "inserted" },
   { HEADSET_BUTTON,    "button" },
   {}    /* last entry */
};

/* ---- Private Function Prototypes -------------------------------------- */
static void apm_mic_in_detect( APM_MICDET_CHAN ch );
/*static void apmMicOnDetected( APM_MICDET_CHAN ch ); */

static void headset_wake( headset_event event, struct headset_info *ch );
static int  headset_open( struct inode *inode, struct file *file );
static int  headset_release( struct inode *inode, struct file *file );
static int  headset_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg );
static unsigned int headset_poll( struct file *file, struct poll_table_struct *poll_table );
static int  headset_readproc( char *buf, char **start, off_t offset, int count, int *eof, void *data );

/* Micrphone detection callbacks */
static struct apm_mic_det_ops detops =
{
   .mic_in  = apm_mic_in_detect,
   /*.mic_on  = apmMicOnDetected, */
};

/* File Operations (these are the device driver entry points) */
struct file_operations apm_headset_fops =
{
   .owner   = THIS_MODULE,
   .open    = headset_open,
   .release = headset_release,
   .ioctl   = headset_ioctl,
   .poll    = headset_poll,
};

/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  Helper routine to reschedule polling timer to detect the headset unplug
*  event.
*
*  @remarks    This routine may be called from an atomic context.
*/
static inline void apm_resched_timer( struct headset_info *ch )
{
   ch->timer.expires = jiffies + (HZ * ch->pollms / 1000);
   add_timer( &ch->timer );
   atomic_set( &ch->timer_active, 1 );
}

/***************************************************************************/
/**
*  APM mic detection read routine to poll the status. The APM hardware
*  circuitry does not provide a microphone removal signal and thus the
*  status registers needs to be polled for this information.
*
*  @return  0 nothing inserted, non-zero if inserted
*/
static inline int apm_micdet_read(
   struct headset_info *ch    /*<< (i) ptr to headset channel info */
)
{
   return ApmHw_MicIsDetected( ch->ch );
}

/***************************************************************************/
/**
*  Helper used to print nice strings from enumerated codes
*
*  @remarks
*/
static const char *tableCodeToString( int code, const TABLE_ENTRY *list )
{
   while ( list->str[0] != '\0' )
   {
      if ( list->code == code )
      {
         return list->str;
      }
      list++;
   }
   return "unknown";
}

/***************************************************************************/
/**
*  Headset Wake state machine
*
*  @remarks This routine is expected to be run in an atomic context.
*/
static void headset_wake(
   headset_event        event,         /*<< (i) headset event */
   struct headset_info *ch             /*<< (i) ptr to headset det chan */
)
{
   /* Only report changes */
   if ( event != ch->event )
   {
      headset_state  old_state;

      old_state   = ch->state;
      ch->event   = event;

      switch ( event )
      {
         case HEADSET_REMOVED:
            ch->state = HEADSET_UNPLUGGED;
            break;

         case HEADSET_INSERTED:
            ch->state = HEADSET_TOGGLE_A;
            if ( !atomic_read( &ch->timer_active ))
            {
               /* Start timer to detect removal if not already active */
               apm_resched_timer( ch );
            }
            break;

#if 0 /* Unsupported */
         case HEADSET_BUTTON:
            /* Toggle button states */
            if ( ch->state == HEADSET_TOGGLE_A )
            {
               ch->state = HEADSET_TOGGLE_B;
            }
            else
            {
               ch->state = HEADSET_TOGGLE_A;
            }
            break;
#endif
         default:
            break;
      }

      if ( old_state != ch->state )
      {
         ch->changed = 1;
         wake_up_interruptible( &ch->waitq );

         HS_DEBUG( "ch=%i event=%s, state=%s\n", ch->ch,
               tableCodeToString( event, gHeadsetEventNames ),
               tableCodeToString( ch->state, gHeadsetStateNames ));
      }
   }
}

/***************************************************************************/
/**
*  Timer expiry routine used to poll whether the headset has been unplugged
*
*  @remarks This timer callback runs in an atomic context
*/
static void micdet_timedout( unsigned long data )
{
   struct headset_info *ch = (void *)data;
   int inserted;

   inserted = apm_micdet_read( ch );

   HS_DEBUG( "ch=%i inserted=%i\n", ch->ch, inserted );

   if ( inserted )
   {
      apm_resched_timer( ch );
   }
   else
   {
      headset_wake( HEADSET_REMOVED, ch );
      atomic_set( &ch->timer_active, 0 );
   }
}

/***************************************************************************/
/**
*  Driver open method
*
*  @remarks
*/
static int headset_open( struct inode *inode, struct file *file )
{
   unsigned int         detchan;
   struct headset_info *ch;
   int                  rc;

   detchan = iminor( inode );
   if ( detchan >= APM_MICDET_MAX_CHANS )
   {
      return -EINVAL;
   }

   ch                   = &gHeadset[detchan];
   file->private_data   = ch;

   /* Allow only 1 user because select() only works for single user */
   if ( atomic_dec_and_test( &ch->avail ) == 0 )
   {
      HS_DEBUG( "ch=%i already opened by another user\n", ch->ch );
      atomic_inc( &ch->avail );
      return -EBUSY;
   }

   /* Initialize state */
   ch->state = (headset_state)-1;
   ch->event = HEADSET_NULL;

   init_timer( &ch->timer );
   ch->timer.function  = micdet_timedout;
   ch->timer.data      = (unsigned long)ch;
   atomic_set( &ch->timer_active, 0 );

   /* Enable mic detection on selected channel */
   rc = apmEnableMicDetect( ch->ch, 1 );
   if ( rc )
   {
      HS_DEBUG( "ch=%i failed to enabled mic detection channel rc=%i\n",
            ch->ch, rc );
      return rc;
   }

   /* Read current status to initialize state. Non-zero = inserted */
   rc = apm_micdet_read( ch );
   headset_wake( rc ? HEADSET_INSERTED : HEADSET_REMOVED, ch );

   return 0;
}

/***************************************************************************/
/**
*  Driver release method
*
*  @remarks
*/
static int headset_release( struct inode *inode, struct file *file )
{
   struct headset_info *ch = file->private_data;

   apmEnableMicDetect( ch->ch, 0 );

   if ( atomic_read( &ch->timer_active ))
   {
      del_timer( &ch->timer );
      atomic_set( &ch->timer_active, 0 );
   }

   atomic_inc( &ch->avail );

   return 0;
}

/***************************************************************************/
/**
*  Driver ioctl method
*
*  @remarks
*/
static int headset_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   struct headset_info *ch = file->private_data;

   switch ( cmd )
   {
      case HEADSET_IOCTL_GET_STATE:
         if ( copy_to_user( (unsigned long *)arg, &ch->state, sizeof(ch->state) ) != 0 )
         {
            return -EFAULT;
         }
         HS_DEBUG( "ch=%i state=%s\n", ch->ch, tableCodeToString( ch->state, gHeadsetStateNames) );
         break;

      default:
         HS_DEBUG( "ch=%i Unrecognized ioctl: '0x%x'\n", ch->ch, cmd );
         return -ENOTTY;
   }
   return 0;
}

/***************************************************************************/
/**
*  Driver poll method to support systel select call
*
*  @remarks
*/
static unsigned int headset_poll( struct file *file, struct poll_table_struct *poll_table )
{
   struct headset_info *ch = file->private_data;

   poll_wait( file, &ch->waitq, poll_table );

   if ( ch->changed )
   {
      ch->changed = 0;
      return POLLIN | POLLRDNORM;
   }

   return 0;
}

/***************************************************************************/
/**
*  Microphone inserted callback.
*
*  @remarks This routine runs in an atomic context
*/
static void apm_mic_in_detect( APM_MICDET_CHAN chan )
{
   struct headset_info *ch;

   if ( chan >= APM_MICDET_MAX_CHANS )
   {
      return;
   }

   ch = &gHeadset[chan];

   /* Only report headset events to opened headset channels */
   if ( !atomic_read( &ch->avail ))
   {
      headset_wake( HEADSET_INSERTED, ch );
   }
}

#if 0
/***************************************************************************/
/**
*  Microphone on state detected
*
*  @remarks This routine runs in an atomic context
*/
static void apmMicOnDetected( APM_MICDET_CHAN chan )
{
}
#endif

/***************************************************************************/
/**
*  Process file read method
*/
static int  headset_readproc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int len = 0;
   int chan;
   struct headset_info *ch;

   ch = gHeadset;
   for ( chan = 0; chan < APM_MICDET_MAX_CHANS; chan++, ch++ )
   {
      if ( atomic_read( &ch->avail ))
      {
         len += sprintf( buf+len, "Ch %c: disabled\n", chan + 'A' );
      }
      else
      {
         len += sprintf( buf+len, "Ch %c: state=%s timer_active=%i pollms=%i\n",
               chan + 'A',
               tableCodeToString( ch->state, gHeadsetStateNames ),
               atomic_read( &ch->timer_active ), ch->pollms );
      }
   }

   *eof = 1;
   return len+1;
}

/***************************************************************************/
/**
*  Initialize APM headset detection
*
*  @remarks
*/
int __init apmHeadsetInit( void )
{
   struct headset_info *ch;
   int                  rc, i;
   /* Just enough space to hold headset name. */
   char headset_buf[15];

   ch = &gHeadset[0];
   for ( i = 0; i < APM_MICDET_MAX_CHANS; i++, ch++ )
   {
      init_waitqueue_head( &ch->waitq );
   }

   apmSetMicDetectOps( &detops );

   rc = register_chrdev( BCM_HEADSET_MAJOR, "headset", &apm_headset_fops );
   if ( rc < 0 )
   {
      HS_ERROR( "failed for to register device major %d\n", BCM_HEADSET_MAJOR );
      return rc;
   }

   gSysCtlHeader = register_sysctl_table( gSysCtl );
   create_proc_read_entry( "headset", 0, NULL, headset_readproc, NULL );

#if CONFIG_SYSFS
   headset_class = class_create(THIS_MODULE,"bcmring-headset");
   if(IS_ERR(headset_class)){
           printk(KERN_ERR "HEADSET: Class create failed\n");
           rc = -EFAULT;
           goto err_unregister_chrdev;
   }

   for ( i = 0; i < APM_MICDET_MAX_CHANS; i++ )
   {
      snprintf( headset_buf, sizeof(headset_buf), "headset%d", i);

      headset_dev = device_create(headset_class, NULL, MKDEV(BCM_HEADSET_MAJOR,i),NULL,headset_buf);
      if(IS_ERR(headset_dev)){
              printk(KERN_ERR "HEADSET: Device(%d) create failed\n", i);
              rc = -EFAULT;
              goto err_class_destroy;
      }
   }
#endif

   return 0;
#if CONFIG_SYSFS
err_class_destroy:
   class_destroy(headset_class);
err_unregister_chrdev:
   unregister_chrdev(BCM_HEADSET_MAJOR, "headset");
   return rc;
#endif
}

/***************************************************************************/
/**
*  Destructor for the APM headset detection driver
*
*  @remarks
*/
void __exit apmHeadsetExit( void )
{
   int i;

   apmSetMicDetectOps( NULL );

   remove_proc_entry( "headset", NULL );
   unregister_chrdev( BCM_HEADSET_MAJOR, "headset" );

#if CONFIG_SYSFS
   for ( i = 0; i < APM_MICDET_MAX_CHANS; i++ )
   {
      device_destroy(headset_class, MKDEV(BCM_HEADSET_MAJOR,i));
   }
   device_destroy(headset_class, MKDEV(BCM_HEADSET_MAJOR,0));
   class_destroy(headset_class);
#endif

   if ( gSysCtlHeader )
   {
      unregister_sysctl_table( gSysCtlHeader );
      gSysCtlHeader = NULL;
   }
}
