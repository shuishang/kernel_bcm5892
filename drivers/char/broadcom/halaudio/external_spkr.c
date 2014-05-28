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
*  @file    apm_external_spkr.c
*
*  @brief   Implements the external speaker detection interface for the
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
#include <linux/device.h>

#include <linux/broadcom/gpio.h>       /* GPIO register accesses */
#include <linux/broadcom/bcmring/gpio_defs.h>

#include <linux/broadcom/bcm_major.h>     /* For BCM_SPEAKER_MAJOR */
#include <linux/broadcom/speaker.h>       /* Speaker API */

#include <mach/csp/apmHw_reg.h>
#include <csp/apmHw.h>

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

#define HS_DEBUG( fmt,args... ) do { if (gSpeakerDbgPrint) printk( KERN_INFO "%s: " fmt, __FUNCTION__, ##args ); } while (0)
#define HS_ERROR( fmt,args... ) printk( KERN_ERR "%s: " fmt, __FUNCTION__, ##args )


/* Table entry structure used to pair string names to codes */
typedef struct TABLE_ENTRY
{
   int         code;
   const char *str;
}
TABLE_ENTRY;

/* Speaker channel info structure */
struct speaker_info
{
   int               extSpkrGpioPin;/* GPIO pin to detect line out or external speaker */
   int               spkrPaPin;     /* GPIO pin to detect line out or external speaker */
   atomic_t          avail;         /* Only allow single user because of select() */
   int               changed;       /* Flag to indicate state changed */
   speaker_state     state;         /* Current state */
   speaker_event     event;         /* Last event */
   wait_queue_head_t waitq;         /* wait queue */
};

/* ---- Private Variables ------------------------------------------------ */
/* Debug print level */
static int gSpeakerDbgPrint = 1;
static int gSpeakerState;

#if CONFIG_SYSFS
static struct class * speaker_class;
static struct device * speaker_dev;
#endif

/* Speaker channel information */
static struct speaker_info gSpeaker =
{
   .avail         = ATOMIC_INIT( 1 ),
   .state         = SPEAKER_UNPLUGGED,
};

static struct ctl_table gSysCtlSpeaker[] =
{
   {
      .procname      = "dbgprint",
      .data          = &gSpeakerDbgPrint,
      .maxlen        = sizeof(int),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec,
   },
   {
      .procname      = "state",
      .data          = &gSpeakerState,
      .maxlen        = sizeof(int),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec,
   },
   {}
};
static struct ctl_table gSysCtl[] =
{
   {
      .procname      = "external_speaker",
      .mode          = 0555,
      .child         = gSysCtlSpeaker,
   },
   {}
};
static struct ctl_table_header *gSysCtlHeader;

static const TABLE_ENTRY gSpeakerStateNames[] =
{
   { SPEAKER_UNPLUGGED, "unplugged" },
   { SPEAKER_TOGGLE,    "plugged" },
   {}    /* last entry */
};

static const TABLE_ENTRY gSpeakerEventNames[] =
{
   { SPEAKER_REMOVED,   "removed" },
   { SPEAKER_INSERTED,  "inserted" },
   {}    /* last entry */
};

/* ---- Private Function Prototypes -------------------------------------- */
static irqreturn_t line_out_det_isr(int irq, void *data);
static int ext_speaker_probe( int extSpkrGpio, int onBoardSpkrGpio );

static void speaker_wake( speaker_event event, struct speaker_info *speaker );
static int  speaker_open( struct inode *inode, struct file *file );
static int  speaker_release( struct inode *inode, struct file *file );
static int  speaker_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg );
static unsigned int speaker_poll( struct file *file, struct poll_table_struct *poll_table );
static int  speaker_readproc( char *buf, char **start, off_t offset, int count, int *eof, void *data );

/* File Operations (these are the device driver entry points) */
struct file_operations apm_speaker_fops =
{
   .owner   = THIS_MODULE,
   .open    = speaker_open,
   .release = speaker_release,
   .ioctl   = speaker_ioctl,
   .poll    = speaker_poll,
};

/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  Line-out detection interrupt service routine
*
*  @return
*     IRQ_HANDLED - Success
*/
static irqreturn_t line_out_det_isr(int irq, void *data)
{
   int removed;

   removed = (gpio_get_value( gSpeaker.extSpkrGpioPin ) != 1 );

   if ( removed )
   {
      gSpeakerState = 0;
   }
   else
   {
      /* Only turn off on board speaker PA when external speaker is plug in */
      gSpeakerState = 1;
      if ( gSpeaker.spkrPaPin >0 )
      {
         gpio_set_value( gSpeaker.spkrPaPin, 0 );
      }
   }

   speaker_wake( removed ? SPEAKER_REMOVED : SPEAKER_INSERTED, &gSpeaker);

   return IRQ_HANDLED;
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
*  Speaker Wake state machine
*
*  @remarks This routine is expected to be run in an atomic context.
*/
static void speaker_wake(
   speaker_event        event,
   struct speaker_info *speaker
)
{
   /* Only report changes */
   if ( event != speaker->event )
   {
      speaker_state  old_state;

      old_state      = speaker->state;
      speaker->event = event;

      switch ( event )
      {
         case SPEAKER_REMOVED:
            speaker->state = SPEAKER_UNPLUGGED;
            break;

         case SPEAKER_INSERTED:
            speaker->state = SPEAKER_TOGGLE;
            break;

         default:
            break;
      }

      if ( old_state != speaker->state )
      {
         speaker->changed = 1;
         wake_up_interruptible( &speaker->waitq );

         HS_DEBUG( "event=%s, state=%s\n",
               tableCodeToString( event, gSpeakerEventNames ),
               tableCodeToString( speaker->state, gSpeakerStateNames ));
      }
   }
}

/***************************************************************************/
/**
*  Extensions support constructor
*/
static int ext_speaker_probe( int extSpkrGpio, int onBoardSpkrGpio  )
{
   int err;
   int removed;

   printk( KERN_INFO "HAL Audio platform Line-out / external speaker detection. Built %s %s\n",
         __DATE__, __TIME__ );


   if ( extSpkrGpio < 0 )
   {
      return 1;
   }

   gSpeaker.extSpkrGpioPin = extSpkrGpio;
   gSpeaker.spkrPaPin      = onBoardSpkrGpio;


   err = gpio_request( extSpkrGpio, "Line-out detection" );
   if ( err )
   {
      printk( KERN_ERR "%s: Failed to request GPIO pin %i\n", __FUNCTION__, extSpkrGpio );
      return err;
   }

   err = gpio_request( onBoardSpkrGpio, "External speaker op-amp" );
   if ( err )
   {
      printk( KERN_ERR "%s: Failed to request GPIO pin %i\n", __FUNCTION__, onBoardSpkrGpio );
      return err;
   }


   /* Set detection GPIO pin as input */
   gpio_direction_input( extSpkrGpio );

   /* Request and enable interrupt */
   err = request_irq( gpio_to_irq( extSpkrGpio ), line_out_det_isr,
                      (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING), "Line-out Det", NULL );
   if ( err )
   {
      printk( KERN_ERR "%s: Failed to request GPIO irq %i\n", __FUNCTION__, extSpkrGpio );
      return err;
   }
   GpioHw_IrqEnable( extSpkrGpio );

   removed = (gpio_get_value( gSpeaker.extSpkrGpioPin ) != 1 );
   if ( removed )
   {
      gSpeakerState = 0;
      if ( gSpeaker.spkrPaPin >0 )
      {
         gpio_set_value( gSpeaker.spkrPaPin, 1 );
      }
   }
   else
   {
      gSpeakerState = 1;
      if ( gSpeaker.spkrPaPin >0 )
      {
         gpio_set_value( gSpeaker.spkrPaPin, 0 );
      }
   }
   speaker_wake( removed ? SPEAKER_REMOVED : SPEAKER_INSERTED,  &gSpeaker );

   return 0;

}
/***************************************************************************/
/**
*  Driver open method
*
*  @remarks
*/
static int speaker_open( struct inode *inode, struct file *file )
{
   struct speaker_info *spkr;
   int removed;

   spkr                 = &gSpeaker;
   file->private_data   = spkr;

   /* Allow only 1 user because select() only works for single user */
   if ( atomic_dec_and_test( &spkr->avail ) == 0 )
   {
      HS_DEBUG( "spkr already opened by another user\n" );
      atomic_inc( &spkr->avail );
      return -EBUSY;
   }

   /* Initialize state */
   spkr->state = (speaker_state)-1;
   spkr->event = SPEAKER_NULL;

   /* Read current status to initialize state. Non-zero = inserted */

   removed = (gpio_get_value( gSpeaker.extSpkrGpioPin ) != 1 );
   HS_DEBUG( "speaker_open: spkr pin(%d)=%d\n", gSpeaker.extSpkrGpioPin, removed ? SPEAKER_REMOVED : SPEAKER_INSERTED );
   speaker_wake( removed ? SPEAKER_REMOVED : SPEAKER_INSERTED,  spkr );

   return 0;
}

/***************************************************************************/
/**
*  Driver release method
*
*  @remarks
*/
static int speaker_release( struct inode *inode, struct file *file )
{

   return 0;
}

/***************************************************************************/
/**
*  Driver ioctl method
*
*  @remarks
*/
static int speaker_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   struct speaker_info *ch = file->private_data;

   switch ( cmd )
   {
      case SPEAKER_IOCTL_GET_STATE:
         if ( copy_to_user( (unsigned long *)arg, &ch->state, sizeof(ch->state) ) != 0 )
         {
            return -EFAULT;
         }
         HS_DEBUG( "state=%s\n", tableCodeToString( ch->state, gSpeakerStateNames) );
         break;

      default:
         HS_DEBUG( "Unrecognized ioctl: '0x%x'\n", cmd );
         return -ENOTTY;
   }
   return 0;
}


/***************************************************************************/
/**
*  Process file read method
*/
static int  speaker_readproc( char *buf, char **start, off_t offset, int count, int *eof, void *data )
{
   int len = 0;
   struct speaker_info *speaker;

   speaker = &gSpeaker;

   if ( atomic_read( &speaker->avail ))
   {
      len += sprintf( buf+len, "Speaker disabled\n" );
   }
   else
   {
      len += sprintf( buf+len, "Speaker state=%s\n",
            tableCodeToString( speaker->state, gSpeakerStateNames ) );
   }

   *eof = 1;
   return len+1;
}

static unsigned int speaker_poll( struct file *file, struct poll_table_struct *poll_table )
{
   struct speaker_info *spkr = file->private_data;

   poll_wait( file, &spkr->waitq, poll_table );

   if ( spkr->changed )
   {
      spkr->changed = 0;
      return POLLIN | POLLRDNORM;
   }

   return 0;
}

/***************************************************************************/
/**
*  Initialize speaker detection
*
*  @remarks
*/
int speakerInit( int extSpkrGpio, int onBoardSpkrGpio )
{
   int                  rc;
   /* Just enough space to hold speaker name. */
   char speaker_buf[15];

   init_waitqueue_head( &gSpeaker.waitq );

   rc = register_chrdev( BCM_SPEAKER_MAJOR, "speaker", &apm_speaker_fops );
   if ( rc < 0 )
   {
      HS_ERROR( "failed for to register device major %d\n", BCM_SPEAKER_MAJOR );
      return rc;
   }

   gSysCtlHeader = register_sysctl_table( gSysCtl );
   create_proc_read_entry( "speaker", 0, NULL, speaker_readproc, NULL );

#if CONFIG_SYSFS
   speaker_class = class_create(THIS_MODULE,"bcmring-spkr");
   if(IS_ERR(speaker_class)){
           printk(KERN_ERR "SPEAKER: Class create failed\n");
           rc = -EFAULT;
           goto err_unregister_chrdev;
   }

   snprintf( speaker_buf, sizeof(speaker_buf), "speaker" );

   speaker_dev = device_create(speaker_class, NULL, MKDEV(BCM_SPEAKER_MAJOR,0),NULL,speaker_buf);
   if(IS_ERR(speaker_dev)){
           printk(KERN_ERR "SPEAKER: Device %s create failed\n", speaker_buf);
           rc = -EFAULT;
           goto err_class_destroy;
   }

#endif

   ext_speaker_probe( extSpkrGpio, onBoardSpkrGpio );

   return 0;
#if CONFIG_SYSFS
err_class_destroy:
   class_destroy(speaker_class);
err_unregister_chrdev:
   unregister_chrdev(BCM_SPEAKER_MAJOR, "speaker");
   return rc;
#endif
}

/***************************************************************************/
/**
*  Destructor for the speaker detection driver
*
*  @remarks
*/
void speakerExit( void )
{
   remove_proc_entry( "speaker", NULL );
   unregister_chrdev( BCM_SPEAKER_MAJOR, "speaker" );

#if CONFIG_SYSFS
   device_destroy(speaker_class, MKDEV(BCM_SPEAKER_MAJOR,0));
   class_destroy(speaker_class);
#endif

   if ( gSysCtlHeader )
   {
      unregister_sysctl_table( gSysCtlHeader );
      gSysCtlHeader = NULL;
   }
}

