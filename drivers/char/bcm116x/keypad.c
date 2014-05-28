/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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



/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/kfifo.h>
#include <linux/kd.h>

#include <asm/irq.h>
#include <mach/reg_irq.h>
#include <mach/reg_keypad.h>
#include <mach/reg_sys.h>
#include <linux/broadcom/regaccess.h>
#include <asm/mach/irq.h>
#include <asm/io.h>

#include <linux/broadcom/hw_cfg.h>
#include <linux/broadcom/keymap.h>
#include <linux/broadcom/timer.h>
#include <linux/broadcom/knllog.h>

/*
 * ---- Public Variables ------------------------------------------------- 
 * ---- Private Constants and Types -------------------------------------- 
 */

/*--------------------------------------------------------------------------*/
/* Driver configuration (compile-time)
*/

#define USE_DYNAMIC_DEV_ALLOC    0

#define DEBUG                    0
#if DEBUG
#define MARKER                   "------->"
#endif

#ifndef MARKER
#define MARKER
#endif

#define myPrintk(f,s...)      printk( MARKER "%s(): " f, __func__, ## s )
#define myPanic(f,s...)       panic( MARKER "%s(): " f, __func__, ## s )

#if DEBUG
#define FUNC_ENTRY(f,s...)    myPrintk( "***ENTRY*** " f, ## s )
#define FUNC_EXIT(f,s...)     myPrintk( "***EXIT*** " f, ## s )
#else
#define FUNC_ENTRY(f,s...)
#define FUNC_EXIT(f,s...)
#endif

#ifdef HW_DEFAULT_KEYMAP
/* This sets the default keymap at compile-time.  The preferred method is to 
*  use loadkeys to load the keymap via a start-up script at run-time
*/
static KEYMAP defaultKeymap[] = HW_DEFAULT_KEYMAP;
#define DEFAULT_KEYMAP        defaultKeymap 
#define DEFAULT_KEYMAP_SIZE   (sizeof( DEFAULT_KEYMAP ) / sizeof( DEFAULT_KEYMAP[0] ))
#else
#define DEFAULT_KEYMAP        NULL
#define DEFAULT_KEYMAP_SIZE   0
#endif

#if USE_DYNAMIC_DEV_ALLOC 
#undef BCM_KEYPAD_MAJOR
#else
#include <linux/broadcom/bcm_major.h>
#endif

/* For compatibility with other Broadcom keypad drivers, the first keypad
*  device (minor 0) should report both key up and key down events.  The second
*  keypad device (minor 1) should report only key down events
*/
#define DEV_NAME        "keypad"
#define DEV_FIRST       0
#define DEV_LAST        1
#define DEV_NUM         (DEV_LAST - DEV_FIRST + 1)

#define KBD_FIFO_SIZE   64       /* Must be power of 2 */

#define ROW_MASK              0xf
#define ROW_NUM               (ROW_MASK + 1)
#define COL_MASK              0xf
#define COL_NUM               (COL_MASK + 1)

typedef unsigned int scancode_t;
typedef unsigned char keycode_t;

#define SCANCODE(r,c)         (scancode_t)((((r) & ROW_MASK) << 4) | \
                                           ((c) & COL_MASK))
#define SCANCODE_UP_MASK      0x80000000
#define SCANCODE_ROW(s)       (((s) >> 4) & ROW_MASK)
#define SCANCODE_COL(s)       ((s) & COL_MASK)

#define SCANCODE_TERM         SCANCODE( ROW_NUM - 1, COL_NUM - 1 )

#define KEYCODE_NONE          -1
#define KEYCODE_UP_MASK       0x80


#define MAX_CLIENTS  4  /* Max number of simultaneous opens */

typedef int KBD_ROW;

typedef struct
{
   int row;
   int col;
} MATRIX;

typedef struct
{                                   
   int major;
   wait_queue_head_t waitQueue;
   struct cdev cdev;
   struct kfifo *fifop[MAX_CLIENTS];
   struct
   {
      MATRIX layout;
      MATRIX first;
      MATRIX last;
      KBD_ROW *prevp;
      int size;
   } matrix;
} CBLK;

timer_tick_count_t  gLastKeypadIrqTick;
timer_tick_count_t  gLastKeypadReadTick;

/* ---- Private Variables ------------------------------------------------ */
static char banner[] __initdata = KERN_INFO "BCM116x Keypad Driver: 1.00\n";
static  int             gOpenCount = 0;
static int keycodeMap[ROW_NUM][COL_NUM];
static CBLK cblk;

/* ---- Private Function Prototypes -------------------------------------- */
static irqreturn_t keypad_interrupt( int irq, void *dev_id );
static int kbdProcess( CBLK *cblkp, KBD_ROW *currentp );
static int keycodeSet( CBLK *cblkp, unsigned int scancode, unsigned int keycode );
static int keycodeGet( CBLK *cblkp, unsigned int scancode, unsigned int *keycodep );

static int keypad_open( struct inode *inode, struct file *file );
static int keypad_release( struct inode *inode, struct file *file );
static ssize_t keypad_read( struct file *file, char __user *buffer, size_t count, loff_t *f_pos );
static unsigned int keypad_poll( struct file *file, struct poll_table_struct *poll_table );
static int keypad_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg ); 

/* ---- Functions -------------------------------------------------------- */
#define kpdlKeyIsDown(m,r,c)     !(((int *)(m))[r] & (1 << (c)))

static int matrix[ROW_NUM];      /* copy of keypad matrix state */

static int kpdlMatrix( void )
{
   register unsigned int keypadVal;

   keypadVal = REG_KEYPAD_KPSSR0;
   matrix[0] = keypadVal & 0xFF;
   keypadVal >>= 8;
   matrix[1] = keypadVal & 0xFF;
   keypadVal >>= 8;
   matrix[2] = keypadVal & 0xFF;
   keypadVal >>= 8;
   matrix[3] = keypadVal & 0xFF;

   keypadVal = REG_KEYPAD_KPSSR1;
   matrix[4] = keypadVal & 0xFF;
   keypadVal >>= 8;
   matrix[5] = keypadVal & 0xFF;
   keypadVal >>= 8;
   matrix[6] = keypadVal & 0xFF;
   keypadVal >>= 8;
   matrix[7] = keypadVal & 0xFF;

   return( (int)&matrix[0] );
}

static int kbdProcess( CBLK *cblkp, KBD_ROW *currentp )
{
   int client;
   int r;
   int c;
   int down;
   int num;
   scancode_t scancode;

   FUNC_ENTRY( "cblkp=0x%x, currentp=0x%x\n", (int)cblkp, (int)currentp );

   num = 0;

   for( r=cblkp->matrix.first.row; r<=cblkp->matrix.last.row; r++ )
   {
      for( c=cblkp->matrix.first.col; c<=cblkp->matrix.last.col; c++ )
      {
         /* Get current key down state of specified row and column */
         down = kpdlKeyIsDown( currentp, r, c );

         /* Compare current key down state with previous state */
         if( down != kpdlKeyIsDown( cblkp->matrix.prevp, r, c ) )
         {
            /* Key state has changed */

            /* Copy scan code to read buffer */
            scancode = SCANCODE( r, c );

            /* Copy key state to buffer holding previous key state */
            if ( down )
            {
#ifdef CONFIG_BCM_KNLLOG_IRQ
   				if (gKnllogIrqSchedEnable & KNLLOG_FB) KNLLOG("keydown\n");
#endif
                gLastKeypadIrqTick = timer_get_tick_count();
            }
            else
            {
               scancode |= SCANCODE_UP_MASK;
            }

            for (client=0; client<gOpenCount; client++) 
            {
               __kfifo_put( cblkp->fifop[client], (unsigned char *)&scancode,
                            sizeof( scancode ) );
            }

            num++;
         }
      }

      /* Copy key state to buffer holding previous key state */
      cblkp->matrix.prevp[r] = currentp[r];
   }

   scancode = SCANCODE_TERM;
   for (client=0; client<gOpenCount; client++) 
   {
      __kfifo_put( cblkp->fifop[client], (unsigned char *)&scancode, sizeof( scancode ) );
   }
   return( num );
}


static int keycodeSet( CBLK *cblkp, unsigned int scancode, unsigned int keycode )
{
   int r;
   int c;

   FUNC_ENTRY( "keycodeSet(): scancode=0x%08x, keycode=0x%08x\n", scancode, keycode );

   if( scancode > SCANCODE_TERM )
   {
      /* Scancode is out of range */
      return( -ERANGE );
   }

   r = SCANCODE_ROW( scancode );
   c = SCANCODE_COL( scancode );

   keycodeMap[r][c] = (int)keycode;

   /* Update keypad matrix dimensions */
   if( r < cblkp->matrix.first.row )
   {
      cblkp->matrix.first.row = r;
   }

   if( c < cblkp->matrix.first.col )
   {
      cblkp->matrix.first.col = c;
   }

   if( r > cblkp->matrix.last.row )
   {
      cblkp->matrix.last.row = r;
      
   }

   if( c > cblkp->matrix.last.col )
   {
      cblkp->matrix.last.col = c;
   }

   regaccess_or_bits( &REG_SYS_IOCR1, 1 << r );
   reg_keypad_set_interrupt_type(r, c, REG_KEYPAD_KPEMR_BOTH_EDGE_INTERRUPT_TRIGGER);
   reg_keypad_set_imr(r,c);
   regaccess_or_bits( &REG_SYS_IOCR1, 1 << (c + 8) );
   
   return( 0 );
}

static int keycodeGet( CBLK *cblkp, unsigned int scancode, unsigned int *keycodep )
{
   int r;
   int c;

   FUNC_ENTRY( "keycodeGet(): scancode=0x%08x\n", scancode );

   if( scancode > SCANCODE_TERM )
   {
      /* Scancode is out of range */
      return( -ERANGE );
   }

   r = SCANCODE_ROW( scancode );
   c = SCANCODE_COL( scancode );

   if( keycodeMap[r][c] == KEYCODE_NONE )
   {
      /* Keycode not mapped */
      return( -ENODEV );
   }

   *keycodep = (unsigned int)keycodeMap[r][c];

   return( 0 );
}

/****************************************************************************
*
*  keypad_interrupt
*
***************************************************************************/
static irqreturn_t keypad_interrupt( int irq, void *dev_id )
{
   CBLK *cblkp = &cblk;
   
    /* We're called with interrupts disabled. */

   FUNC_ENTRY( "irq=%d, dev_id=0x%08x, regs=0x%08x\n", irq, (int)dev_id, (int)regs );
   (void)irq;

   /* Process key presses */
   disable_irq( IRQ_KEYPAD );

   kbdProcess( cblkp, (KBD_ROW *)kpdlMatrix() );

   /* Clear the interrupt source */
   REG_KEYPAD_KPICR0 = ~0;             /* clear all specific keypad interrupt sources */
   REG_KEYPAD_KPICR1 = ~0;
   REG_IRQ_ICR = REG_IRQ_ICR_KPICLR;   /* clear keypad global interrupt source */

   enable_irq( IRQ_KEYPAD );

   if( likely( __kfifo_len( cblkp->fifop[0] ) ) > 0 )
   {
      /* Key events have been queued in FIFO, so unblock read() */
      wake_up_interruptible( &cblkp->waitQueue );
   }
   return( IRQ_HANDLED );
}

/****************************************************************************
*
*  keypad_open
*
***************************************************************************/
static int keypad_open( struct inode *inode, struct file *file )
{
   int rc;
   int dev;
   CBLK *cblkp = &cblk;
   int client = gOpenCount;

   dev = iminor( inode );

   FUNC_ENTRY( "inode=0x%08x, inode->i_cdev=0x%08x, file=0x%08x (major=%d, minor=%d) client=%d\n", 
               (int)inode, (int)inode->i_cdev, (int)file, imajor( inode ), dev, client );

   file->private_data = (void *)client;
   
   if (gOpenCount >= MAX_CLIENTS) 
   {
      myPrintk( KERN_ERR "No more than %d clients allowed\n", MAX_CLIENTS );
      return( -EAGAIN );
   }
   
   /* Allocate FIFO for storing keypresses */
   cblkp->fifop[client] = kfifo_alloc( KBD_FIFO_SIZE, GFP_KERNEL, NULL );
   if( !cblkp->fifop[client] )
   {
      myPrintk( KERN_ERR "Failed to alloc FIFO %d for key presses\n", client );
      return( -ENOMEM );
   }

   if( gOpenCount == 0 )
   {
      /* Get keypad layout information */
      cblkp->matrix.layout.row = 8;
      cblkp->matrix.layout.col = 8;
      cblkp->matrix.size = sizeof( KBD_ROW ) * cblkp->matrix.layout.row;
   
      /* Allocate memory for storing current and previous key state */
      cblkp->matrix.prevp = kmalloc( cblkp->matrix.size, GFP_KERNEL );
      if( !cblkp->matrix.prevp )
      {      
         myPrintk( KERN_ERR "Failed to alloc keyboard matrix memory for "
                   "keystate\n" );
         return( -ENOMEM );
      }
   
      /*
      ** Just before registering the interupt handler, clear any interrupts
      ** that may be pending (this just prevents an initial interrupt
      ** from occuring as soon as request_irq is called).
      */
      REG_KEYPAD_KPICR0 = ~0;             /* clear all specific keypad interrupt sources */
      REG_KEYPAD_KPICR1 = ~0;
      REG_IRQ_ICR = REG_IRQ_ICR_KPICLR;   /* clear keypad global interrupt source */
   
      /* Setup interrupt handler for key presses */
      rc = request_irq( IRQ_KEYPAD, keypad_interrupt, IRQF_DISABLED, DEV_NAME, (void *)cblkp );

      if( rc )
      {
         myPrintk( KERN_ERR "Failed to get IRQ\n" );
         kfree( cblkp->matrix.prevp );
         kfifo_free( cblkp->fifop[client] );
         return( rc );
      } 
   
      /* Initalize starting state of keyboard */
      memcpy( cblkp->matrix.prevp, (KBD_ROW *)kpdlMatrix(),
              cblkp->matrix.size );
   
      /* So that the enable will be balanced. */
      disable_irq( IRQ_KEYPAD );
   
      init_waitqueue_head( &cblkp->waitQueue );
   
      enable_irq( IRQ_KEYPAD );
   }

   gOpenCount++;
   
   return( 0 );
}

/****************************************************************************
*
*  keypad_release
*
***************************************************************************/
static int keypad_release( struct inode *inode, struct file *file )
{
   int dev;
   CBLK *cblkp = &cblk;
   int client = (int)file->private_data;

   dev = iminor( inode );

   FUNC_ENTRY( "inode=0x%08x, file=0x%08x (major=%d, minor=%d) client=%d\n", (int)inode, (int)file, imajor( inode ), dev, client );

   kfifo_free( cblkp->fifop[client] );
 
   if( --gOpenCount > 0 )
   {
      return( 0 );
   }

   disable_irq( IRQ_KEYPAD );
   free_irq( IRQ_KEYPAD, (void *)cblkp );

   kfree( cblkp->matrix.prevp );

   return( 0 );
}

/****************************************************************************
*
*  keypad_read
*
***************************************************************************/
static ssize_t keypad_read( struct file *file, char __user *buffer, size_t count, loff_t *f_pos )
{
   int rc;
   CBLK *cblkp = &cblk;
   int client = (int)file->private_data;
   scancode_t scancode;
   keycode_t keycode;
   unsigned int keycodeInt;
   int bufIndex;

   FUNC_ENTRY( "file=0x%08x, buffer=0x%08x, count=%d, f_pos=0x%08x client=%d\n", (int)file, (int)buffer, count, (int)f_pos, client );

   if( unlikely( (count < sizeof( keycode_t )) || !buffer ) )
   {
      return( -ENOMEM );
   }

   bufIndex = 0;

   while( bufIndex < count )
   {
      if( (file->f_flags & O_NONBLOCK) && (__kfifo_len( cblkp->fifop[client] ) == 0) )
      {
          /* File was opened in non-blocking mode and no data is available,
          ** so don't wait for a key event and exit from the loop
          */
          break;
      }

      rc = wait_event_interruptible( cblkp->waitQueue, __kfifo_len( cblkp->fifop[client] ) );
      if( unlikely( rc ) )
      {
         return( rc );
      }

      disable_irq( IRQ_KEYPAD );

      __kfifo_get( cblkp->fifop[client], (unsigned char *)&scancode,
                   sizeof( scancode ) );

      enable_irq( IRQ_KEYPAD );

      rc = keycodeGet( cblkp, scancode & ~SCANCODE_UP_MASK, &keycodeInt );
      if( rc )
      {
         scancode &= ~SCANCODE_UP_MASK; 

         if( (rc == -ENODEV) && (scancode != SCANCODE_TERM) )
         {
            /* Unmapped scancode */
            myPrintk( KERN_WARNING "No keycode available for scancode "
                      "0x%08x.  Use \"bloadkeys <keymap_file>\" or "
                      "\"bsetkeycodes <scancode> <keycode>\" to set the key"
                      "mapping.\n", scancode );
         }
         else if( rc == -ERANGE )
         {
            /* Unsupported scancode */
            myPrintk( KERN_WARNING "Invalid scancode 0x%08x.  Possible "
                      "hardware configuration problem.\n", scancode );
         }

         continue;
      }

      keycode = (keycode_t)keycodeInt;

      if( scancode & SCANCODE_UP_MASK )
      {
         if( MINOR( file->f_dentry->d_inode->i_rdev ) == 1 )
         {
            /* For compatibility with other Broadcom keypad drivers, the first
            *  keypad device (minor 0) should report both key up and key down
            *  events.  The second keypad device (minor 1) should report only
            *  key down events
            */
            continue;
         }

         keycode |= KEYCODE_UP_MASK;
      }
      else
      {
          gLastKeypadReadTick = timer_get_tick_count();
#ifdef CONFIG_BCM_KNLLOG_IRQ
   		  if (gKnllogIrqSchedEnable & KNLLOG_FB) KNLLOG("keypad_read\n");
#endif
      }

      rc = copy_to_user( &buffer[bufIndex], &keycode, sizeof( keycode ) );
      if( unlikely( rc ) )
      {
         myPrintk( KERN_WARNING "Unable to copy keycode 0x%02x to user "
                   "buffer (rc=%d)\n", keycode, rc );
         break;
      }

      bufIndex += sizeof( keycode );
   }

   if( (file->f_flags & O_NONBLOCK) && (bufIndex == 0) )
   {
       /* File was opened in non-blocking mode and no data was copied */
       return( -EAGAIN );
   }
   return( bufIndex );
}

/****************************************************************************
*
* keypad_poll - used to support the select system call
*
***************************************************************************/
static unsigned int keypad_poll( struct file *file, struct poll_table_struct *poll_table )
{
   CBLK *cblkp = &cblk;
   int client = (int)file->private_data;

   poll_wait( file, &cblkp->waitQueue, poll_table );
   
   if( __kfifo_len( cblkp->fifop[client] ) )
   {
      /* Indicate that data is currently available */
      return( POLLIN | POLLRDNORM );
   }

   return( 0 );
}

/****************************************************************************
*
* keypad_ioctl
*
***************************************************************************/
static int keypad_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
   int rc = 0;
   CBLK *cblkp = &cblk;
   
   FUNC_ENTRY( "inode=0x%08x, file=0x%08x, cmd=0x%08x, arg=0x%08x\n", (int)inode, (int)file, cmd, (int)arg );


   switch( cmd )
   {
      case KDSETKEYCODE:
      {
         struct kbkeycode __user *userp;
         struct kbkeycode local;

         userp = (struct kbkeycode __user *)arg;

         rc = copy_from_user( &local, userp, sizeof( local ) );
         if( rc )
         {
            return( -EFAULT );
         }

         rc = keycodeSet( cblkp, local.scancode, local.keycode );
      }
      break;

      case KDGETKEYCODE:
      {
         struct kbkeycode __user *userp;
         struct kbkeycode local;

         userp = (struct kbkeycode __user *)arg;

         rc = copy_from_user( &local, userp, sizeof( local ) );
         if( rc )
         {
            return( -EFAULT );
         }

         rc = keycodeGet( cblkp, local.scancode, &local.keycode );
         if( !rc )
         {
            rc = put_user( local.keycode, &userp->keycode );
         }
      }
      break;

      default:
      {
         return( -EINVAL );
      }
      break;
   } 

   return( rc );
} 

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations keypad_fops =
{
   owner:      THIS_MODULE,
   open:       keypad_open,
   release:    keypad_release,
   read:       keypad_read,
   poll:       keypad_poll,
   ioctl:      keypad_ioctl 
};

/****************************************************************************
*
*  keypad_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/
static int __init keypad_init( void )
{
   KEYMAP *keymapp = DEFAULT_KEYMAP;
   int keys = DEFAULT_KEYMAP_SIZE;
   dev_t dev;
   int rc;

   printk( banner );

   /* Register our device with Linux */
#ifdef BCM_KEYPAD_MAJOR
   cblk.major = BCM_KEYPAD_MAJOR;
   dev = MKDEV( BCM_KEYPAD_MAJOR, DEV_FIRST );
   rc = register_chrdev_region( dev, DEV_NUM, DEV_NAME );
#else
   rc = alloc_chrdev_region( &dev, DEV_FIRST, DEV_NUM, DEV_NAME );
   cblk.major = MAJOR( dev );
#endif

   if( rc < 0 )
   {
      myPrintk( KERN_WARNING "Unable to init major %d\n", cblk.major );
      return( rc );
   }

   cdev_init( &cblk.cdev, &keypad_fops );
   cblk.cdev.owner = THIS_MODULE;
   cblk.cdev.ops = &keypad_fops;

   rc = cdev_add( &cblk.cdev, dev, DEV_NUM );
   if( rc )
   {
      myPrintk( KERN_WARNING "Unable to add char device\n" );
      return( rc );
   }

   /* Force keypad matrix dimensions to undefined state until keycode mapping
   *  is set
   */
   cblk.matrix.first.row = 9999;
   cblk.matrix.first.col = 9999;
   cblk.matrix.last.row = 0;
   cblk.matrix.last.col = 0;

   /* Clear keycode map */
#if KEYCODE_NONE != -1
#error Default keycode map assumes all keycodes are -1
#endif
   memset( keycodeMap, 0xff, sizeof( keycodeMap ) ); 

   /* Setup the row and column pins as keypad pins */
   REG_KEYPAD_KPEMR0 = 0;
   REG_KEYPAD_KPEMR1 = 0;
   REG_KEYPAD_KPEMR2 = 0;
   REG_KEYPAD_KPEMR3 = 0;
   REG_KEYPAD_KPIMR0 = 0;
   REG_KEYPAD_KPIMR1 = 0;
   regaccess_and_bits( &REG_SYS_IOCR1, 0xFFFF0000 );     /*IOCR1[15:0]={KEY_C[7:0], KEY_R[7:0]} 8x8 keypad */

   /* Set default keycode map (if required) */
   if( keymapp )
   {
      int i;

      for( i=0; i<keys; i++ )
      {
         keycodeSet( &cblk, keymapp->scancode, keymapp->keycode );
         keymapp++;
      }
   }

   /* set rows as outputs, columns as inputs */
   REG_KEYPAD_KPDR = 0xff000000;    
   REG_KEYPAD_KPCR = REG_KEYPAD_KPCR_ENABLE + REG_KEYPAD_KPCR_PULL_UP +
                     REG_KEYPAD_COLFILTER_EN +
                     REG_KEYPAD_STATFILTER_EN +
                     REG_KEYPAD_COLFILTER_32MS +
                     REG_KEYPAD_STATFILTER_32MS +
                     ((ROW_NUM-1) << REG_KEYPAD_ROWWIDTH_SHIFT) +
                     ((COL_NUM-1) << REG_KEYPAD_COLWIDTH_SHIFT);

    return( rc );
}

/****************************************************************************
*
*  keypad_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
static void __exit keypad_exit( void )
{
   FUNC_ENTRY( "\n" );
}

/****************************************************************************/

module_init(keypad_init);
module_exit(keypad_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM116x Keypad Driver");
MODULE_VERSION( "1.0.0" );
MODULE_LICENSE("GPL v2");

