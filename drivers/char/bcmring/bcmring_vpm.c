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
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/semaphore.h>
#include <linux/suspend.h>
#include <linux/freezer.h>
#include <asm/uaccess.h>
#include <mach/csp/mm_io.h>
#include <mach/csp/cap.h>
#include <csp/vpmHw.h>
#include <linux/broadcom/bcmring_vpm.h>
#include <linux/broadcom/bcm_major.h>
#include <linux/sysctl.h>
#include <linux/workqueue.h>
#include <linux/device.h>

#define VPM_DEBUG_TRACE 0

#if VPM_DEBUG_TRACE
#include <linux/broadcom/knllog.h>
#define VPM_TRACE KNLLOG
#else
#define VPM_TRACE(...)
#endif

/* ---- Public Variables -------------------------------------------------*/
/* ---- Private Constants and Types --------------------------------------*/
/* ---- Private Variables ------------------------------------------------*/

DECLARE_MUTEX(vpmLock); /* master lock to protect the VPM resource */

static int gVpmRun = 0;
static int gVpmDelayHalt = 0;

/* Flag to disallow halt on VPM */
static int gVpmNoHalt = 0;

static struct workqueue_struct *gVpmWorkqueue = NULL;

/*  SysCtl data structures */
static struct ctl_table gSysCtlChild[] =
{
   {
      .procname      = "vpm_no_halt",
      .data          = &gVpmNoHalt,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};
static struct ctl_table gSysCtl[] =
{
   {
      .procname      = "vpm",
      .mode          = 0555,
      .child         = gSysCtlChild
   },
   {}
};
static struct ctl_table_header  *gSysCtlHeader;

#define VPM_DELAY_HALT_SECONDS  3

static struct vpmLccFuncs *gLccFuncs;

/* ---- Private Function ------------------------------------------------- */
static void vpmDelayHalt( struct work_struct *unused );
static DECLARE_DELAYED_WORK( gWork, vpmDelayHalt);

/****************************************************************************
*  down_freezable
*
*  Description:
*     Helper function to allow a down() to be freezable
*
***************************************************************************/
#ifdef CONFIG_PM_SLEEP
void down_freezable(struct semaphore *sem)
{
   int rval, signaled = 0;
   unsigned long flags;
   struct task_struct *task = current;

   while (1) {
      rval = down_interruptible(sem);
      if (rval == 0) { /* down now */
         break;
      } else {
         if (freezing(current)) {
            try_to_freeze();
         } else {
            spin_lock_irqsave(&task->sighand->siglock, flags);
            if (test_tsk_thread_flag(task, TIF_SIGPENDING)) {
               clear_tsk_thread_flag(task, TIF_SIGPENDING);
               signaled = 1;
            }
            spin_unlock_irqrestore(&task->sighand->siglock, flags);
         }
      }
   }

   if (signaled) {
      spin_lock_irqsave(&task->sighand->siglock, flags);
      set_tsk_thread_flag(task, TIF_SIGPENDING);
      spin_unlock_irqrestore(&task->sighand->siglock, flags);
   }
}
#else
#define down_freezable down
#endif

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*  vpmLoadAndRun
*
*  Description:
*     Load binary file generated using VPM Binary Archive Utility
*
***************************************************************************/
int vpmLoadAndRun( char *filename )
{
   int rc;

   /* Initialize VPM clocks, de-assert RESET and assert WAIT to halt the
    * TL3 from executing.
    */
   vpmRequest();
   rc = vpmHw_InitandHalt();
   if( rc == -1 )
   {
      printk( "VPM Phase Alignment Failed\n" );
      return rc;
   }

   rc = vpmLoadBinaryFile( filename );
   if( rc )
   {
      printk( "VPM Binary Load Failed rc=%i\n", rc );
      return rc;
   }

   /* Let the VPM run for short time to complete initialization */
   rc = vpmHw_Run();
   if( rc == -1 )
   {
      printk( "VPM Phase Alignment Failed\n" );
      return rc;
   }

   set_current_state(TASK_UNINTERRUPTIBLE);
   /* Note: 20ms works for BCM11107, but BCM11109 and BCM11170 running DSP at 200MHz takes longer, so using 50ms */
   schedule_timeout(msecs_to_jiffies(50));

   vpmHaltAndRelease();

   return(0);
}
EXPORT_SYMBOL( vpmLoadAndRun );


/****************************************************************************
*  vpmLoadBinaryFile
*
*  Description:
*     Load binary file generated using VPM Binary Archive Utility
*
***************************************************************************/
int vpmLoadBinaryFile( char *fileName )
{
   int rc = 0;
   unsigned int start_address;
   unsigned int size;
   short *pBuf, *pBuf_unaligned;

   struct file *filp;

   mm_segment_t old_fs = get_fs();
   set_fs(KERNEL_DS);
   filp = filp_open( fileName, O_RDONLY, 04444 );
   if( IS_ERR(filp) )
   {
      printk( "ERROR: Cannot open VPM image\n" );
      set_fs(old_fs);      
      return -ENOENT;
   }

   /* Read memory sections */
   while( filp->f_op->read( filp, (char*)&start_address, 4, &filp->f_pos ) > 0 )
   {
      int bytesRead;

      /* Read the size of the memory section */
      bytesRead = filp->f_op->read( filp, (char*)&size, 4, &filp->f_pos );
      if( bytesRead <= 0 )
      {
         printk( "VPM Load cannot read size!\n" );
         rc = -EIO;
         goto vpmloader_exit;
      }

      /* 
       * Check that the VPM will not overwrite the kernel code or data.
       */
      if ((start_address != MM_ADDR_IO_VPM_PROG) && (start_address != MM_ADDR_IO_VPM_DATA)) 
      {
         if ((start_address + size) > PHYS_OFFSET) 
         {
            printk("Error: VPM Load at 0x%x with size 0x%x must fit below 0x%x\n", 
                   start_address, size, PHYS_OFFSET);
            rc = -EFAULT; /* Bad address */
            goto vpmloader_exit;
         }
      }

      printk( "VPM Load address=0x%08x size=%u\n", start_address, size );

      /* Allocate buffer to store memory section */
      pBuf_unaligned = kmalloc( size+8, GFP_KERNEL );
      if( pBuf_unaligned <= 0 )
      {
         printk( "VPM Load buffer allocation error!.\n" );
         rc = -ENOMEM;
         goto vpmloader_exit;
      }

      /* Make sure buffer is aligned */
      pBuf = (short*)(((int)pBuf_unaligned + 7) & ~7 );

      /* Read data section */
      bytesRead = filp->f_op->read( filp, (char*)pBuf, size, &filp->f_pos );
      if( bytesRead <= 0 )
      {
         printk( "VPM Load cannot read buffer!\n" );
         rc = -EIO;
         goto vpmloader_exit;
      }

      /* Load VPM memory */
      memcpy( (void*)MM_IO_PHYS_TO_VIRT(start_address), pBuf, size );

      kfree( pBuf_unaligned );
   }

vpmloader_exit:
   filp_close( filp, NULL );
   set_fs(old_fs);

   return rc;
}
EXPORT_SYMBOL(vpmLoadBinaryFile);


/****************************************************************************
*  vpmShutdown
*
*  Description:
*     Shutdown VPM
*
***************************************************************************/
void vpmShutdown( void )
{
   vpmRequest();
   vpmHw_Halt();
   gVpmDelayHalt = 0;
   gVpmRun = 0;
   /* Cancel the delayed halt if active */
   if ( cancel_delayed_work(&gWork) == 0 )
   {
      vpmRelease();
      flush_workqueue( gVpmWorkqueue );
      vpmRequest();
   }
   vpmHw_Shutdown();
   vpmRelease();
}
EXPORT_SYMBOL(vpmShutdown);


/****************************************************************************
*  vpmRequest
*
*  Description:
*     Request to use the VPM resource
*
***************************************************************************/
void vpmRequest( void )
{
   VPM_TRACE("%s", __FUNCTION__);
   down_freezable(&vpmLock);
}

/****************************************************************************
*  vpmRelease
*
*  Description:
*     Release the VPM resource
*
***************************************************************************/
void vpmRelease( void )
{
   VPM_TRACE("%s", __FUNCTION__);
   up(&vpmLock);
}

/****************************************************************************
*  vpmRequestAndRun
*
*  Description:
*     Request the VPM resource and start the VPM running
*
***************************************************************************/
void vpmRequestAndRun( void )
{
   VPM_TRACE("%s", __FUNCTION__);
   vpmRequest();

   /* Cancel the delayed halt if active */
   if ( gVpmDelayHalt )
   {
      gVpmDelayHalt = 0;
      if ( cancel_delayed_work(&gWork) == 0 )
      {
         vpmRelease();
         flush_workqueue( gVpmWorkqueue );
         vpmRequest();
      }
   }
   vpmRun();
}

/****************************************************************************
*  vpmRun
*
*  Description:
*     Start the VPM running, should only be called with VPM already requested
*
***************************************************************************/
void vpmRun( void )
{
   VPM_TRACE("%s", __FUNCTION__);
   if ( !gVpmRun )
   {
      VPM_TRACE("%s run", __FUNCTION__);
      vpmHw_Run();
      gVpmRun = 1;
   }
}

/****************************************************************************
*  vpmHalt
*
*  Description:
*     Stop the VPM from running, should only be called with VPM already requested
*
***************************************************************************/
void vpmHalt( void )
{
   VPM_TRACE("%s", __FUNCTION__);
   if (!gVpmNoHalt && gVpmRun)
   {
      /* Schedule delayed halt */
      gVpmDelayHalt = 1;
      queue_delayed_work( gVpmWorkqueue, &gWork, (VPM_DELAY_HALT_SECONDS*HZ));
   }
}

/****************************************************************************
*  vpmHaltAndRelease
*
*  Description:
*     Stop the VPM from running and release the VPM resource
*
***************************************************************************/
void vpmHaltAndRelease( void )
{
   VPM_TRACE("%s", __FUNCTION__);
   vpmHalt();
   vpmRelease();
}

EXPORT_SYMBOL(vpmRequest);
EXPORT_SYMBOL(vpmRelease);
EXPORT_SYMBOL(vpmRun);
EXPORT_SYMBOL(vpmHalt);
EXPORT_SYMBOL(vpmRequestAndRun);
EXPORT_SYMBOL(vpmHaltAndRelease);

/****************************************************************************
*  vpmLccOpenChannel
*
*  Description:
*     Open LCC channel
*
***************************************************************************/
VPM_LCC_CHANNEL_HANDLE vpmLccOpenChannel( uint16_t moduleId)
{
   if (gLccFuncs && gLccFuncs->openChannel)
      return gLccFuncs->openChannel(moduleId);

   return -EINVAL;
}
EXPORT_SYMBOL(vpmLccOpenChannel);

/****************************************************************************
*  vpmLccCloseChannel
*
*  Description:
*     Close LCC channel
*
***************************************************************************/
int vpmLccCloseChannel( VPM_LCC_CHANNEL_HANDLE handle)
{
   if (gLccFuncs && gLccFuncs->closeChannel)
      return gLccFuncs->closeChannel(handle);

   return -EINVAL;
}
EXPORT_SYMBOL(vpmLccCloseChannel);

/****************************************************************************
*  vpmLccSubmitJob
*
*  Description:
*     Submit LCC job
*
***************************************************************************/
int vpmLccSubmitJob(
   VPM_LCC_CHANNEL_HANDLE   handle,
   int                      priority,
   VPM_LCC_JOB             *lccJobp,
   uint32_t                 headerSizeInBytes,
   uint16_t                *buffer,
   uint32_t                 bufferSizeInBytes
)
{
   if (gLccFuncs && gLccFuncs->submitJob)
      return gLccFuncs->submitJob(handle, priority, lccJobp,
            headerSizeInBytes, buffer, bufferSizeInBytes);

   return -EINVAL;
}
EXPORT_SYMBOL(vpmLccSubmitJob);

/****************************************************************************
*  vpmLccRetrieveJob
*
*  Description:
*     Retrieve LCC job
*
***************************************************************************/
int vpmLccRetrieveJob(
   VPM_LCC_CHANNEL_HANDLE   handle,
   VPM_LCC_JOB            **lccJobpp,
   uint32_t                *numBytesp
)
{
   if (gLccFuncs && gLccFuncs->retrieveJob)
      return gLccFuncs->retrieveJob(handle, lccJobpp, numBytesp);

   return -EINVAL;
}
EXPORT_SYMBOL(vpmLccRetrieveJob);

/****************************************************************************
*  vpmLccFreeJob
*
*  Description:
*     Free LCC job
*
***************************************************************************/
void vpmLccFreeJob( VPM_LCC_CHANNEL_HANDLE handle)
{
   if (gLccFuncs && gLccFuncs->freeJob)
      return gLccFuncs->freeJob(handle);
}
EXPORT_SYMBOL(vpmLccFreeJob);

/****************************************************************************
*  vpmLccReleaseAllJob
*
*  Description:
*     Release all LCC job on channel
*
***************************************************************************/
void vpmLccReleaseAllJob( VPM_LCC_CHANNEL_HANDLE handle)
{
   if (gLccFuncs && gLccFuncs->releaseAllJob)
      return gLccFuncs->releaseAllJob(handle);
}
EXPORT_SYMBOL(vpmLccReleaseAllJob);

/****************************************************************************
*  vpm_lcc_ioctl
*
*  Description:
*     IOCTL handler for VPM LCC functions
*
***************************************************************************/
#define LCC_HEADER_SIZE 100
#define LCC_BUFFER_SIZE 32768
static uint32_t header[LCC_HEADER_SIZE >> 2];
static uint32_t buffer[LCC_BUFFER_SIZE >> 2];

static int vpm_lcc_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
      unsigned long arg)
{
   VPM_LCC_CHANNEL_HANDLE lccHandle;
   int rc = 0;

   /* Guess lccHandle */
   lccHandle = (VPM_LCC_CHANNEL_HANDLE)arg;

   VPM_TRACE("0x%08x (%d): %x", (uint32_t)filp, (int)(filp->private_data), cmd);
   switch (cmd) {
      case VPM_LCC_IOCTL_OPEN_CHANNEL:
      {
         uint16_t moduleId = (uint16_t)arg;

         lccHandle = vpmLccOpenChannel(moduleId);
         /* Save LCC handle in case the device is closed before the channel is properly closed */
         if (lccHandle >= 0)
         {
            filp->private_data = (void *)lccHandle;
         }
         rc = (int)lccHandle;
      }
      break;

      case VPM_LCC_IOCTL_CLOSE_CHANNEL:
      {
         rc = vpmLccCloseChannel(lccHandle);
         filp->private_data = (void *)(-1);
      }
      break;

      case VPM_LCC_IOCTL_SUBMIT_JOB:
      {
         VPM_LCC_SUBMIT_JOB_PARAM param;

         copy_from_user(&param, (void *)arg, sizeof(param));
         if (param.headerSizeInBytes > sizeof(header))
         {
            rc = -EINVAL;
            break;
         }
         if (param.bufferSizeInBytes > sizeof(buffer))
         {
            rc = -EINVAL;
            break;
         }
         copy_from_user(header, param.lccJobp, param.headerSizeInBytes);
         copy_from_user(buffer, param.buffer, param.bufferSizeInBytes);

         rc = vpmLccSubmitJob(param.handle,
                              param.priority,
                              (VPM_LCC_JOB *)header,
                              param.headerSizeInBytes,
                              (uint16_t *)buffer,
                              param.bufferSizeInBytes);
      }
      break;

      case VPM_LCC_IOCTL_RETRIEVE_JOB:
      {
         VPM_LCC_RETRIEVE_JOB_PARAM param;
         VPM_LCC_JOB *lccJobp;
         uint32_t numBytes, userNumBytes;

         copy_from_user(&param, (void *)arg, sizeof(param));
         if (!param.numBytesp)
         {
            rc = -EINVAL;
            break;
         }
         copy_from_user(&userNumBytes, param.numBytesp, sizeof(userNumBytes));

         rc = vpmLccRetrieveJob(param.handle, &lccJobp, &numBytes);
         if (rc < 0)
            break;

         if (numBytes > userNumBytes)
         {
            rc = -ENOMEM;
            break;
         }

         copy_to_user(param.lccJobp, lccJobp, numBytes);
         copy_to_user(param.numBytesp, &numBytes, sizeof(numBytes));
      }
      break;

      case VPM_LCC_IOCTL_FREE_JOB:
      {
         vpmLccFreeJob(lccHandle);
      }
      break;

      case VPM_LCC_IOCTL_RELEASE_ALL_JOB:
      {
         vpmLccReleaseAllJob(lccHandle);
      }
      break;

      default:
         return -EINVAL;
   }
   VPM_TRACE("0x%08x (%d): %x rc = %d", (uint32_t)filp, (int)(filp->private_data), cmd, rc);
   return rc;
}

static int vpm_lcc_open(struct inode *inode, struct file *filp)
{
   VPM_TRACE("0x%08x", (uint32_t)filp);
   filp->private_data = (void *)(-1);
   return 0;
}

static int vpm_lcc_release(struct inode *inode, struct file *filp)
{
   int rc;

   VPM_TRACE("0x%08x (%d)", (uint32_t)filp, (int)(filp->private_data));
   if ((VPM_LCC_CHANNEL_HANDLE)(filp->private_data) >= 0)
   {
      rc = vpmLccCloseChannel((VPM_LCC_CHANNEL_HANDLE)(filp->private_data));
      if ( rc )
      {
         printk( KERN_ERR "Failed to close LCC channel %d: %d\n", (int)(filp->private_data), rc);
      }
      filp->private_data = (void *)(-1);
   }
   return 0;
}

#if CONFIG_SYSFS
static struct class * vpm_lcc_class;
static struct device * vpm_lcc_dev;
#endif

static struct file_operations vpm_lcc_fops =
{
   owner: THIS_MODULE,
   ioctl: vpm_lcc_ioctl,
   open: vpm_lcc_open,
   release: vpm_lcc_release,
};

/****************************************************************************
*  vpmRegisterLccClient
*
*  Description:
*     Register LCC functions for user mode access
*
***************************************************************************/
int vpmRegisterLccClient( struct vpmLccFuncs *funcsp )
{
   int rc = 0;

   if (gLccFuncs != NULL)
      return -EBUSY;

   gLccFuncs = funcsp;

   rc = register_chrdev(BCM_LCC_MAJOR, "lcc", &vpm_lcc_fops);
   if (rc < 0)
   {
      printk(KERN_ERR "VPM: register_chrdev failed for major %u, rc = %d\n", BCM_LCC_MAJOR, rc);
      goto err_exit;
   }

#if CONFIG_SYSFS
   vpm_lcc_class = class_create(THIS_MODULE,"bcmring-vpm-lcc");
   if(IS_ERR(vpm_lcc_class)){
	   printk(KERN_ERR "VPM: Class create failed\n");
	   rc = -EFAULT;
	   goto err_unregister_chrdev;
   }

   vpm_lcc_dev = device_create(vpm_lcc_class, NULL, MKDEV(BCM_LCC_MAJOR,0),NULL,"lcc");
   if(IS_ERR(vpm_lcc_dev)){
	   printk(KERN_ERR "VPM: Device create failed\n");
	   rc = -EFAULT;
	   goto err_class_destroy;
   }
#endif
   return 0;

#if CONFIG_SYSFS
err_class_destroy:
   class_destroy(vpm_lcc_class);
err_unregister_chrdev:
   unregister_chrdev(BCM_LCC_MAJOR, "lcc");
#endif
err_exit:
   gLccFuncs = NULL;
   return rc;
}
EXPORT_SYMBOL(vpmRegisterLccClient);

/****************************************************************************
*  vpmUnregisterLccClient
*
*  Description:
*     Unregister LCC functions for user mode access
*
***************************************************************************/
int vpmUnregisterLccClient( struct vpmLccFuncs *funcsp )
{
   if (gLccFuncs != funcsp)
      return -EINVAL;

#if CONFIG_SYSFS
   device_destroy(vpm_lcc_class,MKDEV(BCM_LCC_MAJOR,0));
   class_destroy(vpm_lcc_class);
#endif
   unregister_chrdev(BCM_LCC_MAJOR, "lcc");

   gLccFuncs = NULL;
   return 0;
}
EXPORT_SYMBOL(vpmUnregisterLccClient);

/****************************************************************************
*  vpm_pm_callback
*
*  Description:
*     Power Management event callback
*
***************************************************************************/
static int vpm_pm_callback(struct notifier_block *nfb, unsigned long action, void *ignored)
{
   switch (action) {
      case PM_HIBERNATION_PREPARE:
      case PM_SUSPEND_PREPARE:
         /* Request for VPM resource to prevent others from using the VPM
          * while suspended.
          */
         vpmRequest();

         /* Halt VPM */
         VPM_TRACE("%s Halt", __FUNCTION__);
         vpmHw_Halt();
         gVpmDelayHalt = 0;
         gVpmRun = 0;

         return NOTIFY_OK;

      case PM_POST_HIBERNATION:
      case PM_POST_SUSPEND:
         /* Release the VPM resource on resume.
          */
         VPM_TRACE("%s Release", __FUNCTION__);
         vpmRelease();
         return NOTIFY_OK;
      }

   return NOTIFY_DONE;
}

static struct notifier_block vpm_pm_notifier;


/****************************************************************************
*  vpm_init
*
*  Description:
*     Module initialization function
*
***************************************************************************/
static int __init vpm_init(void)
{
   int rc = 0;

   /* Check if VPM is present
    */
   if (cap_isPresent(CAP_VPM,0) == CAP_NOT_PRESENT ) {
      printk (KERN_ERR "VPM is not supported\n");
      return -EFAULT;
   }

   /* Register for power management event notifications
    */
   vpm_pm_notifier.notifier_call = &vpm_pm_callback;
   vpm_pm_notifier.priority = 99;
   rc = register_pm_notifier(&vpm_pm_notifier);
   if (rc < 0) {
      printk(KERN_ERR "VPM: register_pm_notifier failed, rc = %d\n", rc);
      return rc;
   }

   gSysCtlHeader = register_sysctl_table( gSysCtl );

   gVpmWorkqueue = create_workqueue( "vpm_power_queue" );

   return 0;
}

/****************************************************************************
*  vpm_exit
*
*  Description:
*     Module exit function
*
***************************************************************************/
static void __exit vpm_exit(void)
{
   /* Unregister power management event notifications
    */
   unregister_pm_notifier(&vpm_pm_notifier);

   if ( gSysCtlHeader )
   {
      unregister_sysctl_table( gSysCtlHeader );
   }

   if ( gVpmWorkqueue )
   {
      destroy_workqueue( gVpmWorkqueue );
   }
}

/****************************************************************************
*  vpmDelayHalt
*
*  Description:
*     Performs halt operation.  Intended to be used in the work queue.
*
***************************************************************************/
static void vpmDelayHalt( struct work_struct *unused )
{
   VPM_TRACE("%s", __FUNCTION__);
   vpmRequest();
   if ( gVpmDelayHalt && gVpmRun )
   {
      VPM_TRACE("%s Halt", __FUNCTION__);
      vpmHw_Halt();
   }
   gVpmDelayHalt = 0;
   gVpmRun = 0;
   vpmRelease();
}

module_init(vpm_init);
module_exit(vpm_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Binary loader for the DSP image and power management for the VPM module");
MODULE_LICENSE("GPL v2");

