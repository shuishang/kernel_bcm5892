/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
#include <mach/irqs.h>
#include <csp/usbHostHw.h>
#include <mach/csp/cap.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/broadcom/bcmring_usb.h>

#define BCM_USBOHCI_MODULE_DESCRIPTION      "Broadcom BCMRING USB OHCI driver"
#define BCM_USBOHCI_MODULE_VERSION          "1.0.0"

/*
 * Disable USB_IRQ while processing the interrupt (IRQF_DISABLED) 
 * USB IRQ is shared between EHCI and OHCI in (IRQF_SHARED) 
 */
#define BCM_USBOHCI_IRQF_FLAGS              (IRQF_DISABLED | IRQF_SHARED)

/*
 * Definitions for the number of OHCI controllers supported. Usually there's
 * just 1, but some Broadcom chips have 2. Note that numbering is
 * 0 .. BCM_USBOHCI_HCD_MAX
 */
#define BCM_USBOHCI_HCD_MAX                 9

/*
 * Name definitions for use with driver and devices. Device names will have
 * a digit appended. Note the correlation to the BCM_USBOHCI_HCD_MAX value
 * (max value is a single digit, hence the sizeof() + 1).
 */
#define BCM_USBOHCI_NAME                    "bcm-ohci"
#define BCM_USBOHCI_DEVICE_NAME_LEN         (sizeof(BCM_USBOHCI_NAME) + 1)

#ifndef BCM_USBOHCI_KMARKER
#define BCM_USBOHCI_KMARKER
#endif


#define BCM_KERROR(fmt, ...)        printk( KERN_ERR BCM_USBOHCI_KMARKER "ERROR: %s(): " fmt, __func__, __VA_ARGS__ )
#define BCM_KINFO(fmt...)           printk( KERN_INFO BCM_USBOHCI_KMARKER fmt )
#define BCM_KPANIC(fmt, ...)        panic( BCM_USBOHCI_KMARKER "%s(): " fmt, __func__, __VA_ARGS__ )
#define BCM_KWARN(fmt...)           printk( KERN_WARNING BCM_USBOHCI_KMARKER fmt )

#ifdef DEBUG
    /* NOTE: Use of this macro requires arguments, i.e. cannot have a format w/o parameters */
    #define BCM_KTRACE(fmt, ...)    printk( KERN_INFO BCM_USBOHCI_KMARKER "%s(): " fmt, __func__, __VA_ARGS__ )
#else
    #define BCM_KTRACE(fmt, ...)
#endif

typedef struct BCM_USBOHCI_HC_CFG
{
   unsigned long ohciRegBaseAddr;
   unsigned int irqNum;
}
BCM_USBOHCI_HC_CFG;

extern int usb_disabled(void);
static int  bcm_UsbOhci_DriverProbe(struct platform_device *devP);
static int  bcm_UsbOhci_DriverRemove(struct platform_device *devP);
static int __devinit bcm_ohci_start (struct usb_hcd *hcdP);
/* @todo Make these IRQ names the same as the RTL. */
static const unsigned bcmIrq[] = { IRQ_USBH1, IRQ_USBHD2 };
static unsigned int bcmUsbHostCnt = 1;
module_param( bcmUsbHostCnt, uint, 0644 );

/*
 * USB HC (Host Controller) driver definition. Note that the only BCM
 * specific parts of this are the "product_desc" and the "start" routine,
 * and that all the "start" does is call Linux ohci_hcd functions.
 */
static const struct hc_driver UsbOhci_HcDriver =
{
   .description = hcd_name,
   .product_desc = BCM_USBOHCI_NAME,
   .hcd_priv_size = sizeof(struct ohci_hcd),
   
   /*
    * generic hardware linkage
    */
   .irq = ohci_irq,
   .flags = HCD_USB11 | HCD_MEMORY,  /* HCD_MEMORY indicates registers are memory mapped */

   /*
    * basic lifecycle operations
    */
   .start = bcm_ohci_start,
   .stop = ohci_stop,
   .shutdown = ohci_shutdown,
#ifdef CONFIG_PM
   .bus_suspend = ohci_bus_suspend,
   .bus_resume = ohci_bus_resume,
#endif

   /*
    * managing i/o requests and associated device resources
    */
   .urb_enqueue = ohci_urb_enqueue,
   .urb_dequeue = ohci_urb_dequeue,
   .endpoint_disable = ohci_endpoint_disable,

   /*
    * scheduling support
    */
   .get_frame_number = ohci_get_frame,

   /*
    * root hub support
    */
   .hub_status_data = ohci_hub_status_data,
   .hub_control = ohci_hub_control,
};

static struct platform_device *gPdev[BCM_USBOHCI_HCD_MAX];

static int UsbOhci_DeviceInit(unsigned hcdNum, unsigned long ohciRegBaseAddr,
      unsigned int irqNum )
{
   struct platform_device *platformDevP;
   BCM_USBOHCI_HC_CFG hcCfg;
   char name[ BCM_USBOHCI_DEVICE_NAME_LEN ];
   int err;

   BCM_KINFO( BCM_USBOHCI_NAME ": init device %u\n", hcdNum );

   if ( hcdNum > BCM_USBOHCI_HCD_MAX )
   {
      return( -E2BIG );
   }

   /*
    * Platform devices need to have the same name as the platform driver, as
    * this is what is used to do the binding between the device and a driver.
    * In theory, the device name could/should be an extension of the driver
    * name, i.e. "<driverName><deviceInstance>", where deviceInstance is
    * 0, 1, etc. However, it appears that this concept is broken in the
    * platform_match() routine contained in drivers/base/platform.c.
    */
   sprintf( name, "%s%u", BCM_USBOHCI_NAME, hcdNum );
   platformDevP = platform_device_alloc( BCM_USBOHCI_NAME, hcdNum );
   if ( IS_ERR(platformDevP) )
   {
      err = PTR_ERR(platformDevP);
      BCM_KERROR( "platform_device_alloc() failed, err=%d\n", err );

      return( err  );
   }
   gPdev[hcdNum] = platformDevP;
   
   /*
    * We need to set the dma_mask to something non-NULL, otherwise the top
    * layer HCD will not set the DMA (physical) address fields in the URBs.
    * The OHCI layer below the HCD uses these DMA addresses when it creates
    * its TDs.
    *
    * We probably should really get the mask from elsewhere (e.g. the platform
    * bus), but this info is not known to be available. As a work-around, we'll
    * use the coherent_dma_mask. See the Linux Documentation/DMA-mappings.txt
    * and Documentation/usb/dma.txt for more details.
    */
   platformDevP->dev.dma_mask = &platformDevP->dev.coherent_dma_mask;
   platformDevP->dev.coherent_dma_mask = DMA_32BIT_MASK;

   hcCfg.ohciRegBaseAddr = ohciRegBaseAddr;
   hcCfg.irqNum = irqNum;

   /*
    * Add the info needed for creating a USB OHCI HCD device. This will get
    * used when the DriverProbe() gets invoked for the device.
    */
   err = platform_device_add_data( platformDevP, &hcCfg, sizeof(hcCfg) );
   if ( err )
   {
      BCM_KERROR( "platform_device_add_data() failed, err=%d\n", err );
      platform_device_put( platformDevP );
      return( err );
   }
   
   err = platform_device_add( platformDevP );
   if ( err )
   {
      BCM_KERROR( "driver_register() failed, err=%d\n", err );
      platform_device_put( platformDevP );
   }
   
   return( err );
}

static int UsbOhci_DeviceTerm(unsigned hcdNum)
{
   struct platform_device *pdev = gPdev[hcdNum];

   platform_device_unregister(pdev);
   gPdev[hcdNum] = NULL;

   return 0;
}

/**
 * bcm_UsbOhci_DriverProbe - initialize BCM based OHCI HCD
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
int bcm_UsbOhci_DriverProbe(struct platform_device *platformDevP)
{
   BCM_USBOHCI_HC_CFG *hcCfg;
   struct usb_hcd *hcdP;
   int err;

   BCM_KTRACE( "%s-%u\n", platformDevP->name, platformDevP->id );

   if (usb_disabled())
   {
      return( -ENODEV );
   }

   hcdP = usb_create_hcd( &UsbOhci_HcDriver, &platformDevP->dev, (char *)platformDevP->name );
   if ( !hcdP )
   {
      BCM_KERROR( "%s: usb_create_hcd() failed\n", platformDevP->name );
      return( -ENOMEM );
   }

   hcCfg = (BCM_USBOHCI_HC_CFG *)platformDevP->dev.platform_data;
   if ( !hcCfg )
   {
      BCM_KERROR( "%s: missing platform_data\n", platformDevP->name );
      /* Release the resources referenced by hcdP */
      usb_put_hcd( hcdP );
      return( -ENODATA );
   }

   /* struct ohci_regs def'd in Linux ohci.h which is included by Linux ohci-hcd.c */
   hcdP->rsrc_start = hcCfg->ohciRegBaseAddr;
   hcdP->rsrc_len = sizeof( struct ohci_regs );
   hcdP->regs = (void *)hcCfg->ohciRegBaseAddr;

   ohci_hcd_init( hcd_to_ohci( hcdP ) );

   err = usb_add_hcd( hcdP, hcCfg->irqNum, BCM_USBOHCI_IRQF_FLAGS );
   if ( err )
   {
      BCM_KERROR( "%s: usb_add_hcd() failed\n", platformDevP->name );
      /* Release the resources referenced by hcdP */
      usb_put_hcd( hcdP );
   }

   return( err );
}

/**
 * bcm_UsbOhci_DriverRemove - shutdown processing for BCM-based OHCI HCD
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of bcm_UsbOhci_DriverProbe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
int bcm_UsbOhci_DriverRemove(struct platform_device *platformDevP)
{
   struct usb_hcd *hcdP;

   BCM_KTRACE( "%s\n", platformDevP->name );

   hcdP = dev_get_drvdata( &platformDevP->dev );

   usb_remove_hcd( hcdP );
   /* Release the resources referenced by hcdP */
   usb_put_hcd( hcdP );

   return( 0 );
}

int __devinit bcm_ohci_start (struct usb_hcd *hcdP)
{
   struct ohci_hcd *ohciP;
   int err;
   
   if ( !hcdP )
   {
      BCM_KERROR( "invalid hcdP=%p\n", hcdP );
      return( -EFAULT );
   }

   BCM_KTRACE( "busnum %d: begin: hcdP=%p\n", hcdP->self.busnum, hcdP );

   ohciP = hcd_to_ohci ( hcdP );
   BCM_KTRACE( "busnum %d: ohciP=%p\n", hcdP->self.busnum, ohciP );

   if ((err = ohci_init( ohciP )) < 0)
   {
      BCM_KERROR( "busnum %d: ohci_init() failed, err=%d\n", hcdP->self.busnum, err );
      return( err );
   }

   BCM_KTRACE( "busnum %d: ohci_init() ok\n", hcdP->self.busnum );
   if ((err = ohci_run ( ohciP )) < 0)
   {
      BCM_KERROR( "busnum %d: ohci_run() failed, err=%d\n", hcdP->self.busnum, err );
      ohci_stop ( hcdP );
      return( err );
   }

   BCM_KTRACE( "busnum %d: ohci_run() ok\n", hcdP->self.busnum );

   return( 0 );
}

#ifdef CONFIG_PM
static int bcm_UsbOhci_DriverSuspend(struct platform_device *dev,
      pm_message_t message)
{
   struct usb_hcd *hcd = platform_get_drvdata(dev);
   struct ohci_hcd *ohci = hcd_to_ohci (hcd);
   unsigned long flags;
   int i, rc = 0;

   /* Root hub was already suspended. Disable irq emission and
    * mark HW unaccessible, bail out if RH has been resumed. Use
    * the spinlock to properly synchronize with possible pending
    * RH suspend or resume activity.
    *
    * This is still racy as hcd->state is manipulated outside of
    * any locks =P But that will be a different fix.
    */
   spin_lock_irqsave(&ohci->lock, flags);
   if (hcd->state != HC_STATE_SUSPENDED) {
      rc = -EINVAL;
      goto bail;
   }
   ohci_writel(ohci, OHCI_INTR_MIE, &ohci->regs->intrdisable);
   (void)ohci_readl(ohci, &ohci->regs->intrdisable);

   /* make sure snapshot being resumed re-enumerates everything */
   if (message.event == PM_EVENT_PRETHAW)
      ohci_usb_reset(ohci);
   
   clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

   for (i = 0; i < bcmUsbHostCnt; i++) {
      rc = bcmring_usb_suspend(i);
      if (rc)
         goto bail;
   }

bail:
   
   spin_unlock_irqrestore(&ohci->lock, flags);
   return rc;
}

static int bcm_UsbOhci_DriverResume(struct platform_device *dev)
{
   int i, rc;
   struct usb_hcd *hcd = platform_get_drvdata(dev);
   
   for (i = 0; i < bcmUsbHostCnt; i++) {
      rc = bcmring_usb_resume(i);
      if (rc)
         return rc;
   }
   
   set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
   ohci_finish_controller_resume(hcd);
   return 0;
}
#else
#define bcm_UsbOhci_DriverSuspend    NULL
#define bcm_UsbOhci_DriverResume     NULL
#endif

/*
 * Generic platform device driver definition.
 */
static struct platform_driver UsbOhci_PlatformDriver =
{
   .probe = bcm_UsbOhci_DriverProbe,
   .remove = bcm_UsbOhci_DriverRemove,
   .shutdown = usb_hcd_platform_shutdown,
   .suspend = bcm_UsbOhci_DriverSuspend,
   .resume = bcm_UsbOhci_DriverResume,
   .driver =
   {
      .name = BCM_USBOHCI_NAME,
      .owner = THIS_MODULE,
   },
};

static int __init bcmring_UsbOhci_ModuleInit (void)
{
   unsigned ohciDevNum;
   unsigned long ohciRegBaseAddr;
   int err;

   if (cap_isPresent(CAP_USB, 0) != CAP_PRESENT) {
      printk(KERN_ERR "USB is not supported\n");
      return -ENODEV;
   }
   else if ((cap_isPresent(CAP_USB, 1) != CAP_PRESENT) && (bcmUsbHostCnt == 2)) {
      /*
       * When this module is initialized with bcmUsbHostCnt set to 2, first
       * check if the second usb host is available, if not, print out warning
       * and reset bcmUsbHostCnt to 1
       */
      printk(KERN_WARNING "USB1 is not support. Fall back to one host only\n");
      bcmUsbHostCnt=1;
   }

   if ( (err = platform_driver_register( &UsbOhci_PlatformDriver )) != 0 )
   {
      return( err );
   }

   for ( ohciDevNum = 0; ohciDevNum < bcmUsbHostCnt; ohciDevNum++ )
   {
      err = bcmring_usb_init(ohciDevNum, 0);
      if (err != 0) {
         platform_driver_unregister( &UsbOhci_PlatformDriver );
         return -EFAULT;
      }
   }

   for ( ohciDevNum = 0; ohciDevNum < bcmUsbHostCnt; ohciDevNum++ )
   {
      ohciRegBaseAddr = ( unsigned long )usbHostHw_OhciRegBaseAddr( ohciDevNum );
      if ( (err = UsbOhci_DeviceInit(ohciDevNum, ohciRegBaseAddr, bcmIrq[ohciDevNum])) != 0 )
         break;
   }
   
   return( err );
}

static void __exit bcmring_UsbOhci_ModuleExit (void)
{
   unsigned ohciDevNum;

   for (ohciDevNum = 0; ohciDevNum < bcmUsbHostCnt; ohciDevNum++)
      UsbOhci_DeviceTerm(ohciDevNum);

   for (ohciDevNum = 0; ohciDevNum < bcmUsbHostCnt; ohciDevNum++)
      bcmring_usb_term(ohciDevNum);
}

module_init (bcmring_UsbOhci_ModuleInit);
module_exit (bcmring_UsbOhci_ModuleExit);

MODULE_DESCRIPTION( BCM_USBOHCI_MODULE_DESCRIPTION );
MODULE_LICENSE( "GPL" );
MODULE_VERSION( BCM_USBOHCI_MODULE_VERSION );
