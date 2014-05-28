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

#define BCM_USBEHCI_MODULE_DESCRIPTION    "Broadcom BCMRING USB EHCI driver"
#define BCM_USBEHCI_MODULE_VERSION        "1.0.0"

/*
 * Disable USB_IRQ while processing the interrupt (IRQF_DISABLED) 
 * USB IRQ is shared between EHCI and OHCI (IRQF_SHARED) 
 */
#define BCM_USBEHCI_IRQF_FLAGS            (IRQF_DISABLED | IRQF_SHARED)

/*
 * Definitions for the number of EHCI controllers supported. Usually there's
 * just 1, but some Broadcom chips have 2. Note that numbering is
 * 0 .. BCM_USBEHCI_HCD_MAX
 */
#define BCM_USBEHCI_HCD_MAX               9

/*
 * Name definitions for use with driver and devices. Device names will have
 * a digit appended. Note the correlation to the BCM_USBEHCI_HCD_MAX value
 * (max value is a single digit, hence the sizeof() + 1).
 */
#define BCM_USBEHCI_NAME                  "bcm-ehci"
#define BCM_USBEHCI_DEVICE_NAME_LEN       (sizeof(BCM_USBEHCI_NAME) + 1)

#ifndef BCM_USBEHCI_KMARKER
#define BCM_USBEHCI_KMARKER
#endif


#define BCM_KERROR(fmt, ...)        printk( KERN_ERR BCM_USBEHCI_KMARKER "ERROR: %s(): " fmt, __func__, __VA_ARGS__ )
#define BCM_KINFO(fmt...)           printk( KERN_INFO BCM_USBEHCI_KMARKER fmt )
#define BCM_KPANIC(fmt, ...)        panic( BCM_USBEHCI_KMARKER "%s(): " fmt, __func__, __VA_ARGS__ )
#define BCM_KWARN(fmt...)           printk( KERN_WARNING BCM_USBEHCI_KMARKER fmt )

#ifdef DEBUG
    /* NOTE: Use of this macro requires arguments, i.e. cannot have a format w/o parameters */
    #define BCM_KTRACE(fmt, ...)    printk( KERN_INFO BCM_USBEHCI_KMARKER "%s(): " fmt, __func__, __VA_ARGS__ )
#else
    #define BCM_KTRACE(fmt, ...)
#endif

extern int usb_disabled(void);
static int  bcm_UsbEhci_DriverProbe(struct platform_device *devP);
static int  bcm_UsbEhci_DriverRemove(struct platform_device *devP);
static int __devinit bcm_ehci_hc_init (struct usb_hcd *hcdP);

/* @todo Make these IRQ names the same as the RTL. */
static const unsigned bcmIrq[] = { IRQ_USBH1, IRQ_USBHD2 };
static unsigned int bcmUsbHostCnt = 1;
module_param( bcmUsbHostCnt, uint, 0644 );

typedef struct BCM_USBEHCI_HC_CFG
{
   unsigned long ehciRegBaseAddr;
   unsigned int irqNum;
}
BCM_USBEHCI_HC_CFG;

/*
 * USB HC (Host Controller) driver definition. Note that the only BCM
 * specific parts of this are the "product_desc" and the "start" routine,
 * and that all the "start" does is call Linux ehci_hcd functions.
 */
static const struct hc_driver UsbEhci_HcDriver =
{
   .description = hcd_name,
   .product_desc = BCM_USBEHCI_NAME,
   .hcd_priv_size = sizeof(struct ehci_hcd),
   
   /*
    * generic hardware linkage
    */
   .irq = ehci_irq,
   .flags = HCD_USB2 | HCD_MEMORY,  /* HCD_MEMORY indicates registers are memory mapped */

   /*
    * basic lifecycle operations
    */
   .reset = bcm_ehci_hc_init,
   .start = ehci_run,
   .stop = ehci_stop,
   .shutdown = ehci_shutdown,

   /*
    * managing i/o requests and associated device resources
    */
   .urb_enqueue = ehci_urb_enqueue,
   .urb_dequeue = ehci_urb_dequeue,
   .endpoint_disable = ehci_endpoint_disable,

   /*
    * scheduling support
    */
   .get_frame_number = ehci_get_frame,

   /*
    * root hub support
    */
   .hub_status_data =  ehci_hub_status_data,
   .hub_control =      ehci_hub_control,
#ifdef CONFIG_PM
   .bus_suspend =      ehci_bus_suspend,
   .bus_resume =       ehci_bus_resume,
#endif
   .relinquish_port =  ehci_relinquish_port,
   .port_handed_over = ehci_port_handed_over,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30)
   .clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
#endif
};

static struct platform_device *gPdev[BCM_USBEHCI_HCD_MAX];

static int UsbEhci_DeviceInit(unsigned hcdNum, unsigned long ehciRegBaseAddr,
      unsigned int irqNum )
{
   struct platform_device *platformDevP;
   BCM_USBEHCI_HC_CFG hcCfg;
   char name[BCM_USBEHCI_DEVICE_NAME_LEN];
   int err;

   BCM_KINFO( BCM_USBEHCI_NAME ": init device %u\n", hcdNum );

   if ( hcdNum > BCM_USBEHCI_HCD_MAX )
      return( -E2BIG );

   /*
    * Platform devices need to have the same name as the platform driver, as
    * this is what is used to do the binding between the device and a driver.
    * In theory, the device name could/should be an extension of the driver
    * name, i.e. "<driverName><deviceInstance>", where driverInstance is
    * 0, 1, etc. However, it appears that this concept is broken in the
    * platform_match() routine contained in drivers/base/platform.c.
    */
   sprintf( name, "%s%u", BCM_USBEHCI_NAME, hcdNum );
   platformDevP = platform_device_alloc(BCM_USBEHCI_NAME, hcdNum);
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
    * The EHCI layer below the HCD uses these DMA addresses when it creates
    * its TDs.
    *
    * We probably should really get the mask from elsewhere (e.g. the platform
    * bus), but this info is not known to be available. As a work-around, we'll
    * use the coherent_dma_mask. See the Linux Documentation/DMA-mappings.txt
    * and Documentation/usb/dma.txt for more details.
    */
   platformDevP->dev.dma_mask = &platformDevP->dev.coherent_dma_mask;
   platformDevP->dev.coherent_dma_mask = DMA_32BIT_MASK;

   hcCfg.ehciRegBaseAddr = ehciRegBaseAddr;
   hcCfg.irqNum = irqNum;

   /*
    * Add the info needed for creating a USB EHCI HCD device. This will get
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

static int UsbEhci_DeviceTerm(unsigned hcdNum)
{
   struct platform_device *pdev = gPdev[hcdNum];

   platform_device_unregister(pdev);
   gPdev[hcdNum] = NULL;

   return 0;
}

/*-------------------------------------------------------------------------*/

/**
 * bcm_UsbEhci_DriverProbe - initialize BCM based EHCI HCD
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
int bcm_UsbEhci_DriverProbe(struct platform_device *platformDevP)
{
   BCM_USBEHCI_HC_CFG *hcCfgP;
   struct ehci_hcd *ehciP;
   struct usb_hcd *hcdP;
   int err;

   BCM_KTRACE( "%s-%u\n", platformDevP->name, platformDevP->id );

   if (usb_disabled())
   {
      return( -ENODEV );
   }

   hcdP = usb_create_hcd( &UsbEhci_HcDriver, &platformDevP->dev, (char *)platformDevP->name );
   if ( !hcdP )
   {
      BCM_KERROR( "%s: usb_create_hcd() failed\n", platformDevP->name );
      return( -ENOMEM );
   }

   hcCfgP = (BCM_USBEHCI_HC_CFG *)platformDevP->dev.platform_data;
   if ( !hcCfgP )
   {
      BCM_KERROR( "%s: missing platform_data\n", platformDevP->name );
      /* Release the resources referenced by hcdP */
      usb_put_hcd( hcdP );
      return( -ENODATA );
   }

   /* struct ehci_regs def'd in Linux ehci.h which is included by Linux ehci-hcd.c */
   hcdP->rsrc_start = hcCfgP->ehciRegBaseAddr;
   hcdP->rsrc_len = sizeof( struct ehci_regs );
   hcdP->regs = (void *)hcCfgP->ehciRegBaseAddr;

   ehciP = hcd_to_ehci( hcdP );
   ehciP->caps = hcdP->regs;
   ehciP->regs = hcdP->regs + HC_LENGTH(ehci_readl(ehciP, &ehciP->caps->hc_capbase));
   /* cache this readonly data; minimize chip reads */
   ehciP->hcs_params = ehci_readl(ehciP, &ehciP->caps->hcs_params);

   err = usb_add_hcd( hcdP, hcCfgP->irqNum, BCM_USBEHCI_IRQF_FLAGS );
   if ( err )
   {
      BCM_KERROR( "%s: usb_add_hcd() failed\n", platformDevP->name );
      /* Release the resources referenced by hcdP */
      usb_put_hcd( hcdP );
   }

   return( err );
}

/**
 * bcm_UsbEhci_DriverRemove - shutdown processing for BCM-based EHCI HCD
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of bcm_UsbEhci_DriverProbe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
int bcm_UsbEhci_DriverRemove(struct platform_device *platformDevP)
{
   struct usb_hcd *hcdP;

   BCM_KTRACE( "%s\n", platformDevP->name );

   hcdP = dev_get_drvdata( &platformDevP->dev );

   usb_remove_hcd( hcdP );
   /* Release the resources referenced by hcdP */
   usb_put_hcd( hcdP );

   return( 0 );
}

int __devinit bcm_ehci_hc_init (struct usb_hcd *hcdP)
{
   struct ehci_hcd *ehciP;
   int err;

   if ( !hcdP )
   {
      BCM_KERROR( "invalid hcdP=%p\n", hcdP );
      return( -EFAULT );
   }

   BCM_KTRACE( "busnum %d: begin: hcdP=%p\n", hcdP->self.busnum, hcdP );

   ehciP = hcd_to_ehci ( hcdP );
   BCM_KTRACE( "busnum %d: ehciP=%p\n", hcdP->self.busnum, ehciP );

   if ((err = ehci_halt( ehciP )) < 0)
   {
      BCM_KERROR( "busnum %d: ehci_halt() failed, err=%d\n", hcdP->self.busnum, err );
      return( err );
   }
   
   if ((err = ehci_init( hcdP )) < 0)
   {
      BCM_KERROR( "busnum %d: ehci_init() failed, err=%d\n", hcdP->self.busnum, err );
      return( err );
   }
   /*
    * Not sure why this is not set by ehci_init(). Convention seems to be to do it here for 
    * reasons unknown. This is a "packed release number". 
    */
   ehciP->sbrn = 0x20;

   BCM_KTRACE( "busnum %d: ehci_init() ok\n", hcdP->self.busnum );

   if ((err = ehci_reset( ehciP )) < 0)
   {
      BCM_KERROR( "busnum %d: ehci_reset() failed, err=%d\n", hcdP->self.busnum, err );
      return( err );
   }

   BCM_KTRACE( "busnum %d: ehci_reset() ok\n", hcdP->self.busnum );

   return( 0 );
}

#ifdef CONFIG_PM
static int bcm_UsbEhci_DriverSuspend(struct platform_device *pdev,
                                        pm_message_t message)
{
   struct usb_hcd *hcd = platform_get_drvdata(pdev);
   struct ehci_hcd *ehci = hcd_to_ehci(hcd);
   unsigned long flags;
   int i, rc = 0;

   if (time_before(jiffies, ehci->next_statechange))
      msleep(10);

   /* Root hub was already suspended. Disable irq emission and
    * mark HW unaccessible, bail out if RH has been resumed. Use
    * the spinlock to properly synchronize with possible pending
    * RH suspend or resume activity.
    *
    * This is still racy as hcd->state is manipulated outside of
    * any locks =P But that will be a different fix.
    */
   spin_lock_irqsave (&ehci->lock, flags);
   if (hcd->state != HC_STATE_SUSPENDED) {
      rc = -EINVAL;
      goto bail;
   }
   ehci_writel(ehci, 0, &ehci->regs->intr_enable);
   (void)ehci_readl(ehci, &ehci->regs->intr_enable);
   
   /* make sure snapshot being resumed re-enumerates everything */
   if (message.event == PM_EVENT_PRETHAW) {
      ehci_halt(ehci);
      ehci_reset(ehci);
   }

   clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

   for (i = 0; i < bcmUsbHostCnt; i++) {
      rc = bcmring_usb_suspend(i);
      if (rc)
         goto bail;
   }

bail:
   spin_unlock_irqrestore (&ehci->lock, flags);
   return rc;
}

static int bcm_UsbEhci_DriverResume(struct platform_device *pdev)
{
   struct usb_hcd *hcd = platform_get_drvdata(pdev);
   struct ehci_hcd *ehci = hcd_to_ehci(hcd);
   int i, rc;
   
   if (time_before(jiffies, ehci->next_statechange))
      msleep(100);

   for (i = 0; i < bcmUsbHostCnt; i++) {
      rc = bcmring_usb_resume(i);
      if (rc)
         return rc;
   }
      
   
   /* Mark hardware accessible again as we are out of D3 state by now */
   set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
   
   ehci_dbg(ehci, "lost power, restarting\n");
   usb_root_hub_lost_power(hcd->self.root_hub);
   
   /*
    * Else reset, to cope with power loss or flush-to-storage
    * style "resume" having let BIOS kick in during reboot.
    */
   (void) ehci_halt(ehci);
   (void) ehci_reset(ehci);
   
   /* emptying the schedule aborts any urbs */
   spin_lock_irq(&ehci->lock);
   if (ehci->reclaim)
      end_unlink_async(ehci);
   ehci_work(ehci);
   spin_unlock_irq(&ehci->lock);

   ehci_writel(ehci, ehci->command, &ehci->regs->command);
   ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
   ehci_readl(ehci, &ehci->regs->command); /* unblock posted writes */

   /* here we "know" root ports should always stay powered */
   ehci_port_power(ehci, 1);

   hcd->state = HC_STATE_SUSPENDED;
   return 0;
}
#else
#define bcm_UsbEhci_DriverSuspend    NULL
#define bcm_UsbEhci_DriverResume     NULL
#endif

/*
 * Generic platform device driver definition.
 */
static struct platform_driver UsbEchi_PlatformDriver =
{
   .probe = bcm_UsbEhci_DriverProbe,
   .remove = bcm_UsbEhci_DriverRemove,
   .shutdown = usb_hcd_platform_shutdown,
   .suspend = bcm_UsbEhci_DriverSuspend,
   .resume = bcm_UsbEhci_DriverResume,
   .driver =
   {
      .name = BCM_USBEHCI_NAME,
      .owner = THIS_MODULE,
   },
};

static int __init bcmring_UsbEhci_ModuleInit(void)
{
   unsigned ehciDevNum;
   unsigned long ehciRegBaseAddr;
   int rval;

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

   /* register the platform driver */
   rval = platform_driver_register(&UsbEchi_PlatformDriver);
   if (rval < 0)
      return rval;

   /* enable clock, PHYs */
   for (ehciDevNum = 0; ehciDevNum < bcmUsbHostCnt; ehciDevNum++) {
      rval = bcmring_usb_init(ehciDevNum, 0);
      if (rval < 0) {
         platform_driver_unregister(&UsbEchi_PlatformDriver );
         return rval;
      }
   }

   for (ehciDevNum = 0; ehciDevNum < bcmUsbHostCnt; ehciDevNum++) {
      ehciRegBaseAddr = (unsigned long)usbHostHw_EhciRegBaseAddr(ehciDevNum);
      if ((rval = UsbEhci_DeviceInit(ehciDevNum, ehciRegBaseAddr,
                  bcmIrq[ehciDevNum])) != 0)
         break;
   }
   return rval;
}

static void __exit bcmring_UsbEhci_ModuleExit(void)
{
   unsigned ehciDevNum;

   for (ehciDevNum = 0; ehciDevNum < bcmUsbHostCnt; ehciDevNum++)
      UsbEhci_DeviceTerm(ehciDevNum);

   platform_driver_unregister( &UsbEchi_PlatformDriver );

   for (ehciDevNum = 0; ehciDevNum < bcmUsbHostCnt; ehciDevNum++)
      bcmring_usb_term(ehciDevNum);
}

MODULE_DESCRIPTION(BCM_USBEHCI_MODULE_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_VERSION(BCM_USBEHCI_MODULE_VERSION);

module_init(bcmring_UsbEhci_ModuleInit);
module_exit(bcmring_UsbEhci_ModuleExit);
