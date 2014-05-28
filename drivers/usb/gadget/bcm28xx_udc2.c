/*                                                            
 * linux/drivers/usb/gadget/bcm28xx_udc2.c
 * Broadcom bcm28xx on-chip high/full speed USB device controllers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#undef DEBUG
#undef VERBOSE

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#include <linux/config.h>
#else
#include <linux/platform_device.h>
#endif
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#include <linux/usb_ch9.h>
#else
#include <linux/usb/ch9.h>
#endif

#include <linux/usb_gadget.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/mach-types.h>
#include <mach/platform.h>
#include <mach/hardware.h>
#include <asm/arch/cm.h>
#include <mach/irqs.h>
#include <asm/arch/posted_write.h>
#include "bcm28xx_udc2.h"


/* 
 * All the RDE/NAK related fix are guarded by this.
 */
#define NAK_RDE_FIX

#undef BCM28XX_UDC2_NAK_DBG 

/*-------------------------------------------------------------------------
 *
 *								Macros
 *
 *-------------------------------------------------------------------------*/

#define	DRIVER_DESC	"BCM28XX UDC2 driver"
#define	DRIVER_VERSION	"2.0"

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)


/*-------------------------------------------------------------------------
 *
 *								Module parameters
 *
 *-------------------------------------------------------------------------*/

static uint dma_burst_len = 16;

module_param_named(dma_burst_size, dma_burst_len, uint, S_IRUGO);
MODULE_PARM_DESC(dma_burst_size, "DMA transfer burst size");


/*-------------------------------------------------------------------------
 *
 *							Global Variables	    
 *
 *-------------------------------------------------------------------------*/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
static const char driver_name [] = "bcm28xx_udc";
#else
static const char driver_name [] = "bcm28xx_udc2";
#endif
static const char driver_desc [] = DRIVER_DESC;

static struct bcm28xx_udc2 *gUdc2;

/* 
 * When the IN/OUT endpoint zero can not clear its nak bit because of the
 * RxFIFO is not empty, it will set this flag to be 1.
 * 
 * And after the bulk out endpoint clears the NAK for it in the next_out(),
 * it will reset this flag back to 0.
 */
static int ep0_in_nak_req_flag = 0;
static int ep0_out_nak_req_flag = 0;


static int start_evnts_cnt_after_status = 0;
static int events_cnt = 0; 
static int n_req_67 = 0;


#define BCM28XX_NUM_MAX_CLEAR_NAK_FIRST_TRY 10
#define BCM28XX_NUM_MAX_CLEAR_NAK_SECOND_TRY 10000

/*------------------------------------------------------------------------
 *
 *  UDC debugging
 *
 *------------------------------------------------------------------------*/

/*#define  UDC_LOGGING */

typedef enum
{
    
    USB_UDC_ISR_ENTER_1             = 0xc1,
    USB_UDC_ISR_ENTER_2             = 0xc2,
    USB_UDC_SETUP_0x64              = 0xc3,
    USB_UDC_SETUP_AFTER_UPPER       = 0xc4,
    USB_UDC_NEXT_OUT                = 0xc5,
    USB_UDC_NEXT_IN                 = 0xc6,
    USB_UDC_FINISH_OUT              = 0xc7,
    USB_UDC_BRKP_SPUR_PKT           = 0xc8,
    USB_UDC_NEXT_OUT_SPUR_PKT       = 0xc9,
    USB_UDC_CRASH_STATE             = 0xca,
    USB_UDC_EP_QUEUE                = 0xcb,
    USB_UDC_FINISH_IN               = 0xcc,


}  UDC_LOGGER_FLAG;


#undef UDC_LOGGING

#ifdef UDC_LOGGING

extern void  __init
udc_logger_init( void );

extern void
udc_logger_do_log(UDC_LOGGER_FLAG log_flag, u32 arg1, u32 arg2, u32 arg3, u32 arg4);

#else  /* if undefined(SDHCI_LOGGING) */
 
#define udc_logger_init( ) 

#define udc_logger_do_log(_log_flag_, _arg1_, _arg2_, _arg3_, _arg4_)

#endif  /* UDC_LOGGING */


#ifdef UDC_LOGGING

#define SDHCI_LOG_TBL_SIZE  (4096 + 2048)

typedef struct _sdhci_logger
{
    u32   curr_idx;
    u64   magic_code;
    u64   timestamp;
    u32   tbl_size;
    u32   timeout_retries;
    struct 
    {
        u32    flag;
        u64    timestamp;
        u32    arg1;
        u32    arg2;
        u32    arg3;
        u32    arg4;
    } tbl[SDHCI_LOG_TBL_SIZE];
} sdhci_logger_t;

static sdhci_logger_t sdhci_logger_struct;

static  sdhci_logger_t * sdhci_logger_p = &sdhci_logger_struct;

extern void __init
udc_logger_init( void )
{
   static unsigned int count = 0;

   if (count == 0)
       count++;
   else
   {
       printk(KERN_ERR "Can not init the logger twice!");
       return;
   }
   
    /*sdhci_logger_p = kmalloc(sizeof(sdhci_logger_t), GFP_KERNEL); */
    
    if (!sdhci_logger_p) 
    {
        printk(KERN_ERR "In %s at %d: kmalloc failed\n", __FILE__, __LINE__);
        return;
    }
    else
    {
        printk(KERN_ERR "The logger starts at 08X%08X\n", (u32)sdhci_logger_p);
    }

    /*memset(sdhci_logger_p, 0, sizeof(sdhci_logger_t)); */
    ((sdhci_logger_t *)sdhci_logger_p)->curr_idx = 0;
    ((sdhci_logger_t *)sdhci_logger_p)->tbl_size = SDHCI_LOG_TBL_SIZE;
    ((sdhci_logger_t *)sdhci_logger_p)->magic_code = 0x12141972; 
    ((sdhci_logger_t *)sdhci_logger_p)->timestamp = 
               *(volatile u64 *)IO_ADDRESS(BCM28XX_RPLC_TIMERLWORD_PHY);  
    ((sdhci_logger_t *)sdhci_logger_p)->timeout_retries = 0;

}


#define sdhci_logger_log( sdhci_logger_p, _flag_, _arg1_, _arg2_, _arg3_, _arg4_)   \
{                                                                   \
    u32 curr_idx, size;                                             \
                                                                    \
    BUG_ON(sdhci_logger_p==NULL);                                   \
    curr_idx =  ((sdhci_logger_t *)sdhci_logger_p)->curr_idx;       \
    size     =  ((sdhci_logger_t *)sdhci_logger_p)->tbl_size;       \
                                                                    \
    BUG_ON(curr_idx >= size);                                       \
                                                                    \
    ((sdhci_logger_t *)sdhci_logger_p)->tbl[curr_idx].flag = _flag_;    \
    ((sdhci_logger_t *)sdhci_logger_p)->tbl[curr_idx].timestamp =       \
        *(volatile u64 *)IO_ADDRESS(BCM28XX_RPLC_TIMERLWORD_PHY);       \
    ((sdhci_logger_t *)sdhci_logger_p)->tbl[curr_idx].arg1 = _arg1_;    \
    ((sdhci_logger_t *)sdhci_logger_p)->tbl[curr_idx].arg2 = _arg2_;    \
    ((sdhci_logger_t *)sdhci_logger_p)->tbl[curr_idx].arg3 = _arg3_;    \
    ((sdhci_logger_t *)sdhci_logger_p)->tbl[curr_idx].arg4 = _arg4_;    \
                                                                        \
    if ((curr_idx+1) >= size)                                           \
        ((sdhci_logger_t *)sdhci_logger_p)->timeout_retries++;          \
    curr_idx = (curr_idx+1) % size;                                     \
    ((sdhci_logger_t *)sdhci_logger_p)->curr_idx = curr_idx;            \
                                                                        \
}


extern void
udc_logger_do_log(UDC_LOGGER_FLAG log_flag, u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
    sdhci_logger_log(sdhci_logger_p, log_flag, arg1, arg2, arg3, arg4);
}


#endif /* UDC_LOGGING */ 




/*-------------------------------------------------------------------------
 *
 *				BCM28XX UDC register access helper functions
 * 
 *-------------------------------------------------------------------------*/
static __inline__ u32 bcm28xx_udc2_readl(u32 reg)
{
	return (readl(IO_ADDRESS(reg)));
}

static __inline__ void bcm28xx_udc2_writel(u32 val, u32 reg)
{
	writel(val, IO_ADDRESS(reg));
}

static __inline__ void bcm28xx_udc2_modifyl(u32 reg, u32 clear_mask, u32 set_mask)
{
	writel((readl(IO_ADDRESS(reg)) & (~clear_mask)) | set_mask, IO_ADDRESS(reg));
}


/*-------------------------------------------------------------------------
 *
 *						BCM28XX UDC UTMI PHY helper functions
 * 
 *-------------------------------------------------------------------------*/
static void bcm28xx_udc2_phy_powerdown(void)
{
	bcm28xx_udc2_writel(USB_PHY_RESET_VAL, BCM28XX_USBH0_PHY_CTRL_PHY);
	bcm28xx_udc2_writel(USB_PHY_POWER_DOWN_VAL, BCM28XX_USBH0_PHY_CTRL_PHY);
	bcm28xx_udc2_writel(USB_PHY_DESELECT_VAL, BCM28XX_USBH0_UTMI_CTRL1_PHY);
}


/*-------------------------------------------------------------------------
 *
 *						BCM28XX UDC helper functions
 * 
 *-------------------------------------------------------------------------*/
static __inline__ void enable_IN_ep_interrupt(int ep_index)
{
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_EPINTMASK_PHY, (1 << ep_index), 0); 
}

static __inline__ void enable_OUT_ep_interrupt(int ep_index)
{
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_EPINTMASK_PHY, (1 << (ep_index+16)), 0); 
}

static __inline__ void disable_IN_ep_interrupt(int ep_index)
{
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_EPINTMASK_PHY, 0, (1 << ep_index)); 
}

static __inline__ void disable_OUT_ep_interrupt(int ep_index)
{
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_EPINTMASK_PHY, 0, (1 << (ep_index+16))); 
}


static void ep0_enable_interrupt(void)
{
	/* Enable Set Configuration and Set Interface interrupts 
	 */
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVINTMASK_PHY, UDC_DEV_INT_SC | UDC_DEV_INT_SI, 0);

	/* Enable EP0 IN and OUT channel interrupts. 
	 */
	enable_IN_ep_interrupt(0);
	enable_OUT_ep_interrupt(0);
}

static void ep0_enable(void)
{
	struct bcm28xx_udc2_setup_desc	*setup_desc;

	/* Set the SETUP descriptor's quadlet to Host Ready
	 */
	setup_desc = (struct bcm28xx_udc2_setup_desc *)gUdc2->SETUP.virt_addr;
	setup_desc->quadlet = UDC2_QUADLET_HOST_READY;
	wmb();
	bcm28xx_udc2_writel(gUdc2->SETUP.dma_addr, BCM28XX_USBD0_EP0O_SETUPPTR_PHY);	 

	/* Enable Rx DMA */
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);

	gUdc2->ep0_current_state = EP0_IDLE;
	gUdc2->ep0_next_state = EP0_SETUP;

	ep0_enable_interrupt();
}

static __inline__ void SNAK_IN_ep(int ep_index)
{
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0I_CTRL_PHY+(0x20 * ep_index), 0, UDC_EP_CTRL_SNAK); 
}

static __inline__ void SNAK_OUT_ep(int ep_index)
{
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * ep_index), 0, UDC_EP_CTRL_SNAK); 
}




static __inline__ int clear_ep0_in_nak( void )
{
    int cnt, ret_val;

    /* After the setup pkt is RXed, HW automactically sets the NAK on both IN endpoint 0 and OUT endpoint 0.
     *
     * OUT endpoint NAK will be and have to be cleared when you are ready to send something; otherwise, once 
     * the RxDMA engine is turned on, we will get BNA error interrupt. 
     *
     * IN endpoint NAK can be cleared in place since it is driven by the device to do the TX; and will not have
     * any negative effect.
     *
     * In order to clear the NAK we have to clear out the RxFIFO; and the way to that is by setting the NAK on 
     * the OUT endpoint 3 if it is not set already, and enabling the RxDMA, and clearing the IN endpoint 0 NAK, and finally clearing
     * the NAK on OUT endpoint 3.  
     */
     int old_ep3_out_nak_state = bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * 3)) & UDC_EP_CTRL_NAK;
     if (!old_ep3_out_nak_state)
         SNAK_OUT_ep(3);
     
     /* Enabling the RxDMA should not bring any harm, because if an OUT endpoint does not want to
      * receive traffic from the host, it should have the NAK bit set.
      */
     bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);
     
     ret_val = 0;
     for (cnt = 0; cnt < BCM28XX_NUM_MAX_CLEAR_NAK_SECOND_TRY; cnt++)
     {
         bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0I_CTRL_PHY, 0, UDC_EP_CTRL_CNAK); 
         if (!(bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY) & UDC_EP_CTRL_NAK))
         {
            ret_val = 1;
            break;
         }
     }
     
     if (!old_ep3_out_nak_state)
         bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * 3), 0, UDC_EP_CTRL_CNAK);
    
    return  ret_val;
 
}




static __inline__ void CNAK_IN_ep(int ep_index)
{
#if 1    
    if (ep_index == 0)
    {  
        int cnt = 0;
        for (cnt =  0; cnt < BCM28XX_NUM_MAX_CLEAR_NAK_FIRST_TRY; cnt++)
        {
             bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0I_CTRL_PHY+(0x20 * ep_index), 0, UDC_EP_CTRL_CNAK); 
             if (!(bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY+(0x20 * ep_index)) & UDC_EP_CTRL_NAK))
                 break;
        }
        
        if (bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY+(0x20 * ep_index)) & UDC_EP_CTRL_NAK)
        {

#if 0
            /* 
             * Enabling the RDE without any checking is not going to do any harm because 
             * in order for the DMA to happen, the RRDY bit has to be set in the endpoint's
             * ctrl register.
             */
            bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);
#endif
            if (clear_ep0_in_nak() == 0)
            {
                ep0_in_nak_req_flag = 1;
                printk(KERN_ERR "\n haha cant clear the INNAK\n");
            }
                
        }

    }
    else
#endif        
    {
	    bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0I_CTRL_PHY+(0x20 * ep_index), 0, UDC_EP_CTRL_CNAK); 
    }
}



static __inline__ int clear_ep0_out_nak( void )
{
    int cnt, ret_val;
    
    /* After the setup pkt is RXed, HW automactically sets the NAK on both IN endpoint 0 and OUT endpoint 0.
     *
     * OUT endpoint NAK will be and have to be cleared when you are ready to send something; otherwise, once 
     * the RxDMA engine is turned on, we will get BNA error interrupt. 
     *
     * IN endpoint NAK can be cleared in place since it is driven by the device to do the TX; and will not have
     * any negative effect.
     *
     * In order to clear the NAK we have to clear out the RxFIFO; and the way to that is by setting the NAK on 
     * the OUT endpoint 3 if it is not set already, and enabling the RxDMA, and clearing the IN endpoint 0 NAK, and finally clearing
     * the NAK on OUT endpoint 3.  
     */
     int old_ep3_out_nak_state = bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * 3)) & UDC_EP_CTRL_NAK;
     if (!old_ep3_out_nak_state)
         SNAK_OUT_ep(3);
     
     /* Enabling the RxDMA should not bring any harm, because if an OUT endpoint does not want to
      * receive traffic from the host, it should have the NAK bit set.
      */
     bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);
     
     ret_val = 0;
     for (cnt = 0; cnt < BCM28XX_NUM_MAX_CLEAR_NAK_SECOND_TRY; cnt++)
     {
         bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0O_CTRL_PHY, 0, UDC_EP_CTRL_CNAK); 
         if (!(bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY) & UDC_EP_CTRL_NAK))
         {
            ret_val = 1;
            break;
         }
     }
     
     if (!old_ep3_out_nak_state)
         bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * 3), 0, UDC_EP_CTRL_CNAK);
    
    return  ret_val;
 
}




static __inline__ void CNAK_OUT_ep(int ep_index)
{
#if 1    
    if (ep_index == 0)
    {
        int cnt = 0;
        for (cnt =  0; cnt < BCM28XX_NUM_MAX_CLEAR_NAK_FIRST_TRY; cnt++)
        {
             bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * ep_index), 0, UDC_EP_CTRL_CNAK); 
             if (!(bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * ep_index)) & UDC_EP_CTRL_NAK))
                 break;
        }

        if (bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * ep_index)) & UDC_EP_CTRL_NAK)
        {
#if 0            
            /* 
             * Enabling the RDE without any checking is not going to do any harm because 
             * in order for the DMA to happen, the RRDY bit has to be set in the endpoint's
             * ctrl register.
             */
            bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);
#endif
            if (clear_ep0_out_nak() == 0)
            {
                ep0_out_nak_req_flag = 1;
                printk(KERN_ERR "\n haha cant clear the OUTNAK\n");
            }
        }

    }
    else
#endif        
    {
        while (1)
        {
	        bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * ep_index), 0, UDC_EP_CTRL_CNAK); 
            if (!(bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * ep_index)) & UDC_EP_CTRL_NAK))
               break;
        }

    }

	/*bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * ep_index), 0, UDC_EP_CTRL_CNAK); */
}

static void bcm28xx_udc2_free_dma_bufs(void)
{
	if (!gUdc2)
		return;

	if (gUdc2->ep_dma_descriptor_buf.virt_addr)
		dma_free_coherent(gUdc2->gadget.dev.parent, 
						  gUdc2->ep_dma_descriptor_buf.size,
						  gUdc2->ep_dma_descriptor_buf.virt_addr, 
						  gUdc2->ep_dma_descriptor_buf.dma_addr);
	
	if (gUdc2->SETUP.virt_addr)
		dma_free_coherent(gUdc2->gadget.dev.parent, 
						  gUdc2->SETUP.size,
						  gUdc2->SETUP.virt_addr, 
						  gUdc2->SETUP.dma_addr);	

	if (gUdc2->SETUP_in_status.virt_addr)
		dma_free_coherent(gUdc2->gadget.dev.parent, 
						  gUdc2->SETUP_in_status.size,
						  gUdc2->SETUP_in_status.virt_addr, 
						  gUdc2->SETUP_in_status.dma_addr);	

	if (gUdc2->SETUP_out_status.virt_addr)
		dma_free_coherent(gUdc2->gadget.dev.parent, 
						  gUdc2->SETUP_out_status.size,
						  gUdc2->SETUP_out_status.virt_addr, 
						  gUdc2->SETUP_out_status.dma_addr);	
}

static int check_req_len(struct bcm28xx_udc2_ep * ep, unsigned len)
{
	unsigned max_len;
	
#ifdef UDC2_BUFFER_FILL_MODE 
	max_len = BCM28XX_UDC2_MAX_TRANSFER_SIZE;
#else
	switch (ep->bmAttributes) {
		case USB_ENDPOINT_XFER_CONTROL:
			max_len = 2 * 1024; 
			break;

		case USB_ENDPOINT_XFER_BULK:
			max_len = 64 * 1024;
			break;

		case USB_ENDPOINT_XFER_INT:
			max_len = 4 * 1024; 
			break;

		default:
			BUG();
			break;
	}	
#endif

	return (len > max_len);
}

/*-------------------------------------------------------------------------
 *
 *			Linux gadget ep operation interface functions.
 * 
 *-------------------------------------------------------------------------*/
static int bcm28xx_udc2_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct bcm28xx_udc2_ep	*ep = container_of(_ep, struct bcm28xx_udc2_ep, ep);
	struct bcm28xx_udc2	*udc2;
	unsigned long	flags;
	u16 	maxp;
	int ep_index;
	int is_in;
	

	/* catch various bogus parameters */
	if (!_ep || !desc || ep->desc
			|| desc->bDescriptorType != USB_DT_ENDPOINT
			|| ep->bEndpointAddress!= desc->bEndpointAddress
			|| ep->maxpacket < le16_to_cpu
						(desc->wMaxPacketSize)) {
		DBG("%s, bad ep or descriptor\n", __FUNCTION__);
		return -EINVAL;
	}
	maxp = le16_to_cpu (desc->wMaxPacketSize);
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
				&& maxp != ep->maxpacket)
			|| le16_to_cpu(desc->wMaxPacketSize) > ep->maxpacket
			|| !desc->wMaxPacketSize) {
		DBG("%s, bad %s maxpacket\n", __FUNCTION__, _ep->name);
		return -ERANGE;
	}

	/* xfer types must match */
	if (ep->bmAttributes != desc->bmAttributes) {
		DBG("%s, %s type mismatch\n", __FUNCTION__, _ep->name);
		return -EINVAL;
	}

	udc2 = ep->udc2;
	if (!udc2->driver || udc2->gadget.speed == USB_SPEED_UNKNOWN) {
		DBG("%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	spin_lock_irqsave(&udc2->lock, flags);

	ep->desc = desc;
	ep->stopped = 0;
	ep->ep.maxpacket = maxp;

	/* Enable endpoint interrupt */
	ep_index = ep->bEndpointAddress & 0xf;
	is_in = ep->bEndpointAddress & USB_DIR_IN;

	if(is_in)
		enable_IN_ep_interrupt(ep_index);
	else
		enable_OUT_ep_interrupt(ep_index);

	spin_unlock_irqrestore(&udc2->lock, flags);
	VDBG("%s enabled\n", _ep->name);
	
	return 0;
}

static int bcm28xx_udc2_ep_disable(struct usb_ep *_ep)
{
	struct bcm28xx_udc2_ep	*ep = container_of(_ep, struct bcm28xx_udc2_ep, ep);
	unsigned long	flags;
	int ep_index;
	int is_in;

	if (!_ep || !ep->desc) {
		DBG("%s, %s not enabled\n", __FUNCTION__,
			_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}
	
	spin_lock_irqsave(&ep->udc2->lock, flags);

	ep->desc = NULL;
	ep->ep.maxpacket = ep->maxpacket;
				
	/* Disable endpoint interrupt */
	ep_index = ep->bEndpointAddress & 0xf;
	is_in = ep->bEndpointAddress & USB_DIR_IN;		

	if(is_in) {
		disable_IN_ep_interrupt(ep_index);
		ep->in_state = IN_INIT;
	}
	else
		disable_OUT_ep_interrupt(ep_index);

	spin_unlock_irqrestore(&ep->udc2->lock, flags);

	VDBG("%s disabled\n", _ep->name);
	return 0;
}


static struct usb_request *
bcm28xx_udc2_alloc_request(struct usb_ep *ep, unsigned gfp_flags)
{
	struct bcm28xx_udc2_req	*req;

	req = kmalloc(sizeof *req, gfp_flags);
	if (req) {
		memset (req, 0, sizeof *req);
		req->req.dma = DMA_ADDR_INVALID;
		INIT_LIST_HEAD (&req->queue);
	}
	return &req->req;
}


static void
bcm28xx_udc2_free_request(struct usb_ep *ep, struct usb_request *_req)
{
	struct bcm28xx_udc2_req	*req = container_of(_req, struct bcm28xx_udc2_req, req);

	if (_req)
		kfree (req);
}


static void *
bcm28xx_udc2_alloc_buffer(
	struct usb_ep	*_ep,
	unsigned	bytes,
	dma_addr_t	*dma,
	unsigned	gfp_flags
)
{
	struct bcm28xx_udc2_ep	*ep;

	ep = container_of(_ep, struct bcm28xx_udc2_ep, ep);	
    return (dma_alloc_coherent(ep->udc2->gadget.dev.parent,
            bytes, dma, gfp_flags));
}

static void 
bcm28xx_udc2_free_buffer(
	struct usb_ep	*_ep,
	void		*buf,
	dma_addr_t	dma,
	unsigned	bytes
)
{
	struct bcm28xx_udc2_ep	*ep;

	ep = container_of(_ep, struct bcm28xx_udc2_ep, ep);
	dma_free_coherent(ep->udc2->gadget.dev.parent, bytes, buf, dma);
}

static void
done(struct bcm28xx_udc2_ep *ep, struct bcm28xx_udc2_req *req, int status)
{
	unsigned		stopped = ep->stopped;

	list_del_init(&req->queue);

	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

#ifndef	USB_TRACE
	if (status && status != -ESHUTDOWN)
#endif
		VDBG("complete %s req %p stat %d len %u/%u\n",
			ep->ep.name, &req->req, status,
			req->req.actual, req->req.length);

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;
	spin_unlock(&ep->udc2->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&ep->udc2->lock);
	ep->stopped = stopped;
}

#ifdef UDC2_BUFFER_FILL_MODE

static void next_in(struct bcm28xx_udc2_ep *ep, struct bcm28xx_udc2_req *req)
{
	unsigned length = req->req.length - req->req.actual;
	int ep_index;
	u32 reg;
	struct bcm28xx_udc2_data_desc *data_desc;		

	ep_index = ep->bEndpointAddress & 0xf;

	VDBG("%s: ep_index = %d, length = %d\n", __FUNCTION__, ep_index, length);		
				
	data_desc = ep->dma_descriptor[0].virt_addr;
	data_desc->quadlet = UDC2_QUADLET_HOST_READY | UDC2_QUADLET_LAST | (length & 0xffff);
	data_desc->buf_ptr = req->req.dma;

	req->dma_bytes = length;

	ep->in_state = IN_POLLDEMAND;
	
	/* Renable interrupt, start DMA and clear CNK */
	enable_IN_ep_interrupt(ep_index);

	wmb();
	reg = BCM28XX_USBD0_EP0I_CTRL_PHY + (0x20 * ep_index);	
	bcm28xx_udc2_modifyl(reg, 0, UDC_EP_CTRL_POLL | UDC_EP_CTRL_CNAK);
}

static void finish_in(struct bcm28xx_udc2_ep *ep, struct bcm28xx_udc2_req *req, int status)
{
	if (status == 0) {
		req->req.actual += req->dma_bytes;

		/* return if this request needs to send more data */
		if (req->req.actual < req->req.length)
			return;
	}

	done(ep, req, status);
}

static void next_out(struct bcm28xx_udc2_ep *ep, struct bcm28xx_udc2_req *req)
{
	unsigned length = req->req.length - req->req.actual;
	int ep_index;
	struct bcm28xx_udc2_data_desc *data_desc;
	u32 reg;

	ep_index = ep->bEndpointAddress & 0xf;

	VDBG("%s: ep_index = %d, length = %d\n", __FUNCTION__, ep_index, length);

	data_desc = ep->dma_descriptor[1].virt_addr;
	data_desc->quadlet = UDC2_QUADLET_HOST_READY | UDC2_QUADLET_LAST | (length & 0xffff);
	data_desc->buf_ptr = req->req.dma;

	req->dma_bytes = length;

	enable_OUT_ep_interrupt(ep_index);
	wmb();

	/* Clear NAK and re-enable Rx DMA */
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);
	wmb();
	reg = BCM28XX_USBD0_EP0O_CTRL_PHY + (0x20 * ep_index);	
	bcm28xx_udc2_modifyl(reg, 0, UDC_EP_CTRL_PRDY);
	wmb();
	CNAK_OUT_ep(ep_index);
}

static void
finish_out(struct bcm28xx_udc2_ep *ep, struct bcm28xx_udc2_req *req, int status)
{
	int ep_index;
	struct bcm28xx_udc2_data_desc *data_desc;	
	u32 rxSize;

	ep_index = ep->bEndpointAddress & 0xf;
	data_desc = ep->dma_descriptor[1].virt_addr;
	rxSize = data_desc->quadlet & 0xffff;
	data_desc->quadlet = UDC2_QUADLET_HOST_BUSY;

	req->req.actual += rxSize;

	done(ep, req, status);
}

#else

static void next_in(struct bcm28xx_udc2_ep *ep, struct bcm28xx_udc2_req *req)
{
	unsigned length = req->req.length - req->req.actual;
	int ep_index;
	unsigned loaded, this_chunk;
	u32 reg;
	struct bcm28xx_udc2_data_desc *data_desc;	
	int desc_index;
	int end_of_chain;
	
	ep_index = ep->bEndpointAddress & 0xf;

	desc_index = 0; loaded = 0;
	end_of_chain = 0;

	VDBG("%s: ep_index = %d, length = %d\n", __FUNCTION__, ep_index, length);		

	while ((!end_of_chain) && (length >= 0))	{
		this_chunk = min(length, (unsigned) ep->maxpacket);
		data_desc = (struct bcm28xx_udc2_data_desc *) (ep->dma_descriptor[0].virt_addr + (desc_index * sizeof(struct bcm28xx_udc2_data_desc)));
		data_desc->quadlet = UDC2_QUADLET_HOST_READY | (this_chunk & 0xffff); 
		if ((length <= ep->maxpacket) || (desc_index == (ep->dma_chain_len - 1))) {
			data_desc->quadlet |= UDC2_QUADLET_LAST; 
			end_of_chain = 1;
		} else 
			data_desc->next_ptr = ep->dma_descriptor[0].dma_addr + ((desc_index + 1) * sizeof(struct bcm28xx_udc2_data_desc));

		data_desc->buf_ptr = req->req.dma + req->req.actual + loaded;

		loaded += this_chunk;	
		length -= this_chunk;
		desc_index++;
	}

	req->dma_bytes = loaded;

	ep->in_state = IN_POLLDEMAND;

    udc_logger_do_log(USB_UDC_NEXT_IN, ep_index, loaded,  bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY), ep0_in_nak_req_flag);

	/* Renable interrupt, start DMA and clear CNK */
	enable_IN_ep_interrupt(ep_index);

	wmb();
	reg = BCM28XX_USBD0_EP0I_CTRL_PHY + (0x20 * ep_index);	
	bcm28xx_udc2_modifyl(reg, 0, UDC_EP_CTRL_POLL | UDC_EP_CTRL_CNAK);

    if (ep_index == 0)
    {
        CNAK_IN_ep(0);
    }

    udc_logger_do_log(USB_UDC_NEXT_IN, ep_index, loaded,  bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY),  bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY));

    udc_logger_do_log(USB_UDC_NEXT_IN, ep_index, bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY), ep0_in_nak_req_flag, bcm28xx_udc2_readl(BCM28XX_USBD0_DEVCTRL_PHY));
            
#ifdef BCM28XX_UDC2_NAK_DBG
    if (ep_index == 0)
        printk(KERN_DEBUG "\n %s %d ep0 o ctrl=0x%08x ep0in ctrl=0x%08x ep0in stat=0x%08x\n",  
          __FUNCTION__, __LINE__, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_STAT_PHY));
#endif  /* BCM28XX_UDC2_NAK_DBG */
        
}

static void finish_in(struct bcm28xx_udc2_ep *ep, struct bcm28xx_udc2_req *req, int status)
{
	if (status == 0) {
		req->req.actual += req->dma_bytes;

		/* return if this request needs to send more data */
		if (req->req.actual < req->req.length)
			return;
	}

    udc_logger_do_log(USB_UDC_FINISH_IN, req->req.actual, status, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY));

	done(ep, req, status);
}

static void next_out(struct bcm28xx_udc2_ep *ep, struct bcm28xx_udc2_req *req)
{
	unsigned length = req->req.length - req->req.actual;
	int ep_index;
	unsigned loaded, this_chunk;
	int end_of_chain;
	struct bcm28xx_udc2_data_desc *data_desc;
	u32 reg;
	int desc_index;
	
	
	ep_index = ep->bEndpointAddress & 0xf;

	end_of_chain = 0; this_chunk = 0; loaded = 0;
	desc_index = 0;

	while ((!end_of_chain) && (length >= 0)) {
		this_chunk = min(length, (unsigned) ep->maxpacket);
		data_desc = (struct bcm28xx_udc2_data_desc *) (ep->dma_descriptor[1].virt_addr + (desc_index * sizeof(struct bcm28xx_udc2_data_desc)));
	
		data_desc->quadlet = UDC2_QUADLET_HOST_READY | (this_chunk & 0xffff);  
	
		if ((length <= ep->maxpacket) || (desc_index == (ep->dma_chain_len - 1))) {
			data_desc->quadlet |= UDC2_QUADLET_LAST; 
			end_of_chain = 1;
		}
		else
			data_desc->next_ptr = ep->dma_descriptor[1].dma_addr + ((desc_index + 1) * sizeof(struct bcm28xx_udc2_data_desc));
			
		data_desc->buf_ptr = req->req.dma + req->req.actual + loaded;
	
		loaded += this_chunk; 
		length -= this_chunk;
		desc_index ++;
	}

	req->dma_bytes = loaded;

	enable_OUT_ep_interrupt(ep_index);
	wmb();

    if (ep_index == 3)
        udc_logger_do_log(USB_UDC_NEXT_OUT, ep_index, loaded, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * 3)),  bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY));
    else
        udc_logger_do_log(USB_UDC_NEXT_OUT, ep_index, loaded, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY));
        

#ifdef NAK_RDE_FIX
    if (ep_index == 3)
    {
        /*
         * Before the RRDY bit is set to 1, even if RDE is turned on, according
         * to spec, the data should not be DMAed into buffer yet. 
         * Therefore, we first set NAK on this endpoint and then drain out
         * the RxFIFO.
         * Last thing is to clear the NAK and let the traffic come into the
         * RxFIFO and also the buffer.
         */ 
        if (ep0_in_nak_req_flag || ep0_out_nak_req_flag)
            SNAK_OUT_ep(ep_index);
      
        /* Now let the RxFIFO on endpoint 3 drain. */
        bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);
        wmb();
        reg = BCM28XX_USBD0_EP0O_CTRL_PHY + (0x20 * ep_index);	
        bcm28xx_udc2_modifyl(reg, 0, UDC_EP_CTRL_PRDY);
        wmb();
        
        if (ep0_in_nak_req_flag)
        {
            int iteration_cnt;
            for (iteration_cnt=0; ;iteration_cnt++)
            {
                bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0I_CTRL_PHY, 0, UDC_EP_CTRL_CNAK); 
                if (!(bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY) & UDC_EP_CTRL_NAK))
                    break;
            }

#ifdef BCM28XX_UDC2_NAK_DBG
           printk(KERN_ERR "\nepoI with cnt =%d\n",  iteration_cnt);
#endif
           
           udc_logger_do_log(USB_UDC_NEXT_OUT, 0xaf1, iteration_cnt, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY));
  
            ep0_in_nak_req_flag = 0;
        }

         if (ep0_out_nak_req_flag)
        {
            int iteration_cnt;
            for (iteration_cnt=0; ;iteration_cnt++)
            {
                bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0O_CTRL_PHY, 0, UDC_EP_CTRL_CNAK); 
                if (!(bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY) & UDC_EP_CTRL_NAK))
                    break;
            }
 
#ifdef BCM28XX_UDC2_NAK_DBG
           printk(KERN_ERR "\nepoO with cnt =%d\n",  iteration_cnt);
#endif
           
           udc_logger_do_log(USB_UDC_NEXT_OUT, 0xaf2, iteration_cnt, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY));
 
            ep0_out_nak_req_flag = 0;
        }
        
        /* 
         * Now let the traffic come in. 
         * FIXME: Shall we be concerned about whether or not the endpoint 3 can clear the NAK???
         * Noop, because once ctrl endpoint started to RX after clearing the NAK as the
         * last thing, it will always have enough buffer to receive all the data sent by the host.
         */
        wmb();

        udc_logger_do_log(USB_UDC_NEXT_OUT, 0xaf3, ep_index, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * 3)),  bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY));
        CNAK_OUT_ep(ep_index);
        udc_logger_do_log(USB_UDC_NEXT_OUT, 0xaf4, ep_index, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * 3)),  bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY));
 

    }
    else
    {
        
        /* Clear NAK and re-enable Rx DMA */
        bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);
        wmb();
        reg = BCM28XX_USBD0_EP0O_CTRL_PHY + (0x20 * ep_index);	
        bcm28xx_udc2_modifyl(reg, 0, UDC_EP_CTRL_PRDY);
        wmb();
        CNAK_OUT_ep(ep_index);

        udc_logger_do_log(USB_UDC_NEXT_IN, 0xaf5, ep_index, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY),  bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY));

        udc_logger_do_log(USB_UDC_NEXT_IN, 0xaf6, ep_index, bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_DEVCTRL_PHY));
    
    }


#else   /* NAK_RDE_FIX */     

	/* Clear NAK and re-enable Rx DMA */
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);
	wmb();
	reg = BCM28XX_USBD0_EP0O_CTRL_PHY + (0x20 * ep_index);	
	bcm28xx_udc2_modifyl(reg, 0, UDC_EP_CTRL_PRDY);
	wmb();
	CNAK_OUT_ep(ep_index);

#endif  /* NAK_RDE_FIX */ 

    if (ep_index == 3)
        udc_logger_do_log(USB_UDC_NEXT_OUT, ep_index, loaded, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * 3)),  bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY));
    else
        udc_logger_do_log(USB_UDC_NEXT_OUT, ep_index, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), ep0_out_nak_req_flag, ep0_in_nak_req_flag);


}

static void
finish_out(struct bcm28xx_udc2_ep *ep, struct bcm28xx_udc2_req *req, int status)
{
	int ep_index;
	struct bcm28xx_udc2_data_desc *data_desc;	
	u32 rxSize = 0;
	int desc_index;

	ep_index = ep->bEndpointAddress & 0xf;

	VDBG("%s: ep_index = %d, dma_chain_len = %d\n", __FUNCTION__, ep_index, ep->dma_chain_len);

	for (desc_index = 0; desc_index < ep->dma_chain_len; desc_index ++) {
		data_desc = (struct bcm28xx_udc2_data_desc *) (ep->dma_descriptor[1].virt_addr + (desc_index * sizeof(struct bcm28xx_udc2_data_desc)));
	
		if (data_desc->quadlet & UDC2_QUADLET_LAST) {
			rxSize = (data_desc->quadlet & 0xffff);  

			/* Take care of overflow issue */
			if ((rxSize == 0) && (req->dma_bytes == (64 * 1024))) 
				rxSize = (64 * 1024);
			
			req->req.actual += rxSize;
			break;
		}
	}	 

    udc_logger_do_log(USB_UDC_FINISH_OUT, ep_index, rxSize,  bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY),bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * 3)));

	VDBG("%s: ep_index = %d, length = %d\n", __FUNCTION__, ep_index, req->req.actual);

	done(ep, req, status);
}

#endif

static int
bcm28xx_udc2_ep_queue(struct usb_ep *_ep, struct usb_request *_req, unsigned gfp_flags)
{
	struct bcm28xx_udc2_ep	*ep = container_of(_ep, struct bcm28xx_udc2_ep, ep);
	struct bcm28xx_udc2_req	*req = container_of(_req, struct bcm28xx_udc2_req, req);
	int ep_index, is_in = 0;
	struct bcm28xx_udc2	*udc2;
	unsigned long	flags;

	/* catch various bogus parameters */
	if (!_req || !req->req.complete || !req->req.buf
			|| !list_empty(&req->queue)) {
		ERR("%s, bad params\n", __FUNCTION__);
		return -EINVAL;
	}

	if (!_ep || (!ep->desc && ep->bEndpointAddress)) {
		ERR("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	if (check_req_len(ep, req->req.length)) {
		ERR("%s, ep %s, bad request length %d\n", __FUNCTION__, ep->name, req->req.length);
		return -EMSGSIZE;
	}

	udc2 = ep->udc2;
	if (!udc2->driver || udc2->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	VDBG("%s: %s queue req %p, len %d buf %p\n",
		__FUNCTION__, ep->ep.name, _req, _req->length, _req->buf);

	spin_lock_irqsave(&udc2->lock, flags);

	req->req.status = -EINPROGRESS;
	req->req.actual = 0;

	ep_index = ep->bEndpointAddress & 0xf;

    udc_logger_do_log(USB_UDC_EP_QUEUE, ep_index, _req->length,  bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY),bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY+(0x20 * 2)));

	/* EP0 is not supposed to have multiple requests pending
	 */
	if (ep_index == 0) {
		VDBG("%s: EP 0 current state = %d, next state = %d\n", __FUNCTION__, 
		udc2->ep0_current_state, udc2->ep0_next_state);

		if ((udc2->ep0_next_state == EP0_IN_STATUS) ||
			(udc2->ep0_next_state == EP0_IN_DATA)) {
			is_in = 1;
		} else if ((udc2->ep0_next_state == EP0_OUT_STATUS) ||
	 		   	   (udc2->ep0_next_state == EP0_OUT_DATA)) {
	 		is_in = 0;
		} else {
			ERR("%s: Invalid EP0 state %d\n", __FUNCTION__, udc2->ep0_next_state);
		}
        
        udc_logger_do_log(USB_UDC_CRASH_STATE, is_in, _req->length, 0x0, 0x0);

#ifdef BCM28XX_UDC2_NAK_DBG
        printk(KERN_DEBUG "\n %s %d EP0 direction=%d, curr state = %d, next state = %d, len = 0x%0x \n",
            __FUNCTION__, __LINE__, is_in, udc2->ep0_current_state, udc2->ep0_next_state, _req->length);
#endif

	} else {
		is_in = ep->bEndpointAddress & USB_DIR_IN;
	}

	if (list_empty(&ep->queue) && !ep->stopped) {
		if (ep_index == 0)	{
			struct bcm28xx_udc2_setup_desc * setup_desc;
			
			if (!udc2->ep0_pending)	{
				ERR("%s: EP0 not pending\n", __FUNCTION__);
                BUG();
				spin_unlock_irqrestore(&udc2->lock, flags);
				return -EL2HLT;
			}

			VDBG("%s: EP0 next state %d\n", __FUNCTION__, udc2->ep0_next_state);

			switch (udc2->ep0_next_state) {
				case EP0_IN_STATUS: 
					if (req->req.length) {
						spin_unlock_irqrestore(&udc2->lock, flags);
						ERR("%s: EP0 IN status packet should be zlp, %d\n", __FUNCTION__, req->req.length);
						BUG();
						return -EL2HLT;
					}

					if (udc2->ep0_current_state == EP0_SETUP) {
						if (udc2->ep0_set_config || udc2->ep0_set_intf) {
							/* Set CSR_DONE bit */
							bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_CSRDONE);
						
							/* UDC core transmits zlp automatically */
							udc2->ep0_set_config = udc2->ep0_set_intf = 0;
						}
					}

					udc2->ep0_current_state = EP0_IN_STATUS;
					udc2->ep0_next_state = EP0_IDLE;	

					/* cleanup */
					udc2->ep0_pending = 0;
					done(ep, req, 0);
					req = NULL;							

					udc2->ep0_current_state = EP0_IDLE;
					udc2->ep0_next_state = EP0_SETUP;	

					setup_desc = (struct bcm28xx_udc2_setup_desc *) udc2->SETUP.virt_addr;
					setup_desc->quadlet = UDC2_QUADLET_HOST_READY;
					
					/* Clear IN & OUT NAK and re-enable Rx DMA, re-enable EP0 OUT interrupt */
					enable_OUT_ep_interrupt(0);
					bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);					
					CNAK_OUT_ep(0);
					CNAK_IN_ep(0);

					break;

				case EP0_IN_DATA: 
					udc2->ep0_current_state = EP0_IN_DATA;
					udc2->ep0_next_state = EP0_OUT_STATUS;

					break;

				case EP0_OUT_DATA:
					udc2->ep0_current_state = EP0_OUT_DATA;
					udc2->ep0_next_state = EP0_IN_STATUS;

					break;

				case EP0_OUT_STATUS:
				default: 
					ERR("%s: Invalid EP0 next state %d\n", __FUNCTION__, udc2->ep0_next_state);
					BUG();

					break;
			} 
		}
		
		if (req) {
	        if (is_in) {
				if(ep->in_state == IN_READY) {
					VDBG("%s: EP %d in_state ready\n", __FUNCTION__, ep_index);	

#ifdef BCM28XX_UDC2_NAK_DBG
                    if (ep_index == 0)
                        printk(KERN_ERR "\n %s %d ep0 o ctrl=0x%08x ep0in ctrl=0x%08x ep0in stat=0x%08x\n",  
                        __FUNCTION__, __LINE__,bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_STAT_PHY));
#endif
                        
					next_in(ep, req);
				} else {
                    
                    if (ep_index == 0)
                    {
                        udc_logger_do_log(USB_UDC_CRASH_STATE, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_STAT_PHY));

#ifdef BCM28XX_UDC2_NAK_DBG                       
                        printk(KERN_ERR "\n %s %d ep0 o ctrl=0x%08x ep0in ctrl=0x%08x ep0in stat=0x%08x\n",  
                        __FUNCTION__, __LINE__,bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_STAT_PHY));
#endif                        
                    }
                    
					VDBG("%s: EP %d in_state not ready\n", __FUNCTION__, ep_index);					
				}  /* == IN_READY */
	        } else
				next_out(ep, req);
		}
	}
	
	/* irq handler advances the queue */
	if (req != NULL)
		list_add_tail(&req->queue, &ep->queue);

	spin_unlock_irqrestore(&udc2->lock, flags);

	return 0;
}


static int bcm28xx_udc2_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct bcm28xx_udc2_ep	*ep = container_of(_ep, struct bcm28xx_udc2_ep, ep);
	struct bcm28xx_udc2_req	*req;
	unsigned long	flags;
	int ep_index, is_in = 0;

	if (!_ep || !_req)
		return -EINVAL;

	spin_lock_irqsave(&ep->udc2->lock, flags);

	ep_index = ep->bEndpointAddress & 0xf;

	if (ep_index == 0) {
		if (gUdc2->ep0_current_state == EP0_IN_DATA)
			is_in = 1;
		else if (gUdc2->ep0_current_state == EP0_OUT_DATA)
			is_in = 0;
		else {
			ERR ("%s: should be no outstanding request on EP0. \n", __FUNCTION__);
			BUG();
			return -EINVAL;
		}
	} else {
		is_in = ep->bEndpointAddress & USB_DIR_IN;
	}

	if (!is_in)
    {
		SNAK_OUT_ep(ep_index);
        /* 
         * We need to make sure that the RX FIFO is empty.
         * Since we are nottaking this path for fixing the Cancel read/write, 
         * let us not touch it for now.
         */
#if 0         
        while (1)
        {
            if (bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY) & UDC_DEV_STAT_RXFIFO_EMPTY)
                break;
        }
#endif        
    }

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&ep->udc2->lock, flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);
	spin_unlock_irqrestore(&ep->udc2->lock, flags);
	return 0;
}


static int bcm28xx_udc2_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct bcm28xx_udc2_ep	*ep = container_of(_ep, struct bcm28xx_udc2_ep, ep);
	unsigned long	flags;
	int		status = -EOPNOTSUPP;

	spin_lock_irqsave(&ep->udc2->lock, flags);

	/* just use protocol stalls for ep0; real halts are annoying */
	if (ep->bEndpointAddress == 0) {
		if (!ep->udc2->ep0_pending)
			status = -EINVAL;
		else if (value)
		{
			if (ep->udc2->ep0_set_config)
			{
				WARN("error changing config?\n");
			}
			ep->udc2->ep0_pending = 0;
			status = 0;
		} 
		else /* NOP */
			status = 0;
	} else {
		int is_in, ep_index;
		u32 reg;

		ep_index = ep->bEndpointAddress & 0xf;
		is_in = ep->bEndpointAddress & USB_DIR_IN;

		reg = is_in ? (BCM28XX_USBD0_EP0I_CTRL_PHY+(0x20*ep_index)) : (BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20*ep_index));
		
		if (value) {
			if (is_in)
				bcm28xx_udc2_modifyl(reg, 0, UDC_EP_CTRL_STALL);
			else
				bcm28xx_udc2_modifyl(reg, 0, UDC_EP_CTRL_STALL | UDC_EP_CTRL_FLUSH);
		} else {
			bcm28xx_udc2_modifyl(reg, UDC_EP_CTRL_STALL, 0);
		}

		status = 0;
	}

	VDBG("%s %s halt stat %d\n", ep->ep.name,
		value ? "set" : "clear", status);

	spin_unlock_irqrestore(&ep->udc2->lock, flags);
	return status;
}

static int bcm28xx_udc2_ep_fifo_status(struct usb_ep *_ep)
{
	struct bcm28xx_udc2_ep	*ep = container_of(_ep, struct bcm28xx_udc2_ep, ep);
	unsigned long	flags;
	int 	status = -EOPNOTSUPP;

	spin_lock_irqsave(&ep->udc2->lock, flags);

	/* The UDC core (non-slave mode) doesn't have a mechanism for 
	 * s/w to probe the FIFO status. All the endpoints share one
	 * RxFIFO. So we simply return 0 here. 
	 */
	status = 0;
	spin_unlock_irqrestore(&ep->udc2->lock, flags);
	return status;
}


static void bcm28xx_udc2_ep_fifo_flush (struct usb_ep *_ep)
{
	struct bcm28xx_udc2_ep	*ep = container_of(_ep, struct bcm28xx_udc2_ep, ep);
	int is_in, ep_index;
	u32 reg;
	unsigned long	flags;

	spin_lock_irqsave(&ep->udc2->lock, flags);

	is_in = ep->bEndpointAddress & USB_DIR_IN;
	ep_index = ep->bEndpointAddress & 0xf;

	/* Flush EP Tx FIFO. Do nothing for OUT EP.
	 */
	if ((ep_index == 0) || is_in) {
		reg = BCM28XX_USBD0_EP0I_CTRL_PHY+(0x20*ep_index);
		bcm28xx_udc2_modifyl(reg, 0, UDC_EP_CTRL_FLUSH);
	}
	 
	spin_unlock_irqrestore(&ep->udc2->lock, flags);
}

static struct usb_ep_ops bcm28xx_udc2_ep_ops = {
	.enable 		= bcm28xx_udc2_ep_enable,
	.disable		= bcm28xx_udc2_ep_disable,

	.alloc_request	= bcm28xx_udc2_alloc_request,
	.free_request	= bcm28xx_udc2_free_request,

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
	.alloc_buffer	= bcm28xx_udc2_alloc_buffer,
	.free_buffer	= bcm28xx_udc2_free_buffer,
#endif

	.queue			= bcm28xx_udc2_ep_queue,
	.dequeue		= bcm28xx_udc2_ep_dequeue,

	.set_halt		= bcm28xx_udc2_ep_set_halt,
	.fifo_status	= bcm28xx_udc2_ep_fifo_status, 
	.fifo_flush		= bcm28xx_udc2_ep_fifo_flush,
};


/*-------------------------------------------------------------------------
 *
 *					BCM28XX UDC ISR functions
 * 
 *-------------------------------------------------------------------------*/
static void ep0_handle_data(struct bcm28xx_udc2 *udc2, u32 ep_irq)
{
	struct bcm28xx_udc2_ep	*ep = &udc2->ep[0];
	struct bcm28xx_udc2_req *req, req_outstatus ;
	struct bcm28xx_udc2_setup_desc *setup_desc;
	u32 ep_stat; 

	VDBG("%s: EP0 current state %d, next state %d\n", __FUNCTION__, 
		 udc2->ep0_current_state, udc2->ep0_next_state);

	if (ep_irq & UDC_EP_0_IN_INT) {
		/* read then clear the ep status register */
		ep_stat = bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_STAT_PHY);
		bcm28xx_udc2_writel(ep_stat, BCM28XX_USBD0_EP0I_STAT_PHY);

        udc_logger_do_log(USB_UDC_ISR_ENTER_2, 0x19721103, ep_stat, 0x0, 0x0);


		VDBG("%s: IN, ep_stat = 0x%x, ep 0 in_state = %d\n", __FUNCTION__, ep_stat, ep->in_state);

		if (ep_stat & UDC_EP_STATUS_TDC) {
			VDBG("%s: EP0 stat = 0x%x, in_state set to TDC\n", __FUNCTION__, ep_stat);
			ep->in_state = IN_TDC;
		}
		else if ((ep_stat & UDC_EP_STATUS_IN) && 
			((ep->in_state == IN_INIT) || (ep->in_state == IN_TDC))) {
			VDBG("%s: EP0 stat = 0x%x, in_state set to IN_READY\n", __FUNCTION__, ep_stat);
			disable_IN_ep_interrupt(0);
			ep->in_state = IN_READY;
		}

		switch (udc2->ep0_current_state) {			
			case EP0_IN_DATA:
				if (ep_stat & UDC_EP_STATUS_TDC) {
					if (!list_empty(&ep->queue)) {
						req = container_of(ep->queue.next, struct bcm28xx_udc2_req, queue);
						finish_in(ep, req, 0);
					}

					udc2->ep0_current_state = EP0_OUT_STATUS;
					udc2->ep0_next_state = EP0_IDLE;

					/* Prepare buffer to receive the zero */
					req_outstatus.req.length = req_outstatus.req.actual = 0;
					req_outstatus.req.buf = udc2->SETUP_out_status.virt_addr;
					req_outstatus.req.dma = udc2->SETUP_out_status.dma_addr;

					next_out(ep, &req_outstatus);
				} else if (ep->in_state == IN_READY) {
					if (!list_empty (&ep->queue)) {
						req = container_of(ep->queue.next, struct bcm28xx_udc2_req, queue);
						next_in(ep, req);
					}
				}
				break;

			case EP0_IN_STATUS:
				/* UDC automatically sends zlp after DATA_OUT for us, hence we should never get here. 
				 */
				break;
			default:
				break;
		}
	}

	if (ep_irq & UDC_EP_0_OUT_INT) {
		/* read then clear the ep status register */
		ep_stat = bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_STAT_PHY);
		bcm28xx_udc2_writel(ep_stat, BCM28XX_USBD0_EP0O_STAT_PHY);

        udc_logger_do_log(USB_UDC_ISR_ENTER_2, 0x19721214, ep_stat, 0x1, 0x1);

		switch(udc2->ep0_current_state) {
			case EP0_OUT_DATA:
				
				if (!list_empty(&ep->queue)) {
					req = container_of(ep->queue.next, struct bcm28xx_udc2_req, queue);
					finish_out(ep, req, 0);
				}

				/* no state change */
				udc2->ep0_current_state = EP0_OUT_DATA;
				udc2->ep0_next_state = EP0_IN_STATUS;

				break;

			case EP0_OUT_STATUS:

				udc2->ep0_current_state = EP0_IDLE;
				udc2->ep0_next_state = EP0_SETUP;


				/* CNAK EP0 and re-enable the RxDMA */
				enable_OUT_ep_interrupt(0);
				setup_desc = (struct bcm28xx_udc2_setup_desc *) udc2->SETUP.virt_addr;
				setup_desc->quadlet = UDC2_QUADLET_HOST_READY;
				bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);
				CNAK_OUT_ep(0);
				CNAK_IN_ep(0);

				break;

			default:
				
				break;
		}
	}	
}


static void ep0_handle_setup(struct bcm28xx_udc2 *udc2, u32 dev_irq, u32 ep_irq)
{
	struct bcm28xx_udc2_ep			*ep0 = &udc2->ep[0];
	struct usb_ctrlrequest			r;
	int 							status = -EINVAL;
	struct bcm28xx_udc2_setup_desc	*setup_desc = NULL;
	u32 							reg;
	int 							ep_index;

	disable_OUT_ep_interrupt(0);

	memset(&r, 0x0, sizeof(r));
	
	if (dev_irq & UDC_DEV_INT_SC) {
		u16 cfg;

		cfg = (u16) (bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY) & 0xf);
		r.bRequestType = USB_DIR_OUT | USB_RECIP_DEVICE;
		r.bRequest = USB_REQ_SET_CONFIGURATION;
		r.wValue = __cpu_to_le16(cfg);
		r.wIndex = 0;
		r.wLength = 0;

		for (ep_index = 0; ep_index < BCM28XX_UDC2_NUM_DATA_EP; ep_index++)	{
			reg = BCM28XX_USBD0_EP0CONFIG_PHY + 0x04 * ep_index;
			bcm28xx_udc2_modifyl(reg, 0xf << UDC_CFG_NUM_OFST, 
								 (cfg & 0xf) << UDC_CFG_NUM_OFST);
		}
		
		udc2->ep0_set_config = 1;
		
		VDBG("set config %d\n", cfg);
	}
	else if (dev_irq & UDC_DEV_INT_SI) {
		u32 dev_stat;
		u16 intf, alt;

		dev_stat = bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY);
		intf = (u16) ((dev_stat >> UDC_DEV_STAT_INTF_OFST) & 0xf);		
		alt  = (u16) ((dev_stat >> UDC_DEV_STAT_ALT_OFST) & 0xf);		

		r.bRequestType = USB_DIR_OUT | USB_RECIP_INTERFACE;
		r.bRequest = USB_REQ_SET_INTERFACE;
		r.wValue = __cpu_to_le16(alt);
		r.wIndex = intf;
		r.wLength = 0;
		
		for (ep_index = 0; ep_index < BCM28XX_UDC2_NUM_DATA_EP; ep_index++)	{
			reg = BCM28XX_USBD0_EP0CONFIG_PHY + 0x04 * ep_index;
			bcm28xx_udc2_modifyl(reg, (0xf << UDC_INTF_NUM_OFST) |  (0xf << UDC_ALT_SETTING_OFST), 
					((intf & 0xf) << UDC_INTF_NUM_OFST) | ((alt & 0xf) << UDC_ALT_SETTING_OFST));
		}

		udc2->ep0_set_intf = 1;		
		
		VDBG("set interface %d, alternate setting %d\n", intf, alt);
	}
	else {
		setup_desc = (struct bcm28xx_udc2_setup_desc *) udc2->SETUP.virt_addr;
		r = * ((struct usb_ctrlrequest *)&(setup_desc->first_data));
	}

#define	w_value		le16_to_cpup (&r.wValue)
#define	w_index		le16_to_cpup (&r.wIndex)
#define	w_length	le16_to_cpup (&r.wLength)

	/* Delegate almost all control requests to the gadget driver,
	 * except for a handful of ch9 status/feature requests that
	 * hardware doesn't autodecode _and_ the gadget API hides.
	 */
	if (udc2->ep0_next_state != EP0_SETUP) {
		ERR("%s: Invalid EP0 next state %d, current state %d\n", __FUNCTION__, 
			udc2->ep0_next_state, udc2->ep0_current_state);
		BUG();
	}

	udc2->ep0_current_state = EP0_SETUP;
	
	if (r.bRequestType & USB_DIR_IN) {
		udc2->ep0_next_state = EP0_IN_DATA;
	}
	else {
		if (w_length)
			udc2->ep0_next_state = EP0_OUT_DATA;
		else
			udc2->ep0_next_state = EP0_IN_STATUS;
	}

	VDBG("%s: EP0 current state %d, next state %d\n", __FUNCTION__, 
		 udc2->ep0_current_state, udc2->ep0_next_state);
						
	ep0->stopped = 0;
	udc2->ep0_pending = 1;

	switch (r.bRequest) {
		case USB_REQ_SET_CONFIGURATION:
			if (r.bRequestType != USB_RECIP_DEVICE)
				goto delegate;
			if (w_length != 0)
				goto do_stall;

			goto delegate;

		case USB_REQ_SET_INTERFACE:
			if (r.bRequestType != USB_RECIP_INTERFACE)
				goto delegate;
			if (w_length != 0)
				goto do_stall;

			goto delegate;

		case USB_REQ_CLEAR_FEATURE:
			if (r.bRequestType != USB_RECIP_ENDPOINT)
				goto delegate;
			/* clear endpoint halt */
			if (w_value != USB_ENDPOINT_HALT
					|| w_length != 0)
				goto do_stall;

			ep_index = w_index & 0xf;
			if (ep_index != 0) {
				u32 reg;
				int is_in;
								
				is_in = udc2->ep[ep_index].bEndpointAddress & USB_DIR_IN;
				reg = is_in ? (BCM28XX_USBD0_EP0I_CTRL_PHY+(0x20*ep_index)) : (BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20*ep_index));
				bcm28xx_udc2_modifyl(reg, UDC_EP_CTRL_STALL, 0);
			}
				
			status = 0;
			udc2->ep0_pending = 0;
			udc2->ep0_current_state = EP0_IDLE;
			udc2->ep0_next_state = EP0_SETUP;

			setup_desc = (struct bcm28xx_udc2_setup_desc *) udc2->SETUP.virt_addr;
			setup_desc->quadlet = UDC2_QUADLET_HOST_READY;
			
			/* Clear IN & OUT NAK and re-enable Rx DMA */
			enable_OUT_ep_interrupt(0);
			bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);					
			CNAK_OUT_ep(0);
			CNAK_IN_ep(0);

			VDBG("%s halt cleared by host\n", udc2->ep[ep_index].name);
			break;

		case USB_REQ_SET_FEATURE:
#if 0
			/* Both MSC and MTP class drivers don't handle
	         * this request. UDC core will send zlp automatically
	         * once EP0 NAK is cleared. 
	         */	
#else
			ERR("Not supported request: SET_FEATURE\n");
			BUG();
#endif
			break;

		case USB_REQ_SET_ADDRESS:
			/* Both MSC and MTP class drivers don't handle
	         * this request. UDC core will send zlp automatically
	         * once EP0 NAK is cleared. 
	         */	
			status = 0;
			udc2->ep0_pending = 0;
			udc2->ep0_current_state = EP0_IDLE;
			udc2->ep0_next_state = EP0_SETUP;

			setup_desc = (struct bcm28xx_udc2_setup_desc *) udc2->SETUP.virt_addr;
			setup_desc->quadlet = UDC2_QUADLET_HOST_READY;
			
			/* Clear IN & OUT NAK and re-enable Rx DMA */
			enable_OUT_ep_interrupt(0);
			bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);					
			CNAK_OUT_ep(0);
			CNAK_IN_ep(0);

			VDBG("Address %d\n", w_value);
			break;
			
		case USB_REQ_GET_STATUS:
#if 0
			/* return interface status.  if we were pedantic,
			 * we'd detect non-existent interfaces, and stall.
			 */
#else
			ERR("Not supported request: GET_STATUS, interface %d\n", w_index);
			BUG();
#endif
			break;

		default:
delegate:
			/* gadget drivers see class/vendor specific requests,
			 * {SET,GET}_{INTERFACE,DESCRIPTOR,CONFIGURATION},
			 * and more
			 */
			VDBG("SETUP %02x.%02x v%04x i%04x l%04x\n",
				r.bRequestType, r.bRequest,
				w_value, w_index, w_length);
            
            if ( r.bRequest == 0x67 )
            {
                start_evnts_cnt_after_status = 1;
                n_req_67++;
                udc_logger_do_log(USB_UDC_SETUP_0x64, n_req_67, w_length, 0xdeadbeef, 0xdeadbeef);
            }
#undef	w_value
#undef	w_index
#undef	w_length

			/* The gadget driver may return an error here,
			 * causing an immediate protocol stall.
			 *
			 * Else it must issue a response, either queueing a
			 * response buffer for the DATA stage, or halting ep0
			 * (causing a protocol stall, not a real halt).  A
			 * zero length buffer means no DATA stage.
			 *
			 * It's fine to issue that response after the setup()
			 * call returns, and this IRQ was handled.
			 */
			udc2->ep0_setup = 1;
			spin_unlock(&udc2->lock);
			status = udc2->driver->setup (&udc2->gadget, &r);

            {
                udc_logger_do_log(USB_UDC_SETUP_AFTER_UPPER, status,  bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_STAT_PHY), 0xdeadbeef, 0xdeadbeef);
            }
			spin_lock(&udc2->lock);
			udc2->ep0_setup = 0;
			break;
	}

	if (status < 0) {
do_stall:
		VDBG("%s: req %02x.%02x protocol STALL; stat %d\n",
			 __FUNCTION__, r.bRequestType, r.bRequest, status);
		/* Stall EP0 IN and OUT */
		bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0I_CTRL_PHY, 0, UDC_EP_CTRL_STALL);
		bcm28xx_udc2_modifyl(BCM28XX_USBD0_EP0O_CTRL_PHY, 0, UDC_EP_CTRL_STALL);
		
		udc2->ep0_current_state = EP0_IDLE;
		udc2->ep0_next_state = EP0_SETUP;
		ep0->in_state = IN_INIT;
		udc2->ep0_pending = 0;

		setup_desc = (struct bcm28xx_udc2_setup_desc *) udc2->SETUP.virt_addr;
		setup_desc->quadlet = UDC2_QUADLET_HOST_READY;
		
		/* Clear IN & OUT NAK and re-enable Rx DMA */
		enable_OUT_ep_interrupt(0);
		bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, 0, UDC_DEV_CTRL_RDE);					
		CNAK_OUT_ep(0);
		CNAK_IN_ep(0);
	}

	if ((r.bRequest != USB_REQ_SET_CONFIGURATION) && 
		(r.bRequest != USB_REQ_SET_INTERFACE))
    {
        udc_logger_do_log(USB_UDC_SETUP_0x64,	bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), 
            bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY), ep0_in_nak_req_flag);
  
		CNAK_IN_ep(0);

        udc_logger_do_log(USB_UDC_SETUP_0x64,	bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), 
            bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY), 	ep0_in_nak_req_flag);

#ifdef BCM28XX_UDC2_NAK_DBG
        printk(KERN_DEBUG "\n%s %d ep0 o ctrl=0x%08x ep0in ctrl=0x%08x ep0in stat=0x%08x\n",  
               __FUNCTION__, __LINE__, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_STAT_PHY));
#endif
        
    }
	
}

static void ep0_irq(struct bcm28xx_udc2 *udc2, u32 dev_irq, u32 ep_irq)
{
	u32 ep0_stat;
	int setup = 0; 

	if (ep_irq & UDC_EP_0_OUT_INT) {
		ep0_stat = bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_STAT_PHY);

		switch ((ep0_stat & (UDC_EP_STATUS_OUT_MASK << UDC_EP_STATUS_OUT_OFST)) >> UDC_EP_STATUS_OUT_OFST) {
			case UDC_EP_STATUS_OUT_SETUP: 
				bcm28xx_udc2_writel(ep0_stat, BCM28XX_USBD0_EP0O_STAT_PHY);
				setup = 1;
				break;

    	    case UDC_EP_STATUS_OUT_DATA:
				break;

			default:
				ERR("%s: invalid EP0 OUT status value 0x%x\n", __FUNCTION__, ep0_stat);
				return;
		} 
	}

	if (setup)
    {
       udc_logger_do_log(USB_UDC_CRASH_STATE, 0xa1, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_STAT_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_STAT_PHY), 
            bcm28xx_udc2_readl(BCM28XX_USBD0_DEVCTRL_PHY));
 

        /*clear_ep0_in_nak(); */
        ep0_handle_setup(udc2, 0, ep_irq);
    }
	else if (ep_irq & (UDC_EP_0_OUT_INT | UDC_EP_0_IN_INT))
    {
       udc_logger_do_log(USB_UDC_CRASH_STATE, 0xa2, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_STAT_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_STAT_PHY), 
            bcm28xx_udc2_readl(BCM28XX_USBD0_DEVCTRL_PHY));
 

		ep0_handle_data(udc2, ep_irq);
    }


	if (dev_irq & (UDC_DEV_INT_SC | UDC_DEV_INT_SI))
		ep0_handle_setup(udc2, dev_irq, 0);
}

static void ep_data_irq(struct bcm28xx_udc2 *udc2, u32 ep_irq)
{
	struct bcm28xx_udc2_ep	*ep;
	struct bcm28xx_udc2_req *req;
	int ep_index;
	u32 ep_stat; 

	/* Check IN EPs first */
	for (ep_index = 1; ep_index < BCM28XX_UDC2_NUM_DATA_EP; ep_index++) {
		if (ep_irq & (1<<ep_index))	{
			ep = &(udc2->ep[ep_index]);

			/* Read and clear ep status register */
			ep_stat = bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_STAT_PHY+(0x20*ep_index));
			bcm28xx_udc2_writel(ep_stat, BCM28XX_USBD0_EP0I_STAT_PHY+(0x20*ep_index)); 

			VDBG("%s: IN EP: ep_index %d, ep_stat = 0x%x\n", __FUNCTION__, ep_index, ep_stat);

			if ((ep_stat & UDC_EP_STATUS_IN) && 
				((ep->in_state == IN_INIT) || (ep->in_state == IN_TDC))) {
				disable_IN_ep_interrupt(ep_index);
				ep->in_state = IN_READY;
			}
			else if (ep_stat & UDC_EP_STATUS_TDC)
				ep->in_state = IN_TDC;

			if (!list_empty(&ep->queue) && (ep_stat & UDC_EP_STATUS_TDC)) {
				req = container_of(ep->queue.next,	struct bcm28xx_udc2_req, queue);
				finish_in(ep, req, 0);
			}

			if (!list_empty (&ep->queue) && (ep->in_state == IN_READY)) {
				req = container_of(ep->queue.next,	struct bcm28xx_udc2_req, queue);
				next_in(ep, req);
			}
		}
	}

	/* Check Out EPs  */
	for (ep_index = 1; ep_index < BCM28XX_UDC2_NUM_DATA_EP; ep_index++)	{
		if (ep_irq & (1<<(ep_index + 16))) {
			
			/* Read and clear ep status register */
			ep_stat = bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_STAT_PHY+(0x20*ep_index));
			bcm28xx_udc2_writel(ep_stat, BCM28XX_USBD0_EP0O_STAT_PHY+(0x20*ep_index));

			VDBG("%s: OUT EP: ep_index %d, ep_stat = 0x%x\n", __FUNCTION__, ep_index, ep_stat);

			ep = &(udc2->ep[ep_index]);

			/* Disable EP interrupt */
			disable_OUT_ep_interrupt(ep_index);

			if (!list_empty(&ep->queue)) {
				req = container_of(ep->queue.next, struct bcm28xx_udc2_req, queue);
				finish_out(ep, req, 0);
			}

			if (!list_empty (&ep->queue)) {
				req = container_of(ep->queue.next, struct bcm28xx_udc2_req, queue);
				next_out(ep, req);
			}
		}
	}
}

static void bcm28xx_udc2_speed_setup(int ep_index, unsigned maxp, int is_hs)
{
	struct bcm28xx_udc2_ep	*ep;
	u32 reg;
	int is_in;

	/* EP0 is fixed.*/
	if (ep_index == 0)
		return;

	ep = &gUdc2->ep[ep_index];

	is_in = ep->bEndpointAddress & USB_DIR_IN;

	/* Set up EP maximum packet size */
	reg = (is_in ? BCM28XX_USBD0_EP0I_MAXPKTSIZE_PHY : BCM28XX_USBD0_EP0O_MAXPKTSIZE_PHY) + (0x20 * ep_index);
	bcm28xx_udc2_writel((maxp & 0xffff), reg);

	/* Set up UDC register */
	reg = BCM28XX_USBD0_EP0CONFIG_PHY + (4 * ep_index);
	bcm28xx_udc2_modifyl(reg, 0x3ff << UDC_MAX_PKT_SIZE_OFST, (maxp & 0x3ff) << UDC_MAX_PKT_SIZE_OFST);

	ep->ep.maxpacket = ep->maxpacket = maxp;

#ifndef UDC2_BUFFER_FILL_MODE
	switch (ep->bmAttributes) 
	{
		case USB_ENDPOINT_XFER_CONTROL:
			ep->dma_chain_len = 32; 
			break;

		case USB_ENDPOINT_XFER_BULK:
			ep->dma_chain_len = is_hs ? 128 : 1024; 
			break;

		case USB_ENDPOINT_XFER_INT:
			ep->dma_chain_len = 64; 
			break;

		default:
			BUG();
			break;
	}	
#endif

	/* NAK data out endpoint */
	SNAK_OUT_ep(ep_index);
}


static void bcm28xx_udc2_speed_enum(struct bcm28xx_udc2	*udc2)
{
    u32 val, speed;

    val = bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY);
    speed = (val & (UDC_DEV_STAT_SPD_MASK << UDC_DEV_STAT_SPD_OFST)) >> UDC_DEV_STAT_SPD_OFST;
    
    switch (speed) 
    {
	    case UDC_DEV_STAT_HIGH_SPD:
	        udc2->gadget.speed = USB_SPEED_HIGH;
			bcm28xx_udc2_speed_setup(1, 64, 1);
			bcm28xx_udc2_speed_setup(2, 512, 1);
			bcm28xx_udc2_speed_setup(3, 512, 1);
	        break;

	    case UDC_DEV_STAT_FULL_SPD:
		case UDC_DEV_STAT_FULL_SPD_1:
	        udc2->gadget.speed = USB_SPEED_FULL;
			bcm28xx_udc2_speed_setup(1, 64, 0);
			bcm28xx_udc2_speed_setup(2, 64, 0);
			bcm28xx_udc2_speed_setup(3, 64, 0);
	        break;

	    case UDC_DEV_STAT_LOW_SPD:
			DBG("%s: Low speed not supported\n", __FUNCTION__);
			BUG();
	        udc2->gadget.speed = USB_SPEED_LOW;
	        break;

	    default:
	        break;
    }
}

static irqreturn_t
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
bcm28xx_udc2_irq(int irq, void *_dev, struct pt_regs *r)
#else
bcm28xx_udc2_irq(int irq, void *_dev)
#endif
{
	irqreturn_t status = IRQ_NONE;
	struct bcm28xx_udc2	*udc2 = (struct bcm28xx_udc2 *)_dev;
	u32		dev_irq, ep_irq;
	unsigned long	flags;

	spin_lock_irqsave(&udc2->lock, flags);

	dev_irq = bcm28xx_udc2_readl(BCM28XX_USBD0_DEVINTR_PHY);
	ep_irq = bcm28xx_udc2_readl(BCM28XX_USBD0_EPINTR_PHY);

    /* Clear interrupts */
    bcm28xx_udc2_writel(dev_irq, BCM28XX_USBD0_DEVINTR_PHY);
	bcm28xx_udc2_writel(ep_irq,  BCM28XX_USBD0_EPINTR_PHY);

    udc_logger_do_log(USB_UDC_ISR_ENTER_1, dev_irq, ep_irq, bcm28xx_udc2_readl(BCM28XX_USBD0_EPINTMASK_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_STAT_PHY));
    udc_logger_do_log(USB_UDC_ISR_ENTER_2, bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_STAT_PHY),  ((struct bcm28xx_udc2_setup_desc *)gUdc2->SETUP.virt_addr)->quadlet,
         ((struct bcm28xx_udc2_setup_desc *)gUdc2->SETUP.virt_addr)->first_data, ((struct bcm28xx_udc2_setup_desc *)gUdc2->SETUP.virt_addr)->secnond_data); 
    udc_logger_do_log(USB_UDC_ISR_ENTER_2,	bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_STAT_PHY+(0x20*2)), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_STAT_PHY+(0x20*3)), 
            bcm28xx_udc2_readl(BCM28XX_USBD0_DEVSTAT_PHY), 	bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY+(0x20 * 3)));
    udc_logger_do_log(USB_UDC_ISR_ENTER_2,	bcm28xx_udc2_readl(BCM28XX_USBD0_EP0O_CTRL_PHY), bcm28xx_udc2_readl(BCM28XX_USBD0_EP0I_CTRL_PHY),
            bcm28xx_udc2_readl(BCM28XX_USBD0_DEVCTRL_PHY), 0x12141972);

	VDBG("%s: dev_irq = 0x%x, ep_irq = 0x%x\n", __FUNCTION__, dev_irq, ep_irq);


    if (start_evnts_cnt_after_status)
        events_cnt++;

	/* Return immediately if no class driver registered.
	 */
	if (udc2->driver == NULL) {
		spin_unlock_irqrestore(&udc2->lock, flags);
		return IRQ_HANDLED;
	}
	
	/* 
	 * Flush the BCM28XX SyncTop FIFO
	 */
	FLUSH_SYNC_AHB(); 

	if (dev_irq & UDC_DEV_INT_UR) {
		DBG("%s: reset\n", __FUNCTION__);

		/* TODO: add support for reset */
		status = IRQ_HANDLED;
	   spin_unlock_irqrestore(&udc2->lock, flags);
	   return status;
	}

	if (dev_irq & UDC_DEV_INT_US) {
		DBG("%s: suspend\n", __FUNCTION__);

		/* TODO: add support for suspend */
		status = IRQ_HANDLED;
	}

	if (dev_irq & UDC_DEV_INT_ES) {
		DBG("%s: idle for more than 3ms \n", __FUNCTION__);

		/* TODO: add support for 3ms idle. Suspend is rquired. */
		status = IRQ_HANDLED;
	}

    if (dev_irq & UDC_DEV_INT_ENUM) {
		DBG("%s: speed enumeation\n", __FUNCTION__);
		bcm28xx_udc2_speed_enum(udc2);
        status = IRQ_HANDLED;
    }
   
    /* Interrupts from control endpoint */
    if ((dev_irq & (UDC_DEV_INT_SC | UDC_DEV_INT_SI)) || (ep_irq & (UDC_EP_0_IN_INT | UDC_EP_0_OUT_INT))) {
        ep0_irq(udc2, dev_irq, ep_irq & (UDC_EP_0_IN_INT | UDC_EP_0_OUT_INT));
        status = IRQ_HANDLED;
     }

	/* Interrupts from data endpoints */
	if (ep_irq & (~(UDC_EP_0_IN_INT | UDC_EP_0_OUT_INT))) {
		ep_data_irq(udc2, ep_irq & (~(UDC_EP_0_IN_INT | UDC_EP_0_OUT_INT)));
		status = IRQ_HANDLED;
	}

	if (status != IRQ_HANDLED) 
		DBG("%s, unhandled interrupt, dev irq 0x%x, ep irq 0x%x\n",
		    __FUNCTION__, dev_irq, ep_irq);

	spin_unlock_irqrestore(&udc2->lock, flags);
	return status;
}


/*-------------------------------------------------------------------------
 *
 *					Linux gadget components
 * 
 *-------------------------------------------------------------------------*/
/* dequeue ALL requests; caller holds udc->lock */
static void nuke(struct bcm28xx_udc2_ep *ep, int status)
{
	struct bcm28xx_udc2_req	*req;

	ep->stopped = 1;

	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct bcm28xx_udc2_req, queue);
		done(ep, req, status);
	}
}

/* caller holds udc->lock */
static void udc_quiesce(struct bcm28xx_udc2 *udc2)
{
	struct bcm28xx_udc2_ep	*ep;

	udc2->gadget.speed = USB_SPEED_UNKNOWN;
	nuke(&udc2->ep[0], -ESHUTDOWN);
	list_for_each_entry (ep, &udc2->gadget.ep_list, ep.ep_list)
		nuke(ep, -ESHUTDOWN);
}

int usb_gadget_register_driver (struct usb_gadget_driver *driver)
{
	int		status = -ENODEV;
	unsigned long	flags;

	/* basic sanity tests */
	if (!gUdc2)
		return -ENODEV;
	
	if (!driver
			|| driver->speed < USB_SPEED_FULL
			|| !driver->bind
			|| !driver->unbind
			|| !driver->setup) {
		return -EINVAL;
	}
	
	
	spin_lock_irqsave(&gUdc2->lock, flags);
	if (gUdc2->driver) {
		spin_unlock_irqrestore(&gUdc2->lock, flags);
		return -EBUSY;
	}

	/* reset state */
	gUdc2->ep0_pending = 0;

	/* hook up the driver */
	driver->driver.bus = NULL;
	gUdc2->driver = driver;
	gUdc2->gadget.dev.driver = &driver->driver;
	spin_unlock_irqrestore(&gUdc2->lock, flags);

	status = driver->bind (&gUdc2->gadget);
	

	if (status) {
		DBG("bind to %s --> %d\n", driver->driver.name, status);
		gUdc2->gadget.dev.driver = NULL;
		gUdc2->driver = NULL;
		goto done;
	}
	DBG("bound to driver %s\n", driver->driver.name);
	writel(0x0300, IO_ADDRESS(BCM28XX_USBH0_UTMI_CTRL1_PHY));

done:
	return status;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
	unsigned long	flags;
	int		status = -ENODEV;

	if (!gUdc2)
		return -ENODEV;
	if (!driver || driver != gUdc2->driver)
		return -EINVAL;
	writel(0x1300, IO_ADDRESS(BCM28XX_USBH0_UTMI_CTRL1_PHY));
	udelay(20);

	spin_lock_irqsave(&gUdc2->lock, flags);
	udc_quiesce(gUdc2);
	spin_unlock_irqrestore(&gUdc2->lock, flags);

	driver->unbind(&gUdc2->gadget);
	gUdc2->gadget.dev.driver = NULL;
	gUdc2->driver = NULL;

	DBG("unregistered driver '%s'\n", driver->driver.name);
	return status;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);


static struct usb_gadget_ops bcm28xx_udc2_gadget_ops = {
	.get_frame			= NULL,
	.wakeup				= NULL,
	.set_selfpowered	= NULL, 
	.vbus_session		= NULL,
	.vbus_draw			= NULL,
	.pullup				= NULL,
	.ioctl				= NULL,
};


/*-------------------------------------------------------------------------
 *
 *			BCM28XX UDC gadget and hardware initialization 
 * 			and release functions
 * 
 *-------------------------------------------------------------------------*/

#ifdef UDC2_BUFFER_FILL_MODE
static void bcm28xx_udc2_ep_config(int ep_index, int is_in, u32 type, u32 buf_size, unsigned maxp)
{
	u32 reg, val;
	u32 desc_index;
	struct bcm28xx_udc2_data_desc	*dma_descriptor;
	struct bcm28xx_udc2_dma_buf 	*descriptor_buf;

	reg = (is_in ? BCM28XX_USBD0_EP0I_CTRL_PHY : BCM28XX_USBD0_EP0O_CTRL_PHY) + (0x20 * ep_index);

	/* Configure Control Register */
	val = (type << 4) | (is_in ? UDC_EP_CTRL_FLUSH : UDC_EP_CTRL_PRDY);
	bcm28xx_udc2_writel(val, reg);

	/* Configure Buffer Size */
	val = is_in ? (buf_size >> 2 /* 32-bit word count */) : 0;
	bcm28xx_udc2_writel(val, reg + 0x08);
	
	/* Configure maximum packet size */
	val = maxp & 0xffff;
	bcm28xx_udc2_writel(val, reg + 0x0c);
		
	/* Set up data dma descriptor
	 * Note: Elements 0 is always assigned to IN pipe and element 1
	 *       is always assigned to OUT pipe.
	 */
	desc_index = is_in ? 0 : 1;
	descriptor_buf = &(gUdc2->ep[ep_index].dma_descriptor[desc_index]);

	descriptor_buf->virt_addr = gUdc2->ep_dma_descriptor_buf.virt_addr + 
								   (sizeof(struct bcm28xx_udc2_data_desc) * ((2*ep_index) + desc_index));
	descriptor_buf->dma_addr  = gUdc2->ep_dma_descriptor_buf.dma_addr + 
								   (sizeof(struct bcm28xx_udc2_data_desc) * ((2*ep_index) + desc_index));
	descriptor_buf->size	  = sizeof(struct bcm28xx_udc2_data_desc);

    dma_descriptor = (struct bcm28xx_udc2_data_desc *) descriptor_buf->virt_addr;
	dma_descriptor->quadlet = UDC2_QUADLET_HOST_BUSY;

	wmb();
	reg = (is_in ? BCM28XX_USBD0_EP0I_DESCPTR_PHY : BCM28XX_USBD0_EP0O_DESCPTR_PHY) + (0x20 * ep_index);
	bcm28xx_udc2_writel(descriptor_buf->dma_addr, reg);
}

#else

static void bcm28xx_udc2_ep_config(int ep_index, int is_in, u32 type, u32 buf_size, unsigned maxp)
{
	u32 reg, val;
	u32 desc_index;
	struct bcm28xx_udc2_data_desc	*dma_descriptor;
	struct bcm28xx_udc2_dma_buf 	*descriptor_buf;

	reg = (is_in ? BCM28XX_USBD0_EP0I_CTRL_PHY : BCM28XX_USBD0_EP0O_CTRL_PHY) + (0x20 * ep_index);

	/* Configure Control Register */
	val = (type << 4) | (is_in ? UDC_EP_CTRL_FLUSH : UDC_EP_CTRL_PRDY);
	bcm28xx_udc2_writel(val, reg);

	/* Configure Buffer Size */
	val = is_in ? (buf_size >> 2 /* 32-bit word count */) : 0;
	bcm28xx_udc2_writel(val, reg + 0x08);
	
	/* Configure maximum packet size */
	val = maxp & 0xffff;
	bcm28xx_udc2_writel(val, reg + 0x0c);
		
	/* DMA descriptor buffer assignment:
	 * Byte 0 - 511: EP0 in descriptor chain. (Up to 2KB in HS and FS).
	 * Byte 512 - 1023: EP0 out descriptor chain. (Up to 2KB in HS and FS).
	 * Byte 1024 - 2047: EP1 in descriptor chain. (Up to 4KB in HS and FS).
	 * Byte 2048 - 18431: EP2 in descriptor chian. (Up to 64KB in FS and 512 KB in HS).
	 * Byte 18432 - 34815: EP3 out descriptor chian. (Up to 64KB in FS and 512 KB in HS).
	 * Byte 34816 - 65535: reserved.	
	 */
	desc_index = is_in ? 0 : 1;
	descriptor_buf = &(gUdc2->ep[ep_index].dma_descriptor[desc_index]);

	switch (ep_index) {
		case 0:
			if (is_in) {
				descriptor_buf->virt_addr = gUdc2->ep_dma_descriptor_buf.virt_addr + 0;
				descriptor_buf->dma_addr  = gUdc2->ep_dma_descriptor_buf.dma_addr + 0;
				descriptor_buf->size	  = 512;
			} else {
				descriptor_buf->virt_addr = gUdc2->ep_dma_descriptor_buf.virt_addr + 512;
				descriptor_buf->dma_addr  = gUdc2->ep_dma_descriptor_buf.dma_addr + 512;
				descriptor_buf->size	  = 512;
			}
			break;
		
		case 1:
			descriptor_buf->virt_addr = gUdc2->ep_dma_descriptor_buf.virt_addr + 1024;
			descriptor_buf->dma_addr  = gUdc2->ep_dma_descriptor_buf.dma_addr + 1024;
			descriptor_buf->size	  = 1024;
			break;

		case 2:
			descriptor_buf->virt_addr = gUdc2->ep_dma_descriptor_buf.virt_addr + 2048;
			descriptor_buf->dma_addr  = gUdc2->ep_dma_descriptor_buf.dma_addr + 2048;
			descriptor_buf->size	  = 16384;
			break;

		case 3:
			descriptor_buf->virt_addr = gUdc2->ep_dma_descriptor_buf.virt_addr + 18432;
			descriptor_buf->dma_addr  = gUdc2->ep_dma_descriptor_buf.dma_addr + 18432;
			descriptor_buf->size	  = 16384;
			break;

		default:
			ERR("%s: EP-%d not supported yet\n", __FUNCTION__, ep_index);
			BUG();
			break;
	}


	dma_descriptor = (struct bcm28xx_udc2_data_desc *) descriptor_buf->virt_addr;
	dma_descriptor->quadlet = UDC2_QUADLET_HOST_BUSY;

	wmb();
	reg = (is_in ? BCM28XX_USBD0_EP0I_DESCPTR_PHY : BCM28XX_USBD0_EP0O_DESCPTR_PHY) + (0x20 * ep_index);
	bcm28xx_udc2_writel(descriptor_buf->dma_addr, reg);
}

#endif

static void bcm28xx_udc2_ep_setup(char *name, u32 addr, u32 type, unsigned maxp)
{
	struct bcm28xx_udc2_ep	*ep;
	int ep_index;
	u32 reg, val;
	int is_in;
	u32 buf_size; 

	ep_index = addr & 0xf;
	ep = &gUdc2->ep[ep_index];
	ep->bEndpointAddress = addr;

	is_in = addr & USB_DIR_IN;
	
	/* in case of ep init table bugs */
	BUG_ON(ep->name[0]);

	/* Default buffer size in Alt 0 */
	switch (type) 
	{
		case USB_ENDPOINT_XFER_CONTROL:
			buf_size = 64; /* in bytes */
#ifndef UDC2_BUFFER_FILL_MODE
			ep->dma_chain_len = 32; /* HS & FS */
#endif
			break;

		case USB_ENDPOINT_XFER_BULK:
			buf_size = is_in ? 512 : 1024; /* in bytes */
#ifndef UDC2_BUFFER_FILL_MODE
			ep->dma_chain_len = 128; /* HS */
#endif
			break;

		case USB_ENDPOINT_XFER_INT:
			buf_size = 64; /* in bytes */
#ifndef UDC2_BUFFER_FILL_MODE			
			ep->dma_chain_len = 64; /* HS & FS */
#endif
			break;

		default:
			BUG();
			break;
	}

	if (ep_index == 0)
	{
		struct bcm28xx_udc2_setup_desc	*setup_desc;

		bcm28xx_udc2_ep_config(0, 1, type, buf_size, maxp);
		bcm28xx_udc2_ep_config(0, 0, type, buf_size, maxp);

		/* Set SETUP Descriptor
		 */
		setup_desc = (struct bcm28xx_udc2_setup_desc *)gUdc2->SETUP.virt_addr;
		setup_desc->quadlet = UDC2_QUADLET_HOST_BUSY;
		wmb();
		bcm28xx_udc2_writel(gUdc2->SETUP.dma_addr, BCM28XX_USBD0_EP0O_SETUPPTR_PHY);
	}
	else
		bcm28xx_udc2_ep_config(ep_index, is_in, type, buf_size, maxp);


	/* Set up UDC register */
	reg = BCM28XX_USBD0_EP0CONFIG_PHY + (4 * ep_index);
	val = (ep_index & 0xf) | ((is_in ? 1 : 0) << UDC_EP_DIR_OFST)
		  | ((type & 0x3) << UDC_EP_TYPE_OFST) | (1 << UDC_CFG_NUM_OFST) 
		  | ((maxp & 0x3ff) << UDC_MAX_PKT_SIZE_OFST);
	wmb();
	bcm28xx_udc2_writel(val, reg);
	
	/* set up driver data structures */
	strlcpy(ep->name, name, sizeof(ep->name));
	INIT_LIST_HEAD(&ep->queue);
	ep->bEndpointAddress = addr;
	ep->bmAttributes = type;
	ep->udc2 = gUdc2; 
	ep->ep.name = ep->name;
	ep->ep.ops = &bcm28xx_udc2_ep_ops;
	ep->ep.maxpacket = ep->maxpacket = maxp;
	ep->in_state = IN_INIT;
	ep->stopped = 0;
	list_add_tail (&ep->ep.ep_list, &gUdc2->gadget.ep_list);
}


static void bcm28xx_udc2_release(struct device *dev)
{
	/* Mask out all the interrupts.
	 */
	bcm28xx_udc2_writel(~0, BCM28XX_USBD0_DEVINTMASK_PHY);
	bcm28xx_udc2_writel(~0, BCM28XX_USBD0_EPINTMASK_PHY);

	bcm28xx_udc2_free_dma_bufs();
	complete(gUdc2->done);
	kfree (gUdc2);
	gUdc2 = NULL;
}


static int __init
bcm28xx_udc2_gadget_setup(struct device *dev)
{
	gUdc2 = kmalloc (sizeof *gUdc2, GFP_KERNEL);
	if (!gUdc2)
		return -ENOMEM;

	memset(gUdc2, 0, sizeof *gUdc2);
	spin_lock_init (&gUdc2->lock);

	gUdc2->gadget.ops = &bcm28xx_udc2_gadget_ops;
	gUdc2->gadget.ep0 = &gUdc2->ep[0].ep;
	INIT_LIST_HEAD(&gUdc2->gadget.ep_list);
	gUdc2->gadget.speed = USB_SPEED_UNKNOWN;
	gUdc2->gadget.name = driver_name;
	device_initialize(&gUdc2->gadget.dev);
	dev_set_name(&(gUdc2->gadget.dev), "gadget");
	gUdc2->gadget.dev.release = bcm28xx_udc2_release;
	gUdc2->gadget.dev.parent = dev;
	gUdc2->gadget.dev.dma_mask = dev->dma_mask;
	gUdc2->gadget.is_dualspeed = 1;

	return 0;
}

void bcm28xx_reset_device(int dev_reset_id)
{
   u32 reset_ctrl;
   u32 reset_clear;

      reset_ctrl = readl(IO_ADDRESS(BCM28XX_RSTSR0_PHY));
      reset_ctrl |= 1 << dev_reset_id;
      writel(reset_ctrl, IO_ADDRESS(BCM28XX_RSTSR0_PHY));
      udelay(1);
  
       /* Restart */
       reset_clear = readl(IO_ADDRESS(BCM28XX_RSTSRC0_PHY));
       reset_clear |= 1 << dev_reset_id;
       writel(reset_clear, IO_ADDRESS(BCM28XX_RSTSRC0_PHY));
       udelay(1);
}
  
static void __init bcm28xx_udc2_reset(void)
{
	/* 
	 * Switch the USB core, UTMI PHY to device mode. 
	 */
	bcm28xx_udc2_modifyl(BCM28XX_CHIPMGR_USB_Ctrl_PHY, BCM28XX_CM_USB_HOST_MODE, 0);

	/* 
	 *	Reset USB core
	 */
    /* Resetting the USB core will reset its PHY as well, 
     * which will turn the PHY on. We deselect the PHY again,
     * and wait till the class driver ready then select it. 
     */
	bcm28xx_reset_device(USB_RESET_ID);
	bcm28xx_udc2_writel(USB_PHY_DESELECT_VAL, BCM28XX_USBH0_UTMI_CTRL1_PHY);

	/* 
	 *	Reset and restart USB PHY
	 */
	bcm28xx_udc2_writel(USB_PHY_RESET_VAL, BCM28XX_USBH0_PHY_CTRL_PHY);
	bcm28xx_udc2_modifyl(BCM28XX_CHIPMGR_USB_Ctrl_PHY, BCM28XX_CM_USB_PHY_RESETB_I, BCM28XX_CM_USB_PHY_RESET_PLL);
	bcm28xx_udc2_modifyl(BCM28XX_CHIPMGR_USB_Ctrl_PHY, BCM28XX_CM_USB_PHY_RESET_PLL, 0);
	bcm28xx_udc2_modifyl(BCM28XX_CHIPMGR_USB_Ctrl_PHY, 0, BCM28XX_CM_USB_PHY_RESETB_I);	
	bcm28xx_udc2_writel(USB_PHY_ENABLE_VAL, BCM28XX_USBH0_PHY_CTRL_PHY);
}


static int __init 
bcm28xx_udc2_config(void)
{

	/* Mask out all the interrupts first
	 */
	bcm28xx_udc2_writel(~0, BCM28XX_USBD0_DEVINTMASK_PHY);
	bcm28xx_udc2_writel(~0, BCM28XX_USBD0_EPINTMASK_PHY);

	/* Clear all interrupt bits */
	bcm28xx_udc2_writel(~0, BCM28XX_USBD0_DEVINTR_PHY);
	bcm28xx_udc2_writel(~0, BCM28XX_USBD0_EPINTR_PHY);	

	/* Respond STALL when received a Clear_Feature for EP0, 
	 * Self-Powered mode, 
	 * UTMI PHY has 8 bit interface,
	 * Support dynamic CSR programming. 
	 */
	bcm28xx_udc2_writel(UDC_DEV_CFG_HLT_STATUS | UDC_DEV_CFG_PI | UDC_DEV_CFG_SP | UDC_DEV_CFG_CSR_PRG, 
		   				BCM28XX_USBD0_DEVCONFIG_PHY); 

		/* Setup and configure endpoints 
		 * Note: Max packet size is configured to high speed size first. Will
		 * be reconfigured based on the real speed detected. 
		 */
		 
	/* ep0 is special. It has both IN and OUT channels */
	bcm28xx_udc2_ep_setup("ep0", 0, USB_ENDPOINT_XFER_CONTROL, 64);
	list_del_init(&gUdc2->ep[0].ep.ep_list);

#define BCM28XX_BULK_EP(name,addr) \
	bcm28xx_udc2_ep_setup(name "-bulk", addr, USB_ENDPOINT_XFER_BULK, 512);  
#define BCM28XX_INT_EP(name,addr) \
	bcm28xx_udc2_ep_setup(name "-int", addr, USB_ENDPOINT_XFER_INT, 64); 

	BCM28XX_INT_EP("ep1in", USB_DIR_IN | 1);
	BCM28XX_BULK_EP("ep2in", USB_DIR_IN | 2);
	BCM28XX_BULK_EP("ep3out", USB_DIR_OUT | 3);

#ifdef UDC2_BUFFER_FILL_MODE
	/* Set up device control reigster. 
	 * Disable Threshold. Set to little endian mode. 
	 * Enable and set burst length, dictate the operation in DMA mode,
	 * enable Buffer Fill mode, enable Tx DMA, enable Rx DMA. 
	 */
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, ~0, 
						 ((dma_burst_len-1) << UDC_DEV_CTRL_BRLEN_POS) | UDC_DEV_CTRL_BREN 
						 | UDC_DEV_CTRL_MODE | UDC_DEV_CTRL_BF
						 | UDC_DEV_CTRL_TDE | UDC_DEV_CTRL_RDE);
#else
	/* Buffer-per-packet mode */
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVCTRL_PHY, ~0, 
						 ((dma_burst_len-1) << UDC_DEV_CTRL_BRLEN_POS) | UDC_DEV_CTRL_BREN 
						 | UDC_DEV_CTRL_MODE
						 | UDC_DEV_CTRL_TDE | UDC_DEV_CTRL_RDE);
#endif

	/* Enable EP0. This includes enabling control packets related
	 * interrupts, including EP0-IN, EP0-OUT, Set Configuration
	 * and Set Interface interrupts. Set the DMA descriptor 
	 * quadlet to ready. 
	 */
	ep0_enable();

	/* Enable speed enumeration completion interrupt, 
	 * enable suspend interrupt, enable 3 ms idle interrupt, 
	 * enable reset interrupt. 
	 * Keep SOF interrupt disabled. Keep 3ms idle interrupt disable.  
	 */
	bcm28xx_udc2_modifyl(BCM28XX_USBD0_DEVINTMASK_PHY, 
						 UDC_DEV_INT_ENUM | UDC_DEV_INT_US | UDC_DEV_INT_UR, 
						 0);
	return 0;
}



static int __init
bcm28xx_udc2_setup(struct device *dev)
{
	int status;
	unsigned long flags;


	/* Setup for registering with Linux gadget framework. 
	 */
	status = bcm28xx_udc2_gadget_setup(dev);
	if(status) {
		INFO(" failed to register with gadget framework, status 0x%x", status);
		return status;
	}

#ifdef UDC2_BUFFER_FILL_MODE
	/* Allocate DMA buffer for each endpoint to build
	 * its DMA desriptor. 
	 */
	gUdc2->ep_dma_descriptor_buf.virt_addr = 
		dma_alloc_coherent(gUdc2->gadget.dev.parent,
			 PAGE_SIZE, &gUdc2->ep_dma_descriptor_buf.dma_addr, GFP_KERNEL);
	if (!gUdc2->ep_dma_descriptor_buf.virt_addr) {
		INFO(" failed to allocate EP DMA descriptor buffer\n");
		return -ENOMEM;
	}
	else {

		gUdc2->ep_dma_descriptor_buf.size = PAGE_SIZE;
	}
#else
	/* Allocate DMA buffer for each endpoint to build
	 * its DMA descriptor. 
	 */

	/* DMA descriptor buffer assignment:
	 * Byte 0 - 511: EP0 in descriptor chain. (Up to 2KB in HS and FS).
	 * Byte 512 - 1023: EP0 out descriptor chain. (Up to 2KB in HS and FS).
	 * Byte 1024 - 2047: EP1 in descriptor chain. (Up to 4KB in HS and FS).
	 * Byte 2048 - 18431: EP2 in descriptor chian. (Up to 64KB in FS and 512 KB in HS).
	 * Byte 18432 - 34815: EP3 out descriptor chian. (Up to 64KB in FS and 512 KB in HS).
	 * Byte 34816 - 65535: reserved.	
	 */
	gUdc2->ep_dma_descriptor_buf.virt_addr = 
		dma_alloc_coherent(gUdc2->gadget.dev.parent,
			 64 * 1024, &gUdc2->ep_dma_descriptor_buf.dma_addr, GFP_KERNEL);
	if (!gUdc2->ep_dma_descriptor_buf.virt_addr) {
		ERR(" failed to allocate EP DMA descriptor buffer\n");
		return -ENOMEM;
	}
	else {

		gUdc2->ep_dma_descriptor_buf.size = 64 * 1024;
	}
#endif
		
	/* Allocate DMA buffer for EP0 SETUP data. 
	 * 4KB (PAGE_SIZE) is more than what we need, but
	 * it is the minimum unit by doing dma_alloc_coherent(). 
	 */
	gUdc2->SETUP.virt_addr = 
		dma_alloc_coherent(gUdc2->gadget.dev.parent,
			 PAGE_SIZE, &gUdc2->SETUP.dma_addr, GFP_KERNEL);
	if (!gUdc2->SETUP.virt_addr) {
		bcm28xx_udc2_free_dma_bufs();
		ERR(" failed to allocate SETUP buffer\n");
		return -ENOMEM;
	}
	else {

		gUdc2->SETUP.size = PAGE_SIZE;
	}

	/* Allocate DMA buffer for EP0 in status data. 
	 * 4KB (PAGE_SIZE) is more than what we need, but
	 * it is the minimum unit by doing dma_alloc_coherent(). 
	 */
	gUdc2->SETUP_in_status.virt_addr = 
		dma_alloc_coherent(gUdc2->gadget.dev.parent,
			 PAGE_SIZE, &gUdc2->SETUP_in_status.dma_addr, GFP_KERNEL);
	if (!gUdc2->SETUP_in_status.virt_addr) {
		bcm28xx_udc2_free_dma_bufs();
		ERR(" failed to allocate SETUP in_status buffer\n");
		return -ENOMEM;
	}
	else {

		gUdc2->SETUP_in_status.size = PAGE_SIZE;
	}

	/* Allocate DMA buffer for EP0 out status data. 
	 * 4KB (PAGE_SIZE) is more than what we need, but
	 * it is the minimum unit by doing dma_alloc_coherent(). 
	 */
	gUdc2->SETUP_out_status.virt_addr = 
		dma_alloc_coherent(gUdc2->gadget.dev.parent,
			 PAGE_SIZE, &gUdc2->SETUP_out_status.dma_addr, GFP_KERNEL);
	if (!gUdc2->SETUP_out_status.virt_addr) {
		bcm28xx_udc2_free_dma_bufs();
		ERR(" failed to allocate SETUP out_status buffer\n");
		return -ENOMEM;
	}
	else {

		gUdc2->SETUP_out_status.size = PAGE_SIZE;
	}

	/* Disable interrupt to make sure device init 
	 * won't be interrupted. 
	 */
	spin_lock_irqsave(&gUdc2->lock, flags);

	bcm28xx_udc2_reset();
	bcm28xx_udc2_config();
	
	spin_unlock_irqrestore(&gUdc2->lock, flags);
	return 0;
}



/*-------------------------------------------------------------------------
 *
 *					Linux proc file system functions
 * 
 *-------------------------------------------------------------------------*/

#ifdef CONFIG_USB_GADGET_DEBUG_FILES

#include <linux/seq_file.h>

static const char proc_filename[] = "driver/udc2";

static int proc_udc2_show(struct seq_file *s, void *_)
{
	return 0;
}

static int proc_udc2_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_udc2_show, NULL);
}

static struct file_operations proc_ops = {
	.open		= proc_udc2_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_proc_file(void)
{
	struct proc_dir_entry *pde;

	pde = create_proc_entry (proc_filename, 0, NULL);
	if (pde)
		pde->proc_fops = &proc_ops;
}

static void remove_proc_file(void)
{
	remove_proc_entry(proc_filename, NULL);
}

#else

static inline void create_proc_file(void) {}
static inline void remove_proc_file(void) {}

#endif


/*-------------------------------------------------------------------------
 *
 *						Linux driver model functions
 * 
 *-------------------------------------------------------------------------*/

static int bcm28xx_udc2_probe(struct device *dev)
{
	int			status = -ENODEV;

	/* A "gadget" abstracts/virtualizes the controller */
	status = bcm28xx_udc2_setup(dev);
	if (status) {
		ERR(" failed to set up udc, status 0x%x", status);
		goto cleanup0;
	}

	/* Register IRQ */
	status = request_irq(IRQ_USB, bcm28xx_udc2_irq, 0, "BCM28XX_UDC2", (void *)gUdc2);
	if (status != 0) {
		ERR( "can't get irq %d, err %d\n",
			IRQ_USB, status);
		goto cleanup1;
	}
	
	create_proc_file();
	device_add(&gUdc2->gadget.dev);
	return 0;

cleanup1:
	bcm28xx_udc2_free_dma_bufs();
	kfree (gUdc2);
	gUdc2 = NULL;

cleanup0:
	return status;
}

static int __exit bcm28xx_udc2_remove(struct device *dev)
{
	DECLARE_COMPLETION(done);

	if (!gUdc2)
		return -ENODEV;

	gUdc2->done = &done;

	remove_proc_file();

	free_irq(IRQ_USB, gUdc2);

	device_unregister(&gUdc2->gadget.dev);
	wait_for_completion(&done);

	return 0;
}


static struct device_driver bcm28xx_udc2_driver = {
	.name		= (char *) driver_name,
	.bus		= &platform_bus_type,
	.probe		= bcm28xx_udc2_probe,
	.remove		= __exit_p(bcm28xx_udc2_remove),
};

static int __init bcm28xx_udc2_init(void)
{
	INFO("%s version: %s\n", driver_desc, DRIVER_VERSION);

    udc_logger_init();
    
	return driver_register(&bcm28xx_udc2_driver);
}

static void __exit bcm28xx_udc2_exit(void)
{
	bcm28xx_udc2_phy_powerdown();
	driver_unregister(&bcm28xx_udc2_driver);	
}


module_init(bcm28xx_udc2_init);
module_exit(bcm28xx_udc2_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
                                 
