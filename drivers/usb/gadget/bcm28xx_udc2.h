/*
 * linux/drivers/usb/gadget/bcm28xx_udc2.h
 * Broadcom BCM28xx on-chip high/full speed USB device controllers
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef _BCM28XX_UDC2_H
#define _BCM28XX_UDC2_H

#undef UDC2_BUFFER_FILL_MODE	

#define UDC2_QUADLET_LAST		(1 << 27)
#define UDC2_QUADLET_HOST_BUSY	0xc0000000
#define UDC2_QUADLET_HOST_READY	0x00000000

struct bcm28xx_udc2_data_desc {
	volatile __u32 quadlet;
	volatile __u32 r;
	volatile __u32 buf_ptr;
	volatile __u32 next_ptr;
} __attribute__ ((packed));

struct bcm28xx_udc2_setup_desc {
	volatile __u32 quadlet;
	volatile __u32 r;
	volatile __u32 first_data;
	volatile __u32 secnond_data;
} __attribute__ ((packed));


#define BCM28XX_UDC2_NUM_DATA_EP	8

#ifdef UDC2_BUFFER_FILL_MODE
#define BCM28XX_UDC2_MAX_TRANSFER_SIZE	(32 * 1024)
#endif
/*-------------------------------------------------------------------------*/


struct bcm28xx_udc2_req {
	struct usb_request		req;
	struct list_head		queue;
	unsigned				dma_bytes;
	unsigned				mapped:1;
};

struct bcm28xx_udc2_dma_buf {
	u32 		size;
	void 		*virt_addr;
	dma_addr_t	dma_addr;
};

typedef enum _bcm28xx_udc2_state {
	INIT,
	RESET,
	ADDRESS,
	CONFIGURED,
	POWERED,
	SUSPENDED
} bcm28xx_udc2_state;

typedef enum _bcm28xx_udc2_ep0_state {
	EP0_IDLE,
	EP0_SETUP,
	EP0_IN_DATA,
	EP0_OUT_DATA,
	EP0_IN_STATUS,
	EP0_OUT_STATUS,
	EP0_STOPPED
} bcm28xx_udc2_ep0_state;

typedef enum _bcm28xx_udc2_in_state {
	IN_INIT,
	IN_READY,
	IN_POLLDEMAND,
	IN_TDC
} bcm28xx_udc2_in_state;


struct bcm28xx_udc2_ep {
	struct usb_ep				ep;
	struct list_head			queue;
	const struct usb_endpoint_descriptor	*desc;
	char						name[32];
	u8							bEndpointAddress;
	u8							bmAttributes;
	u16							maxpacket;
	unsigned					stopped:1;	
	bcm28xx_udc2_in_state		in_state; /* used only by IN endpoints */
#ifndef UDC2_BUFFER_FILL_MODE
	int							dma_chain_len;
#endif
	struct bcm28xx_udc2_dma_buf dma_descriptor[2];
	struct bcm28xx_udc2			*udc2;
};


struct bcm28xx_udc2 {
	struct usb_gadget			gadget;
	struct usb_gadget_driver	*driver;
	spinlock_t					lock;
	struct completion			*done;
	struct bcm28xx_udc2_ep		ep[BCM28XX_UDC2_NUM_DATA_EP];
	struct bcm28xx_udc2_dma_buf ep_dma_descriptor_buf;
	struct bcm28xx_udc2_dma_buf	SETUP;
	struct bcm28xx_udc2_dma_buf	SETUP_in_status;
	struct bcm28xx_udc2_dma_buf	SETUP_out_status;
	unsigned 					ep0_setup:1;   
	unsigned					ep0_pending:1;
	bcm28xx_udc2_ep0_state		ep0_current_state;
	bcm28xx_udc2_ep0_state		ep0_next_state;
	unsigned					ep0_in:1;
	unsigned					ep0_set_config:1;
	unsigned					ep0_set_intf:1;
};

/*-------------------------------------------------------------------------*/

/* UDC regusters */

#define    BCM28XX_USBD_ADDRBASE0 0x0c070000
#define    BCM28XX_CHIPMGR_ARM_ADDRBASE0 0x0c0cf000
#define    BCM28XX_CHIPMGR_USB_Ctrl_PHY ((BCM28XX_CHIPMGR_ARM_ADDRBASE0 + 0x020))
#define    BCM28XX_USBD0_EP0I_CTRL_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x000))
#define    BCM28XX_USBD0_EP0I_STAT_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x004))
#define    BCM28XX_USBD0_EP0I_MAXPKTSIZE_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x00C))
#define    BCM28XX_USBD0_EP0I_DESCPTR_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x014))
#define    BCM28XX_USBD0_EP0O_CTRL_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x200))
#define    BCM28XX_USBD0_EP0O_STAT_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x204))
#define    BCM28XX_USBD0_EP0O_SETUPPTR_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x210))
#define    BCM28XX_USBD0_EP0O_DESCPTR_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x214))
#define    BCM28XX_USBD0_EP0O_MAXPKTSIZE_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x20C))

#define    BCM28XX_USBD0_DEVCONFIG_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x400))
#define    BCM28XX_USBD0_DEVCTRL_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x404))
#define    BCM28XX_USBD0_DEVSTAT_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x408))
#define    BCM28XX_USBD0_DEVINTMASK_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x410))
#define    BCM28XX_USBD0_EPINTR_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x414))
#define    BCM28XX_USBD0_EPINTMASK_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x418))
#define    BCM28XX_USBD0_DEVINTR_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x40C))
#define    BCM28XX_USBD0_EP0CONFIG_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x504))

#define    BCM28XX_USBH0_UTMI_CTRL1_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x910))
#define    BCM28XX_USBH0_PHY_CTRL_PHY ((BCM28XX_USBD_ADDRBASE0 + 0x91C))

/*Clock manager registers */

#define    BCM28XX_CLKMGR_ARM_ADDRBASE0 0x0c0d1000
#define    BCM28XX_RSTSR0_PHY ((BCM28XX_CLKMGR_ARM_ADDRBASE0 + 0x0108))
#define    BCM28XX_RSTSRC0_PHY ((BCM28XX_CLKMGR_ARM_ADDRBASE0 + 0x0110))


/* 
 * Register bits definitions
 */

	/* 
	 * BCM28XX Chip Manager USB Control Registers 
	 */
#define BCM28XX_CM_USB_HOST_MODE		(1 << 19)
#define BCM28XX_CM_USB_PHY_RESET_PLL	(1 << 17)
#define BCM28XX_CM_USB_PHY_RESETB_I		(1 << 16)

	/*
	 * USB Device Controller Registers 
	 */

	/* EP Control Register */
#define UDC_EP_CTRL_STALL		(1 << 0)
#define UDC_EP_CTRL_FLUSH		(1 << 1)
#define UDC_EP_CTRL_SNOOP		(1 << 2)
#define UDC_EP_CTRL_POLL		(1 << 3)
#define UDC_EP_CTRL_ET			(1 << 4)
#define UDC_EP_CTRL_NAK			(1 << 6)
#define UDC_EP_CTRL_SNAK		(1 << 7)
#define UDC_EP_CTRL_CNAK		(1 << 8)
#define UDC_EP_CTRL_PRDY		(1 << 9)
#define UDC_EP_CTRL_SNDNULL		(1 << 10)
#define UDC_EP_CTRL_CLSDESC		(1 << 11)

	/* EP Status Register */
#define UDC_EP_STATUS_OUT_OFST	4
  #define UDC_EP_STATUS_OUT_MASK	0x03
  #define UDC_EP_STATUS_OUT_DATA	0x01
  #define UDC_EP_STATUS_OUT_SETUP	0x02
#define UDC_EP_STATUS_IN		(1 << 6)
#define UDC_EP_STATUS_BNA		(1 << 7)
#define UDC_EP_STATUS_HE		(1 << 9)
#define UDC_EP_STATUS_TDC		(1 << 10)
#define UDC_EP_STATUS_OUT		(1 << 4)

	/* EP Buffer Size Register */
#define UDC_EP_MAXPKT_SIZE_OFST	0
#define UDC_EP_BUF_SIZE_OFST	16

	/* Device Configuration Register */
#define UDC_DEV_CFG_SPD			(1 << 0)
#define UDC_DEV_CFG_RWKP		(1 << 2)
#define UDC_DEV_CFG_SP			(1 << 3)
#define UDC_DEV_CFG_SS			(1 << 4)
#define UDC_DEV_CFG_PI			(1 << 5)
#define UDC_DEV_CFG_DIR			(1 << 6)
#define UDC_DEV_CFG_STATUS		(1 << 7)
#define UDC_DEV_CFG_STATUS_1	(1 << 8)
#define UDC_DEV_CFG_PHY_ERR		(1 << 9)
#define UDC_DEV_CFG_HLT_STATUS	(1 << 16)
#define UDC_DEV_CFG_CSR_PRG		(1 << 17)
#define UDC_DEV_CFG_SET_DESC	(1 << 18)
#define UDC_DEV_CFG_DDR			(1 << 19)

	/* Device Control Register */
#define UDC_DEV_CTRL_RES		(1 << 0)
#define UDC_DEV_CTRL_RDE		(1 << 2)
#define UDC_DEV_CTRL_TDE		(1 << 3)
#define UDC_DEV_CTRL_DU			(1 << 4)
#define UDC_DEV_CTRL_BE			(1 << 5)
#define UDC_DEV_CTRL_BF			(1 << 6)
#define UDC_DEV_CTRL_THE		(1 << 7)
#define UDC_DEV_CTRL_BREN		(1 << 8)
#define UDC_DEV_CTRL_MODE		(1 << 9)
#define UDC_DEV_CTRL_SD			(1 << 10)
#define UDC_DEV_CTRL_SCALE		(1 << 11)
#define UDC_DEV_CTRL_DEVNAK		(1 << 12)
#define UDC_DEV_CTRL_CSRDONE	(1 << 13)
#define UDC_DEV_CTRL_DU			(1 << 4)
#define UDC_DEV_CTRL_BRLEN_POS	(16)
#define UDC_DEV_CTRL_THLEN		(0x0 << 24)

	/* Device Status Register */
#define UDC_DEV_STAT_RXFIFO_EMPTY (0x1 << 15)    
#define UDC_DEV_STAT_SPD_OFST   13
    
#define UDC_DEV_STAT_SPD_MASK   0x3
#define UDC_DEV_STAT_HIGH_SPD   0x0
#define UDC_DEV_STAT_FULL_SPD   0x1
#define UDC_DEV_STAT_LOW_SPD    0x2
#define UDC_DEV_STAT_FULL_SPD_1 0x3

#define UDC_DEV_STAT_INTF_OFST   4
#define UDC_DEV_STAT_ALT_OFST    8

	/* Device Interrupt (Mask) Register */
#define UDC_DEV_INT_SC			(1 << 0)
#define UDC_DEV_INT_SI			(1 << 1)
#define UDC_DEV_INT_ES			(1 << 2)
#define UDC_DEV_INT_UR			(1 << 3)
#define UDC_DEV_INT_US			(1 << 4)
#define UDC_DEV_INT_SOF			(1 << 5)
#define UDC_DEV_INT_ENUM		(1 << 6)

	/* Endpoint Interrupt (Mask) Register */
#define UDC_EP_0_IN_INT			(1 << 0)
#define UDC_EP_1_IN_INT			(1 << 1)
#define UDC_EP_2_IN_INT			(1 << 2)
#define UDC_EP_3_IN_INT			(1 << 3)
#define UDC_EP_0_OUT_INT		(1 << 16)
#define UDC_EP_1_OUT_INT		(1 << 17)
#define UDC_EP_2_OUT_INT		(1 << 18)
#define UDC_EP_3_OUT_INT		(1 << 19)

	/* UDC Registers (0x5xx) */
#define UDC_EP_DIR_OFST			4
#define UDC_EP_TYPE_OFST		5
#define UDC_CFG_NUM_OFST		7
#define UDC_INTF_NUM_OFST		11
#define UDC_ALT_SETTING_OFST	15
#define UDC_MAX_PKT_SIZE_OFST	19

/* Values for USB UTMI/PHY registers */
#define USB_PHY_RESET_VAL		0x3c
#define USB_PHY_ENABLE_VAL		0x3f
#define USB_PHY_POWER_DOWN_VAL	0x0
#define USB_PHY_DESELECT_VAL	0x1300

/*-------------------------------------------------------------------------*/

#ifdef DEBUG
#define DBG(stuff...)		printk(KERN_DEBUG "bcm28xx_udc2: " stuff)
#else
#define DBG(stuff...)		do{}while(0)
#endif

#ifdef VERBOSE
#    define VDBG		DBG
#else
#    define VDBG(stuff...)	do{}while(0)
#endif

#define ERR(stuff...)		printk(KERN_ERR "bcm28xx_udc2: " stuff)
#define WARN(stuff...)		printk(KERN_WARNING "bcm28xx_udc2: " stuff)
#define INFO(stuff...)		printk(KERN_INFO "bcm28xx_udc2: " stuff)


#endif /* _BCM28XX_UDC2_H */
