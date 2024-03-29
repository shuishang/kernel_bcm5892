/* ==========================================================================
 * $File: /*dwh/usb_iip/dev/software/otg_ipmate/linux/drivers/dwc_otg_pcd.c $
 * $Revision: #18 $
 * $Date: 2007/02/07 $
 * $Change: 791271 $
 *
 * Synopsys HS OTG Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 * 
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 * 
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================== */

/*****************************************************************************  
 *  Portions Copyright 2008 Broadcom Corporation.
 *****************************************************************************/ 


#ifndef DWC_HOST_ONLY

/** @file 
 * This file implements the Peripheral Controller Driver.
 *
 * The Peripheral Controller Driver (PCD) is responsible for
 * translating requests from the Function Driver into the appropriate
 * actions on the DWC_otg controller. It isolates the Function Driver
 * from the specifics of the controller by providing an API to the
 * Function Driver. 
 *
 * The Peripheral Controller Driver for Linux will implement the
 * Gadget API, so that the existing Gadget drivers can be used.
 * (Gadget Driver is the Linux terminology for a Function Driver.)
 * 
 * The Linux Gadget API is defined in the header file
 * <code><linux/usb_gadget.h></code>.  The USB EP operations API is
 * defined in the structure <code>usb_ep_ops</code> and the USB
 * Controller API is defined in the structure
 * <code>usb_gadget_ops</code>.
 *
 * An important function of the PCD is managing interrupts generated
 * by the DWC_otg controller. The implementation of the DWC_otg device
 * mode interrupt service routines is in dwc_otg_pcd_intr.c.
 *
 * @todo Add Device Mode test modes (Test J mode, Test K mode, etc).
 * @todo Does it work when the request size is greater than DEPTSIZ
 * transfer size
 *
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/dma-mapping.h>
#include <mach/lm.h>
#include <mach/irqs.h>
#include <asm/arch/reg_sys.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#include <linux/usb_ch9.h>
#else
#include <linux/usb/ch9.h>
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
#include <linux/usb_gadget.h>
#else
#include <linux/usb/gadget.h>
#endif

#include "dwc_otg_driver.h"
#include "dwc_otg_pcd.h"



/**
 * Static PCD pointer for use in usb_gadget_register_driver and
 * usb_gadget_unregister_driver.  Initialized in dwc_otg_pcd_init.
 */
static	 dwc_otg_pcd_t *s_pcd = 0;


/* Display the contents of the buffer */
extern void dump_msg(const u8 *buf, unsigned int length);

void dump_buffer(void *buffer, uint32_t size, char * header);

/**
 * This function completes a request.  It call's the request call back.
 */
void request_done(dwc_otg_pcd_ep_t *_ep, dwc_otg_pcd_request_t *_req, 
				  int _status)
{
	unsigned stopped = _ep->stopped;
    dwc_otg_pcd_t	*pcd;

	DWC_DEBUGPL(DBG_PCDV|DBG_USRV, "(%p)\n", _ep);
	list_del_init(&_req->queue);

	if (_req->req.status == -EINPROGRESS) 
	{
		_req->req.status = _status;
	}
	else
	{
		_status = _req->req.status;
	}

	/* don't modify queue heads during completion callback */
	_ep->stopped = 1;

    pcd = _ep->pcd;

    if (GET_CORE_IF(pcd)->dma_enable)
    {      
        if(_req->relocated)
        {    
            DWC_DEBUGPL(DBG_PCD, "cp back EP %s, L2 Buf (%08x), local buf = %8x, size=%d\n",
                        _ep->ep.name, (int)_req->req.buf, (int)_req->new_buf, _req->req.length);
    
            memcpy(_req->req.buf, _req->new_buf, _req->req.length);
            _req->relocated = 0;
            _req->req.dma = DMA_ADDR_INVALID;
            wmb();
    
            dma_free_coherent (NULL, _req->req.length, _req->new_buf, _req->req.dma);
        }
        else
        {    
            if(_req->mapped)
        	{
                dma_unmap_single(NULL, _req->req.dma, _req->req.length,
                                 DMA_BIDIRECTIONAL);
        
                _req->req.dma = DMA_ADDR_INVALID;
                _req->mapped = 0;    
            }
            else
                dma_sync_single_for_cpu(NULL,
                                        _req->req.dma, _req->req.length,
                                        DMA_BIDIRECTIONAL);
        }
    }

	SPIN_UNLOCK(&_ep->pcd->lock);
	_req->req.complete(&_ep->ep, &_req->req);
	SPIN_LOCK(&_ep->pcd->lock);

	if (_ep->pcd->request_pending > 0)
	{
		--_ep->pcd->request_pending;
	}

	_ep->stopped = stopped;
}

/**
 * This function terminates all the requsts in the EP request queue.
 */
void request_nuke( dwc_otg_pcd_ep_t *_ep )
{
	dwc_otg_pcd_request_t *req;

	_ep->stopped = 1;

	/* called with irqs blocked?? */
	while (!list_empty(&_ep->queue)) 
	{
		req = list_entry(_ep->queue.next, dwc_otg_pcd_request_t,
				 queue);
		request_done(_ep, req, -ESHUTDOWN );
	}
}

/* USB Endpoint Operations */
/* 
 * The following sections briefly describe the behavior of the Gadget
 * API endpoint operations implemented in the DWC_otg driver
 * software. Detailed descriptions of the generic behavior of each of
 * these functions can be found in the Linux header file
 * include/linux/usb_gadget.h.
 *
 * The Gadget API provides wrapper functions for each of the function
 * pointers defined in usb_ep_ops. The Gadget Driver calls the wrapper
 * function, which then calls the underlying PCD function. The
 * following sections are named according to the wrapper
 * functions. Within each section, the corresponding DWC_otg PCD
 * function name is specified.
 *
 */

/**
 * This function assigns periodic Tx FIFO to an periodic EP
 * in shared Tx FIFO mode
 */
static uint32_t assign_perio_tx_fifo(dwc_otg_core_if_t	*core_if)
{
	uint32_t PerTxMsk = 1;
	int i;
	for(i = 0; i < core_if->hwcfg4.b.num_dev_perio_in_ep; ++i)
	{
		if((PerTxMsk & core_if->p_tx_msk) == 0)
		{
			core_if->p_tx_msk |= PerTxMsk;
			return i + 1;
		}
		PerTxMsk <<= 1;
	}
	return 0;
}
/**
 * This function releases periodic Tx FIFO 
 * in shared Tx FIFO mode
 */
static void release_perio_tx_fifo(dwc_otg_core_if_t *core_if, uint32_t fifo_num)
{
	core_if->p_tx_msk = (core_if->p_tx_msk & (1 << (fifo_num - 1))) ^ core_if->p_tx_msk;
}
/**
 * This function assigns periodic Tx FIFO to an periodic EP
 * in shared Tx FIFO mode
 */
static uint32_t assign_tx_fifo(dwc_otg_core_if_t *core_if)
{
	uint32_t TxMsk = 1;
	int i;

	for(i = 0; i < core_if->hwcfg4.b.num_in_eps; ++i)
	{
		if((TxMsk & core_if->tx_msk) == 0)
		{
			core_if->tx_msk |= TxMsk;
			return i + 1;
		}
		TxMsk <<= 1;
	}
	return 0;
}
/**
 * This function releases periodic Tx FIFO 
 * in shared Tx FIFO mode
 */
static void release_tx_fifo(dwc_otg_core_if_t	*core_if, uint32_t fifo_num)
{
	core_if->tx_msk = (core_if->tx_msk & (1 << (fifo_num - 1))) ^ core_if->tx_msk;
}
/**
 * This function is called by the Gadget Driver for each EP to be
 * configured for the current configuration (SET_CONFIGURATION).  
 * 
 * This function initializes the dwc_otg_ep_t data structure, and then
 * calls dwc_otg_ep_activate.
 */
static int dwc_otg_pcd_ep_enable(struct usb_ep *_ep, 
								 const struct usb_endpoint_descriptor *_desc)
{
	dwc_otg_pcd_ep_t *ep = 0;
	dwc_otg_pcd_t *pcd = 0;
	unsigned long flags;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p,%p)\n", __func__, _ep, _desc );

	ep = container_of(_ep, dwc_otg_pcd_ep_t, ep);
	if (!_ep || !_desc || ep->desc || 
			_desc->bDescriptorType != USB_DT_ENDPOINT) 
	{
		DWC_WARN( "%s, bad ep or descriptor\n", __func__);
		return -EINVAL;
	}
	if (ep == &ep->pcd->ep0)
	{
		DWC_WARN("%s, bad ep(0)\n", __func__);
		return -EINVAL;
	}
		
	/* Check FIFO size? */
	if (!_desc->wMaxPacketSize) 
	{
		DWC_WARN("%s, bad %s maxpacket\n", __func__, _ep->name);
		return -ERANGE;
	}

	pcd = ep->pcd;
	if (!pcd->driver || pcd->gadget.speed == USB_SPEED_UNKNOWN) 
	{
		DWC_WARN("%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

	SPIN_LOCK_IRQSAVE(&pcd->lock, flags);
		
	ep->desc = _desc;
	ep->ep.maxpacket = le16_to_cpu (_desc->wMaxPacketSize);
		
	/*
	 * Activate the EP
	 */
	ep->stopped = 0;
		
	ep->dwc_ep.is_in = (USB_DIR_IN & _desc->bEndpointAddress) != 0;
	ep->dwc_ep.maxpacket = ep->ep.maxpacket;
	
	ep->dwc_ep.type = _desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

	if(ep->dwc_ep.is_in)
	{
		if(!pcd->otg_dev->core_if->en_multiple_tx_fifo)
		{
			ep->dwc_ep.tx_fifo_num = 0;
		
			if ((_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == 
				USB_ENDPOINT_XFER_ISOC ) 
			{
				/* 
				 * if ISOC EP then assign a Periodic Tx FIFO.
				 */
				ep->dwc_ep.tx_fifo_num = assign_perio_tx_fifo(pcd->otg_dev->core_if);
			 }
		}
		else
		{
			/* 
			 * if Dedicated FIFOs mode is on then assign a Tx FIFO.
			 */
			ep->dwc_ep.tx_fifo_num = assign_tx_fifo(pcd->otg_dev->core_if);
		}
	}		 
	/* Set initial data PID. */
	if ((_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == 
			USB_ENDPOINT_XFER_BULK ) 
	{
		ep->dwc_ep.data_pid_start = 0;	
	}
		
	DWC_DEBUGPL(DBG_PCD, "Activate %s-%s: type=%d, mps=%d desc=%p\n", 
					ep->ep.name, (ep->dwc_ep.is_in ?"IN":"OUT"),
					ep->dwc_ep.type, ep->dwc_ep.maxpacket, ep->desc );
		
	dwc_otg_ep_activate( GET_CORE_IF(pcd), &ep->dwc_ep );
	SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);
	return 0;
}

/** 
 * This function is called when an EP is disabled due to disconnect or
 * change in configuration. Any pending requests will terminate with a
 * status of -ESHUTDOWN.
 *
 * This function modifies the dwc_otg_ep_t data structure for this EP,
 * and then calls dwc_otg_ep_deactivate.
 */
static int dwc_otg_pcd_ep_disable(struct usb_ep *_ep)
{
	dwc_otg_pcd_ep_t *ep;
	unsigned long flags;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n", __func__, _ep);
	ep = container_of(_ep, dwc_otg_pcd_ep_t, ep);
	if (!_ep || !ep->desc) 
	{
		DWC_DEBUGPL(DBG_PCD, "%s, %s not enabled\n", __func__,
			_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}
		
	SPIN_LOCK_IRQSAVE(&ep->pcd->lock, flags);

	request_nuke( ep );		   

	dwc_otg_ep_deactivate( GET_CORE_IF(ep->pcd), &ep->dwc_ep );
	ep->desc = 0;
	ep->stopped = 1;
	
	if(ep->dwc_ep.is_in)
	{
		release_perio_tx_fifo(GET_CORE_IF(ep->pcd), ep->dwc_ep.tx_fifo_num);
		release_tx_fifo(GET_CORE_IF(ep->pcd), ep->dwc_ep.tx_fifo_num);
	}	
	
	SPIN_UNLOCK_IRQRESTORE(&ep->pcd->lock, flags);

	DWC_DEBUGPL(DBG_PCD, "%s disabled\n", _ep->name);
	return 0;
}


/**
 * This function allocates a request object to use with the specified
 * endpoint.
 *
 * @param _ep The endpoint to be used with with the request
 * @param _gfp_flags the GFP_* flags to use.
 */
static struct usb_request *dwc_otg_pcd_alloc_request(struct usb_ep *_ep,
					gfp_t _gfp_flags)
{
	dwc_otg_pcd_request_t *req;

    DWC_DEBUGPL(DBG_PCDV,"%s(%p,%d)\n", __func__, _ep, _gfp_flags);
	if (0 == _ep ) 
	{
		DWC_WARN("%s() %s\n", __func__, "Invalid EP!\n");
		return 0;
	}
	req = kmalloc( sizeof(dwc_otg_pcd_request_t), _gfp_flags);
	if (0 == req)
	{
		DWC_WARN("%s() %s\n", __func__, 
				 "request allocation failed!\n");
		return 0;
	}
	memset(req, 0, sizeof(dwc_otg_pcd_request_t));
	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

/**
 * This function frees a request object.
 *
 * @param _ep The endpoint associated with the request
 * @param _req The request being freed
 */
static void dwc_otg_pcd_free_request(struct usb_ep *_ep,
									 struct usb_request *_req)
{
	dwc_otg_pcd_request_t *req;
	DWC_DEBUGPL(DBG_PCDV,"%s(%p,%p)\n", __func__, _ep, _req);

	if (0 == _ep || 0 == _req) 
	{
		DWC_WARN("%s() %s\n", __func__, 
				 "Invalid ep or req argument!\n");
		return;
	}
		
	req = container_of(_req, dwc_otg_pcd_request_t, req);
	kfree(req);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
/**
 * This function allocates an I/O buffer to be used for a transfer
 * to/from the specified endpoint.
 * 
 * @param _ep The endpoint to be used with with the request
 * @param _bytes The desired number of bytes for the buffer
 * @param _dma Pointer to the buffer's DMA address; must be valid
 * @param _gfp_flags the GFP_* flags to use.
 * @return address of a new buffer or null is buffer could not be allocated.
 */
static void *dwc_otg_pcd_alloc_buffer(struct usb_ep *_ep, unsigned _bytes,
			dma_addr_t *_dma, gfp_t _gfp_flags)
{
	void *buf;
	dwc_otg_pcd_ep_t *ep;
	dwc_otg_pcd_t *pcd = 0;

	ep = container_of(_ep, dwc_otg_pcd_ep_t, ep);
	pcd = ep->pcd;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p,%d,%p,%0x)\n", __func__, _ep, _bytes, 
				_dma, _gfp_flags);

	/* Check dword alignment */
	if ((_bytes & 0x3UL) != 0) 
	{
		DWC_WARN("%s() Buffer size is not a multiple of" 
				 "DWORD size (%d)",__func__, _bytes);
	}

	if (GET_CORE_IF(pcd)->dma_enable) 
	{
		buf = dma_alloc_coherent (NULL, _bytes, _dma, _gfp_flags);
	}
	else 
	{
		buf = kmalloc( _bytes, _gfp_flags);
	}

	/* Check dword alignment */
	if (((int)buf & 0x3UL) != 0) 
	{
		DWC_WARN("%s() Buffer is not DWORD aligned (%p)",
					__func__, buf);
	}
		
	return buf;
}

/**
 * This function frees an I/O buffer that was allocated by alloc_buffer.
 *
 * @param _ep the endpoint associated with the buffer
 * @param _buf address of the buffer
 * @param _dma The buffer's DMA address
 * @param _bytes The number of bytes of the buffer
 */
static void dwc_otg_pcd_free_buffer(struct usb_ep *_ep, void *_buf,
									dma_addr_t _dma, unsigned _bytes)
{
	dwc_otg_pcd_ep_t *ep;
	dwc_otg_pcd_t *pcd = 0;

	ep = container_of(_ep, dwc_otg_pcd_ep_t, ep);
	pcd = ep->pcd;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p,%p,%0x,%d)\n", __func__, _ep, _buf, _dma, _bytes);
	
	if (GET_CORE_IF(pcd)->dma_enable) 
	{
		dma_free_coherent (NULL, _bytes, _buf, _dma);
	}
	else 
	{
		kfree( _buf );
	}
}
#endif


/**
 * This function is used to submit an I/O Request to an EP.
 *
 *	- When the request completes the request's completion callback
 *	  is called to return the request to the driver.
 *	- An EP, except control EPs, may have multiple requests
 *	  pending.
 *	- Once submitted the request cannot be examined or modified.
 *	- Each request is turned into one or more packets.
 *	- A BULK EP can queue any amount of data; the transfer is
 *	  packetized.
 *	- Zero length Packets are specified with the request 'zero'
 *	  flag.
 */
static int dwc_otg_pcd_ep_queue(struct usb_ep *_ep, 
			struct usb_request *_req, gfp_t _gfp_flags)
{
	int prevented = 0;
	dwc_otg_pcd_request_t *req;
	dwc_otg_pcd_ep_t *ep;
	dwc_otg_pcd_t	*pcd;
	unsigned long flags = 0;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p,%p,%d)\n", 
				__func__, _ep, _req, _gfp_flags);

	req = container_of(_req, dwc_otg_pcd_request_t, req);
	if (!_req || !_req->complete || !_req->buf || 
			!list_empty(&req->queue)) 
	{
		DWC_WARN("%s, bad params\n", __func__);
		return -EINVAL;
	}
		
	ep = container_of(_ep, dwc_otg_pcd_ep_t, ep);
	if (!_ep || (!ep->desc && ep->dwc_ep.num != 0)) 
	{
		DWC_WARN("%s, bad ep\n", __func__);
		return -EINVAL;
	}
	pcd = ep->pcd;
	if (!pcd->driver || pcd->gadget.speed == USB_SPEED_UNKNOWN) 
	{
		DWC_DEBUGPL(DBG_PCDV, "gadget.speed=%d\n", pcd->gadget.speed);
		DWC_WARN("%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

    DWC_DEBUGPL(DBG_PCDV, "dwc_otg_pcd_ep_queue, _req->dma=0x%08x\n", _req->dma);

    req->mapped    = 0;
    req->relocated = 0;

    if (GET_CORE_IF(pcd)->dma_enable)
    {    
        /* Check dword alignment */    
        if (((int)_req->buf & 0x3UL) != 0) 
        {
            DWC_DEBUGPL(DBG_PCD, "Warning: EP %s, Buf not DWORD aligned (%08x), size=%d, alloc&fix\n",
                        _ep->name, (int)_req->buf, _req->length);
    
            req->new_buf = dma_alloc_coherent (NULL, _req->length, &(_req->dma), GFP_KERNEL);
            if(req->new_buf==NULL)
                return -ENOMEM;
    
            memcpy(req->new_buf, _req->buf, _req->length);
            req->relocated = 1;
            wmb();
    
            DWC_DEBUGPL(DBG_PCD, "Fixed: EP %s, Buf (%08x), DMA = %8x, size=%d\n",
                        _ep->name, (int)req->new_buf, (int)_req->dma, _req->length);
        }
    
        else /* when buffer IS aligned, used it. */
        {    
            if((_req->dma == DMA_ADDR_INVALID)||(_req->dma == 0))
            {           
                _req->dma = dma_map_single(
                        NULL,
                        _req->buf,
                        _req->length,
                        DMA_BIDIRECTIONAL);
        
                req->mapped = 1;
                DWC_DEBUGPL(DBG_PCDV, "in-map, buf=%p, len=%d, dma=0x%08x\n", _req->buf, _req->length, _req->dma);
            }
            else
            {
                dma_sync_single_for_device(
                    NULL,
                    _req->dma, _req->length,
                    DMA_BIDIRECTIONAL);
        
                req->mapped = 0;
                DWC_DEBUGPL(DBG_PCDV, "No mapping, %s, buf=%p, len=%d, dma=0x%08x\n", _ep->name, _req->buf, _req->length, _req->dma);
            }
        }
    } /* GET_CORE_IF(pcd)->dma_enable */

    DWC_DEBUGPL(DBG_PCD, "%s eq_queue req, len %d buf %p, dma=%08x\n",
                      _ep->name, _req->length, _req->buf, _req->dma);        
	
	if (!GET_CORE_IF(pcd)->core_params->opt) 
	{
		if (ep->dwc_ep.num != 0) 
		{
			DWC_ERROR("%s queue req %p, len %d buf %p\n",
					  _ep->name, _req, _req->length, _req->buf);
		}
	}

	SPIN_LOCK_IRQSAVE(&ep->pcd->lock, flags);

#if defined(DEBUG) & defined(VERBOSE)
	dump_msg(_req->buf, _req->length);
#endif	

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* 
	 * For EP0 IN without premature status, zlp is required?
	 */
	if (ep->dwc_ep.num == 0 && ep->dwc_ep.is_in) 
	{		
		DWC_DEBUGPL(DBG_PCDV, "%s-OUT ZLP\n", _ep->name);
        /*_req->zero = 1; */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)
        DWC_DEBUGPL(DBG_PCDV, "%s-OUT ZLP, start req-zero=%d\n", _ep->name, _req->zero);
        _req->zero = 0;
#endif

        DWC_DEBUGPL(DBG_PCDV, "%s-OUT ZLP, req-zero=%d\n", _ep->name, _req->zero);
	}

	/* Start the transfer */
	if (list_empty(&ep->queue) && !ep->stopped) 
	{
		/* EP0 Transfer? */
		if (ep->dwc_ep.num == 0) 
		{
			DWC_DEBUGPL(DBG_PCDV, "EP0 Transfer: %s\n", _ep->name);
			switch (pcd->ep0state) 
			{
			case EP0_IN_DATA_PHASE:
				DWC_DEBUGPL(DBG_PCD,
								"%s ep0: EP0_IN_DATA_PHASE\n", 
								__func__);
				break;

			case EP0_OUT_DATA_PHASE:
				DWC_DEBUGPL(DBG_PCD, 
								"%s ep0: EP0_OUT_DATA_PHASE\n", 
								__func__);
				if (pcd->request_config) 
				{ 
					DWC_DEBUGPL(DBG_PCD, 
									"%s ep0: Complete STATUS_PHASE\n", 
									__func__);
					/* Complete STATUS PHASE */
					ep->dwc_ep.is_in = 1;
					pcd->ep0state = EP0_STATUS;
				}
				break;
						
			default:
				DWC_DEBUGPL(DBG_ANY, "ep0: odd state %d\n", 
											pcd->ep0state);
				SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);
				return -EL2HLT;
			}

			ep->dwc_ep.dma_addr = _req->dma;
			ep->dwc_ep.start_xfer_buff = _req->buf;
			ep->dwc_ep.xfer_buff = _req->buf;
			ep->dwc_ep.xfer_len = _req->length;
			ep->dwc_ep.xfer_count = 0;
			ep->dwc_ep.sent_zlp = 0;
			ep->dwc_ep.total_len = _req->length;
			dwc_otg_ep0_start_transfer( GET_CORE_IF(pcd), 
										&ep->dwc_ep );
		}
		else
		{
			DWC_DEBUGPL(DBG_PCDV, "EP%d Transfer: %s\n", ep->dwc_ep.num, _ep->name);
			/* Setup and start the Transfer */
			ep->dwc_ep.dma_addr = _req->dma;
			ep->dwc_ep.start_xfer_buff = _req->buf;
			ep->dwc_ep.xfer_buff = _req->buf;
			ep->dwc_ep.xfer_len = _req->length;
			ep->dwc_ep.xfer_count = 0;
			ep->dwc_ep.sent_zlp = 0;
			ep->dwc_ep.total_len = ep->dwc_ep.xfer_len;
			dwc_otg_ep_start_transfer( GET_CORE_IF(pcd), 
									   &ep->dwc_ep );
		}
	}

	if ((req != 0) || prevented) 
	{
		++pcd->request_pending;
		DWC_DEBUGPL(DBG_PCD, "%s ep%d-%s: Request Pending: %d\n", __func__, ep->dwc_ep.num, _ep->name, pcd->request_pending );
		list_add_tail(&req->queue, &ep->queue);
		if (ep->dwc_ep.is_in && ep->stopped && !(GET_CORE_IF(pcd)->dma_enable)) 
		{
			/** @todo NGS Create a function for this. */
			diepmsk_data_t diepmsk = { .d32 = 0};
			diepmsk.b.intktxfemp = 1;
			dwc_modify_reg32( &GET_CORE_IF(pcd)->dev_if->dev_global_regs->diepmsk, 0, diepmsk.d32 );
		}
	}
		
	SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);
	return 0;
}

/**
 * This function cancels an I/O request from an EP.
 */
static int dwc_otg_pcd_ep_dequeue(struct usb_ep *_ep,
								  struct usb_request *_req)
{
	dwc_otg_pcd_request_t *req;
	dwc_otg_pcd_ep_t *ep;
	dwc_otg_pcd_t	*pcd;
	unsigned long flags;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p,%p)\n", __func__, _ep, _req);
		
	ep = container_of(_ep, dwc_otg_pcd_ep_t, ep);
	if (!_ep || !_req || (!ep->desc && ep->dwc_ep.num != 0)) 
	{
		DWC_WARN("%s, bad argument\n", __func__);
		return -EINVAL;
	}
	pcd = ep->pcd;
	if (!pcd->driver || pcd->gadget.speed == USB_SPEED_UNKNOWN) 
	{
		DWC_WARN("%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

	SPIN_LOCK_IRQSAVE(&pcd->lock, flags);
	DWC_DEBUGPL(DBG_PCDV, "%s %s %s %p\n", __func__, _ep->name,
					ep->dwc_ep.is_in ? "IN" : "OUT",
					_req);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry( req, &ep->queue, queue) 
	{
		if (&req->req == _req) 
		{
			break;
		}
	}

	if (&req->req != _req) 
	{
		SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);
		return -EINVAL;
	}

	if (!list_empty(&req->queue)) 
	{		 
		request_done(ep, req, -ECONNRESET);
	} 
	else 
	{
		req = 0;
	}
		
	SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);

	return req ? 0 : -EOPNOTSUPP;
}

/**
 * usb_ep_set_halt stalls an endpoint. 
 *
 * usb_ep_clear_halt clears an endpoint halt and resets its data
 * toggle.
 *
 * Both of these functions are implemented with the same underlying
 * function. The behavior depends on the value argument.
 * 
 * @param[in] _ep the Endpoint to halt or clear halt.
 * @param[in] _value 
 *	- 0 means clear_halt.
 *	- 1 means set_halt, 
 *	- 2 means clear stall lock flag.
 *	- 3 means set  stall lock flag.
 */
static int dwc_otg_pcd_ep_set_halt(struct usb_ep *_ep, int _value)
{
	int retval = 0;
	unsigned long flags;
	dwc_otg_pcd_ep_t *ep = 0;
		
		
	DWC_DEBUGPL(DBG_PCD,"HALT %s %d\n", _ep->name, _value);

	ep = container_of(_ep, dwc_otg_pcd_ep_t, ep);

	if (!_ep || (!ep->desc && ep != &ep->pcd->ep0) ||
			ep->desc->bmAttributes == USB_ENDPOINT_XFER_ISOC) 
	{
		DWC_WARN("%s, bad ep\n", __func__);
		return -EINVAL;
	}
		
	SPIN_LOCK_IRQSAVE(&ep->pcd->lock, flags);
	if (!list_empty(&ep->queue))
	{
		DWC_WARN("%s() %s XFer In process\n", __func__, _ep->name);
		retval = -EAGAIN;
	}
	else if (_value == 0) 
	{
		dwc_otg_ep_clear_stall( ep->pcd->otg_dev->core_if, 
									&ep->dwc_ep );		 
	}
	else if(_value == 1)
	{
		if (ep->dwc_ep.num == 0) 
		{
			ep->pcd->ep0state = EP0_STALL;
		}
		
		ep->stopped = 1;
		dwc_otg_ep_set_stall( ep->pcd->otg_dev->core_if, 
								&ep->dwc_ep );
	}
	else if (_value == 2) 
	{
		ep->dwc_ep.stall_clear_flag = 0;
	}
	else if (_value == 3) 
	{
		ep->dwc_ep.stall_clear_flag = 1;
	}
	
	SPIN_UNLOCK_IRQRESTORE(&ep->pcd->lock, flags);
	return retval;
}


static struct usb_ep_ops dwc_otg_pcd_ep_ops = 
{
	.enable		= dwc_otg_pcd_ep_enable,
	.disable	= dwc_otg_pcd_ep_disable,

	.alloc_request	= dwc_otg_pcd_alloc_request,
	.free_request	= dwc_otg_pcd_free_request,

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
	.alloc_buffer	= dwc_otg_pcd_alloc_buffer,
	.free_buffer	= dwc_otg_pcd_free_buffer,
#endif

	.queue		= dwc_otg_pcd_ep_queue,
	.dequeue	= dwc_otg_pcd_ep_dequeue,

	.set_halt	= dwc_otg_pcd_ep_set_halt,
	.fifo_status	= 0,
	.fifo_flush = 0,
};

/*	Gadget Operations */
/**
 * The following gadget operations will be implemented in the DWC_otg
 * PCD. Functions in the API that are not described below are not
 * implemented.
 *
 * The Gadget API provides wrapper functions for each of the function
 * pointers defined in usb_gadget_ops. The Gadget Driver calls the
 * wrapper function, which then calls the underlying PCD function. The
 * following sections are named according to the wrapper functions
 * (except for ioctl, which doesn't have a wrapper function). Within
 * each section, the corresponding DWC_otg PCD function name is
 * specified.
 *
 */

/**
 *Gets the USB Frame number of the last SOF.
 */
static int dwc_otg_pcd_get_frame(struct usb_gadget *_gadget)
{
	dwc_otg_pcd_t *pcd;
	
	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n", __func__, _gadget);
		
	if (_gadget == 0)
	{
		return -ENODEV;
	} 
	else 
	{
		pcd = container_of(_gadget, dwc_otg_pcd_t, gadget);
		dwc_otg_get_frame_number( GET_CORE_IF(pcd) );
	}
		
	return 0;
}

void dwc_otg_pcd_initiate_srp(dwc_otg_pcd_t *_pcd)
{
	uint32_t *addr = (uint32_t *)&(GET_CORE_IF(_pcd)->core_global_regs->gotgctl);
	gotgctl_data_t mem;
	gotgctl_data_t val;

	val.d32 = dwc_read_reg32( addr );
	if (val.b.sesreq) 
	{
		DWC_ERROR("Session Request Already active!\n");
			return;
	}

	DWC_NOTICE("Session Request Initated\n");
	mem.d32 = dwc_read_reg32(addr);
	mem.b.sesreq = 1;
	dwc_write_reg32(addr, mem.d32);

	/* Start the SRP timer */
	dwc_otg_pcd_start_srp_timer( _pcd );
	return;
}

void dwc_otg_pcd_remote_wakeup(dwc_otg_pcd_t *_pcd, int set)
{
	dctl_data_t dctl = {.d32=0};
	volatile uint32_t *addr = 
				&(GET_CORE_IF(_pcd)->dev_if->dev_global_regs->dctl);

	if (dwc_otg_is_device_mode(GET_CORE_IF(_pcd))) 
	{
		if (_pcd->remote_wakeup_enable) 
		{
			if (set) 
			{
				dctl.b.rmtwkupsig = 1;
				dwc_modify_reg32( addr, 0, dctl.d32 );
				DWC_DEBUGPL(DBG_PCD, "Set Remote Wakeup\n");
				mdelay(1);
				dwc_modify_reg32( addr, dctl.d32, 0 );
				DWC_DEBUGPL(DBG_PCD, "Clear Remote Wakeup\n");
			}
			else 
			{
			}
		}
		else 
		{
			DWC_DEBUGPL(DBG_PCD, "Remote Wakeup is disabled\n");
		}
	}

	return;
}

/**
 * Initiates Session Request Protocol (SRP) to wakeup the host if no
 * session is in progress. If a session is already in progress, but
 * the device is suspended, remote wakeup signaling is started.
 *
 */
static int dwc_otg_pcd_wakeup(struct usb_gadget *_gadget)
{
	unsigned long flags;
	dwc_otg_pcd_t *pcd;
	dsts_data_t		dsts;
	gotgctl_data_t	gotgctl;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n", __func__, _gadget);
		
	if (_gadget == 0)
	{
		return -ENODEV;
	} 
	else 
	{
		pcd = container_of(_gadget, dwc_otg_pcd_t, gadget);
	}
	SPIN_LOCK_IRQSAVE(&pcd->lock, flags);

	/*
	 * This function starts the Protocol if no session is in progress. If
	 * a session is already in progress, but the device is suspended,
	 * remote wakeup signaling is started.
	 */

	/* Check if valid session */
	gotgctl.d32 = dwc_read_reg32(&(GET_CORE_IF(pcd)->core_global_regs->gotgctl));
	if (gotgctl.b.bsesvld) 
	{
		/* Check if suspend state */
		dsts.d32 = dwc_read_reg32(&(GET_CORE_IF(pcd)->dev_if->dev_global_regs->dsts));
		if (dsts.b.suspsts) 
		{
			dwc_otg_pcd_remote_wakeup(pcd, 1);
		}
	}
	else 
	{
		dwc_otg_pcd_initiate_srp(pcd);
	}

	SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);
	return 0;
}

static const struct usb_gadget_ops dwc_otg_pcd_ops = 
{
	.get_frame	 = dwc_otg_pcd_get_frame,
	.wakeup		 = dwc_otg_pcd_wakeup,
	/* current versions must always be self-powered */
};

/**
 * This function updates the otg values in the gadget structure. 
 */
void dwc_otg_pcd_update_otg( dwc_otg_pcd_t *_pcd, const unsigned _reset )
{
		
	if (!_pcd->gadget.is_otg)
		return;

	if (_reset) 
	{
		_pcd->b_hnp_enable = 0;
		_pcd->a_hnp_support = 0;
		_pcd->a_alt_hnp_support = 0;
	}

	_pcd->gadget.b_hnp_enable = _pcd->b_hnp_enable;
	_pcd->gadget.a_hnp_support =  _pcd->a_hnp_support;
	_pcd->gadget.a_alt_hnp_support = _pcd->a_alt_hnp_support;
}

/** 
 * This function is the top level PCD interrupt handler.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t 
dwc_otg_pcd_irq(int _irq, void *_dev, struct pt_regs *_r)
#else
static irqreturn_t 
dwc_otg_pcd_irq(int _irq, void *_dev)
#endif
{
	dwc_otg_pcd_t *pcd = _dev;
	int32_t retval = IRQ_NONE;
        
	retval = dwc_otg_pcd_handle_intr( pcd );        
	return IRQ_RETVAL(retval);
}

/**
 * PCD Callback function for initializing the PCD when switching to
 * device mode.
 *
 * @param _p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_start_cb( void *_p )
{
	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t *)_p;
	
	/*
	 * Initialized the Core for Device mode.
	 */
	if (dwc_otg_is_device_mode( GET_CORE_IF(pcd) ))
	{
			dwc_otg_core_dev_init(GET_CORE_IF(pcd));
	}
	
	return 1;
}

/**
 * PCD Callback function for stopping the PCD when switching to Host
 * mode.
 *
 * @param _p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_stop_cb( void *_p )
{
	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t *)_p;
	extern void dwc_otg_pcd_stop(dwc_otg_pcd_t *_pcd);
	
	dwc_otg_pcd_stop( pcd );
	return 1;
}

static int32_t dwc_otg_pcd_autoresume( void *_p )
{
	dwc_otg_core_if_t *_core_if = (dwc_otg_core_if_t *)_p;
	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t *)(_core_if->pcd_cb->p);
        dctl_data_t dctl = {.d32=0};
        gintsts_data_t gintsts;

	DWC_DEBUGPL(DBG_PCD, "DSTS=0x%0x\n", dwc_read_reg32( &_core_if->dev_if->dev_global_regs->dsts));

        if (_core_if->hwcfg4.b.power_optimiz) {
            pcgcctl_data_t power = {.d32=0};

            power.d32 = dwc_read_reg32( _core_if->pcgcctl );
            DWC_DEBUGPL(DBG_CIL, "PCGCCTL=%0x\n", power.d32);

            power.b.stoppclk = 0;
            dwc_write_reg32( _core_if->pcgcctl, power.d32);

        }

        /* Clear the Remote Wakeup Signalling */
        dctl.b.rmtwkupsig = 1;
        dwc_modify_reg32( &_core_if->dev_if->dev_global_regs->dctl, dctl.d32, 0 );

	/* Call upper layer resume functions. */
	if (pcd->driver && pcd->driver->resume) 
	{
			SPIN_UNLOCK(&pcd->lock);
			pcd->driver->resume(&pcd->gadget);
			SPIN_LOCK(&pcd->lock);
	}
	
	/* Stop the SRP timeout timer. */
	if ((GET_CORE_IF(pcd)->core_params->phy_type != DWC_PHY_TYPE_PARAM_FS) ||
		(!GET_CORE_IF(pcd)->core_params->i2c_enable))
	{
		if (GET_CORE_IF(pcd)->srp_timer_started) 
		{
			GET_CORE_IF(pcd)->srp_timer_started = 0;
			del_timer( &pcd->srp_timer );
		}
	}

        gintsts.d32 = 0;
        gintsts.b.disconnect = 1;
        dwc_write_reg32 (&_core_if->core_global_regs->gintsts, gintsts.d32);

	return 1;
}

/**
 * PCD handling of suspend intr.
 *
 * @param _p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_autosuspend( void *_p )
{
	dwc_otg_core_if_t *_core_if = (dwc_otg_core_if_t *)_p;
	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t *)(_core_if->pcd_cb->p);
	dsts_data_t dsts ;
    gintsts_data_t gintsts;
    /*dctl_data_t dctl = {.d32=0}; */

        /* Check the Device status register to determine if the Suspend
        * state is active. */
	/* If everything is settable to suspend state, turn off the clock, which is 
	 * putting usb core into power save mode. */
        dsts.d32 = dwc_read_reg32( &_core_if->dev_if->dev_global_regs->dsts);

        if (dsts.b.suspsts && _core_if->hwcfg4.b.power_optimiz) {
            pcgcctl_data_t power = {.d32=0};

            power.b.stoppclk = 1;
            dwc_modify_reg32( _core_if->pcgcctl, 0, power.d32);
        } else {
                DWC_DEBUGPL(DBG_ANY,"disconnect?\n");
        }

	/* We are done with this layer of power save in usb subsystem. So go to upper layers, to optimize more. */
	/* This will be at a class layer, file_storage/rndis/mtp layer, so call it's suspend function. 
	 * This suspend function is defined inside the gadget_driver structure, so call it directly. 
	 */
	if (pcd->driver && pcd->driver->resume) 
	{
		SPIN_UNLOCK(&pcd->lock);
		pcd->driver->suspend(&pcd->gadget);
		SPIN_LOCK(&pcd->lock);
	}

        /* Clear interrupt */
        gintsts.d32 = 0;
        gintsts.b.usbsuspend = 1;
        dwc_write_reg32( &_core_if->core_global_regs->gintsts, gintsts.d32);

	return 1;
}


/**
 * PCD Callback function for notifying the PCD when resuming from
 * suspend.
 *
 * @param _p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_suspend_cb( void *_p )
{
	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t *)_p;

	if (pcd->driver && pcd->driver->resume) 
	{
		SPIN_UNLOCK(&pcd->lock);
		pcd->driver->suspend(&pcd->gadget);
		SPIN_LOCK(&pcd->lock);
	}
	
	return 1;
}


/**
 * PCD Callback function for notifying the PCD when resuming from
 * suspend.
 *
 * @param _p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_resume_cb( void *_p )
{
	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t *)_p;
	
	if (pcd->driver && pcd->driver->resume) 
	{
			SPIN_UNLOCK(&pcd->lock);
			pcd->driver->resume(&pcd->gadget);
			SPIN_LOCK(&pcd->lock);
	}
	
	/* Stop the SRP timeout timer. */
	if ((GET_CORE_IF(pcd)->core_params->phy_type != DWC_PHY_TYPE_PARAM_FS) ||
		(!GET_CORE_IF(pcd)->core_params->i2c_enable))
	{
		if (GET_CORE_IF(pcd)->srp_timer_started) 
		{
			GET_CORE_IF(pcd)->srp_timer_started = 0;
			del_timer( &pcd->srp_timer );
		}
	}
	return 1;
}


/**
 * PCD Callback structure for handling mode switching.
 */
static dwc_otg_cil_callbacks_t pcd_callbacks = 
{
	.start = dwc_otg_pcd_start_cb,
	.stop = dwc_otg_pcd_stop_cb,
	.suspend = dwc_otg_pcd_suspend_cb,
	.resume_wakeup = dwc_otg_pcd_resume_cb,
	.auto_suspend = dwc_otg_pcd_autosuspend,
	.auto_resume = dwc_otg_pcd_autoresume,
	.p = 0, /* Set at registration */
};

/**
 * This function is called when the SRP timer expires.	The SRP should
 * complete within 6 seconds. 
 */
static void srp_timeout( unsigned long _ptr )
{
	gotgctl_data_t gotgctl;
	dwc_otg_core_if_t *core_if = (dwc_otg_core_if_t *)_ptr;
	volatile uint32_t *addr = &core_if->core_global_regs->gotgctl;

	gotgctl.d32 = dwc_read_reg32(addr);

	core_if->srp_timer_started = 0;

	if ((core_if->core_params->phy_type == DWC_PHY_TYPE_PARAM_FS) && 
		(core_if->core_params->i2c_enable))
	{
		DWC_PRINT( "SRP Timeout\n");

		if ((core_if->srp_success) && 
			(gotgctl.b.bsesvld))
		{
			if (core_if->pcd_cb && core_if->pcd_cb->resume_wakeup ) 
			{
				core_if->pcd_cb->resume_wakeup(core_if->pcd_cb->p);
			}
			
			/* Clear Session Request */
			gotgctl.d32 = 0;
			gotgctl.b.sesreq = 1;
			dwc_modify_reg32( &core_if->core_global_regs->gotgctl, 
					  gotgctl.d32, 0);
	
			core_if->srp_success = 0;
		}
		else 
		{
			DWC_ERROR( "Device not connected/responding\n");
			gotgctl.b.sesreq = 0;
			dwc_write_reg32(addr, gotgctl.d32);
		}
	}
	else if (gotgctl.b.sesreq) 
	{
		DWC_PRINT( "SRP Timeout\n");

		DWC_ERROR( "Device not connected/responding\n");
		gotgctl.b.sesreq = 0;
		dwc_write_reg32(addr, gotgctl.d32);
	} 
	else 
	{
		DWC_PRINT( " SRP GOTGCTL=%0x\n", gotgctl.d32);
	} 
}

/**
 * Start the SRP timer to detect when the SRP does not complete within 
 * 6 seconds.
 *
 * @param _pcd the pcd structure.
 */
void dwc_otg_pcd_start_srp_timer(dwc_otg_pcd_t *_pcd )
{
	struct timer_list *srp_timer = &_pcd->srp_timer;
	GET_CORE_IF(_pcd)->srp_timer_started = 1;
	init_timer( srp_timer );
	srp_timer->function = srp_timeout;
	srp_timer->data = (unsigned long)GET_CORE_IF(_pcd);
	srp_timer->expires = jiffies + (HZ*6);
	add_timer( srp_timer );
}

/**
 * Tasklet
 *
 */
extern void start_next_request( dwc_otg_pcd_ep_t *_ep );

static void start_xfer_tasklet_func (unsigned long data)
{
	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t*)data;
	dwc_otg_core_if_t *core_if = pcd->otg_dev->core_if;

	int i;
	depctl_data_t diepctl;

	DWC_DEBUGPL(DBG_PCDV, "Start xfer tasklet\n");

	diepctl.d32 = dwc_read_reg32( &core_if->dev_if->in_ep_regs[0]->diepctl);

	if (pcd->ep0.queue_sof) 
	{
		pcd->ep0.queue_sof = 0;
		start_next_request (&pcd->ep0);
		/* break; */
	}

	for (i=0; i<core_if->dev_if->num_in_eps; i++) 
	{
		depctl_data_t diepctl;
		diepctl.d32 = dwc_read_reg32( &core_if->dev_if->in_ep_regs[i]->diepctl);

		if (pcd->in_ep[i].queue_sof) 
		{
			pcd->in_ep[i].queue_sof = 0;
			start_next_request (&pcd->in_ep[i]);
			/* break; */
		}
	}

	return;
}


static struct tasklet_struct start_xfer_tasklet = {
	.next = NULL,
	.state = 0,
	.count = ATOMIC_INIT(0),
	.func = start_xfer_tasklet_func,
	.data = 0,
};
/**
 * This function initialized the pcd Dp structures to there default
 * state.
 *
 * @param _pcd the pcd structure.
 */
void dwc_otg_pcd_reinit(dwc_otg_pcd_t *_pcd)
{
	static const char * names[] = 
		{
			
			"ep0",
			"ep1in",	
			"ep2in",	
			"ep3in",	
			"ep4in",	
			"ep5in",	
			"ep6in",	
			"ep7in",	
			"ep8in",	
			"ep9in",	
			"ep10in",	
			"ep11in",	
			"ep12in",	
			"ep13in",	
			"ep14in",	
			"ep15in",	
			"ep1out",  
			"ep2out",  
			"ep3out",
			"ep4out",
			"ep5out",
			"ep6out",
			"ep7out",
			"ep8out",
			"ep9out",
			"ep10out",
			"ep11out",
			"ep12out",
			"ep13out",
			"ep14out",
			"ep15out"
			
	};
		
	int i;
	int in_ep_cntr, out_ep_cntr;
	uint32_t hwcfg1;
	uint32_t num_in_eps = (GET_CORE_IF(_pcd))->dev_if->num_in_eps;
	uint32_t num_out_eps = (GET_CORE_IF(_pcd))->dev_if->num_out_eps;
	dwc_otg_pcd_ep_t *ep;
	
	DWC_DEBUGPL(DBG_PCDV, "%s(%p)\n", __func__, _pcd);
	
	INIT_LIST_HEAD (&_pcd->gadget.ep_list);
	_pcd->gadget.ep0 = &_pcd->ep0.ep;
	_pcd->gadget.speed = USB_SPEED_UNKNOWN;

	INIT_LIST_HEAD (&_pcd->gadget.ep0->ep_list);

	/**
	 * Initialize the EP0 structure.
	 */
	ep = &_pcd->ep0;

	/* Init EP structure */
	ep->desc = 0;
	ep->pcd = _pcd;
	ep->stopped = 1;

	/* Init DWC ep structure */
	ep->dwc_ep.num = 0;
	ep->dwc_ep.active = 0;
	ep->dwc_ep.tx_fifo_num = 0;
	/* Control until ep is actvated */
	ep->dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL; 
	ep->dwc_ep.maxpacket = MAX_PACKET_SIZE;
	ep->dwc_ep.dma_addr = 0;
	ep->dwc_ep.start_xfer_buff = 0;
	ep->dwc_ep.xfer_buff = 0;
	ep->dwc_ep.xfer_len = 0;
	ep->dwc_ep.xfer_count = 0;
	ep->dwc_ep.sent_zlp = 0;
	ep->dwc_ep.total_len = 0;
	ep->queue_sof = 0;

	/* Init the usb_ep structure. */
	ep->ep.name = names[0];
	ep->ep.ops = &dwc_otg_pcd_ep_ops;

	/**
	 * @todo NGS: What should the max packet size be set to
	 * here?  Before EP type is set?
	 */
	ep->ep.maxpacket = MAX_PACKET_SIZE;

	list_add_tail (&ep->ep.ep_list, &_pcd->gadget.ep_list);
		
	INIT_LIST_HEAD (&ep->queue);
	/**
	 * Initialize the EP structures.
	 */
	in_ep_cntr = 0;
	hwcfg1 = (GET_CORE_IF(_pcd))->hwcfg1.d32 >> 3;
	 
	for (i = 1; in_ep_cntr < num_in_eps; i++) 
	{
		if((hwcfg1 & 0x1) == 0)
		{
			dwc_otg_pcd_ep_t *ep = &_pcd->in_ep[in_ep_cntr];
			in_ep_cntr ++;
			
			/* Init EP structure */
			ep->desc = 0;
			ep->pcd = _pcd;
			ep->stopped = 1;
	
			/* Init DWC ep structure */
			ep->dwc_ep.is_in = 1;
			ep->dwc_ep.num = i;
			ep->dwc_ep.active = 0;
			ep->dwc_ep.tx_fifo_num = 0;
			
			/* Control until ep is actvated */
			ep->dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL; 
			ep->dwc_ep.maxpacket = MAX_PACKET_SIZE;
			ep->dwc_ep.dma_addr = 0;
			ep->dwc_ep.start_xfer_buff = 0;
			ep->dwc_ep.xfer_buff = 0;
			ep->dwc_ep.xfer_len = 0;
			ep->dwc_ep.xfer_count = 0;
			ep->dwc_ep.sent_zlp = 0;
			ep->dwc_ep.total_len = 0;
			ep->queue_sof = 0;
	
			/* Init the usb_ep structure. */
			/**
			 * @todo NGS: Add direction to EP, based on contents
			 * of HWCFG1.  Need a copy of HWCFG1 in pcd structure?
			 * sprintf( ";r
			 */
			ep->ep.name = names[i];
			ep->ep.ops = &dwc_otg_pcd_ep_ops;	
			
			/**
			 * @todo NGS: What should the max packet size be set to
			 * here?  Before EP type is set?
			 */
			ep->ep.maxpacket = MAX_PACKET_SIZE;
	
			list_add_tail (&ep->ep.ep_list, &_pcd->gadget.ep_list);
				
			INIT_LIST_HEAD (&ep->queue);
		}
		hwcfg1 >>= 2;
	}

	out_ep_cntr = 0;
	hwcfg1 = (GET_CORE_IF(_pcd))->hwcfg1.d32 >> 2;

	for (i = 1; out_ep_cntr < num_out_eps; i++) 
	{
		if((hwcfg1 & 0x1) == 0)
		{
			dwc_otg_pcd_ep_t *ep = &_pcd->out_ep[out_ep_cntr];
			out_ep_cntr++;
	
			/* Init EP structure */
			ep->desc = 0;
			ep->pcd = _pcd;
			ep->stopped = 1;
	
			/* Init DWC ep structure */
			ep->dwc_ep.is_in = 0;
			ep->dwc_ep.num = i;
			ep->dwc_ep.active = 0;
			ep->dwc_ep.tx_fifo_num = 0;
			/* Control until ep is actvated */
			ep->dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL; 
			ep->dwc_ep.maxpacket = MAX_PACKET_SIZE;
			ep->dwc_ep.dma_addr = 0;
			ep->dwc_ep.start_xfer_buff = 0;
			ep->dwc_ep.xfer_buff = 0;
			ep->dwc_ep.xfer_len = 0;
			ep->dwc_ep.xfer_count = 0;
			ep->dwc_ep.sent_zlp = 0;
			ep->dwc_ep.total_len = 0;
			ep->queue_sof = 0;
	
			/* Init the usb_ep structure. */
			/**
			 * @todo NGS: Add direction to EP, based on contents
			 * of HWCFG1.  Need a copy of HWCFG1 in pcd structure?
			 * sprintf( ";r
			 */
			ep->ep.name = names[15 + i];
			ep->ep.ops = &dwc_otg_pcd_ep_ops;
			/**
			 * @todo NGS: What should the max packet size be set to
			 * here?  Before EP type is set?
			 */
			ep->ep.maxpacket = MAX_PACKET_SIZE;
	
			list_add_tail (&ep->ep.ep_list, &_pcd->gadget.ep_list);
				
			INIT_LIST_HEAD (&ep->queue);
		}
		hwcfg1 >>= 2;
	}
	
	/* remove ep0 from the list.  There is a ep0 pointer.*/
	list_del_init (&_pcd->ep0.ep.ep_list);
   
	_pcd->ep0state = EP0_DISCONNECT;
	_pcd->ep0.ep.maxpacket = MAX_EP0_SIZE;		  
	_pcd->ep0.dwc_ep.maxpacket = MAX_EP0_SIZE;
	_pcd->ep0.dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL;
}

/**
 * This function releases the Gadget device.
 * required by device_unregister().
 *
 * @todo Should this do something?	Should it free the PCD? 
 */
static void dwc_otg_pcd_gadget_release(struct device *_dev)
{
	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n", __func__, _dev);
}


/** 
 * This function initialized the PCD portion of the driver.
 *
 */ 
/*int __init dwc_otg_pcd_init(struct lm_device *_lmdev) */
int __init_or_module dwc_otg_pcd_init(struct lm_device *_lmdev)
{
	static char pcd_name[] = "dwc_otg_pcd";
	dwc_otg_pcd_t *pcd;
	dwc_otg_device_t *otg_dev = lm_get_drvdata(_lmdev);
	int retval = 0;


	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n",__func__, _lmdev );
    otg_dev = g_dwc_otg_device;

	/*
	 * Allocate PCD structure
	 */
	pcd = kmalloc( sizeof(dwc_otg_pcd_t), GFP_KERNEL);
	
	if (pcd == 0) 
	{
			return -ENOMEM;
	}
	
	memset( pcd, 0, sizeof(dwc_otg_pcd_t));
	spin_lock_init( &pcd->lock );
	
	otg_dev->pcd = pcd;
	s_pcd = pcd;
	pcd->gadget.name = pcd_name;
	dev_set_name(&(pcd->gadget.dev), "gadget");
	
	pcd->otg_dev = lm_get_drvdata(_lmdev);
	pcd->otg_dev = g_dwc_otg_device;

	pcd->gadget.dev.parent = &_lmdev->dev;
	pcd->gadget.dev.release = dwc_otg_pcd_gadget_release;
	pcd->gadget.ops = &dwc_otg_pcd_ops;
	
	
	if(GET_CORE_IF(pcd)->hwcfg4.b.ded_fifo_en)
	{
		DWC_PRINT("Dedicated Tx FIFOs mode\n");
	}
	else
	{
		DWC_PRINT("Shared Tx FIFO mode\n");
	}
	
	/* If the module is set to FS or if the PHY_TYPE is FS then the gadget
	 * should not report as dual-speed capable.	 replace the following line
	 * with the block of code below it once the software is debugged for
	 * this.  If is_dualspeed = 0 then the gadget driver should not report
	 * a device qualifier descriptor when queried. */
	if ((GET_CORE_IF(pcd)->core_params->speed == DWC_SPEED_PARAM_FULL) ||
		((GET_CORE_IF(pcd)->hwcfg2.b.hs_phy_type == 2) &&
		 (GET_CORE_IF(pcd)->hwcfg2.b.fs_phy_type == 1) &&
		 (GET_CORE_IF(pcd)->core_params->ulpi_fs_ls)))
	{
		pcd->gadget.is_dualspeed = 0;
	}
	else 
	{
		pcd->gadget.is_dualspeed = 1;
	}
	
	if ((otg_dev->core_if->hwcfg2.b.op_mode == DWC_HWCFG2_OP_MODE_NO_SRP_CAPABLE_DEVICE) || 
	(otg_dev->core_if->hwcfg2.b.op_mode == DWC_HWCFG2_OP_MODE_NO_SRP_CAPABLE_HOST) || 
	(otg_dev->core_if->hwcfg2.b.op_mode == DWC_HWCFG2_OP_MODE_SRP_CAPABLE_DEVICE) || 
	(otg_dev->core_if->hwcfg2.b.op_mode == DWC_HWCFG2_OP_MODE_SRP_CAPABLE_HOST))
	{
		pcd->gadget.is_otg = 0;
	}
	else 
	{
		pcd->gadget.is_otg = 1;
	}
	  
    /* Add a hack here for testing, need para + config reg to decide,  */
    pcd->gadget.is_otg = 0;

	pcd->driver = 0;
	/* Register the gadget device */
    device_register( &pcd->gadget.dev );

	/*
	 * Initialized the Core for Device mode.
	 */
	if (dwc_otg_is_device_mode( GET_CORE_IF(pcd) )) 
	{
		dwc_otg_core_dev_init( GET_CORE_IF(pcd) );
	}

	/*
	 * Initialize EP structures
	 */
	dwc_otg_pcd_reinit( pcd );

	/*
	 * Register the PCD Callbacks. 
	 */
	dwc_otg_cil_register_pcd_callbacks( otg_dev->core_if, &pcd_callbacks, 
										pcd );
	/*
	 * Setup interupt handler
	 */
	DWC_DEBUGPL( DBG_ANY, "registering handler for irq%d\n", _lmdev->irq);
	retval = request_irq(_lmdev->irq, dwc_otg_pcd_irq,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
      SA_SHIRQ,
#else
      IRQF_SHARED,
#endif
						  pcd->gadget.name, pcd);

	if (retval != 0) 
	{
		DWC_ERROR("request of irq%d failed\n", _lmdev->irq);
		kfree (pcd);
		return -EBUSY;
	}

	/* 
	 * Initialize the DMA buffer for SETUP packets
	 */
	if (GET_CORE_IF(pcd)->dma_enable) 
	{
        DWC_DEBUGPL( DBG_ANY, "Setup DMA coherent buffer\n");
		pcd->setup_pkt = dma_alloc_coherent (NULL, sizeof (*pcd->setup_pkt) * 5, &pcd->setup_pkt_dma_handle, 0);
		pcd->status_buf = dma_alloc_coherent (NULL, sizeof (uint16_t), &pcd->status_buf_dma_handle, 0);

        DWC_PRINT("pcd->setup_pkt=%p, pcd->setup_pkt_dma_handle=%x\n", pcd->setup_pkt, pcd->setup_pkt_dma_handle);
        DWC_PRINT("pcd->status_buf=%p, pcd->status_buf_dma_handle=%x\n", pcd->status_buf, pcd->status_buf_dma_handle);
	}
	else 
	{
		pcd->setup_pkt = kmalloc (sizeof (*pcd->setup_pkt) * 5, GFP_KERNEL);
		pcd->status_buf = kmalloc (sizeof (uint16_t), GFP_KERNEL);
	}

	if (pcd->setup_pkt == 0) 
	{
		kfree (pcd);
		return -ENOMEM;
	}

	/* Initialize tasklet */
	start_xfer_tasklet.data = (unsigned long)pcd;
	pcd->start_xfer_tasklet = &start_xfer_tasklet;

	return 0;
}

/**
 * Cleanup the PCD.
 */
void dwc_otg_pcd_remove( struct lm_device *_lmdev )
{
	dwc_otg_device_t *otg_dev = lm_get_drvdata(_lmdev);
	dwc_otg_pcd_t *pcd = otg_dev->pcd;
	
	DWC_DEBUGPL(DBG_PCD, "%s(%p), otg_dev=%p, pcd=%p\n", __func__, _lmdev, otg_dev, pcd);
    otg_dev = g_dwc_otg_device;
    pcd = otg_dev->pcd;

	/*
	 * Free the IRQ 
	 */
        free_irq( _lmdev->irq, pcd );
	
	 /* start with the driver above us */
	if (pcd->driver) 
	{
		/* should have been done already by driver model core */
		DWC_WARN("driver '%s' is still registered\n",
					 pcd->driver->driver.name);
		usb_gadget_unregister_driver( pcd->driver);
	}
	device_unregister(&pcd->gadget.dev);
		
	if (GET_CORE_IF(pcd)->dma_enable) 
	{
		dma_free_coherent (NULL, sizeof (*pcd->setup_pkt) * 5, pcd->setup_pkt, pcd->setup_pkt_dma_handle);
		dma_free_coherent (NULL, sizeof (uint16_t), pcd->status_buf, pcd->status_buf_dma_handle);
	}
	else 
	{
		kfree (pcd->setup_pkt);
		kfree (pcd->status_buf);
	}
	
	kfree( pcd );
	otg_dev->pcd = 0;
}

/**
 * This function registers a gadget driver with the PCD.
 *
 * When a driver is successfully registered, it will receive control
 * requests including set_configuration(), which enables non-control
 * requests.  then usb traffic follows until a disconnect is reported.
 * then a host may connect again, or the driver might get unbound.
 *
 * @param _driver The driver being registered
 */
int usb_gadget_register_driver(struct usb_gadget_driver *_driver)
{
	int retval;

	DWC_DEBUGPL(DBG_PCD, "registering gadget driver '%s'\n", _driver->driver.name);
		
	if (!_driver || _driver->speed == USB_SPEED_UNKNOWN || 
		!_driver->bind || 
		!_driver->unbind || 
		!_driver->disconnect || 
		!_driver->setup) 
	{
		DWC_ERROR("EINVAL\n");	
		return -EINVAL;
	}
	if (s_pcd == 0) 
	{
		DWC_ERROR("ENODEV\n");	
		return -ENODEV;
	}
	if (s_pcd->driver != 0) 
	{
		DWC_ERROR("EBUSY (%p)\n", s_pcd->driver);   
		return -EBUSY;
	}
		
	/* hook up the driver */
	s_pcd->driver = _driver;
	s_pcd->gadget.dev.driver = &_driver->driver;

	retval = _driver->bind(&s_pcd->gadget);
	if (retval) 
	{
        if(retval == -EISNAM)
            DWC_NOTICE("bind to driver %s\n",
                    _driver->driver.name);
        else
            DWC_ERROR("bind to driver %s --> error %d\n",
					_driver->driver.name, retval);
		s_pcd->driver = 0;
		s_pcd->gadget.dev.driver = 0;
		return retval;
	}
	DWC_NOTICE("registered gadget driver '%s'\n", 
					_driver->driver.name);

	DWC_NOTICE("Enable USB globle interrupt\n");
	/* Now we have all drivers ready, Show USB cable to PC host */
        REG_SYS_ANACR9 &= ~(0x1<<28);     /* non-driving */
        REG_SYS_ANACR9 |= (0x1<<30);      /* suspend_eco */
        udelay(20);
	
	dwc_otg_enable_global_interrupts( );
	return 0;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

/**
 * This function unregisters a gadget driver
 *
 * @param _driver The driver being unregistered
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *_driver)
{
	/*DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n", __func__, _driver); */

	if (s_pcd == 0) 
	{
		DWC_DEBUGPL(DBG_ANY, "%s Return(%d): s_pcd==0\n", __func__, 
					-ENODEV);
		return -ENODEV;
	}
	if (_driver == 0 || _driver != s_pcd->driver) 
	{
		DWC_DEBUGPL( DBG_ANY, "%s Return(%d): driver?\n", __func__, 
					-EINVAL);
		return -EINVAL;
	}

        /* Disable USB interrupt before class driver unbind */
	REG_SYS_ANACR9 |= (0x1<<28);     /* non-driving */
	dwc_otg_disable_global_interrupts();

	_driver->unbind(&s_pcd->gadget);
	s_pcd->driver = 0;

	DWC_DEBUGPL(DBG_ANY, "unregistered driver '%s'\n", 
					_driver->driver.name);

	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

#endif /* DWC_HOST_ONLY */
