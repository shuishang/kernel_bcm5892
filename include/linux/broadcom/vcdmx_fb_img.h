/*****************************************************************************
* Copyright 2001 - 2009 Broadcom Corporation.  All rights reserved.
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
*  VCDMX_fb_img.h
*
*  PURPOSE:
*
*   This implements 
*
*  NOTES:
*
*****************************************************************************/


#if !defined( BCM_LINUX_VCDMX_FB_IMG_H )
#define BCM_LINUX_VCDMX_FB_IMG_H

/* ---- Include Files ---------------------------------------------------- */
#include <linux/ioctl.h>

#include "interface/vmcs_host/vcdmx.h"


#define VCDMX_MAGIC	'V'
#define VCDMX_CMD_ACC_FIRST			0x80
#define VCDMX_CMD_ACC_CREATEIMAGE	0x80
#define VCDMX_CMD_ACC_PUTPIXELS		0x81
#define VCDMX_CMD_ACC_GETPIXELS		0x82
#define VCDMX_CMD_ACC_FILLRECT		0x83	
#define VCDMX_CMD_ACC_COPYRECT		0x84
#define VCDMX_CMD_ACC_COPYRECT_II	0x85
#define VCDMX_CMD_ACC_SYNC			0x86
#define VCDMX_CMD_ACC_DELETEIMAGE	0x87
#define VCDMX_CMD_ACC_DISPLAYIMAGE	0x88
#define VCDMX_CMD_ACC_REMOVEIMAGE   0x89
#define VCDMX_CMD_ACC_CREATERESOURCE 0x8A

#define VCDMX_CMD_ACC_COPYRECTBULK	0x8C
#define VCDMX_CMD_ACC_FILLRECTBULK  0x8D
#define VCDMX_CMD_ACC_CMDBULK		0x8E
#define VCDMX_CMD_ACC_CREATEIMAGEBULK 0x8F
#define VCDMX_CMD_ACC_DELETEIMAGEBULK 0x90
#define VCDMX_CMD_ACC_SWAPIMAGE		0x91
#define VCDMX_CMD_ACC_INIT              0x92
#define VCDMX_CMD_ACC_DEINIT            0x93
#define VCDMX_CMD_ACC_RENDER			0x94

#define VCDMX_CMD_ACC_LAST			0x94



typedef union {
   VCDMX_IMAGE_T              image;
   VCDMX_PIXEL_XFER_T         pixel_xfer;
   VCDMX_FILL_INFO_T          fill_info;
   VCDMX_COPY_INFO_T          copy_info;
   VCDMX_COPY_TYPE_II_INFO_T  typeII_info;
   VCDMX_MARKER_T             marker;
   VCDMX_DISPLAY_IMG_INFO_T   display_img;
   VCDMX_COPYRECT_BULK_XFER_T copyrect_bulk;
   VCDMX_FILLRECT_BULK_XFER_T  fillrect_bulk;
   VCDMX_IMAGE_BULK_XFER_T    image_bulk;
   VCDMX_SWAP_IMG_INFO_T      swap_img;
   VCDMX_RENDER_INFO_T        render_info;

} VCDMX_Request;

/* maximum number of cmd in VCDMX_CMD_BULK_XFER_T. 
   violation of this will cause unexpected behaviour 
   such as the first 1024 cmds are executed by videocore */
#ifndef VCDMX_CMD_BULK_MAX_ENTRY
#define VCDMX_CMD_BULK_MAX_ENTRY	1024    /*this is redundant. It is already defined in vcdmx.h*/
#endif

#define VCDMX_IOCTL_ACC_CREATEIMAGE 				_IOWR( VCDMX_MAGIC, VCDMX_CMD_ACC_CREATEIMAGE, VCDMX_IMAGE_T				) 
#define VCDMX_IOCTL_ACC_PUTPIXELS					_IOW( VCDMX_MAGIC, VCDMX_CMD_ACC_PUTPIXELS,	VCDMX_PIXEL_XFER_T			)
#define VCDMX_IOCTL_ACC_GETPIXELS					_IOWR( VCDMX_MAGIC,  VCDMX_CMD_ACC_GETPIXELS,	VCDMX_PIXEL_XFER_T			)
#define VCDMX_IOCTL_ACC_FILLRECT					_IOW( VCDMX_MAGIC, VCDMX_CMD_ACC_FILLRECT,	VCDMX_FILL_INFO_T			)
#define VCDMX_IOCTL_ACC_COPYRECT					_IOW( VCDMX_MAGIC, VCDMX_CMD_ACC_COPYRECT,	VCDMX_COPY_INFO_T			)
#define VCDMX_IOCTL_ACC_COPYRECT_II					_IOW( VCDMX_MAGIC, VCDMX_CMD_ACC_COPYRECT_II,	VCDMX_COPY_TYPE_II_INFO_T)
#define VCDMX_IOCTL_ACC_SYNC						_IOW( VCDMX_MAGIC,  VCDMX_CMD_ACC_SYNC,		VCDMX_MARKER_T				)
#define VCDMX_IOCTL_ACC_DELETEIMAGE					_IOW( VCDMX_MAGIC,  VCDMX_CMD_ACC_DELETEIMAGE,	VCDMX_IMAGE_T				)
#define VCDMX_IOCTL_ACC_DISPLAYIMAGE				_IOWR( VCDMX_MAGIC,  VCDMX_CMD_ACC_DISPLAYIMAGE,	VCDMX_DISPLAY_IMG_INFO_T	)
#define VCDMX_IOCTL_ACC_REMOVEIMAGE					_IOW( VCDMX_MAGIC,  VCDMX_CMD_ACC_REMOVEIMAGE,	VCDMX_DISPLAY_IMG_INFO_T		)
#define VCDMX_IOCTL_ACC_CREATERESOURCE 				_IOW( VCDMX_MAGIC, VCDMX_CMD_ACC_CREATERESOURCE, VCDMX_IMAGE_T				) 

#define VCDMX_IOCTL_ACC_COPYRECTBULK				_IOW( VCDMX_MAGIC, VCDMX_CMD_ACC_COPYRECTBULK,   VCDMX_COPYRECT_BULK_XFER_T )
#define VCDMX_IOCTL_ACC_FILLRECTBULK				_IOW( VCDMX_MAGIC, VCDMX_CMD_ACC_FILLRECTBULK,   VCDMX_FILLRECT_BULK_XFER_T )
#define VCDMX_IOCTL_ACC_CMDBULK					_IOW( VCDMX_MAGIC, VCDMX_CMD_ACC_CMDBULK,		VCDMX_CMD_BULK_XFER_T)
#define VCDMX_IOCTL_ACC_CREATEIMAGEBULK					_IOW( VCDMX_MAGIC, VCDMX_CMD_ACC_CREATEIMAGEBULK,		VCDMX_IMAGE_BULK_XFER_T)
#define VCDMX_IOCTL_ACC_DELETEIMAGEBULK					_IOW( VCDMX_MAGIC, VCDMX_CMD_ACC_DELETEIMAGEBULK,		VCDMX_IMAGE_BULK_XFER_T)
#define VCDMX_IOCTL_ACC_SWAPIMAGE					_IOWR( VCDMX_MAGIC,  VCDMX_CMD_ACC_SWAPIMAGE,	VCDMX_SWAP_IMG_INFO_T		)
#define VCDMX_IOCTL_ACC_INIT                                    _IOW( VCDMX_MAGIC,  VCDMX_CMD_ACC_INIT,	int		)
#define VCDMX_IOCTL_ACC_DEINIT				        _IOW( VCDMX_MAGIC,  VCDMX_CMD_ACC_DEINIT,	int		)
#define VCDMX_IOCTL_ACC_RENDER						_IOW( VCDMX_MAGIC, VCDMX_CMD_ACC_RENDER,	VCDMX_RENDER_INFO_T)

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

#endif /* BCM_LINUX_VCDMX_FB_IMG_H*/ 
