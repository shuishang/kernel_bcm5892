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

/*
 * Description: Header of the Multimedia (MM) DMA module that should be used
 * for multimedia related DMA operations. Drivers that should use this module
 * include the Linux framebuffer driver, Graphic Engine (GE) driver, LCD
 * driver, Hantro driver, etc.
 */

#ifndef _MMDMA_H
#define _MMDMA_H

#include <linux/ioctl.h>

#ifndef __KERNEL__
#include <stdint.h>
#endif

#define MMDMA_LOG_ENABLED    0
#if MMDMA_LOG_ENABLED
#define MMDMA_LOG            KNLLOG
#else
#define MMDMA_LOG(c,args...)
#endif

typedef int MMDMA_HDL;

/*
 * MMDMA memory chunck lookup table
 */
struct mmdma_hw_cfg {
   unsigned int *mem_table;
   unsigned int mem_cnt;
   unsigned int tot_mem;
};

/*
 * Data buffer structure
 */
struct mmdma_buf {
   /* virtual address in the kernel */
   void *data;

   /* physical address */
   uint32_t bus_addr;

   /* offset used for mmap */
   uint32_t offset;

   /* buffer size in bytes */
   unsigned int len; 
};

struct mmdma_ioctl_buf_alloc_param {
   struct mmdma_buf buf;
};

struct mmdma_ioctl_buf_get_param {
   struct mmdma_buf buf;
};

struct mmdma_ioctl_buf_free_param {
   uint32_t bus_addr;
};

#define MMDMA_MAGIC            'G'
#define MMDMA_CMD_BUF_ALLOC    0xA1
#define MMDMA_CMD_BUF_GET      0xA2
#define MMDMA_CMD_BUF_FREE     0xA3

#define MMDMA_IOCTL_BUF_ALLOC    _IOWR(MMDMA_MAGIC, MMDMA_CMD_BUF_ALLOC, struct mmdma_ioctl_buf_alloc_param)
#define MMDMA_IOCTL_BUF_GET      _IOWR(MMDMA_MAGIC, MMDMA_CMD_BUF_GET, struct mmdma_ioctl_buf_get_param)
#define MMDMA_IOCTL_BUF_FREE     _IOW(MMDMA_MAGIC, MMDMA_CMD_BUF_FREE, struct mmdma_ioctl_buf_free_param)

#ifdef __KERNEL__
/*
 * Perform the data transfer using a dedicated DMA channel. This routine block
 * wait until DMA finishes
 */
extern int mmdma_transfer(dma_addr_t src_addr, dma_addr_t dst_addr,
      uint32_t len);
#endif

#endif /* _MMDMA_H */
