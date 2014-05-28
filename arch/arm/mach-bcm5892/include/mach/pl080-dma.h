/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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


#ifndef BCM5892_PL022_DMA_H
#define BCM5892_PL022_DMA_H

#include <linux/types.h>

#define DMA_NUM_CHANNELS (8)


#define PL080_INT_STATUS      (0x00)
#define PL080_TC_INT_STATUS   (0x04)
#define PL080_TC_INT_CLEAR    (0x08)
#define PL080_ERR_INT_STATUS  (0x0c)
#define PL080_ERR_INT_CLEAR   (0x10)
#define PL080_CONFIG          (0x30)

#define PL080_CONFIG_ENABLE   (0x01)

/* Offsets to calculate channel base addresses */
#define PL080_CHAN0_OFFSET         (0x100)
#define PL080_INTER_CHANNEL_OFFSET (0x20)

/* Most of the channel's registers we will access as if it were an LLI, but
 * the CCFG is extra:
*/
#define PL080_CHAN_CCFG (0x10)

/* Channel / LLI CCTL fields */

#define PL080_CCTL_TCI       (0x80000000) /* terminal count interrupt */
#define PL080_CCTL_DI        (0x08000000) /* dest increment */
#define PL080_CCTL_SI        (0x04000000) /* source increment */
#define PL080_CCTL_D         (0x02000000) /* dest master select */
#define PL080_CCTL_S         (0x01000000) /* src master select */
#define PL080_CCTL_DWIDTH8   (0x00000000) /* dest width = 8 bits */
#define PL080_CCTL_DWIDTH16  (0x00200000) /* dest width = 16 bits */
#define PL080_CCTL_DWIDTH32  (0x00400000) /* dest width = 32 bits */
#define PL080_CCTL_SWIDTH8   (0x00000000) /* dest width = 8 bits */
#define PL080_CCTL_SWIDTH16  (0x00040000) /* dest width = 16 bits */
#define PL080_CCTL_SWIDTH32  (0x00080000) /* dest width = 32 bits */
#define PL080_CCTL_DBSIZE1   (0x00000000) /* dest burst size */
#define PL080_CCTL_DBSIZE4   (0x00008000)
#define PL080_CCTL_DBSIZE8   (0x00010000)
#define PL080_CCTL_DBSIZE16  (0x00018000)
#define PL080_CCTL_SBSIZE1   (0x00000000) /* src burst size */
#define PL080_CCTL_SBSIZE4   (0x00001000)
#define PL080_CCTL_SBSIZE8   (0x00002000)
#define PL080_CCTL_SBSIZE16  (0x00003000)
#define PL080_CCTL_LENMASK   (0x00000fff)

#define PL080_CCTL_DWIDTH_MASK (0x00600000)
#define PL080_CCTL_DWIDTH_SHIFT 21
#define PL080_CCTL_SWIDTH_MASK (0x000c0000)
#define PL080_CCTL_SWIDTH_SHIFT 18

/* Channel CCFG fields */
#define PL080_CCFG_H         (0x00040000)
#define PL080_CCFG_A         (0x00020000)
#define PL080_CCFG_L         (0x00010000)
#define PL080_CCFG_ITC       (0x00008000)
#define PL080_CCFG_IE        (0x00004000)
#define PL080_CCFG_MEM_MEM     (0 << 11)
#define PL080_CCFG_MEM_PER     (1 << 11)
#define PL080_CCFG_PER_MEM     (2 << 11)
#define PL080_CCFG_PER_PER     (3 << 11)
#define PL080_CCFG_PER_PER_DST (4 << 11)  /* flow control by dest periph */
#define PL080_CCFG_MEM_PER_DST (5 << 11)  /* flow control by dest periph */
#define PL080_CCFG_PER_MEM_SRC (6 << 11)  /* flow control by src periph */
#define PL080_CCFG_PER_PER_SRC (7 << 11)  /* flow control by src periph */
/* alternative view of those last few: bits for src or dest being periph,
   if DMA is the controller (i.e. is transferring a set number of words): */
#define PL080_CCFG_DEST_IS_PER (1 << 11)
#define PL080_CCFG_SRC_IS_PER  (2 << 11)

#define PL080_CCFG_DEST(idx) ((idx) << 6)
#define PL080_CCFG_SRC(idx)  ((idx) << 1)
#define PL080_CCFG_E         (0x00000001)

/* Channel CCFG Dest/Src periph numbers */
#define PL080_DMA_UART0_TX    (0)
#define PL080_DMA_UART0_RX    (1)
#define PL080_DMA_UART1_TX    (2)
#define PL080_DMA_UART1_RX    (3)
#define PL080_DMA_UART2_TX    (4)
#define PL080_DMA_UART2_RX    (5)
#define PL080_DMA_UART3_TX    (6)
#define PL080_DMA_UART3_RX    (7)
#define PL080_DMA_SPI0_TX     (8)
#define PL080_DMA_SPI0_RX     (9)
#define PL080_DMA_SPI1_TX     (10)
#define PL080_DMA_SPI1_RX     (11)
#define PL080_DMA_SPI3_TX     (14)
#define PL080_DMA_SPI3_RX     (15)
#define PL080_DMA_SC0_TX      (16)
#define PL080_DMA_SC0_RX      (17)
#define PL080_DMA_I2S_TX      (18)
#define PL080_DMA_I2S_RX      (19)
#define PL080_DMA_TP_SPI4_TX  (20)
#define PL080_DMA_TP_SPI4_RX  (21)
#define PL080_DMA_SC1_TX      (22)
#define PL080_DMA_SC1_RX      (23)
#define PL080_DMA_MEM         (31) /* used by sw, now hw, to signal mem xfer */


#define STATUS_TC  (1)
#define STATUS_ERR  (2)

typedef void (*dma_channel_cb_t) (int channel, int status_mask, void *cb_data);

/* Request a channel.  Returns the highest/lowest priority channel that is available,
 *  as specified by "high_priority".  Returns a channel index, or -EBUSY if no channels
 *  are available.
 */
int pl080_request_channel(int high_priority);

/* Releases a previously requested channel */
void pl080_release_channel(int channel_idx);

/*
 * Channel operations:
 */

/* Set the source and destination peripherals for a channel */
void pl080_channel_set_src_dest(unsigned channel_idx, int src, int dest);

/* Set a callback for a channel: */
void pl080_channel_set_irq_callback(unsigned channel_idx, dma_channel_cb_t cb, void *cb_data);

/* Add to the transfer chain for a channel.  Will break up into multiple LLIs as appropriate */
int pl080_channel_add_xfer(unsigned channel_idx, dma_addr_t src, dma_addr_t dest, unsigned length, uint32_t base_cctl);

/* Indicates that all following xfers will be circular, i.e. will loop endlessly
 */
void pl080_channel_begin_circular_xfer(unsigned channel_idx);

/* Add a single LLI to the transfer chain for a channel */
int pl080_channel_add_single_xfer(unsigned channel_idx, dma_addr_t src, dma_addr_t dest, uint32_t cctl);

/* Adds a single transfer in a transfer chain for a channel.  Will do a short transfer to try to achieve alignment, if
 *   that is sensible
 */
int pl080_channel_add_xfer_stage(unsigned channel_idx, dma_addr_t src, dma_addr_t dest, unsigned length, uint32_t base_cctl);

/* Clear all arranged transfers.  Typically called from callback when xfer is done */
void pl080_channel_clear_xfers(unsigned channel_idx);

/* Start DMA transfer */
void pl080_channel_enable(unsigned channel_idx);

/* Cancel DMA transfer */
void pl080_channel_disable(unsigned channel_idx, int nicely);


/* Set bits in CCFG */
#define CHAN_CCFG_BIS(val, chan) iowrite32(ioread32(IO_ADDRESS((chan)->base) + PL080_CHAN_CCFG) | (val), IO_ADDRESS((chan)->base) + PL080_CHAN_CCFG)
/* Clear bits in CCFG */
#define CHAN_CCFG_BIC(val, chan) iowrite32(ioread32(IO_ADDRESS((chan)->base) + PL080_CHAN_CCFG) & ~(val), IO_ADDRESS((chan)->base) + PL080_CHAN_CCFG)
#define CHAN_CCFG_READ(chan) ioread32(IO_ADDRESS((chan)->base) + PL080_CHAN_CCFG)

#define PL080_READ_REG(reg) ioread32(IO_ADDRESS(global_dma_dev->base) + (reg))
#define PL080_WRITE_REG(val, reg) iowrite32((val), IO_ADDRESS(global_dma_dev->base) + (reg))

/*   /\* conservative guesses for SPI TX: no bursting, bytes only.  XXX DV has used 8 byte bursts with SPI... *\/ */
/* #define SPI0_TX_PL080_CCTL_OPT  (CCTL_TCI | CCTL_SI | CCTL_DWIDTH8 | CCTL_SWIDTH8 | CCTL_DBSIZE1 | CCTL_SBSIZE1) */
/* #define SPI0_TX_PL080_CONFIG_BASE (CCFG_ITC | CCFG_IE | CCFG_MEM_PER | CCFG_DEST(DMA_SPI0_TX)) */

/*   /\* conservative guesses for SPI TX: no bursting, bytes only.  XXX DV has used 8 byte bursts with SPI... *\/ */
/* #define SPI0_RX_PL080_CCTL_OPT  (CCTL_TCI | CCTL_SI | CCTL_DWIDTH8 | CCTL_SWIDTH8 | CCTL_DBSIZE1 | CCTL_SBSIZE1) */
/* #define SPI0_RX_PL080_CONFIG_BASE (CCFG_ITC | CCFG_IE | CCFG_PER_MEM | CCFG_SRC(DMA_SPI0_RX)) */

#endif /* #ifndef BCM5892_PL080_DMA_H */
