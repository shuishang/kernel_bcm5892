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


//#include "pl080-dma.h"
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <mach/shm.h>
#include <mach/reg_gpio.h>
#include <linux/amba/bus.h>
#include <linux/platform_device.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/timer.h>

#include <mach/pl080-dma.h>

#define PRINTK (!debug_on) ?: printk

#define MAX_LLI_NUM (8)

/* TDM buffers in scratch RAM assigned to open mode.  Used here for testing only */
#define TDM3_BUF_BA (0x1bb000)

static int debug_on = 0;
extern uint32_t hw_timer_get_counter(int);

struct pl080_lli {
	dma_addr_t src;
	dma_addr_t dest;
	dma_addr_t next_lli;
	uint32_t cctl;
};

struct pl080_channel {
	unsigned allocated:1;
	unsigned is_circular:1;
	unsigned adding_multi:1;

	void __iomem * base;
	int num_llis;
	int circular_start;

	struct pl080_lli * lli_va[MAX_LLI_NUM];
	dma_addr_t lli_ba[MAX_LLI_NUM];

	dma_channel_cb_t cb;
	void *cb_data;
};

struct pl080_device {
	struct device *dev;
	void __iomem *base;
	struct pl080_channel channel[DMA_NUM_CHANNELS];

	struct dma_pool *pool;

	void * null_src_va;
	dma_addr_t null_src_ba;
	void * null_dest_va;
	dma_addr_t null_dest_ba;

	spinlock_t channel_list_lock;
};

static struct pl080_device *global_dma_dev;


/* Called when an example transfer is done */
static void example_callback(int channel, int status_mask, void *cb_data) {
	struct completion *c = cb_data;

	complete(c);
}

int example_memcpy(dma_addr_t from_ba, dma_addr_t to_ba, int len) {
	int chan = pl080_request_channel(0);
	uint32_t base_cctl =  PL080_CCTL_TCI | PL080_CCTL_SI | PL080_CCTL_DI |
		              PL080_CCTL_DWIDTH32 | PL080_CCTL_SWIDTH32 | PL080_CCTL_DBSIZE4 | PL080_CCTL_SBSIZE4;
	struct completion dma_completion;
	int retval = 0;
	uint32_t start, finish;
	int iter;

	init_completion(&dma_completion);

	start = hw_timer_get_counter(3);

	for (iter = 0; iter < 100000; ++iter) {
		/* DMA map the memory that we are going to use.
		 * note: first argument should be the device requesting the mapping */
		pl080_channel_set_src_dest(chan, PL080_DMA_MEM, PL080_DMA_MEM);
		pl080_channel_set_irq_callback(chan, example_callback, &dma_completion);

		pl080_channel_add_xfer(chan, from_ba, to_ba, 25, base_cctl);

		pl080_channel_enable(chan);
		wait_for_completion(&dma_completion);

		pl080_release_channel(chan);
	}

	finish = hw_timer_get_counter(3);

	{
		/* note: timer counts down, so use start - finish */
		int ticks_per_iter_times_100 = 100 * (int32_t)(start - finish) / iter;
		int ticks_per_iter = ticks_per_iter_times_100 / 100;
		int frac = ticks_per_iter_times_100 - 100 * ticks_per_iter;

		printk(KERN_INFO "DMA Test complete, %d.%02d ticks per iteration) (%08x, %08x)\n", ticks_per_iter, frac, from_ba, to_ba);
	}

	return retval;
}

int example_memcpy_main_mem(void) {
	int size = 0x3f00;
	uint8_t *from = kzalloc(size, GFP_KERNEL);
	uint8_t *to = kzalloc(size, GFP_KERNEL);
	dma_addr_t from_ba, to_ba;
	int retval;

	from_ba = dma_map_single(global_dma_dev->dev, from, sizeof(from), DMA_TO_DEVICE);
	to_ba = dma_map_single(global_dma_dev->dev, to, sizeof(to), DMA_FROM_DEVICE);

	retval = example_memcpy(from_ba, to_ba, size);

	dma_unmap_single(global_dma_dev->dev, from_ba, sizeof(from), DMA_TO_DEVICE);
	dma_unmap_single(global_dma_dev->dev, to_ba, sizeof(to), DMA_FROM_DEVICE);

	retval = retval || memcmp(to, from, size);

	if (retval)
		printk(KERN_WARNING "DMA Test failed! (%08x, %08x)\n", from_ba, to_ba);

	kfree(from);
	kfree(to);

	return retval;
}

int example_memcpy_to_scratch(void) {
	int size = 0x3f00;
	uint8_t *from = kzalloc(size, GFP_KERNEL);
	dma_addr_t from_ba, to_ba;
	int retval;

	from_ba = dma_map_single(global_dma_dev->dev, from, sizeof(from), DMA_TO_DEVICE);
	to_ba = TDM3_BUF_BA;

	retval = example_memcpy(from_ba, to_ba, size);

	dma_unmap_single(global_dma_dev->dev, from_ba, sizeof(from), DMA_TO_DEVICE);

	kfree(from);

	return retval;
}

static ssize_t show_debug_sysfs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t set_debug_sysfs(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(debug, S_IWUSR | S_IRUGO, show_debug_sysfs, set_debug_sysfs);

static ssize_t show_debug_sysfs(struct device *pDev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", debug_on);
}

static ssize_t set_debug_sysfs(struct device *pDev, struct device_attribute *attr, const char *buf, size_t count)
{
	int new_debug =  simple_strtoul(buf, NULL, 10);
	switch (new_debug) {
	case 10:
		debug_on = example_memcpy_main_mem();
		break;
	case 20:
		debug_on = example_memcpy_to_scratch();
		break;
	default:
		debug_on = new_debug;
	}

	return count;
}

int pl080_request_channel(int high_priority)
{
	int i;
	unsigned long flags;
	int channel_idx = -EBUSY;

	PRINTK(KERN_INFO "request channel(%d), %p\n", high_priority, global_dma_dev);

	spin_lock_irqsave(&global_dma_dev->channel_list_lock, flags);

	if (high_priority) {
		for (i = 0; i < DMA_NUM_CHANNELS; ++i) {
			if (!global_dma_dev->channel[i].allocated) {
				global_dma_dev->channel[i].allocated = 1;
				channel_idx = i;
				break;
			}
		}
	} else {
		for (i = DMA_NUM_CHANNELS - 1; i >= 0; --i) {
			if (!global_dma_dev->channel[i].allocated) {
				global_dma_dev->channel[i].allocated = 1;
				channel_idx = i;
				break;
			}
		}
	}

	spin_unlock_irqrestore(&global_dma_dev->channel_list_lock, flags);

	return channel_idx;
}
EXPORT_SYMBOL(pl080_request_channel);

void pl080_release_channel(int channel_idx)
{
	struct pl080_channel *chan = &global_dma_dev->channel[channel_idx];

	PRINTK("Releasing channel %d\n", channel_idx);
	CHAN_CCFG_BIC(PL080_CCFG_E, chan);

	pl080_channel_clear_xfers(channel_idx);

	chan->cb = 0;
	chan->cb_data = 0;

	/* effectively atomic, so no need to lock */
	global_dma_dev->channel[channel_idx].allocated = 0;
}
EXPORT_SYMBOL(pl080_release_channel);

/* Adds a single transfer to the transfer chain for a yet-to-be-activated channel.  If either 'src' or
   'dest' are null, they are set to be internal values capable of holding a 4-byte value.
 * Note: does not lock, on the theory that if multiple folks are adding to a DMA at the same
 *  time, bad things will happen even if things aren't getting corrupted
 */
int pl080_channel_add_single_xfer(unsigned channel_idx, dma_addr_t src, dma_addr_t dest, uint32_t cctl)
{
	struct pl080_channel *chan = &global_dma_dev->channel[channel_idx];
	struct pl080_lli *new_lli_va;
	struct pl080_lli *prev_lli_va = NULL;
	dma_addr_t new_lli_ba = 0;

	BUG_ON(channel_idx >= DMA_NUM_CHANNELS);

	PRINTK(KERN_INFO "Adding single xfer (%d, %08x %08x, %08x)\n", channel_idx, src, dest, cctl);

	if (src == 0)
		src = global_dma_dev->null_src_ba;

	if (dest == 0)
		dest = global_dma_dev->null_dest_ba;

	if (chan->num_llis == -1) {
		new_lli_va = IO_ADDRESS(chan->base);
	} else {
		PRINTK(KERN_INFO "Getting lli space from pool for %d\n", chan->num_llis);
		new_lli_va = dma_pool_alloc(global_dma_dev->pool, GFP_KERNEL, &new_lli_ba);

		PRINTK(KERN_INFO "Pool: %p, %08x\n", new_lli_va, new_lli_ba);

		if (!new_lli_va)
			return -EBUSY;

		chan->lli_va[chan->num_llis] = new_lli_va;
		chan->lli_ba[chan->num_llis] = new_lli_ba;

		if (chan->num_llis == 0)
			prev_lli_va = IO_ADDRESS(chan->base);
		else
			prev_lli_va = chan->lli_va[chan->num_llis-1];

		PRINTK(KERN_INFO "Previous va: %p\n", prev_lli_va);
	}

	PRINTK(KERN_INFO "new_lli_va: %p\n", new_lli_va);

	new_lli_va->src = src;
	new_lli_va->dest = dest;
	new_lli_va->cctl = cctl;
	new_lli_va->next_lli = 0;

	if (prev_lli_va)
		prev_lli_va->next_lli = new_lli_ba;

	chan->num_llis++;

	return 0;
}
EXPORT_SYMBOL(pl080_channel_add_single_xfer);


int pl080_channel_add_xfer_stage(unsigned channel_idx, dma_addr_t src, dma_addr_t dest,
				unsigned length, uint32_t base_cctl)
{
	int sw_lg2, dw_lg2;  /* src/dest widths log base 2, i.e. shifts to mult/div by width */
	uint32_t sw_mask, dw_mask; /* mask of invalid address bits according to src/dest widths */

	int num_this_time, sw_lg2_this_time, bytes_this_time, max_bytes;
	uint32_t cctl_this_time;
	int rc = 0;

	base_cctl &= ~0xfff;  /* mask out the length field in the base cctl */

	/* given that max width <= 32 bits, we can do the following to get our width shifts: */
	sw_lg2 = (base_cctl & PL080_CCTL_SWIDTH_MASK) >> PL080_CCTL_SWIDTH_SHIFT;
	dw_lg2 = (base_cctl & PL080_CCTL_DWIDTH_MASK) >> PL080_CCTL_DWIDTH_SHIFT;

	sw_mask = (sw_lg2 | (sw_lg2 >> 1));
	dw_mask = (dw_lg2 | (dw_lg2 >> 1));

	PRINTK(KERN_INFO "Xfer stage [%d, %d]. Adding LLIs up to %d bytes\n", sw_lg2, dw_lg2, length);

	if (length > 0 && rc == 0) {
		cctl_this_time = base_cctl;

		/* if source is not aligned, or length < swidth, do a small unaligned LLI */
		if (src & sw_mask) {
			num_this_time = (src & sw_mask);

			if (length <= num_this_time)
				num_this_time = length;

			sw_lg2_this_time = 0;
			cctl_this_time &= ~PL080_CCTL_SWIDTH_MASK;
		} else if ((length & sw_mask) == length) {
			num_this_time = length;
			sw_lg2_this_time = 0;
			cctl_this_time &= ~PL080_CCTL_SWIDTH_MASK;
		} else {
			/* an aligned transfer */
			num_this_time = length >> sw_lg2;

			/* we need to limit the number of transfers to less than LENMASK */
		        /* (or even fewer, if dest has a larger alignment than src */

			bytes_this_time = (num_this_time << sw_lg2);
			max_bytes = (PL080_CCTL_LENMASK << sw_lg2) & ~dw_mask;
			if (bytes_this_time > max_bytes) {
				num_this_time = max_bytes >> sw_lg2;
			}
			sw_lg2_this_time = sw_lg2;
		}

		bytes_this_time = num_this_time << sw_lg2_this_time;

		/* for unaligned destination or xfer, fall back to byte dest width */
		if ((dest & 1) || (bytes_this_time & 1))
			cctl_this_time &= ~PL080_CCTL_DWIDTH_MASK;
		else if ((dest & dw_mask) == 0x2 || (bytes_this_time & dw_mask) == 0x2) {
			cctl_this_time &= ~PL080_CCTL_DWIDTH_MASK;
			cctl_this_time |= PL080_CCTL_DWIDTH16;
		}

		PRINTK(KERN_INFO "  stage: %d %d %d\n", sw_lg2_this_time, num_this_time, bytes_this_time);

		if (bytes_this_time != length && global_dma_dev->channel[channel_idx].adding_multi)
			cctl_this_time &= ~PL080_CCTL_TCI;

		rc = pl080_channel_add_single_xfer(channel_idx, src, dest, cctl_this_time | num_this_time);

		rc = bytes_this_time;
	}
	return rc;
}
EXPORT_SYMBOL(pl080_channel_add_xfer_stage);

/* pl080_channel_add_xfer - Adds a transfer to the  transfer chain for a
 *   yet-to-be-activated channel.
 * @channel_idx: the channel index (returned from pl080_request_channel)
 * @src: the source address (bus address, i.e. physical address for this arch)
 *       if null, an internal address is used to source zeros (SI should NOT
 *       be set in base_cctl in this case).
 * @dest: the destination address (bus address).  If null, an internal
 *        address is used (DI should NOT be set in base_cctl in this case).
 * @length: the length of the transfer in bytes
 * @base_cctl: the cctl info without the length field.  Source and Dest widths
               are taken to be maximum widths- if the transfer is not aligned
	       with this max width then it may be broken into multiple transfers
	       so the bulk of it can be done at max width.
	       If the transfer is broken up, only the last one will cause a
	       TC interrupt.
 * Note: does not lock, on the theory that if multiple folks are adding to a DMA at the same
 *  time, bad things will happen even if things aren't getting corrupted
 */
int pl080_channel_add_xfer(unsigned channel_idx, dma_addr_t src, dma_addr_t dest, unsigned length, uint32_t base_cctl)
{
	int rc = 0;

	PRINTK(KERN_INFO "Multi-xfer adding LLIs totalling %d bytes\n", length);

	global_dma_dev->channel[channel_idx].adding_multi = 1;
	do {
		rc = pl080_channel_add_xfer_stage(channel_idx, src, dest, length, base_cctl);
		if (rc < 0)
			break;
		if (base_cctl & PL080_CCTL_SI)
			src += rc;
		if (base_cctl & PL080_CCTL_DI)
			dest += rc;
		length -= rc;
	} while (length > 0 && rc >= 0);

	global_dma_dev->channel[channel_idx].adding_multi = 0;
	if (rc > 0)
		rc = 0;

	return rc;
}
EXPORT_SYMBOL(pl080_channel_add_xfer);

void pl080_channel_begin_circular_xfer(unsigned channel_idx)
{
	struct pl080_channel *chan = &global_dma_dev->channel[channel_idx];
	chan->is_circular = 1;
	chan->circular_start = chan->num_llis;
	if (chan->num_llis < 0) {
		/* In order to be able to loop back to an LLI, it must be in memory
		 * not in the channel registers.  So leave the registers blank for now,
		 * we'll fill them in at the end
		 */
		chan->num_llis = 0;
	}
}
EXPORT_SYMBOL(pl080_channel_begin_circular_xfer);

void pl080_channel_clear_xfers(unsigned channel_idx)
{
	struct pl080_channel *chan = &global_dma_dev->channel[channel_idx];
	int i;

	PRINTK(KERN_INFO "Clearing xfers for chan %d\n", channel_idx);
	for (i = 0; i < chan->num_llis; ++i) {
		PRINTK(KERN_INFO "Freeing pool member: %p, %08x\n", chan->lli_va[i], chan->lli_ba[i]);
		dma_pool_free(global_dma_dev->pool, chan->lli_va[i], chan->lli_ba[i]);
	}
	chan->num_llis = -1;
	chan->is_circular = 0;
}
EXPORT_SYMBOL(pl080_channel_clear_xfers);

void pl080_channel_clear_all_but_current_xfer(unsigned channel_idx)
{
	struct pl080_channel *chan = &global_dma_dev->channel[channel_idx];
	struct pl080_lli *reg_lli = IO_ADDRESS(chan->base);

	/* Current xfer will be the last one.  Since we're about to free any LLI
	 * that this might point to, terminate the list first */
	reg_lli->next_lli = 0;

	pl080_channel_clear_xfers(channel_idx);
	chan->num_llis = 0;  /* since we actually have the one current xfer going */
}
EXPORT_SYMBOL(pl080_channel_clear_all_but_current_xfer);

void pl080_channel_set_src_dest(unsigned channel_idx, int src, int dest)
{
	struct pl080_channel *chan = &global_dma_dev->channel[channel_idx];

	uint32_t ccfg = PL080_CCFG_ITC | PL080_CCFG_IE | PL080_CCFG_DEST(dest) | PL080_CCFG_SRC(src);

	if (src != PL080_DMA_MEM) {
		ccfg |= PL080_CCFG_SRC_IS_PER;
	}
	if (dest != PL080_DMA_MEM) {
		ccfg |= PL080_CCFG_DEST_IS_PER;
	}

	iowrite32(ccfg, IO_ADDRESS(chan->base + PL080_CHAN_CCFG));
}
EXPORT_SYMBOL(pl080_channel_set_src_dest);

void pl080_channel_enable(unsigned channel_idx)
{
	struct pl080_channel *chan = &global_dma_dev->channel[channel_idx];

	PRINTK(KERN_INFO "Enabling channel %d\n", channel_idx);

	if (chan->is_circular) {
		int workaround_single_block = (chan->circular_start < 0);
		if (workaround_single_block)
			chan->circular_start = 0;

		chan->lli_va[chan->num_llis-1]->next_lli = chan->lli_ba[chan->circular_start];

		if (workaround_single_block) {
			struct pl080_lli *reg_lli = IO_ADDRESS(chan->base);
			reg_lli->src      = chan->lli_va[0]->src;
			reg_lli->dest     = chan->lli_va[0]->dest;
			reg_lli->next_lli = chan->lli_va[0]->next_lli;
			reg_lli->cctl     = chan->lli_va[0]->cctl;
		}
	}

	CHAN_CCFG_BIS(PL080_CCFG_E, chan);
}
EXPORT_SYMBOL(pl080_channel_enable);

void pl080_channel_disable(unsigned channel_idx, int nicely)
{
	struct pl080_channel *chan = &global_dma_dev->channel[channel_idx];
	if (nicely) {
		CHAN_CCFG_BIS(PL080_CCFG_H, chan);  /* halt the current transfer */
		while (CHAN_CCFG_READ(chan) | PL080_CCFG_A)
			;
	}
	CHAN_CCFG_BIC(PL080_CCFG_E, chan);
	CHAN_CCFG_BIC(PL080_CCFG_H, chan);  /* no longer halted */
}

EXPORT_SYMBOL(pl080_channel_disable);

void pl080_channel_set_irq_callback(unsigned channel_idx, dma_channel_cb_t cb, void * cb_data)
{
	struct pl080_channel *chan = &global_dma_dev->channel[channel_idx];
	chan->cb = cb;
	chan->cb_data = cb_data;
}
EXPORT_SYMBOL(pl080_channel_set_irq_callback);

/*
 * Interrupt handler
 */
static irqreturn_t bcm5892_dma_int(int irq, void *dev_id)
{
	int handled = 0;
	int i;
	uint32_t int_status = PL080_READ_REG(PL080_INT_STATUS);
	uint32_t tc_status = PL080_READ_REG(PL080_TC_INT_STATUS);
	uint32_t err_status = PL080_READ_REG(PL080_ERR_INT_STATUS);
	unsigned chan_stat;
	struct pl080_channel *chan;

	for (i = 0; i < DMA_NUM_CHANNELS; ++i) {
		if (int_status & (1 << i)) {
			chan_stat =((tc_status & (1 << i)) ? STATUS_TC : 0)
				+ ((err_status & (1 << i)) ? STATUS_ERR : 0);
			if (chan_stat) {
				chan = &global_dma_dev->channel[i];
				if (chan->cb)
					(chan->cb)(i, chan_stat, chan->cb_data);
				handled = 1;
			}
		}
	}

	PL080_WRITE_REG(int_status & tc_status, PL080_TC_INT_CLEAR);
	PL080_WRITE_REG(int_status & err_status, PL080_ERR_INT_CLEAR);
	return IRQ_RETVAL(handled);
}


/* ================================================================================================ */
/*  AMBA Functions */
/* ================================================================================================ */

static int pl080_amba_probe(struct amba_device *pdev, void *id)
{
	int retval = 0;
	int i;

	PRINTK(KERN_INFO "PL080-DMA: pl080_amba_probe(pdev:%p, id:%p)\n", pdev, id);

	pdev->dma_mask = DMA_BIT_MASK(32);
	*pdev->dev.dma_mask = DMA_BIT_MASK(32);
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	/* Request the AMBA memory region */
	retval = amba_request_regions(pdev, NULL);
	if (retval) {
		dev_err(&pdev->dev, "PL080-DMA: amba_request_regions(%p, NULL) failed\n", pdev);
		return retval;
	}

	global_dma_dev = kzalloc(sizeof(struct pl080_device), GFP_KERNEL);
	if (!global_dma_dev) {
		dev_err(&pdev->dev, "PL080-DMA: could not alloc space for DMA device.\n");
		retval = -ENOMEM;
		goto err_release_region;
	}

	global_dma_dev->dev = &pdev->dev;

	retval = request_irq(pdev->irq[0], bcm5892_dma_int, 0, "dma", global_dma_dev);
	if (retval) {
		dev_err(&pdev->dev, "PL022-DMA: request_irq(%d, %p, %d, dma, %p) failed with code %d.\n",
			pdev->irq[0], bcm5892_dma_int, 0, global_dma_dev, retval);
		goto err_release_memory;
	}

	retval = bcm5892_device_create_file(&pdev->dev, &dev_attr_debug);
	if (retval) {
		dev_err(&pdev->dev, "PL022-DMA: device_create_file(0x%p, 0x%p) failed\n",
			&pdev->dev, &dev_attr_debug);
		goto err_release_irq;
	}

	global_dma_dev->pool = dma_pool_create("pl080lli", &pdev->dev, 16, 16, 0);
	if (!global_dma_dev->pool) {
		retval = -ENOMEM;
		dev_err(&pdev->dev, "PL022-DMA: dma_pool_create(pl080lli, %p, %d, %d, %d) failed.\n",
			&pdev->dev, 16, 16, 0);
		goto err_remove_debug_file;
	}

	global_dma_dev->null_src_va = dma_pool_alloc(global_dma_dev->pool, GFP_KERNEL, &global_dma_dev->null_src_ba);
	*(uint32_t*)global_dma_dev->null_src_va = 0;

	/* pool gave us 16 bytes of memory, use the second 4 bytes for the rx null xfer */
	global_dma_dev->null_dest_va = global_dma_dev->null_src_va + 4;
	global_dma_dev->null_dest_ba = global_dma_dev->null_src_ba + 4;

	global_dma_dev->base = (void* __iomem) (pdev->res.start);

	for (i = 0; i < DMA_NUM_CHANNELS; ++i) {
		global_dma_dev->channel[i].base = global_dma_dev->base + PL080_CHAN0_OFFSET +
						  PL080_INTER_CHANNEL_OFFSET * i;
		global_dma_dev->channel[i].num_llis = -1;
	}

	spin_lock_init(&global_dma_dev->channel_list_lock);

	PL080_WRITE_REG(PL080_CONFIG_ENABLE, PL080_CONFIG);


	PRINTK(KERN_INFO "PL080-DMA: pl080_amba_prob() done\n");

	return retval;

	/* Error Handling... */
err_remove_debug_file:
	bcm5892_device_remove_file(&pdev->dev, &dev_attr_debug);

err_release_irq:
	free_irq(pdev->irq[0], global_dma_dev);

err_release_memory:
	kfree(global_dma_dev);

err_release_region:
	amba_release_regions(pdev);
	return retval;
}


static int pl080_amba_remove(struct amba_device *pdev)
{
	PRINTK(KERN_INFO "PL080-DMA: pl080_amba_remove\n");

	dma_pool_free(global_dma_dev->pool, global_dma_dev->null_src_va, global_dma_dev->null_src_ba);

	dma_pool_destroy(global_dma_dev->pool);

	bcm5892_device_remove_file(&pdev->dev, &dev_attr_debug);

 	free_irq(pdev->irq[0], global_dma_dev);

	kfree(global_dma_dev);

	amba_release_regions(pdev);

	PRINTK(KERN_INFO "PL080-DMA: pl080_amba_remove() done.\n");

	return 0;
}


static struct amba_id amba_id_table[] = {
	{
		.id   = 0x00041081,
		.mask = 0x000fffff
	},
	{}
};

static struct amba_driver pl080_amba_driver = {
	.drv = {
		.name = "bcm5892-dma",
	},
	.id_table       = amba_id_table,
	.probe          = pl080_amba_probe,
	.remove         = pl080_amba_remove,
};


/* ================================================================================================
 *  Module Functions & Macros
 * ================================================================================================
 */


static int __init pl080_dma_init(void)
{
	int retval;

	PRINTK(KERN_INFO "PL080-DMA: pl080_dma_init()\n");

	PRINTK(KERN_INFO "PL080-DMA:	amba_driver_register(%p)\n", &pl080_amba_driver);
	if ((retval = amba_driver_register(&pl080_amba_driver))) {
		printk(KERN_WARNING "PL080-DMA: DMA AMBA driver register failed\n");
		return retval;
	}

	printk(KERN_INFO "PL080-DMA: Module loaded.\n");

	return 0;
}
arch_initcall(pl080_dma_init);

