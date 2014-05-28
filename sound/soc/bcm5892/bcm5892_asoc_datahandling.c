/*****************************************************************************
*  Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
*  http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*
*  Datahandling: for DMA or FIFO
*****************************************************************************/


#include <linux/module.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/unistd.h>
#include <linux/time.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/version.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>
/*#include <asm/atomic.h> */

#include <mach/bcm5892_reg.h>
#include <mach/reg_gpio.h>

#include "bcm5892_asoc.h"

#define PCMLOG(...)
extern bcm5892_asoc_ctx_t bcm5892_asoc_ctx;

#if BCM5892_ASOC_DMA
#include <mach/pl080-dma.h>
static void bcm5892_asoc_dma_callback(int channel, int status_mask, void *cb_data);
#else
static int bcm5892_asoc_tx_fifo_task( void *unused );
#endif


int bcm5892_asoc_datahandling_setup(void *cb)
{
#if BCM5892_ASOC_DMA

	if (bcm5892_asoc_ctx.dma_channel < 0)
	{
		bcm5892_asoc_ctx.dma_channel = pl080_request_channel(1); /* 1 is for high priority */
		if (bcm5892_asoc_ctx.dma_channel < 0) return 1; /* fail */
		
		printk(KERN_INFO "got dma channel %d for audio\n", bcm5892_asoc_ctx.dma_channel);

		//init_completion(&bcm5892_asoc_ctx.dma_completion);
		pl080_channel_set_src_dest(bcm5892_asoc_ctx.dma_channel, PL080_DMA_MEM, PL080_DMA_I2S_TX);
		pl080_channel_set_irq_callback(bcm5892_asoc_ctx.dma_channel, bcm5892_asoc_dma_callback, cb);
	}

#else
	/* use a thread to do fifo filling */

	sema_init(&bcm5892_asoc_ctx.fifo_fill_sema, 0);

	bcm5892_asoc_ctx.fifo_fill_task_running = 1;
	bcm5892_asoc_ctx.fifo_fill_task_id = kthread_run(bcm5892_asoc_tx_fifo_task, 0, "bcm5892_asoc_tx_fifo_task");

	if(NULL == bcm5892_asoc_ctx.fifo_fill_task_id || IS_ERR(bcm5892_asoc_ctx.fifo_fill_task_id))
	{
		return -1;
	}

#endif

	return 0;
}


#if BCM5892_ASOC_DMA

/* Called when an dma transfer is done */
static void bcm5892_asoc_dma_callback(int channel, int status_mask, void *cb_data) 
{
	PCMLOG("DMA ISR, status =%x \n", status_mask);

	if (status_mask & STATUS_TC) /* transfer complete */
	{
#if 1
		pl080_channel_clear_xfers(channel);
		pl080_channel_disable(channel, 0/*not nicely*/);

		/* if it's still enabled, we need to tell PCM and get the next period */
		if (atomic_read(&bcm5892_asoc_ctx.aEnabled))
		{
			bcm5892_asoc_int_handler(cb_data);

			bcm5892_asoc_send_data();
		}
#endif
	}
	else /* STATUS_ERR */
	{
	}
}
#endif


/* Note the data are 16bit */
/* pVirData: virtual address
   pPhyData: physical address
*/
void bcm5892_asoc_datahandling_tx(int16_t *pVirData, uint16_t *pPhyData, unsigned int nLen)
{
	PCMLOG("fill a new buffer \n");

#if BCM5892_ASOC_DMA
	{
		uint32_t base_cctl;

		base_cctl =  PL080_CCTL_TCI | PL080_CCTL_SI | PL080_CCTL_DWIDTH16 | PL080_CCTL_SWIDTH16 | PL080_CCTL_DBSIZE1 | PL080_CCTL_SBSIZE1;

		//pl080_channel_disable(bcm5892_asoc_ctx.dma_channel, 1 /* disable nicely */);

		pl080_channel_add_xfer(bcm5892_asoc_ctx.dma_channel, pPhyData, (I2S_REG_BASE_ADDR+0x08) /* need to use physical address, not I2SREG_DAFIFO_DATA*/, nLen, base_cctl);

		pl080_channel_enable(bcm5892_asoc_ctx.dma_channel);

		//wait_for_completion(&bcm5892_asoc_ctx.dma_completion);
	}
#else
	{
		if (bcm5892_asoc_ctx.fifo_fill_len != 0) printk(KERN_CRIT "NOT 0\n");

		bcm5892_asoc_ctx.fifo_fill_data = pVirData;
		bcm5892_asoc_ctx.fifo_fill_len  = nLen;

		up(&bcm5892_asoc_ctx.fifo_fill_sema); /*signal new buffer comes */
	}
#endif

	/* this is data dump
	{
	uint32_t i;
	printk("got new buf, vir=%x phy=%x data=\n", pVirData, pPhyData);
	for (i=0; i < nLen/2; i++) printk("%x ", pVirData[i]);
	printk("\n");
	}
	*/

}


void bcm5892_asoc_datahandling_free(void)
{
	PCMLOG("bcm5892_asoc_datahandling_free\n");
	
#if BCM5892_ASOC_DMA
	if (bcm5892_asoc_ctx.dma_channel >= 0)
	{
		pl080_release_channel(bcm5892_asoc_ctx.dma_channel);

		bcm5892_asoc_ctx.dma_channel = -1;
	}

#else

	if (bcm5892_asoc_ctx.fifo_fill_task_running)
	{
		bcm5892_asoc_ctx.fifo_fill_task_running = 0;
	}

#endif

}


#if !BCM5892_ASOC_DMA

/****************************************************************************
*	bcm5892_asoc_tx_fifo_task:
*		thread to fill FIFO
***************************************************************************/
static int bcm5892_asoc_tx_fifo_task( void *unused )
{
	int nFifoDepth = 0;
	bool bThresholdChanged = 0;
	unsigned long flags;

	PCMLOG ("fifo fill task started \n");

	while (bcm5892_asoc_ctx.fifo_fill_task_running)
	{
		/*sleep wait for start trigger */
		if (down_interruptible(&bcm5892_asoc_ctx.fifo_fill_sema) == 0)
		{
			if (bThresholdChanged)
				bcm5892_i2s_tx_set_threshold(I2S_FIFO_THRESHOLD);

			PCMLOG ("waked up, start to fill a new buffer, len=%d \n", bcm5892_asoc_ctx.fifo_fill_len);

			for (; bcm5892_asoc_ctx.fifo_fill_len > 0; )
			{

				nFifoDepth = I2S_GET_TXFIFO_CNT();
				if (nFifoDepth < (I2S_FIFO_DEPTH - 1))
				{ 	/* there are still room to fill */
					*I2SREG_DAFIFO_DATA = *bcm5892_asoc_ctx.fifo_fill_data;

					bcm5892_asoc_ctx.fifo_fill_len -= 2;
					bcm5892_asoc_ctx.fifo_fill_data ++;

					continue;
				}

				/* if the remaining length is larger than threshold then we wait for interrupt
				   otherwise don't wait, so we can always make sure we get the last interrupt for this period
				*/
				if (down_interruptible(&bcm5892_asoc_ctx.fifo_fill_sema) == 0)
				{
					PCMLOG ("waked up from waiting irq, partial fill,len=%d\n", bcm5892_asoc_ctx.fifo_fill_len);
				}
				else
					break;
			}


			spin_lock_irqsave(&bcm5892_asoc_ctx.lock,flags);

			nFifoDepth = I2S_GET_TXFIFO_CNT();
			if (nFifoDepth < I2S_FIFO_THRESHOLD)
			{
				bcm5892_i2s_tx_set_threshold(nFifoDepth/2);
				bThresholdChanged = 1;
				/*output during spinlock? PCMLOG ("reset fifo depth to =%d \n", nFifoDepth/2); */
			}

			spin_unlock_irqrestore(&bcm5892_asoc_ctx.lock,flags);

			/* Now wait for the interrupt */
			if (down_interruptible(&bcm5892_asoc_ctx.fifo_fill_sema) == 0)
			{
				PCMLOG ("waked up from waiting irq, fifo depth before wait is %d, ready for next buffer\n", nFifoDepth);
			}
			else
				break;
				
		}
	}

	bcm5892_asoc_ctx.fifo_fill_task_id = NULL;
	return 0;
}

#endif
