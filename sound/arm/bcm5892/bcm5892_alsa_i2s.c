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

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>
/*#include <asm/atomic.h> */

#include <mach/bcm5892_reg.h>
#include <mach/reg_gpio.h>


#define BCM5892_ALSA_I2S_DMA      1

#if BCM5892_ALSA_I2S_DMA
#include <mach/pl080-dma.h>
#endif

#include "bcm5892_alsa.h"
#include "bcm5892_alsa_i2s.h"
#include "bcm5892_alsa_dac.h"

struct i2s_info  gI2Sctx;


static int fifo_fill_task( void *unused );
static irqreturn_t bcm5892_i2s_irq(int irq, void *drv_ctx);
static unsigned int bcm5892_i2s_get_bclk( void );
static unsigned int bcm5892_i2s_set_sampling_rate(unsigned int nSamplingRate);
static void bcm5892_i2s_tx_set_threshold(unsigned int nThreshold);


#define DMU_BCLK_11_2896M         0
#define DMU_BCLK_12_0000M         1
#define DMU_BCLK_12_2880M         2
#define DMU_BCLK_24_5760M         3
#define DMU_BCLK_12_0000M_REFCLK  4

unsigned int aSamplingRate[][10] = {
	/* DMU_BCLK_11_2896M */ {8001, 8018, 11025, 11997, 15991, 22050, 24030, 31982, 44100, 48041},
	/* DMU_BCLK_12_0000M */ {8000, 8021, 11029, 12000, 16000, 22058, 24000, 32000, 44118, 48000},
	/* DMU_BCLK_12_2880M */ {8000, 8021, 11021, 12000, 16000, 22061, 24000, 32000, 44043, 48000},
	/* DMU_BCLK_24_5760M */ {8000, 8021, 11021, 12000, 16000, 22041, 24000, 32000, 44122, 48000}};

#define I2S_FIFO_THRESHOLD   (I2S_FIFO_DEPTH/4)

unsigned int bcm5892_i2s_init()
{
	int err;

	memset(&gI2Sctx, sizeof(struct i2s_info), 0);

	gI2Sctx.i2s_reg_base = IO_ADDRESS (I2S_REG_BASE_ADDR);
	
	
#if BCM5892_ALSA_I2S_DMA

	gI2Sctx.dma_channel = pl080_request_channel(1); /* 1 is for high priority */
	if (gI2Sctx.dma_channel < 0) return 1; /* fail */

	//init_completion(&gI2Sctx.dma_completion);
#else
	/* use a thread to do fifo filling */

	sema_init(&gI2Sctx.fifo_fill_sema, 0);

	gI2Sctx.fifo_fill_task_running = 1;
	gI2Sctx.fifo_fill_task_id = kthread_run(fifo_fill_task, 0, "fifo_fill_task");

	if(NULL == gI2Sctx.fifo_fill_task_id || IS_ERR(gI2Sctx.fifo_fill_task_id))
	{
		return 1;
	}

#endif

	err = request_irq(IRQ_OI2S, bcm5892_i2s_irq, IRQF_DISABLED, "i2s", 0);
	if (err) {
   		printk( KERN_ERR "cannot get irq - err %d\n", err);
   		return err;
   	}

	return 0;
}

void bcm5892_i2s_softreset()
{
	volatile int i;

	/* Set the reset bit and then clear it */
	*I2SREG_DATXCTRL = I2S_DATXCTRL_SRST;
	for (i= 0; i< 100; i++);
	*I2SREG_DATXCTRL = 0;
}

/* return: the DMU register BCLK field value */
static unsigned int bcm5892_i2s_get_bclk()
{
	return 3; /* FIXME: should read DMU register */
}

/* return: success 0, fail 1 */
static unsigned int bcm5892_i2s_set_sampling_rate(unsigned int nSamplingRate)
{
	unsigned int i;
	unsigned int nBlkVal;

	nBlkVal = bcm5892_i2s_get_bclk();
	if (nBlkVal == DMU_BCLK_12_0000M_REFCLK) nBlkVal = DMU_BCLK_12_0000M;

	if ((nBlkVal >= DMU_BCLK_11_2896M) && (nBlkVal <= DMU_BCLK_24_5760M))
	{
		/* valid blk values, now search in the arrary for best match */
		for (i=0; i < sizeof(aSamplingRate[0])/sizeof(aSamplingRate[0][0]); i++)
		{
			if ((aSamplingRate[nBlkVal][i] - nSamplingRate) < 200)
			{
				printk (KERN_INFO "set sampling rate to %d\n", aSamplingRate[nBlkVal][i]);
				/* found a match */
				*I2SREG_DAI2S &= ~I2S_DAIS_TX_SAMPLE_MASK;
				*I2SREG_DAI2S |= (i << I2S_DAIS_TX_SAMPLE_SHIFT);

				return 0; /* success */
			}
		}
	}

	return 1; /* fail */
}


static void bcm5892_i2s_tx_set_threshold(unsigned int nThreshold)
{
	unsigned int nVal;

	nVal = *I2SREG_DATXCTRL;
	nVal &= ~I2S_DATXCTRL_INT_PERIOD;
	nVal |= (nThreshold << I2S_DATXCTRL_INT_PERIOD_SHIFT);
	*I2SREG_DATXCTRL = nVal;
}

#if BCM5892_ALSA_I2S_DMA

/* Called when an dma transfer is done */
static void bcm5892_i2s_dma_callback(int channel, int status_mask, void *cb_data) 
{
	I2SLOG("\DMA ISR, status =%x \n", status_mask);

	if (status_mask & STATUS_TC) /* transfer complete */
	{
#if 1
		pl080_channel_clear_xfers(channel);
		pl080_channel_disable(channel, 0/*not nicely*/);

		/* if it's still enabled, we need to tell PCM and get the next period */
		if (atomic_read(&g_brcm_alsa_chip->aEnabled[PCM_TYPE_PLAYBACK]))
		{
			snd_brcm_int_handler(PCM_TYPE_PLAYBACK);

			brcm_alsa_feed_data();
		}
#endif
	}
	else /* STATUS_ERR */
	{
	}
}
#endif


unsigned int bcm5892_i2s_tx_config(bool bMono, unsigned int nSamplingRate)
{
	unsigned int nRetVal = 0;

	if (bMono)
		printk(KERN_INFO "\nconfig for mono\n");
	else
		printk(KERN_INFO "\nconfig for stero\n");

	/* Tx config */
	*I2SREG_DAI2S = (I2S_DAIS_TX_START_RIGHT | (bMono ? I2S_DAIS_TX_MONO : I2S_DAIS_TX_STEREO));


	/* Enable TX FIFO (this will not flush fifo and make the I2SREG_DADEBUG_TXP register to reflect */
	/* Set Tx interrupt threshold as well, this field can not be cleared (0 is an invalid value) so need to set by assignment */
	*I2SREG_DATXCTRL = (I2S_DATXCTRL_TX_EN | I2S_DATXCTRL_INT_EN);
	bcm5892_i2s_tx_set_threshold(I2S_FIFO_THRESHOLD);

	/* Set sampling rate */
	nRetVal = bcm5892_i2s_set_sampling_rate(nSamplingRate);

#if BCM5892_ALSA_I2S_DMA

	pl080_channel_set_src_dest(gI2Sctx.dma_channel, PL080_DMA_MEM, PL080_DMA_I2S_TX);

	pl080_channel_set_irq_callback(gI2Sctx.dma_channel, bcm5892_i2s_dma_callback, NULL /* we do not use completion since we can't wait */);
#endif

	return nRetVal;
}

unsigned int bcm5892_i2s_tx_flush()
{
	volatile int i;

	/* Flush FIFO */
	*I2SREG_DATXCTRL |= I2S_DATXCTRL_TX_FLUSH;
	for (i= 0; i< 100; i++);
	*I2SREG_DATXCTRL &= ~I2S_DATXCTRL_TX_FLUSH;

	return 0;
}

unsigned int bcm5892_i2s_tx_enable()
{

    /* I2S_DAIS_TX_EN will empty out the values in FIFO, according to Manoj's email */
	*I2SREG_DAI2S |= I2S_DAIS_TX_EN;

#if BCM5892_ALSA_I2S_DMA
	*I2SREG_DADMACTRL |= (I2S_DADMACTRL_TX_EN | I2S_DADMACTRL_TX_SIZE_1);	
#else
	gI2Sctx.fifo_fill_len  = 0;
#endif

	atomic_set( &g_brcm_alsa_chip->aEnabled[PCM_TYPE_PLAYBACK], 1 );

	return 0;
}

unsigned int bcm5892_i2s_tx_disable()
{
#if BCM5892_ALSA_I2S_DMA
	*I2SREG_DADMACTRL &= ~(I2S_DADMACTRL_TX_EN);	
#endif

	atomic_set( &g_brcm_alsa_chip->aEnabled[PCM_TYPE_PLAYBACK], 0 );

/************ We do not disable it right away, we wait till this period is done
	*I2SREG_DAI2S &= ~I2S_DAIS_TX_EN;
*******/

	return 0;
}

/* Note the data are 16bit */
/* pVirData: virtual address
   pPhyData: physical address
*/
void bcm5892_i2s_tx_buffer(int16_t *pVirData, uint16_t *pPhyData, unsigned int nLen)
{
	I2SLOG("fill a new buffer \n");

#if BCM5892_ALSA_I2S_DMA
	{
		uint32_t base_cctl;

		base_cctl =  PL080_CCTL_TCI | PL080_CCTL_SI | PL080_CCTL_DWIDTH16 | PL080_CCTL_SWIDTH16 | PL080_CCTL_DBSIZE1 | PL080_CCTL_SBSIZE1;

		//pl080_channel_disable(gI2Sctx.dma_channel, 1 /* disable nicely */);

		pl080_channel_add_xfer(gI2Sctx.dma_channel, pPhyData, (I2S_REG_BASE_ADDR+0x08) /* need to use physical address, not I2SREG_DAFIFO_DATA*/, nLen, base_cctl);

		pl080_channel_enable(gI2Sctx.dma_channel);

		//wait_for_completion(&gI2Sctx.dma_completion);
	}
#else
	{
		if (gI2Sctx.fifo_fill_len != 0) printk(KERN_CRIT "NOT 0\n");

		gI2Sctx.fifo_fill_data = pVirData;
		gI2Sctx.fifo_fill_len  = nLen;

		up(&gI2Sctx.fifo_fill_sema); /*signal new buffer comes */
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


unsigned int bcm5892_i2s_exit()
{
	I2SLOG("bcm5892_i2s_exit\n");
	
#if BCM5892_ALSA_I2S_DMA
	if (gI2Sctx.dma_channel >= 0)
	{
		pl080_release_channel(gI2Sctx.dma_channel);

		gI2Sctx.dma_channel = 0;
	}
#endif

	if (gI2Sctx.fifo_fill_task_running)
		gI2Sctx.fifo_fill_task_running = 0;

	free_irq(IRQ_OI2S, 0);

	return 0;
}

/* return: 1: tx interrupt happened; 0: no interrupt */
unsigned int bcm5892_i2s_tx_int_happened()
{
	return ((*I2SREG_DATXCTRL & I2S_DATXCTRL_INT_STATUS) ? 1: 0);
}

void bcm5892_i2s_tx_int_clear()
{
	*I2SREG_DATXCTRL |= I2S_DATXCTRL_INT_STATUS;
}


void bcm5892_dac_config_i2s(bool bI2S)
{
	unsigned int dac_reg_base = IO_ADDRESS (DAC_REG_BASE_ADDR);

#if 0 /* do not need to since DAC is internal */
	/* setup GPIO for I2S */
    reg_gpio_iotr_set_pin_type( GPIO_AUX_I2S, GPIO_PIN_TYPE_ALTERNATIVE_FUNC0 );
#endif

	/* reset DAC first */
	*DACREG_DAC_CONFIG = DAC_SFT_RST;

	if (bI2S)
	{
		/* set DAC to take I2S data */
		*DACREG_DAC_CONFIG = DAC_I2S_ENABLE | DAC_I2S_SAMPLE_RIGHT | DAC_INTERPOLATION_BYP;
	}
	else
	{
		*DACREG_DAC_CONFIG &= ~DAC_I2S_ENABLE;
	}
}

/****************************************************************************
*	i2s irq related
*
***************************************************************************/

static irqreturn_t bcm5892_i2s_irq(int irq, void *drv_ctx)
{
	I2SLOG( "bcm5892_i2s_irq");

	if (bcm5892_i2s_tx_int_happened())
		bcm5892_i2s_tx_int_clear();

#if !BCM5892_ALSA_I2S_DMA

	/* tell fifo fill task irq happened */
	up(&gI2Sctx.fifo_fill_sema);

	/* If this period is not done */
	if (!bcm5892_i2s_tx_buffer_empty())
		goto ends;

	/* if it's still enabled, we need to tell PCM and get the next period */
	if (atomic_read(&g_brcm_alsa_chip->aEnabled[PCM_TYPE_PLAYBACK]))
	{
		snd_brcm_int_handler(PCM_TYPE_PLAYBACK);

		brcm_alsa_feed_data();
	}
#else

#endif

ends:
	return IRQ_HANDLED;
}

#if !BCM5892_ALSA_I2S_DMA

/****************************************************************************
*	fifo_fill_task:
*		thread to fill FIFO
***************************************************************************/
static int fifo_fill_task( void *unused )
{
	int nFifoDepth = 0;
	bool bThresholdChanged = 0;
	unsigned long flags;

	I2SLOG ("fifo fill task started \n");

	while (gI2Sctx.fifo_fill_task_running)
	{
		/*sleep wait for start trigger */
		if (down_interruptible(&gI2Sctx.fifo_fill_sema) == 0)
		{
			if (bThresholdChanged)
				bcm5892_i2s_tx_set_threshold(I2S_FIFO_THRESHOLD);

			I2SLOG ("waked up, start to fill a new buffer, len=%d \n", gI2Sctx.fifo_fill_len);

			for (; gI2Sctx.fifo_fill_len > 0; )
			{

				nFifoDepth = I2S_GET_TXFIFO_CNT();
				if (nFifoDepth < (I2S_FIFO_DEPTH - 1))
				{ 	/* there are still room to fill */
					*I2SREG_DAFIFO_DATA = *gI2Sctx.fifo_fill_data;

					gI2Sctx.fifo_fill_len -= 2;
					gI2Sctx.fifo_fill_data ++;

					continue;
				}

				/* if the remaining length is larger than threshold then we wait for interrupt
				   otherwise don't wait, so we can always make sure we get the last interrupt for this period
				*/
				if (down_interruptible(&gI2Sctx.fifo_fill_sema) == 0)
				{
					I2SLOG ("waked up from waiting irq, partial fill,len=%d\n", gI2Sctx.fifo_fill_len);
				}
				else
					break;
			}


			spin_lock_irqsave(&g_brcm_alsa_chip->lock,flags);

			nFifoDepth = I2S_GET_TXFIFO_CNT();
			if (nFifoDepth < I2S_FIFO_THRESHOLD)
			{
				bcm5892_i2s_tx_set_threshold(nFifoDepth/2);
				bThresholdChanged = 1;
				/*output during spinlock? I2SLOG ("reset fifo depth to =%d \n", nFifoDepth/2); */
			}

			spin_unlock_irqrestore(&g_brcm_alsa_chip->lock,flags);

			/* Now wait for the interrupt */
			if (down_interruptible(&gI2Sctx.fifo_fill_sema) == 0)
			{
				I2SLOG ("waked up from waiting irq, fifo depth before wait is %d, ready for next buffer\n", nFifoDepth);
			}
			else
				break;

			/* If playback is cancelled, need to break here */
			if (!atomic_read(&g_brcm_alsa_chip->aEnabled[PCM_TYPE_PLAYBACK]))
				*I2SREG_DAI2S &= ~I2S_DAIS_TX_EN;
		}
	}

	gI2Sctx.fifo_fill_task_id = NULL;
	return 0;
}

unsigned int bcm5892_i2s_tx_buffer_empty()
{
	return (gI2Sctx.fifo_fill_len == 0);
}

#endif
