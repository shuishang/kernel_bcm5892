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


#define BCM5892_ALSA_I2S_DMA      1

#if BCM5892_ALSA_I2S_DMA
#include <mach/pl080-dma.h>
#endif

#include "bcm5892_asoc.h"
#include "bcm5892_asoc_i2s.h"

static unsigned int i2s_reg_base;


static void bcm5892_i2s_softreset(void);
static unsigned int bcm5892_i2s_set_sampling_rate(unsigned int nSamplingRate);
static void bcm5892_i2s_tx_set_threshold(unsigned int nThreshold);
static unsigned int bcm5892_i2s_tx_config(unsigned int nChannel, unsigned int nSamplingRate);
static unsigned int bcm5892_i2s_tx_flush(void);
static unsigned int bcm5892_i2s_tx_enable(void);
static unsigned int bcm5892_i2s_tx_disable(void);
static irqreturn_t bcm5892_asoc_i2s_irq(int irq, void *drv_ctx);


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

/****************************************************************************
*	low layer i2s functions
*
***************************************************************************/

static void bcm5892_i2s_softreset(void)
{
	volatile int i;

	/* Set the reset bit and then clear it */
	*I2SREG_DATXCTRL = I2S_DATXCTRL_SRST;
	for (i= 0; i< 100; i++);
	*I2SREG_DATXCTRL = 0;
}


/* return: success 0, fail 1 */
static unsigned int bcm5892_i2s_set_sampling_rate(unsigned int nSamplingRate)
{
	unsigned int i;
	unsigned int nBlkVal;

	nBlkVal = 3; /* the DMU register BCLK field value */
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

static unsigned int bcm5892_i2s_tx_config(unsigned int nChannel, unsigned int nSamplingRate)
{
	unsigned int nRetVal = 0;

	/* Tx config */
	*I2SREG_DAI2S = (I2S_DAIS_TX_START_RIGHT | ((nChannel == 1) ? I2S_DAIS_TX_MONO : I2S_DAIS_TX_STEREO));


	/* Enable TX FIFO (this will not flush fifo and make the I2SREG_DADEBUG_TXP register to reflect */
	/* Set Tx interrupt threshold as well, this field can not be cleared (0 is an invalid value) so need to set by assignment */
	*I2SREG_DATXCTRL = (I2S_DATXCTRL_TX_EN | I2S_DATXCTRL_INT_EN);
	bcm5892_i2s_tx_set_threshold(I2S_FIFO_THRESHOLD);

	/* Set sampling rate */
	nRetVal = bcm5892_i2s_set_sampling_rate(nSamplingRate);

	return nRetVal;
}

static unsigned int bcm5892_i2s_tx_flush(void)
{
	volatile int i;

	/* Flush FIFO */
	*I2SREG_DATXCTRL |= I2S_DATXCTRL_TX_FLUSH;
	for (i= 0; i< 100; i++);
	*I2SREG_DATXCTRL &= ~I2S_DATXCTRL_TX_FLUSH;

	return 0;
}

static unsigned int bcm5892_i2s_tx_enable(void)
{

    /* I2S_DAIS_TX_EN will empty out the values in FIFO, according to Manoj's email */
	*I2SREG_DAI2S |= I2S_DAIS_TX_EN;

#if BCM5892_ALSA_I2S_DMA
	*I2SREG_DADMACTRL |= (I2S_DADMACTRL_TX_EN | I2S_DADMACTRL_TX_SIZE_1);	
#else
	gI2Sctx.fifo_fill_len  = 0;
#endif

	return 0;
}

static unsigned int bcm5892_i2s_tx_disable(void)
{
#if BCM5892_ALSA_I2S_DMA
	*I2SREG_DADMACTRL &= ~(I2S_DADMACTRL_TX_EN);	
#endif

/************ We do not disable it right away, we wait till this period is done
	*I2SREG_DAI2S &= ~I2S_DAIS_TX_EN;
*******/

	return 0;
}


static irqreturn_t bcm5892_asoc_i2s_irq(int irq, void *drv_ctx)
{
	/* if tx interrupt happened then ack it */
	if ((*I2SREG_DATXCTRL & I2S_DATXCTRL_INT_STATUS))
		*I2SREG_DATXCTRL |= I2S_DATXCTRL_INT_STATUS;

#if !BCM5892_ALSA_I2S_DMA /* fixme: this section should be moved to pcm part of code and register a handler from there */

	/* tell fifo fill task irq happened */
	up(&gI2Sctx.fifo_fill_sema);

	/* If this period is not done */
	if (gI2Sctx.fifo_fill_len != 0)
		goto ends;

	/* if it's still enabled, we need to tell PCM and get the next period */
	if (atomic_read(&g_brcm_alsa_chip->aEnabled))
	{
		bcm5892_asoc_int_handler();

		brcm_alsa_feed_data();
	}
#endif

	return IRQ_HANDLED;
}


/****************************************************************************
*	DAI ops functions
*
***************************************************************************/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
static int bcm5892_daiops_i2s_prepare(struct snd_pcm_substream *substream)
#else
static int bcm5892_daiops_i2s_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *cpu_dai)
#endif
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;

	bcm5892_i2s_softreset();

	ret = bcm5892_i2s_tx_config(runtime->channels, runtime->rate);
	if ( ret != 0 )
	{
		printk( KERN_ERR "bcm5892 asoc: failed to config tx rc=%i\n", ret );
		return ret;
	}

	ret = bcm5892_i2s_tx_flush();
	if ( ret != 0 )
	{
		printk( KERN_ERR "bcm5892 asoc: failed to flush tx rc=%i\n", ret );
		return ret;
	}
	
	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
static int bcm5892_daiops_i2s_trigger(struct snd_pcm_substream *substream, int cmd)
#else
static int bcm5892_daiops_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *cpu_dai)
#endif
{
	int ret = 0;

	switch (cmd)
	{
		case SNDRV_PCM_TRIGGER_START:
			bcm5892_i2s_tx_enable();
			break;
		
		case SNDRV_PCM_TRIGGER_STOP:
			bcm5892_i2s_tx_disable();
			bcm5892_i2s_tx_flush();
			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}

/* interface format */
static int bcm5892_daiops_i2s_setfmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	int ret = 0;
	
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S: /* we only have this format */
			break;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBS_CFM:
		case SND_SOC_DAIFMT_CBS_CFS:
	        /* Set us as I2S master */
			break;
	    case SND_SOC_DAIFMT_CBM_CFM:
		case SND_SOC_DAIFMT_CBM_CFS:
		default:
			/* We can not be I2S slave */
			ret = -EINVAL;
			break;
	}
	return ret;
}

/****************************************************************************
*	SoC structures
*
***************************************************************************/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
static struct snd_soc_dai_ops bcm5892_daiops_i2s = {
	//.startup = bcm5892_asoc_i2s_startup,
	//.shutdown = bcm5892_asoc_i2s_shutdown,
	.prepare   = bcm5892_daiops_i2s_prepare,
	.trigger   = bcm5892_daiops_i2s_trigger,
    //.hw_params = bcm5892_daiops_i2s_hwparams,
	.set_fmt   = bcm5892_daiops_i2s_setfmt,
    //.set_sysclk = bcm5892_asoc_i2s_set_dai_sysclk,
};
#endif

#define BCM5892_I2S_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |  \
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

struct snd_soc_dai bcm5892_dai_i2s = {
	.name = "bcm5892-dai-i2s",
	.id = 0,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	.type = SND_SOC_DAI_I2S,
#endif
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = BCM5892_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = BCM5892_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE,},
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	.ops = {
		.prepare = bcm5892_daiops_i2s_prepare,
		.trigger = bcm5892_daiops_i2s_trigger,
		},
	.dai_ops = {
		.set_fmt = bcm5892_daiops_i2s_setfmt,
        //.set_sysclk = bcm_i2s_set_dai_sysclk,
        },
#else
	.ops = &bcm5892_daiops_i2s,
#endif
};

EXPORT_SYMBOL_GPL(bcm5892_dai_i2s);



/****************************************************************************
*	Platform driver related
*
***************************************************************************/

static int bcm5892_asoc_i2s_probe(struct platform_device *dev)
{
    return 0;
}

static int __devexit bcm5892_asoc_i2s_remove(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver bcm5892_asoc_i2s_driver = {
	.probe = bcm5892_asoc_i2s_probe,
	.remove = __devexit_p(bcm5892_asoc_i2s_remove),

	.driver = {
		.name = "bcm5892-asoc-i2s",
		.owner = THIS_MODULE,
	},
};


static int __init bcm5892_asoc_i2s_init(void)
{
	int ret = 0;

	i2s_reg_base = IO_ADDRESS (I2S_REG_BASE_ADDR);
	
	ret = request_irq(IRQ_OI2S, bcm5892_asoc_i2s_irq, IRQF_DISABLED, "i2s", 0);
	if (ret) {
   		printk( KERN_ERR "cannot get i2s irq - err %d\n", ret);
   		return ret;
   	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
    snd_soc_register_dai(&bcm5892_dai_i2s);
#endif

	return platform_driver_register(&bcm5892_asoc_i2s_driver);
}

static void __exit bcm5892_asoc_i2s_exit(void)
{
	free_irq(IRQ_OI2S, 0);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
    snd_soc_unregister_dai(&bcm5892_dai_i2s);
#endif
	
	platform_driver_unregister(&bcm5892_asoc_i2s_driver);
}

module_init(bcm5892_asoc_i2s_init);
module_exit(bcm5892_asoc_i2s_exit);

/* Module information */
MODULE_DESCRIPTION("bcm5892 I2S SoC Interface");
MODULE_LICENSE("GPL");
