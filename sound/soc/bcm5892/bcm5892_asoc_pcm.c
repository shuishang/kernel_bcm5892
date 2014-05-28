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


#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>


#include "bcm5892_asoc.h"

extern int bcm5892_asoc_datahandling_setup(void *cb);
extern void bcm5892_asoc_datahandling_tx(int16_t *pVirData, uint16_t *pPhyData, unsigned int nLen);

/* the main context */
bcm5892_asoc_ctx_t bcm5892_asoc_ctx = {
#if BCM5892_ASOC_DMA
	.dma_channel = -1,
#endif	
};

#define BRCM_ALLOCATE_DMAMEM           1 /* we allocate memory ourself, not let Alsa do it. For some reason if Alsa do it the data are all 0s */


#define BRCM_ALSA_PLAYBACK_STREAM_NUM  1 /* one substream for playback */
#define BRCM_ALSA_CAPTURE_STREAM_NUM   1 /* one substream for record */

/******************* DAC hw paramters **********************/

#define BRCM_ALSA_DAC_PERIOD_MS               5      /* one period is set to be 10ms */
#define BRCM_ALSA_DAC_FREQHZ_MIN              8000
#define BRCM_ALSA_DAC_FREQHZ_MAX              48000
#define BRCM_ALSA_DAC_SAMPLE_BYTES            2       /* 16 bit sample */
#define BRCM_ALSA_DAC_FRAME_BYTES_MIN         2*1     /* mono */
#define BRCM_ALSA_DAC_FRAME_BYTES_MAX         2*1     /* mono */
#define BRCM_ALSA_DAC_PERIOD_BYTES_MIN        (BRCM_ALSA_DAC_PERIOD_MS * BRCM_ALSA_DAC_FREQHZ_MIN / 1000 * BRCM_ALSA_DAC_FRAME_BYTES_MIN) /* = 80, mono 8khz sampling */
#define BRCM_ALSA_DAC_PERIOD_BYTES_MAX        (BRCM_ALSA_DAC_PERIOD_MS * BRCM_ALSA_DAC_FREQHZ_MAX / 1000 * BRCM_ALSA_DAC_FRAME_BYTES_MAX) /* = 480,mono 48khz sampling */
#define BRCM_ALSA_DAC_BUFFER_PERIOD_MIN       1      /* number of periods in one buffer */
#define BRCM_ALSA_DAC_BUFFER_PERIOD_MAX       32
#define BRCM_ALSA_DAC_BUFFER_BYTES_MAX        BRCM_ALSA_DAC_PERIOD_BYTES_MAX * BRCM_ALSA_DAC_BUFFER_PERIOD_MAX

#if 0
#define BRCM_ALSA_PERIOD_NUM (8)         /* number of period per buffer */
#define BRCM_ALSA_PERIOD_FRAME (1024)    /* number of frames per period */ /* 1024*4: underrun */
#define BRCM_ALSA_FRAME_BYTE (2)         /* number of bytes per frame   */
#define BRCM_ALSA_PERIOD_BYTE (BRCM_ALSA_PERIOD_FRAME*BRCM_ALSA_FRAME_BYTE)
#define BRCM_ALSA_BUFFER_BYTE (BRCM_ALSA_PERIOD_BYTE*BRCM_ALSA_PERIOD_NUM)
#define BRCM_ALSA_BUFFER_FRAME (BRCM_ALSA_PERIOD_FRAME*BRCM_ALSA_PERIOD_NUM)
#endif

/* hardware definition */
static struct snd_pcm_hardware bcm5892_dac_hw =
{
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |	SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_U16_LE, /* unsigned 16bit */
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = BRCM_ALSA_DAC_FREQHZ_MIN,
	.rate_max = BRCM_ALSA_DAC_FREQHZ_MAX,
	.channels_min = 1,
	.channels_max = 1, /* mono only */
	.buffer_bytes_max = BRCM_ALSA_DAC_BUFFER_BYTES_MAX,
	.period_bytes_min = BRCM_ALSA_DAC_PERIOD_BYTES_MIN,
	.period_bytes_max = BRCM_ALSA_DAC_PERIOD_BYTES_MAX,
	.periods_min = BRCM_ALSA_DAC_BUFFER_PERIOD_MIN,
	.periods_max = BRCM_ALSA_DAC_BUFFER_PERIOD_MAX,
};


/******************* CODEC hw paramters **********************/

/* the parameters are mostly same as DAC except use 2 channels instead of 1 */
#define BRCM_ALSA_CODEC_PERIOD_MS               5      /* one period is set to be 10ms */
#define BRCM_ALSA_CODEC_FREQHZ_MIN              8000
#define BRCM_ALSA_CODEC_FREQHZ_MAX              48000
#define BRCM_ALSA_CODEC_SAMPLE_BYTES            2       /* 16 bit sample */
#define BRCM_ALSA_CODEC_FRAME_BYTES_MIN         2*2     /* mono */
#define BRCM_ALSA_CODEC_FRAME_BYTES_MAX         2*2     /* stereo */
#define BRCM_ALSA_CODEC_PERIOD_BYTES_MIN        (BRCM_ALSA_CODEC_PERIOD_MS * BRCM_ALSA_CODEC_FREQHZ_MIN / 1000 * BRCM_ALSA_CODEC_FRAME_BYTES_MIN) /* = 80, mono 8khz sampling */
#define BRCM_ALSA_CODEC_PERIOD_BYTES_MAX        (BRCM_ALSA_CODEC_PERIOD_MS * BRCM_ALSA_CODEC_FREQHZ_MAX / 1000 * BRCM_ALSA_CODEC_FRAME_BYTES_MAX) /* = 480,mono 48khz sampling */
#define BRCM_ALSA_CODEC_BUFFER_PERIOD_MIN       1      /* number of periods in one buffer */
#define BRCM_ALSA_CODEC_BUFFER_PERIOD_MAX       32
#define BRCM_ALSA_CODEC_BUFFER_BYTES_MAX        BRCM_ALSA_CODEC_PERIOD_BYTES_MAX * BRCM_ALSA_CODEC_BUFFER_PERIOD_MAX


/* hardware definition */
static struct snd_pcm_hardware bcm5892_codec_hw =
{
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |	SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE, /* signed 16bit */
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = BRCM_ALSA_CODEC_FREQHZ_MIN,
	.rate_max = BRCM_ALSA_CODEC_FREQHZ_MAX,
	.channels_min = 1, /*1,*/
	.channels_max = 2, /*1,*/
	.buffer_bytes_max = BRCM_ALSA_CODEC_BUFFER_BYTES_MAX,
	.period_bytes_min = BRCM_ALSA_CODEC_PERIOD_BYTES_MIN,    /* 4096: underrun */
	.period_bytes_max = BRCM_ALSA_CODEC_PERIOD_BYTES_MAX,
	.periods_min = BRCM_ALSA_CODEC_BUFFER_PERIOD_MIN,
	.periods_max = BRCM_ALSA_CODEC_BUFFER_PERIOD_MAX,
};


/* We only need to use one set of parameters. We use codec since it's larger. */
#define BRCM_ALSA_BUFFER_BYTES_MAX BRCM_ALSA_CODEC_BUFFER_BYTES_MAX


/******************* DAC specific callbacks **********************/

static int bcm5892_asoc_dac_pcm_open(struct snd_pcm_substream * substream)
{
	printk(KERN_INFO ":%s:\n",__FUNCTION__);

	snd_soc_set_runtime_hwparams(substream, &bcm5892_dac_hw);

	return 0;
}


/******************* CODEC specific callbacks **********************/

static int bcm5892_asoc_codec_pcm_open(struct snd_pcm_substream * substream)
{
	printk(KERN_INFO ":%s:\n",__FUNCTION__);

	snd_soc_set_runtime_hwparams(substream, &bcm5892_codec_hw);

	return 0;
}

/******************* Common callbacks **********************/

/* Note this function could be called multiple times */
static int bcm5892_asoc_pcm_hw_params(struct snd_pcm_substream * substream, struct snd_pcm_hw_params * params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err;
	
	printk(KERN_INFO ":%s: Enter\n",__FUNCTION__);

	DEBUG("\t params_access=%d\n",params_access(hw_params));
	DEBUG("\t params_format=%d\n",params_format(hw_params));
	DEBUG("\t params_subformat=%d\n",params_subformat(hw_params));
	DEBUG("\t params_channels=%d\n",params_channels(hw_params));
	DEBUG("\t params_rate=%d\n",params_rate(hw_params));
	DEBUG("\t params_period_size=%d\n",params_period_size(hw_params));
	DEBUG("\t params_period_bytes=%d\n",params_period_bytes(hw_params));
	DEBUG("\t params_periods=%d\n",params_periods(hw_params));
	DEBUG("\t params_buffer_size=%d\n",params_buffer_size(hw_params));
	DEBUG("\t params_buffer_bytes=%d\n",params_buffer_bytes(hw_params));

	err = bcm5892_asoc_datahandling_setup((void *)substream); /* this will setup DMA or FIFO task */
	if (err != 0)
	{
		printk(KERN_ERR "%s: datahandling setup failed\n",__FUNCTION__);
		return err;
	}
	
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	return 0;
}

static int bcm5892_asoc_pcm_hw_free(struct snd_pcm_substream * substream)
{
	printk(KERN_INFO ":%s: Enter\n",__FUNCTION__);

	bcm5892_asoc_datahandling_free();
	
	snd_pcm_set_runtime_buffer(substream, NULL);
	
	return 0;
}

static int bcm5892_asoc_pcm_close(struct snd_pcm_substream * substream)
{

	printk(KERN_INFO ":%s:\n",__FUNCTION__);

	return 0;
}

static int bcm5892_asoc_pcm_prepare(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	printk(KERN_INFO ":%s:\n",__FUNCTION__);
	DEBUG("\n %lx:prepare \n",jiffies);

	DEBUG("\t rate=%d\n",runtime->rate);
	DEBUG("\t format=%d\n",runtime->format);
	DEBUG("\t channels=%d\n",runtime->channels);
	DEBUG("\t dma_area=%x\n",(unsigned int)runtime->dma_area);
	DEBUG("\t dma_addr=%x\n",(unsigned int)runtime->dma_addr);
	DEBUG("\t dma_bytes=%d\n",runtime->dma_bytes);
	DEBUG("\t period_size=%d\n",runtime->period_size);
	DEBUG("\t period_bytes=%d\n",frames_to_bytes(runtime, runtime->period_size));
	DEBUG("\t avail_min=%d\n",frames_to_bytes(runtime, runtime->control->avail_min));

	printk(KERN_INFO "dma_area=%x\n",(unsigned int)runtime->dma_area);
	printk(KERN_INFO "dma_addr=%x\n",(unsigned int)runtime->dma_addr);
	
	bcm5892_asoc_ctx.rate         = runtime->rate;
	bcm5892_asoc_ctx.channels     = runtime->channels;
	bcm5892_asoc_ctx.buffer_len   = runtime->dma_bytes;
	bcm5892_asoc_ctx.buffer_start_vir = (int)runtime->dma_area;
	bcm5892_asoc_ctx.buffer_start_phy = (int)runtime->dma_addr;
	bcm5892_asoc_ctx.buffer_pos   = 0;
	bcm5892_asoc_ctx.period_bytes = frames_to_bytes(runtime, runtime->period_size);

	printk(KERN_INFO "prepare: buflen=%d, period len=%d\n", bcm5892_asoc_ctx.buffer_len, bcm5892_asoc_ctx.period_bytes);

	return 0;
}


static int bcm5892_asoc_pcm_trigger(struct snd_pcm_substream * substream, int cmd)
{
	DEBUG("\n %lx:trigger cmd=%d\n",jiffies,cmd);

	switch (cmd)
	{
		case SNDRV_PCM_TRIGGER_START:
			printk(KERN_INFO ":%s: SNDRV_PCM_TRIGGER_START\n",__FUNCTION__);

			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
				atomic_set( &bcm5892_asoc_ctx.aEnabled, 1 );
				bcm5892_asoc_send_data();
			}
			break;

		case SNDRV_PCM_TRIGGER_STOP:
			printk(KERN_INFO ":%s: SNDRV_PCM_TRIGGER_STOP\n",__FUNCTION__);

			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
				atomic_set( &bcm5892_asoc_ctx.aEnabled, 0 );
			}
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t bcm5892_asoc_pcm_pointer(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t ret;

	ret = bytes_to_frames(runtime, bcm5892_asoc_ctx.buffer_pos);
	DEBUG("\n pointer buf_pos=%d\n", bcm5892_asoc_ctx.buffer_pos);

    return ret;
}

static int bcm5892_asoc_pcm_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}


/******************* PCM Functions **********************/

static struct snd_pcm_ops bcm5892_asoc_dac_pcm_ops = {
   .open =        bcm5892_asoc_dac_pcm_open,
   .close =       bcm5892_asoc_pcm_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   bcm5892_asoc_pcm_hw_params,
   .hw_free =     bcm5892_asoc_pcm_hw_free,
   .prepare =     bcm5892_asoc_pcm_prepare,
   .trigger =     bcm5892_asoc_pcm_trigger,
   .pointer =     bcm5892_asoc_pcm_pointer,
   .mmap	=     bcm5892_asoc_pcm_mmap,
};

static struct snd_pcm_ops bcm5892_asoc_codec_pcm_ops = {
   .open =        bcm5892_asoc_codec_pcm_open,
   .close =       bcm5892_asoc_pcm_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   bcm5892_asoc_pcm_hw_params,
   .hw_free =     bcm5892_asoc_pcm_hw_free,
   .prepare =     bcm5892_asoc_pcm_prepare,
   .trigger =     bcm5892_asoc_pcm_trigger,
   .pointer =     bcm5892_asoc_pcm_pointer,
   .mmap	=     bcm5892_asoc_pcm_mmap,
};

static u64 bcm5892_pcm_dmamask = DMA_BIT_MASK(64);

static int bcm5892_asoc_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = BRCM_ALSA_BUFFER_BYTES_MAX;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, BRCM_ALSA_BUFFER_BYTES_MAX, &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;
	return 0;
}

static void bcm5892_asoc_pcm_free_dma_buffer(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes, buf->area, buf->addr);
		buf->area = NULL;
	}
}

static int bcm5892_asoc_pcm_new(struct snd_card *card, struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int ret;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &bcm5892_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(64);

	if (dai->playback.channels_min) {
		ret = bcm5892_asoc_pcm_preallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			return ret;
	}

	if (dai->capture.channels_min) {
		ret = bcm5892_asoc_pcm_preallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			return ret;
	}

	return ret;

}

/* index: 0 - playback, 1 - record */
void bcm5892_asoc_int_handler(struct snd_pcm_substream *substream)
{
	unsigned long flags;

	spin_lock_irqsave(&bcm5892_asoc_ctx.lock,flags);
	bcm5892_asoc_ctx.buffer_pos += bcm5892_asoc_ctx.period_bytes;
	bcm5892_asoc_ctx.buffer_pos %= bcm5892_asoc_ctx.buffer_len;

	snd_pcm_period_elapsed(substream);
	spin_unlock_irqrestore(&bcm5892_asoc_ctx.lock,flags);

	DEBUG("\n (index %d):elapsed  pcm_pos=%d hw_ptr=%d\n", index, bcm5892_asoc_ctx.pcm_pos,(int)runtime->status->hw_ptr);
}


void bcm5892_asoc_send_data( void )
{
	unsigned char          *pVirBuffer = (unsigned char *)((unsigned int)bcm5892_asoc_ctx.buffer_start_vir + (unsigned int)bcm5892_asoc_ctx.buffer_pos); /* virtual address */
	unsigned char          *pPhyBuffer = (unsigned char *)((unsigned int)bcm5892_asoc_ctx.buffer_start_phy + (unsigned int)bcm5892_asoc_ctx.buffer_pos); /* physical address */
	unsigned int            nFilledLen = bcm5892_asoc_ctx.period_bytes;

	DEBUG("\n feed, period=%d, frames2bytes=%d, dma size=%d\n", bcm5892_asoc_ctx.period_bytes, frames_to_bytes(runtime, runtime->period_size), runtime->dma_bytes);

	bcm5892_asoc_datahandling_tx((uint16_t *)pVirBuffer, (int16_t *)pPhyBuffer, nFilledLen);
}


struct snd_soc_platform bcm5892_asoc_platform = {
	.name		= "bcm5892-asoc-audio",
	.pcm_ops 	= &bcm5892_asoc_codec_pcm_ops, /* we use codec ops here to match with BRCM_ALSA_BUFFER_BYTES_MAX */
	.pcm_new	= bcm5892_asoc_pcm_new,
	.pcm_free	= bcm5892_asoc_pcm_free_dma_buffer,
};
EXPORT_SYMBOL_GPL(bcm5892_asoc_platform);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
static int __init bcm5892_asoc_platform_init(void)
{
	return snd_soc_register_platform(&bcm5892_asoc_platform);
}
module_init(bcm5892_asoc_platform_init);

static void __exit bcm5892_asoc_platform_exit(void)
{
	snd_soc_unregister_platform(&bcm5892_asoc_platform);
}
module_exit(bcm5892_asoc_platform_exit);
#endif

MODULE_AUTHOR("Broadcom Corp.");
MODULE_DESCRIPTION("BCM5892 ASOC PCM module");
MODULE_LICENSE("GPL");

