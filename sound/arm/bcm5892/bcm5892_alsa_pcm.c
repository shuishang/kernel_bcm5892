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

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>


#include "bcm5892_alsa.h"
#include "bcm5892_alsa_dac.h"
#include "bcm5892_alsa_codec.h"
#include "bcm5892_alsa_i2s.h"

extern struct platform_device brcm_alsa_device;

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
static struct snd_pcm_hardware brcm_dac_playback_hw =
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

static struct snd_pcm_hardware brcm_dac_capture_hw =
{
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |	SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 32768,
	.period_bytes_min = 4096,
	.period_bytes_max = 32768,
	.periods_min = 1,
	.periods_max = 1024,
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
static struct snd_pcm_hardware brcm_codec_playback_hw =
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

static struct snd_pcm_hardware brcm_codec_capture_hw =
{
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |	SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 32768,
	.period_bytes_min = 4096,
	.period_bytes_max = 32768,
	.periods_min = 1,
	.periods_max = 1024,
};

/******************* DAC specific callbacks **********************/

/***
static unsigned int buffer_size[] = {BRCM_ALSA_DAC_BUFFER_BYTES_MAX/BRCM_ALSA_DAC_FRAME_BYTES_MAX};  //frames BRCM_ALSA_DAC_BUFFER_FRAME
static struct snd_pcm_hw_constraint_list constraints_buffer_size =
{
	.count = ARRAY_SIZE(buffer_size),
	.list  = buffer_size,
	.mask  = 0,
};

static unsigned int period_size[] = {BRCM_ALSA_DAC_PERIOD_BYTES_MAX/BRCM_ALSA_DAC_FRAME_BYTES_MAX}; //BRCM_ALSA_DAC_PERIOD_FRAME
static struct snd_pcm_hw_constraint_list constraints_period_size =
{
	.count = ARRAY_SIZE(period_size),
	.list  = period_size,
	.mask  = 0,
};
****/

static int brcm_alsa_dac_pcm_playback_open(struct snd_pcm_substream * substream)
{
	brcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err=0;

	printk(KERN_INFO ":%s:\n",__FUNCTION__);
	DEBUG("\n %lx:dac_playback_open \n",jiffies);

	runtime->hw = brcm_dac_playback_hw;
	chip->substream[PCM_TYPE_PLAYBACK] = substream;

/****
	//make sure buffer-size is multiple of period-size
	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
										&constraints_buffer_size);

	if (err<0)
		return err;

	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
										&constraints_period_size);
****/

	DEBUG ("disable CODEC\n");
	bcm5892_codec_disable();
	DEBUG ("init DAC \n");
	bcm5892_dac_config_i2s(1); /* enable DAC I2S mode */
	g_brcm_alsa_chip->tDac = DAC_TYPE_INTERNAL_DAC;


	return err;
}

static unsigned int rate[]={8001, 8018, 11025, 11997, 15991, 22050, 24030, 31982, 44100, 48041}; /* fye FIXME, more to add here */
static struct snd_pcm_hw_constraint_list constraints_rate =
{
	.count = ARRAY_SIZE(rate),
	.list  = rate,
	.mask  = 0,
};

static int brcm_alsa_dac_pcm_capture_open(struct snd_pcm_substream * substream)
{
	brcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err=0;
	printk(KERN_INFO ":%s:\n",__FUNCTION__);

	runtime->hw = brcm_dac_capture_hw;
	chip->substream[PCM_TYPE_CAPTURE] = substream;

	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_RATE,
										&constraints_rate);

	DEBUG ("disable CODEC\n");
	bcm5892_codec_disable();
	DEBUG ("init DAC \n");
	bcm5892_dac_config_i2s(1); /* enable DAC I2S mode */
	g_brcm_alsa_chip->tDac = DAC_TYPE_INTERNAL_DAC;

	DEBUG("\n %lx:capture_open \n",jiffies);

	return err;
}


/******************* CODEC specific callbacks **********************/

static int brcm_alsa_codec_pcm_playback_open(struct snd_pcm_substream * substream)
{
	brcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err=0;

	printk(KERN_INFO ":%s:\n",__FUNCTION__);
	DEBUG("\n %lx:codec_playback_open \n",jiffies);

	runtime->hw = brcm_codec_playback_hw;
	chip->substream[PCM_TYPE_PLAYBACK] = substream;

	/* if we were using DAC, then disable DAC I2S mode */
	DEBUG ("unset internal DAC\n");
	bcm5892_dac_config_i2s(0);
	DEBUG ("init CODEC \n");
	bcm5892_codec_init();
	g_brcm_alsa_chip->tDac = DAC_TYPE_EXTERNAL_CODEC;


	return err;
}

static int brcm_alsa_codec_pcm_capture_open(struct snd_pcm_substream * substream)
{
	brcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err=0;
	printk(KERN_INFO ":%s:\n",__FUNCTION__);

	runtime->hw = brcm_codec_capture_hw;
	chip->substream[PCM_TYPE_CAPTURE] = substream;

	DEBUG("\n %lx:capture_open \n",jiffies);

	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_RATE,
										&constraints_rate);

	DEBUG ("unset internal DAC\n");
	bcm5892_dac_config_i2s(0);

	DEBUG ("init CODEC \n");
	bcm5892_codec_init();
	g_brcm_alsa_chip->tDac = DAC_TYPE_EXTERNAL_CODEC;

	return err;
}

/******************* Common callbacks **********************/

static int brcm_alsa_pcm_hw_params(struct snd_pcm_substream * substream,
                                   struct snd_pcm_hw_params * hw_params)
{
	int index;

	printk(KERN_INFO ":%s:\n",__FUNCTION__);
	DEBUG("\n %lx:hw_params %d\n",jiffies,(int)substream->stream);


	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		index = 0;
	}
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
		index = 1;
	}
	else
		return -EINVAL;

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

	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

static int brcm_alsa_pcm_hw_free(struct snd_pcm_substream * substream)
{

	DEBUG("\n %lx:hw_free \n",jiffies);
	printk(KERN_INFO ":%s:\n",__FUNCTION__);

	return snd_pcm_lib_free_pages(substream);
}

static int brcm_alsa_pcm_playback_close(struct snd_pcm_substream * substream)
{
	brcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);

	printk(KERN_INFO ":%s:\n",__FUNCTION__);
	DEBUG("\n %lx:playback_close \n",jiffies);

	bcm5892_dac_config_i2s(0);
	bcm5892_codec_disable();

	chip->substream[PCM_TYPE_PLAYBACK] = NULL;

	return 0;
}

static int brcm_alsa_pcm_playback_prepare(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int    rc;
	bool bMono;


	printk(KERN_INFO ":%s:\n",__FUNCTION__);
	DEBUG("\n %lx:playback_prepare \n",jiffies);

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
	
	g_brcm_alsa_chip->rate[PCM_TYPE_PLAYBACK]         = runtime->rate;
	g_brcm_alsa_chip->channels[PCM_TYPE_PLAYBACK]     = runtime->channels;
	g_brcm_alsa_chip->buffer_len[PCM_TYPE_PLAYBACK]   = runtime->dma_bytes;
	g_brcm_alsa_chip->buffer_start_vir[PCM_TYPE_PLAYBACK] = (int)runtime->dma_area;
	g_brcm_alsa_chip->buffer_start_phy[PCM_TYPE_PLAYBACK] = (int)runtime->dma_addr;
	g_brcm_alsa_chip->buffer_pos[PCM_TYPE_PLAYBACK]   = 0;
	g_brcm_alsa_chip->period_bytes[PCM_TYPE_PLAYBACK] = frames_to_bytes(runtime, runtime->period_size);

	printk(KERN_INFO "prepare: buflen=%d, period len=%d\n", g_brcm_alsa_chip->buffer_len[PCM_TYPE_PLAYBACK], g_brcm_alsa_chip->period_bytes[PCM_TYPE_PLAYBACK]);

   /* FIXME: Soft reset the I2S channel to ensure FIFO entirely flushed.
    *        As a consequence, hardware parameters setup previously may have to
    *        be setup again.
    */
	bcm5892_i2s_softreset();

	bMono = ((g_brcm_alsa_chip->channels[PCM_TYPE_PLAYBACK] == 1) ? true : false);

	rc = bcm5892_i2s_tx_config(bMono, g_brcm_alsa_chip->rate[PCM_TYPE_PLAYBACK]);
	if ( rc != 0 )
	{
		printk( KERN_ERR "bcm5892hal: failed to config tx rc=%i\n", rc );
		return rc;
	}

	rc = bcm5892_i2s_tx_flush();
	if ( rc != 0 )
	{
		printk( KERN_ERR "bcm5892hal: failed to config tx rc=%i\n", rc );
		return rc;
	}

	return 0;
}


static int brcm_alsa_pcm_playback_trigger(struct snd_pcm_substream * substream, int cmd)
{
	DEBUG("\n %lx:playback_trigger cmd=%d\n",jiffies,cmd);

	switch (cmd)
	{
		case SNDRV_PCM_TRIGGER_START:
			printk(KERN_INFO ":%s: SNDRV_PCM_TRIGGER_START\n",__FUNCTION__);

			bcm5892_i2s_tx_enable();
			brcm_alsa_feed_data();
			break;

		case SNDRV_PCM_TRIGGER_STOP:
			printk(KERN_INFO ":%s: SNDRV_PCM_TRIGGER_STOP\n",__FUNCTION__);

			bcm5892_i2s_tx_disable();
			bcm5892_i2s_tx_flush();
			break;

		default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t brcm_alsa_pcm_playback_pointer(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t ret;

	ret = bytes_to_frames(runtime, g_brcm_alsa_chip->buffer_pos[PCM_TYPE_PLAYBACK]);
	DEBUG("\n %lx:playback_pointer %d, buf_pos[0]=%d\n",jiffies,(int)ret, g_brcm_alsa_chip->buffer_pos[PCM_TYPE_PLAYBACK]);

    return ret;
}


static int brcm_alsa_pcm_capture_close(struct snd_pcm_substream * substream)
{
	brcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	printk(KERN_INFO ":%s:\n",__FUNCTION__);


	chip->substream[PCM_TYPE_CAPTURE] = NULL;

	DEBUG("\n %lx:capture_close \n",jiffies);
	return 0;
}

static int brcm_alsa_pcm_capture_prepare(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	DEBUG("\n %lx:capture_prepare \n",jiffies);
	printk(KERN_INFO ":%s:\n",__FUNCTION__);

	DEBUG("\t rate=%d\n",runtime->rate);
	DEBUG("\t format=%d\n",runtime->format);
	DEBUG("\t channels=%d\n",runtime->channels);
	DEBUG("\t dma_area=%x\n",(unsigned int)runtime->dma_area);
	DEBUG("\t dma_bytes=%d\n",runtime->dma_bytes);
	DEBUG("\t period_bytes=%d\n",frames_to_bytes(runtime, runtime->period_size));
	DEBUG("\t avail_min=%d\n",frames_to_bytes(runtime, runtime->control->avail_min));

	g_brcm_alsa_chip->rate[PCM_TYPE_CAPTURE]         = runtime->rate;
	g_brcm_alsa_chip->buffer_len[PCM_TYPE_CAPTURE]   = runtime->dma_bytes;
	g_brcm_alsa_chip->buffer_start_vir[PCM_TYPE_CAPTURE] = (int)runtime->dma_area;
	g_brcm_alsa_chip->buffer_start_phy[PCM_TYPE_CAPTURE] = (int)runtime->dma_addr;
	g_brcm_alsa_chip->buffer_pos[PCM_TYPE_CAPTURE]   = 0;
	g_brcm_alsa_chip->period_bytes[PCM_TYPE_CAPTURE] = frames_to_bytes(runtime, runtime->period_size);

	return 0;
}


static int brcm_alsa_pcm_capture_trigger(struct snd_pcm_substream * substream, int cmd)
{
	DEBUG("\n %lx:capture_trigger cmd=%d\n",jiffies,cmd);
	printk(KERN_INFO ":%s:\n",__FUNCTION__);

	switch (cmd)
	{
		case SNDRV_PCM_TRIGGER_START:
			/* currently empty */

		case SNDRV_PCM_TRIGGER_STOP:
			/* currently empty */

		default:
			return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t brcm_alsa_pcm_capture_pointer(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t ret;

	ret = bytes_to_frames(runtime, g_brcm_alsa_chip->buffer_pos[PCM_TYPE_CAPTURE]);
	DEBUG("\n %lx:capture_pointer %d\n",jiffies,(int)ret);

	return ret;
}


/******************* PCM Functions **********************/

static struct snd_pcm_ops brcm_alsa_dac_pcm_playback_ops = {
   .open =        brcm_alsa_dac_pcm_playback_open,
   .close =       brcm_alsa_pcm_playback_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   brcm_alsa_pcm_hw_params,
   .hw_free =     brcm_alsa_pcm_hw_free,
   .prepare =     brcm_alsa_pcm_playback_prepare,
   .trigger =     brcm_alsa_pcm_playback_trigger,
   .pointer =     brcm_alsa_pcm_playback_pointer,
};

static struct snd_pcm_ops brcm_alsa_dac_pcm_capture_ops = {
   .open =        brcm_alsa_dac_pcm_capture_open,
   .close =       brcm_alsa_pcm_capture_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   brcm_alsa_pcm_hw_params,
   .hw_free =     brcm_alsa_pcm_hw_free,
   .prepare =     brcm_alsa_pcm_capture_prepare,
   .trigger =     brcm_alsa_pcm_capture_trigger,
   .pointer =     brcm_alsa_pcm_capture_pointer,
};

static struct snd_pcm_ops brcm_alsa_codec_pcm_playback_ops = {
   .open =        brcm_alsa_codec_pcm_playback_open,
   .close =       brcm_alsa_pcm_playback_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   brcm_alsa_pcm_hw_params,
   .hw_free =     brcm_alsa_pcm_hw_free,
   .prepare =     brcm_alsa_pcm_playback_prepare,
   .trigger =     brcm_alsa_pcm_playback_trigger,
   .pointer =     brcm_alsa_pcm_playback_pointer,
};

static struct snd_pcm_ops brcm_alsa_codec_pcm_capture_ops = {
   .open =        brcm_alsa_codec_pcm_capture_open,
   .close =       brcm_alsa_pcm_capture_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   brcm_alsa_pcm_hw_params,
   .hw_free =     brcm_alsa_pcm_hw_free,
   .prepare =     brcm_alsa_pcm_capture_prepare,
   .trigger =     brcm_alsa_pcm_capture_trigger,
   .pointer =     brcm_alsa_pcm_capture_pointer,
};


int __devinit brcm_alsa_pcm_new(struct snd_card *card)
{
	struct snd_pcm *pcm;
	int err;

    /***** first the internal DAC device *****/

	err = snd_pcm_new(card, "Broadcom ALSA PCM Using Internal DAC", 0 /* index */, BRCM_ALSA_PLAYBACK_STREAM_NUM, BRCM_ALSA_CAPTURE_STREAM_NUM, &pcm);
	if (err < 0) return err;

    pcm->private_data = card->private_data;
	strcpy(pcm->name, "Broadcom ALSA PCM with Internal DAC");

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &brcm_alsa_dac_pcm_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &brcm_alsa_dac_pcm_capture_ops);

	pcm->info_flags = 0;

#if BRCM_ALLOCATE_DMAMEM /* if we allocate DMA memory then we let Alsa allocate nonDMA memory */
	err = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS, snd_dma_continuous_data(GFP_KERNEL), 0, BRCM_ALSA_DAC_BUFFER_BYTES_MAX);
#else
	err = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, &(brcm_alsa_device.dev), BRCM_ALSA_DAC_BUFFER_BYTES_MAX, BRCM_ALSA_DAC_BUFFER_BYTES_MAX);
#endif
	if (err < 0) return err;


    /***** second the external CODEC device *****/

	err = snd_pcm_new(card, "Broadcom ALSA PCM Using External Codec", 1 /* index */, BRCM_ALSA_PLAYBACK_STREAM_NUM, BRCM_ALSA_CAPTURE_STREAM_NUM, &pcm);
	if (err < 0) return err;

    pcm->private_data = card->private_data;
	strcpy(pcm->name, "Broadcom ALSA PCM with External Codec");

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &brcm_alsa_codec_pcm_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &brcm_alsa_codec_pcm_capture_ops);

	pcm->info_flags = 0;

#if BRCM_ALLOCATE_DMAMEM /* if we allocate DMA memory then we let Alsa allocate nonDMA memory */
	err = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS, snd_dma_continuous_data(GFP_KERNEL), 0, BRCM_ALSA_DAC_BUFFER_BYTES_MAX);
#else
	err = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, &(brcm_alsa_device.dev), BRCM_ALSA_CODEC_BUFFER_BYTES_MAX, BRCM_ALSA_CODEC_BUFFER_BYTES_MAX);
#endif
	if (err < 0) return err;

#if BRCM_ALLOCATE_DMAMEM
	g_brcm_alsa_chip->pBuf_CPU = (u32 *)dma_alloc_writecombine(&(brcm_alsa_device.dev), BRCM_ALSA_CODEC_BUFFER_BYTES_MAX, &g_brcm_alsa_chip->pBuf_DMA, GFP_KERNEL | GFP_DMA);
	g_brcm_alsa_chip->nBuf_Len = BRCM_ALSA_CODEC_BUFFER_BYTES_MAX;
	printk("allocated dma memory for alsa. cpu=%x dma=%x\n", g_brcm_alsa_chip->pBuf_CPU, g_brcm_alsa_chip->pBuf_DMA);
#endif

	return err;

}

/* index: 0 - playback, 1 - record */
void snd_brcm_int_handler(int index)
{
	struct snd_pcm_substream *substream = g_brcm_alsa_chip->substream[index];
	unsigned long flags;

	spin_lock_irqsave(&g_brcm_alsa_chip->lock,flags);
	g_brcm_alsa_chip->buffer_pos[index] += g_brcm_alsa_chip->period_bytes[index];
	g_brcm_alsa_chip->buffer_pos[index] %= g_brcm_alsa_chip->buffer_len[index];

	snd_pcm_period_elapsed(substream);
	spin_unlock_irqrestore(&g_brcm_alsa_chip->lock,flags);

	DEBUG("\n %lx (index %d):elapsed  pcm_pos=%d hw_ptr=%d\n",jiffies, index,
		g_brcm_alsa_chip->pcm_pos[index],(int)runtime->status->hw_ptr);
}


void brcm_alsa_feed_data( void )
{
	unsigned char          *pVirBuffer = (unsigned char *)((unsigned int)g_brcm_alsa_chip->buffer_start_vir[0] + (unsigned int)g_brcm_alsa_chip->buffer_pos[0]); /* virtual address */
	unsigned char          *pPhyBuffer = (unsigned char *)((unsigned int)g_brcm_alsa_chip->buffer_start_phy[0] + (unsigned int)g_brcm_alsa_chip->buffer_pos[0]); /* physical address */
	unsigned int            nFilledLen = g_brcm_alsa_chip->period_bytes[0];

	DEBUG("\n feed, period=%d, frames2bytes=%d, dma size=%d\n", g_brcm_alsa_chip->period_bytes[0], frames_to_bytes(runtime, runtime->period_size), runtime->dma_bytes);

#if BRCM_ALLOCATE_DMAMEM
	memcpy(g_brcm_alsa_chip->pBuf_CPU, pVirBuffer, nFilledLen);
	bcm5892_i2s_tx_buffer((uint16_t *)g_brcm_alsa_chip->pBuf_CPU, (int16_t *)g_brcm_alsa_chip->pBuf_DMA, nFilledLen);
#else	
	bcm5892_i2s_tx_buffer((uint16_t *)pVirBuffer, (int16_t *)pPhyBuffer, nFilledLen);
#endif
}


