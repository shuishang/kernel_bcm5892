/*****************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
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

//#include <sound/driver.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

#include "bcm_alsa_internal.h"

#define ALSA_PERIOD_NUM (8)
#define ALSA_PERIOD_FRAME (1024*4)
#define ALSA_FRAME_BYTE (4)

/* 16K */
#define ALSA_PERIOD_BYTE (ALSA_PERIOD_FRAME*ALSA_FRAME_BYTE)
/* 128K */
#define ALSA_BUFFER_BYTE (ALSA_PERIOD_BYTE*ALSA_PERIOD_NUM)
/* 32K */
#define ALSA_BUFFER_FRAME (ALSA_PERIOD_FRAME*ALSA_PERIOD_NUM)

/* hardware definition */
static struct snd_pcm_hardware bcm_playback_hw = 
{
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |	SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_RESUME),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000_96000,
	.rate_min = 8000,
	.rate_max = 96000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = ALSA_BUFFER_BYTE,
	.period_bytes_min = 4096, /* 32  .. 2 periods worked */
	.period_bytes_max = ALSA_PERIOD_BYTE,
	.periods_min = 1,
	.periods_max = ALSA_PERIOD_FRAME,
};

static struct snd_pcm_hardware bcm_capture_hw = 
{
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |	SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_RESUME),
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


//common

static int index_map[BCM_ALSA_SUBSTREAM_NUM];
bcm_alsa_wq_t bcm_alsa_wq;
static spinlock_t bcm_alsa_lock;


//we don't take chance to use substream index to access our internal data structure
//so we do a mapping
static int bcm_alsa_index_set(int ss_index)
{
	int ii;
	int index = -1;
	unsigned long flags;

	spin_lock_irqsave(&g_bcm_alsa_chip->lock,flags);

	for (ii=0;ii<BCM_ALSA_SUBSTREAM_NUM;ii++)
	{
		if (index_map[ii] == -1)
		{
			index_map[ii] = ss_index;
			index = ii;
			break;
		}
	}

	spin_unlock_irqrestore(&g_bcm_alsa_chip->lock,flags);
	
	if (index < 0)
		DEBUG("ERROR to assign an index for this substream %d", ss_index);
	
	return index;
}

static int bcm_alsa_index_map(int ss_index)
{
	int ii;
	int index = -1;

	for (ii=0;ii<BCM_ALSA_SUBSTREAM_NUM;ii++)
	{
		if (index_map[ii] == ss_index)
		{
			index = ii;
			break;
		}
	}
	
	if (index < 0)
		DEBUG("ERROR to find an index for this substream %d", ss_index);

	return index;


}


/*
 * PCM HW Params callback
 * This can be called multiple times per initialization!
 * Watch out for memory leaks. snd_pcm_lib_malloc_pages
 */
static int bcm_alsa_pcm_hw_params(struct snd_pcm_substream * substream,
                                  struct snd_pcm_hw_params * hw_params)
{
  	int index;
	struct snd_pcm_runtime *runtime = substream->runtime;

	index = bcm_alsa_index_map(substream->number);
	if (index < 0)
		return -1;
	
	DEBUG("\n %lx:hw_params [%d] %d\n",jiffies,(int)substream->number,index);
		
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
			
	g_bcm_alsa_chip->sample[index].rate         = params_rate(hw_params);
	g_bcm_alsa_chip->sample[index].period_bytes = params_period_bytes(hw_params);

	if (runtime->dma_area == NULL) 
	{
		unsigned int dma_size;
		void *dma_area;
		
		dma_size = params_buffer_bytes(hw_params);
		dma_area = vmalloc(dma_size);
		if (dma_area == NULL) {
			DEBUG("Unable to allocate buffer.\n");
			return -EINVAL;
		}

		memset(dma_area,0,dma_size);
		runtime->dma_area = dma_area;
		runtime->dma_bytes= dma_size;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		g_bcm_alsa_chip->user_ops->play_substream_start(g_bcm_alsa_chip->user_data,
                                                        index,
                                                        &g_bcm_alsa_chip->sample[index]);
	}
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
		g_bcm_alsa_chip->user_ops->rec_substream_start(g_bcm_alsa_chip->user_data,
                                                       index);
	}
	else {
		DEBUG("Unknown stream type.\n");
		return -EINVAL;
	}

	return 0;
}

/* PCM HW Free callback
 * Release resources allocated via hw_params
 * Can be called multiple times.
 * Called before close callback.
 */
static int bcm_alsa_pcm_hw_free(struct snd_pcm_substream * substream)
{
  	int index;
	struct snd_pcm_runtime *runtime = substream->runtime;

	index = bcm_alsa_index_map(substream->number);
	if (index < 0)
		return -1;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		g_bcm_alsa_chip->user_ops->play_substream_stop(g_bcm_alsa_chip->user_data,
                                                       index);
	}
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
		g_bcm_alsa_chip->user_ops->rec_substream_stop(g_bcm_alsa_chip->user_data,
                                                      index);
	}
	else {
		DEBUG("\n %lx:hw_free failed\n",jiffies);
		return -EINVAL;
	}
	
	DEBUG("\n %lx:hw_free [%d] %d\n",jiffies,substream->number,index);
	
	if (runtime->dma_area)
	{
		vfree(runtime->dma_area);
		runtime->dma_area = NULL;
		runtime->dma_bytes = 0;
	}
	return 0;
}

//static unsigned int buffer_size[]={(4096), (4096*2), ALSA_BUFFER_FRAME, (16 * 1024), (64 * 1024), (128 * 1024)};  //frames
//
#ifdef CONFIG_MACH_RAINBOW
    static unsigned int buffer_size[]={(16*1024), ALSA_BUFFER_FRAME, (64 * 1024), (128 * 1024)};
#else
    static unsigned int buffer_size[]={(16*1024), ALSA_BUFFER_FRAME};
#endif
static struct snd_pcm_hw_constraint_list constraints_buffer_size = 
{
	.count = ARRAY_SIZE(buffer_size),
	.list  = buffer_size,
	.mask  = 0,
};

//static unsigned int period_size[]={(32), (64), (128), (256), (512), (1024), (2048), ALSA_PERIOD_FRAME, (4096*2), (4096*4)};
static unsigned int period_size[]={ALSA_PERIOD_FRAME};
static struct snd_pcm_hw_constraint_list constraints_period_size = 
{
	.count = ARRAY_SIZE(period_size),
	.list  = period_size,
	.mask  = 0,
};


/**
 * Open pcm playback device callback
 */
static int bcm_alsa_pcm_playback_open(struct snd_pcm_substream * substream)
{
	bcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err = 0;
  	int index;

	index = bcm_alsa_index_set(substream->number);
	if (index < 0) {
		return -1;
	}

	runtime->hw = bcm_playback_hw;	
	runtime->dma_area = NULL;
	runtime->dma_bytes = 0;
	
	chip->sample[index].pcm_substream = substream;
	g_bcm_alsa_chip->sample[index].pcm_playback_mute = 0;
	g_bcm_alsa_chip->sample[index].pcm_playback_volume =
		PCM_PLAYBACK_VOLUME_DEFAULT;

	//make sure buffer-size is multiple of period-size
	err = snd_pcm_hw_constraint_list(runtime, 0,
			SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
			&constraints_buffer_size);

	if (err < 0) {
		DEBUG("Buffer constraint mismatch\n");
		return err;
	}

	err = snd_pcm_hw_constraint_list(runtime, 0,
			SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
			&constraints_period_size);
	if (err < 0) {
		DEBUG("Period constraint mismatch\n");
	}

	DEBUG("\n %lx:playback_open substream[%d] %d\n", jiffies,
			substream->number, index);

	return err;
}

/*
 * Close pcm playback device callback
 */
static int bcm_alsa_pcm_playback_close(struct snd_pcm_substream * substream)
{
	bcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
  	int index;

	index = bcm_alsa_index_map(substream->number);
	if (index < 0)
		return -1;

	index_map[index] = -1;
	chip->sample[index].pcm_substream = NULL;

	DEBUG("\n %lx:playback_close substream[%d] %d\n", jiffies,
			substream->number, index);

	return 0;
}

/*
 * Prepare pcm playback device callback
 * Used to setup format type, sample rate, etc.
 * Called when recovering from underruns
 */
static int bcm_alsa_pcm_playback_prepare(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
  	int index;

	index = bcm_alsa_index_map(substream->number);
	if (index < 0)
		return -1;

	DEBUG("\n %lx:playback_prepare substream[%d] %d\n",jiffies,substream->number,index);
	
	
	DEBUG("\t rate=%d\n",runtime->rate);
	DEBUG("\t format=%d\n",runtime->format);
	DEBUG("\t channels=%d\n",runtime->channels);
	DEBUG("\t dma_area=%x\n",(unsigned int)runtime->dma_area);
	DEBUG("\t dma_bytes=%d\n",runtime->dma_bytes);
	DEBUG("\t period_bytes=%d\n",frames_to_bytes(runtime, runtime->period_size));
	DEBUG("\t avail_min=%d\n",frames_to_bytes(runtime, runtime->control->avail_min));

	g_bcm_alsa_chip->sample[index].rate         = runtime->rate;
	g_bcm_alsa_chip->sample[index].buffer_bytes = runtime->dma_bytes;
	g_bcm_alsa_chip->sample[index].period_bytes = frames_to_bytes(runtime, runtime->period_size);

	return 0;
}

/**
 * Work queue handler for trigger callback bottom half
 */
static void bcm_alsa_wq_handler(struct work_struct *work)
{
	unsigned long flags;
	int cmd[BCM_ALSA_WQ_LENGTH];
	int index[BCM_ALSA_WQ_LENGTH];
	int active, i = 0;
	bcm_alsa_wq_t *info = container_of(work, bcm_alsa_wq_t, wq);

	spin_lock_irqsave(&bcm_alsa_lock, flags);
	memcpy(cmd, info->cmd, sizeof(cmd));
	memcpy(index, info->index, sizeof(index));
	active = info->active;
	info->active = 0;
	spin_unlock_irqrestore(&bcm_alsa_lock, flags);
	DEBUG("Num events: %d\n", active);

	// FIXME: Task handler can't handle multiple events elegantly :(
	// Need to serialize, or rewrite task handler.. why is it in a thread anyways?
	// This is already a hack though, my max queue length is 8(randomly choosen), but why?
	for (i = 0; i < active; i++) {
		DEBUG("\n %lx:playback_handler cmd: %d index: %d\n", jiffies,
				cmd[i], index[i]);
		switch (cmd[i]) {
		case SNDRV_PCM_TRIGGER_START:
			/* do something to start the PCM engine */
			g_bcm_alsa_chip->user_ops->play_start(g_bcm_alsa_chip->user_data,
			                                       index[i]);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			/* do something to stop the PCM engine */
			g_bcm_alsa_chip->user_ops->play_stop(g_bcm_alsa_chip->user_data,
			                                      index[i]);
			break;

		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			DEBUG("Not implemented");
			break;
		case SNDRV_PCM_TRIGGER_SUSPEND:
			{

				DEBUG("ALSA: SUSPENDING");

			}
			break;
		case SNDRV_PCM_TRIGGER_RESUME:
			{
				DEBUG("ASLA: RESUME");
				g_bcm_alsa_chip->user_ops->play_substream_start(g_bcm_alsa_chip->user_data,
				                                                index[i],
				                                                &g_bcm_alsa_chip->sample[index[i]]);
			}
			break;
		default:
			DEBUG("Invalid trigger type.\n");
			break;
			//return -EINVAL;
		}

	}
}

/**
 * Pcm playback trigger callback
 * This is callback is atomic!
 */
static int bcm_alsa_pcm_playback_trigger(struct snd_pcm_substream * substream, int cmd)
{
	int index;

	index = bcm_alsa_index_map(substream->number);
	if (index < 0)
		return -1;
	
	DEBUG("\n %lx:playback_trigger substream[%d] %d cmd=%d\n",jiffies,
			substream->number,index,cmd);

	/* Check if cmd is valid before scheduling */
	switch (cmd) 
	{
		case SNDRV_PCM_TRIGGER_STOP:
			break;
		case SNDRV_PCM_TRIGGER_START:
			break;
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_RESUME:
			break;
		default:
			return -EINVAL;
	}

	
	/* FIXME: Do we enqueue faster than we handle? */
	if(schedule_work(&bcm_alsa_wq.wq) == 0) {
		DEBUG(KERN_ERR "Playback trigger still in queue! %d\n",
				bcm_alsa_wq.active);
		//return -1;
	}
	if (bcm_alsa_wq.active >= BCM_ALSA_WQ_LENGTH) {
		return -1;
	}
	bcm_alsa_wq.index[bcm_alsa_wq.active] = index;
	bcm_alsa_wq.cmd[bcm_alsa_wq.active] = cmd;
	bcm_alsa_wq.active++;

	return 0;
}

/**
 * Current hardware position on the buffer in frames. 0 to buffer_size -1
 * This callback is atomic!
 */
static snd_pcm_uframes_t bcm_alsa_pcm_playback_pointer(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t ret;
  	int index;

	index = bcm_alsa_index_map(substream->number);
	if (index < 0)
		return -1;

	ret = bytes_to_frames(runtime, g_bcm_alsa_chip->sample[index].pcm_ptr);
	DEBUG("\n %lx:playback_pointer substream[%d] %d\n",jiffies,index,(int)ret);
    
    return ret;
}


//need to check every VC OMX release
static unsigned int rate[]={16000};
static struct snd_pcm_hw_constraint_list constraints_rate = 
{
	.count = ARRAY_SIZE(rate),
	.list  = rate,
	.mask  = 0,
};


//capture
static int bcm_alsa_pcm_capture_open(struct snd_pcm_substream * substream)
{
	bcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err;
  	int index;

	index = bcm_alsa_index_set(substream->number);
	
	runtime->hw = bcm_capture_hw;	
	chip->sample[index].pcm_substream = substream;
	
	//make sure buffer-size is multiple of period-size
	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
										&constraints_buffer_size);
	
	if (err<0)
		return err;
											
	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
										&constraints_period_size);
										
	if (err<0)
		return err;
											
	err = snd_pcm_hw_constraint_list(runtime,0,SNDRV_PCM_HW_PARAM_RATE,
										&constraints_rate);
										
	DEBUG("\n %lx:capture_open substream[%d] %d\n",jiffies,substream->number,index);
	return 0;
}

static int bcm_alsa_pcm_capture_close(struct snd_pcm_substream * substream)
{
	bcm_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
  	int index;

	index = bcm_alsa_index_map(substream->number);
	if (index < 0)
		return -1;
    
	index_map[index] = -1;
	chip->sample[index].pcm_substream = NULL;
	
	DEBUG("\n %lx:capture_close substream[%d] %d\n",jiffies,substream->number,index);
	return 0;
}

static int bcm_alsa_pcm_capture_prepare(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
  	int index;

	index = bcm_alsa_index_map(substream->number);
	if (index < 0)
		return -1;

	DEBUG("\n %lx:capture_prepare substream[%d] %d\n",jiffies,substream->number,index);
	
	DEBUG("\t rate=%d\n",runtime->rate);
	DEBUG("\t format=%d\n",runtime->format);
	DEBUG("\t channels=%d\n",runtime->channels);
	DEBUG("\t dma_area=%x\n",(unsigned int)runtime->dma_area);
	DEBUG("\t dma_bytes=%d\n",runtime->dma_bytes);
	DEBUG("\t period_bytes=%d\n",frames_to_bytes(runtime, runtime->period_size));
	DEBUG("\t avail_min=%d\n",frames_to_bytes(runtime, runtime->control->avail_min));

	g_bcm_alsa_chip->sample[index].rate         = runtime->rate;
	g_bcm_alsa_chip->sample[index].buffer_bytes = runtime->dma_bytes;
	g_bcm_alsa_chip->sample[index].period_bytes = frames_to_bytes(runtime, runtime->period_size);
	
	return 0;
}

static int bcm_alsa_pcm_capture_trigger(struct snd_pcm_substream * substream, int cmd)
{
  	int index;

	index = bcm_alsa_index_map(substream->number);
	if (index < 0)
		return -1;

	DEBUG("\n %lx:capture_trigger substream[%d] %d cmd=%d\n",jiffies,substream->number,index,cmd);
	
	switch (cmd) 
	{
		case SNDRV_PCM_TRIGGER_START:
			/* do something to start the PCM engine */
    		g_bcm_alsa_chip->user_ops->rec_start(g_bcm_alsa_chip->user_data,
                                                  index);
		break;
		
		case SNDRV_PCM_TRIGGER_STOP:
			/* do something to stop the PCM engine */
    		g_bcm_alsa_chip->user_ops->rec_stop(g_bcm_alsa_chip->user_data,
                                                 index);
		break;
		
		default:
		return -EINVAL;
	}	
	return 0;
}

static snd_pcm_uframes_t bcm_alsa_pcm_capture_pointer(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t ret;
  	int index;

	index = bcm_alsa_index_map(substream->number);
	if (index < 0)
		return -1;

	ret = bytes_to_frames(runtime, g_bcm_alsa_chip->sample[index].pcm_ptr);
	DEBUG("\n %lx:capture_pointer substream[%d] %d\n",jiffies,index,(int)ret);

	return ret;
}



//

static struct snd_pcm_ops bcm_alsa_pcm_playback_ops = {
   .open =        bcm_alsa_pcm_playback_open,
   .close =       bcm_alsa_pcm_playback_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   bcm_alsa_pcm_hw_params,
   .hw_free =     bcm_alsa_pcm_hw_free,
   .prepare =     bcm_alsa_pcm_playback_prepare,
   .trigger =     bcm_alsa_pcm_playback_trigger,
   .pointer =     bcm_alsa_pcm_playback_pointer,
};

static struct snd_pcm_ops bcm_alsa_pcm_capture_ops = {
   .open =        bcm_alsa_pcm_capture_open,
   .close =       bcm_alsa_pcm_capture_close,
   .ioctl =       snd_pcm_lib_ioctl,
   .hw_params =   bcm_alsa_pcm_hw_params,
   .hw_free =     bcm_alsa_pcm_hw_free,
   .prepare =     bcm_alsa_pcm_capture_prepare,
   .trigger =     bcm_alsa_pcm_capture_trigger,
   .pointer =     bcm_alsa_pcm_capture_pointer,
};

int __devinit bcm_alsa_pcm_new(struct snd_card *card)
{
	struct snd_pcm *pcm;
	int err;
	int ii;

	for (ii=0;ii<BCM_ALSA_SUBSTREAM_NUM;ii++)
		index_map[ii] = -1;

	//Speaker/HDMI PCM output
	err = snd_pcm_new(card, "Broadcom ALSA PCM", 0, BCM_ALSA_SUBSTREAM_PLAY_NUM, BCM_ALSA_SUBSTREAM_RECD_NUM, &pcm);
	if (err<0)
		return err;

	pcm->private_data = card->private_data;
	strcpy(pcm->name, "Broadcom ALSA PCM Speaker/HDMI");		

	INIT_WORK(&bcm_alsa_wq.wq, bcm_alsa_wq_handler);
	spin_lock_init(&bcm_alsa_lock);

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &bcm_alsa_pcm_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &bcm_alsa_pcm_capture_ops);

	pcm->info_flags = 0;
	DEBUG("\n %lx:done snd_pcm_new %s\n",jiffies,"Speaker/HDMI");
	//g_bcm_alsa_chip->pcm = pcm;
	//HDMI PCM output
/*	err = snd_pcm_new(card, "Broadcom ALSA PCM", 1, BCM_ALSA_SUBSTREAM_PLAY_NUM, BCM_ALSA_SUBSTREAM_RECD_NUM, &pcm);
	if (err<0)
		return err;
    
	pcm->private_data = card->private_data;
	strcpy(pcm->name, "Broadcom ALSA PCM HDMI");		
    
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &bcm_alsa_pcm_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &bcm_alsa_pcm_capture_ops);

	pcm->info_flags = 0;
	DEBUG("\n %lx:done snd_pcm_new %s\n",jiffies,"HDMI");*/
	
	return err;
}

/*
 * PCM Interrupt handler
 */

void bcm_alsa_pcm_int_handler(int index)
{
	struct snd_pcm_substream *substream = g_bcm_alsa_chip->sample[index].pcm_substream;
	//struct snd_pcm_runtime *runtime = substream->runtime;

	g_bcm_alsa_chip->sample[index].pcm_ptr += g_bcm_alsa_chip->sample[index].period_bytes;
	if (g_bcm_alsa_chip->sample[index].pcm_ptr >= g_bcm_alsa_chip->sample[index].buffer_bytes)
		g_bcm_alsa_chip->sample[index].pcm_ptr = 0;		
	
	snd_pcm_period_elapsed(substream);
		
	DEBUG("\n %lx:elapsed  pcm_ptr=%d\n",jiffies,
		g_bcm_alsa_chip->sample[index].pcm_ptr);
}

EXPORT_SYMBOL(bcm_alsa_pcm_int_handler);

