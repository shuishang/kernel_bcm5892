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

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

#include "bcm_alsa_internal.h"

static int bcm_alsa_ctrl_info(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_info * uinfo)
{
	DEBUG("\n%lx:bcm_alsa_ctrl_info\n",jiffies);
	
	if  (kcontrol->private_value == PCM_TYPE_PLAYBACK)
	{	     	 
		uinfo->type					= SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count				= 1;
		uinfo->value.integer.min	= 0;
		uinfo->value.integer.max	= AUDIO_MAX_OUTPUT_VOLUME;
	}
	else if (kcontrol->private_value == PCM_TYPE_PLAYBACK_MUTE)
	{
		uinfo->type					= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
		uinfo->count				= 1;
		uinfo->value.integer.min	= 0;
		uinfo->value.integer.max	= 1;		
	}
	else if (kcontrol->private_value == PCM_TYPE_DEVICE)
	{
		uinfo->type					= SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count				= 1;
		uinfo->value.integer.min	= PCM_DEVICE_SPEAKER;
		uinfo->value.integer.max	= PCM_DEVICE_HDMI;		
	}
/*	
	else if  (kcontrol->private_value == PCM_TYPE_CAPTURE) 
	{	     	 
		uinfo->type					= SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count					= BCM_ALSA_SUBSTREAM_RECD_NUM;
		uinfo->value.integer.min	= 0;
		uinfo->value.integer.max	= AUDIO_MAX_OUTPUT_VOLUME;
	}
*/	
	return 0;
	
}

static int bcm_alsa_ctrl_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	int index;

	index = snd_ctl_get_ioffidx(kcontrol, &ucontrol->id);
	DEBUG("\n%lx:bcm_alsa_ctrl_get %d\n",jiffies,index);
	
	if (kcontrol->private_value == PCM_TYPE_PLAYBACK)
	{
		assert(index>=0 && index<BCM_ALSA_SUBSTREAM_PLAY_NUM);
		ucontrol->value.integer.value[0] = g_bcm_alsa_chip->sample[index].pcm_playback_volume;
	}
	else if (kcontrol->private_value == PCM_TYPE_PLAYBACK_MUTE)
	{
		assert(index>=0 && index<BCM_ALSA_SUBSTREAM_PLAY_NUM);
		ucontrol->value.integer.value[0] = g_bcm_alsa_chip->sample[index].pcm_playback_mute;
	}
	else if (kcontrol->private_value == PCM_TYPE_DEVICE)
	{
		assert(index>=0 && index<BCM_ALSA_SUBSTREAM_PLAY_NUM);
		ucontrol->value.integer.value[0] = g_bcm_alsa_chip->device_in_use;
	}
/*
	else if (kcontrol->private_value == PCM_TYPE_CAPTURE)
	{
		assert(index>=0 && index<BCM_ALSA_SUBSTREAM_RECD_NUM);
		ucontrol->value.integer.value[0] = g_bcm_alsa_chip->pcm_capture_volume[index];
	}
*/		
	return 0;
}

static int bcm_alsa_ctrl_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	int changed = 0;
	int index;

	index = snd_ctl_get_ioffidx(kcontrol, &ucontrol->id);
	DEBUG("\n%lx:bcm_alsa_ctrl_put %d\n",jiffies,index);
	
	if (kcontrol->private_value == PCM_TYPE_PLAYBACK)
	{
		assert(index>=0 && index<BCM_ALSA_SUBSTREAM_PLAY_NUM);
		//reset mute: i treat any volume change event as unmute
		if (g_bcm_alsa_chip->sample[index].pcm_playback_mute & 1)
		{
			g_bcm_alsa_chip->sample[index].pcm_playback_mute = 0;
			changed = 1;
		}
		
		if ( changed || (ucontrol->value.integer.value[0] != g_bcm_alsa_chip->sample[index].pcm_playback_volume) )
		{
			g_bcm_alsa_chip->sample[index].pcm_playback_volume = ucontrol->value.integer.value[0];
			changed=1;
    		g_bcm_alsa_chip->user_ops->param_changed(g_bcm_alsa_chip->user_data,
                                                      index, PCM_EVENT_VOLM);
		}	
		
	}
	else if (kcontrol->private_value == PCM_TYPE_PLAYBACK_MUTE)
	{
		assert(index>=0 && index<BCM_ALSA_SUBSTREAM_PLAY_NUM);
		if (ucontrol->value.integer.value[0] != (g_bcm_alsa_chip->sample[index].pcm_playback_mute & 1))
		{
			g_bcm_alsa_chip->sample[index].pcm_playback_mute = ucontrol->value.integer.value[0];
			changed=1;
    		g_bcm_alsa_chip->user_ops->param_changed(g_bcm_alsa_chip->user_data,
                                                      index, PCM_EVENT_MUTE);
		}		
	}
	else if (kcontrol->private_value == PCM_TYPE_DEVICE)
	{
		assert(index>=0 && index<BCM_ALSA_SUBSTREAM_PLAY_NUM);
		if (ucontrol->value.integer.value[0] != g_bcm_alsa_chip->device_in_use)
		{
			g_bcm_alsa_chip->device_in_use= ucontrol->value.integer.value[0];
			changed=1;
    		g_bcm_alsa_chip->user_ops->param_changed(g_bcm_alsa_chip->user_data,
                                                      index, PCM_EVENT_DEVICE);
		}		
	}
/*
	else if (kcontrol->private_value == PCM_TYPE_CAPTURE)
	{
		if (ucontrol->value.integer.value[0] != g_bcm_alsa_chip->pcm_capture_volume)
		{
			g_bcm_alsa_chip->pcm_capture_volume = ucontrol->value.integer.value[0];
			changed=1;
			g_bcm_alsa_chip->pcm_param_changed |= PCM_EVENT_RECD_VOLM;
		}		
	}
*/	
	return changed;
}


static struct snd_kcontrol_new bcm_alsa_ctrl[] __devinitdata = 
{
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "PCM Playback Volume",
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = PCM_TYPE_PLAYBACK,
		.info  = bcm_alsa_ctrl_info,
		.get   = bcm_alsa_ctrl_get,
		.put   = bcm_alsa_ctrl_put,		
		.count = BCM_ALSA_SUBSTREAM_PLAY_NUM,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "PCM Playback Switch",
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = PCM_TYPE_PLAYBACK_MUTE,
		.info  = bcm_alsa_ctrl_info,
		.get   = bcm_alsa_ctrl_get,
		.put   = bcm_alsa_ctrl_put,		
		.count = BCM_ALSA_SUBSTREAM_PLAY_NUM,
	},		
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "PCM Playback Route",
		.index = 0,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = PCM_TYPE_DEVICE,
		.info  = bcm_alsa_ctrl_info,
		.get   = bcm_alsa_ctrl_get,
		.put   = bcm_alsa_ctrl_put,		
		.count = 1,
	},		
/*
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "PCM Capture Volume",
		.index = 1,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = PCM_TYPE_CAPTURE,
		.info  = bcm_alsa_ctrl_info,
		.get   = bcm_alsa_ctrl_get,
		.put   = bcm_alsa_ctrl_put,	
		.count = BCM_ALSA_SUBSTREAM_RECD_NUM,
	},
*/
};

int __devinit bcm_alsa_ctl_new(struct snd_card *card)
{
	unsigned int idx;
	int err;

	strcpy(card->mixername, "Broadcom ALSA Mixer");

	for (idx = 0; idx < ARRAY_SIZE(bcm_alsa_ctrl); idx++)
	{
		if ((err = snd_ctl_add(card, snd_ctl_new1(&bcm_alsa_ctrl[idx], g_bcm_alsa_chip))) < 0)
			return err;
	}
   
	for (idx = 0; idx < BCM_ALSA_SUBSTREAM_PLAY_NUM; idx++)
	{
		g_bcm_alsa_chip->sample[idx].pcm_playback_volume = PCM_PLAYBACK_VOLUME_DEFAULT;
		g_bcm_alsa_chip->sample[idx].pcm_playback_mute   = 0;
	}
/*
	for (idx = 0; idx < BCM_ALSA_SUBSTREAM_RECD_NUM; idx++)
	{
		g_bcm_alsa_chip->pcm_capture_volume  = PCM_CAPTURE_VOLUME_DEFAULT;
 	}
 */
	return 0;
}
