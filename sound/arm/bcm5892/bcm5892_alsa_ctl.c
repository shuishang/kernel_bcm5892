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

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>


#include "bcm5892_alsa.h"
#include "bcm5892_alsa_codec.h"

#define PCM_PLAYBACK_VOLUME_DEFAULT 100
#define PCM_CAPTURE_VOLUME_DEFAULT  50

static int brcm_alsa_ctrl_info(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_info * uinfo)
{
	DEBUG("\n%lx:brcm_alsa_ctrl_info",jiffies);

	if ( (kcontrol->private_value == PCM_TYPE_PLAYBACK) || (kcontrol->private_value == PCM_TYPE_CAPTURE) )
	{
		uinfo->type			     = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count			 = 1;
		uinfo->value.integer.min = 0;
		uinfo->value.integer.max = AUDIO_MAX_OUTPUT_VOLUME;
	}
	else if (kcontrol->private_value == PCM_TYPE_PLAYBACK_MUTE)
	{
		uinfo->type			     = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
		uinfo->count			 = 1;
		uinfo->value.integer.min = 0;
		uinfo->value.integer.max = 1;
	}

	return 0;

}

static int brcm_alsa_ctrl_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	DEBUG("\n%lx:brcm_alsa_ctrl_get",jiffies);

	if (kcontrol->private_value == PCM_TYPE_PLAYBACK)
	{
		ucontrol->value.integer.value[0] = g_brcm_alsa_chip->pcm_playback_volume;
	}
	else if (kcontrol->private_value == PCM_TYPE_CAPTURE)
	{
		ucontrol->value.integer.value[0] = g_brcm_alsa_chip->pcm_capture_volume;
	}
	else if (kcontrol->private_value == PCM_TYPE_PLAYBACK_MUTE)
	{
		ucontrol->value.integer.value[0] = g_brcm_alsa_chip->pcm_playback_mute;
	}

	return 0;
}

static int brcm_alsa_ctrl_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	int changed = 0;

	DEBUG("\n%lx:brcm_alsa_ctrl_put",jiffies);

	if (kcontrol->private_value == PCM_TYPE_PLAYBACK)
	{
		if (ucontrol->value.integer.value[0] != g_brcm_alsa_chip->pcm_playback_volume)
		{
			g_brcm_alsa_chip->pcm_playback_volume = ucontrol->value.integer.value[0];
			bcm5892_codec_set_volume(g_brcm_alsa_chip->pcm_playback_volume);
		}
	}
	else if (kcontrol->private_value == PCM_TYPE_CAPTURE)
	{
		if (ucontrol->value.integer.value[0] != g_brcm_alsa_chip->pcm_capture_volume)
		{
			g_brcm_alsa_chip->pcm_capture_volume = ucontrol->value.integer.value[0];
			changed=1;
			g_brcm_alsa_chip->pcm_param_changed |= PCM_EVENT_RECD_VOLM;
		}
	}
	else if (kcontrol->private_value == PCM_TYPE_PLAYBACK_MUTE)
	{
		if (ucontrol->value.integer.value[0] != g_brcm_alsa_chip->pcm_playback_mute)
		{
			g_brcm_alsa_chip->pcm_playback_mute = ucontrol->value.integer.value[0];
			changed=1;
			g_brcm_alsa_chip->pcm_param_changed |= PCM_EVENT_PLAY_MUTE;
		}
	}

	return changed;
}


static struct snd_kcontrol_new brcm_alsa_ctrl[] __devinitdata =
{
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "PCM Playback Volume",
		.index = 1,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = PCM_TYPE_PLAYBACK,
		.info  = brcm_alsa_ctrl_info,
		.get   = brcm_alsa_ctrl_get,
		.put   = brcm_alsa_ctrl_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "PCM Capture Volume",
		.index = 1,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = PCM_TYPE_CAPTURE,
		.info  = brcm_alsa_ctrl_info,
		.get   = brcm_alsa_ctrl_get,
		.put   = brcm_alsa_ctrl_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "PCM Playback Mute",
		.index = 1,
		.access= SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = PCM_TYPE_PLAYBACK_MUTE,
		.info  = brcm_alsa_ctrl_info,
		.get   = brcm_alsa_ctrl_get,
		.put   = brcm_alsa_ctrl_put,
	},
};

int __devinit brcm_alsa_ctl_new(struct snd_card *card)
{
	unsigned int idx;
	int err;

	strcpy(card->mixername, "Broadcom ALSA Mixer");

	for (idx = 0; idx < ARRAY_SIZE(brcm_alsa_ctrl); idx++)
	{
		if ((err = snd_ctl_add(card, snd_ctl_new1(&brcm_alsa_ctrl[idx], g_brcm_alsa_chip))) < 0)
			return err;
	}

	g_brcm_alsa_chip->pcm_playback_volume = PCM_PLAYBACK_VOLUME_DEFAULT;
	g_brcm_alsa_chip->pcm_capture_volume  = PCM_CAPTURE_VOLUME_DEFAULT;
	g_brcm_alsa_chip->pcm_playback_mute   = 0;
	g_brcm_alsa_chip->pcm_param_changed   = 0;

   return 0;
}
