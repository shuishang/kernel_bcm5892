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
#ifndef __BCM_ALSA_H__
#define __BCM_ALSA_H__

/* ---- Include Files ---------------------------------------------------- */

#include <sound/pcm.h>

/* ---- Constants and Types ---------------------------------------------- */

#if defined( __KERNEL__ )

//total number of substream we support
#define BCM_ALSA_SUBSTREAM_PLAY_NUM (4) //3  //4 playback 
#define BCM_ALSA_SUBSTREAM_RECD_NUM (1) //3  //1 capture
#define BCM_ALSA_SUBSTREAM_NUM \
	(BCM_ALSA_SUBSTREAM_PLAY_NUM + BCM_ALSA_SUBSTREAM_RECD_NUM) //3

typedef enum
{
	PCM_DEVICE_NOT_SET=0,
	PCM_DEVICE_SPEAKER,
	PCM_DEVICE_HDMI
	
} PCM_DEVICE_TYPE;

typedef enum 
{
	PCM_TYPE_PLAYBACK=0,
	PCM_TYPE_CAPTURE,
	PCM_TYPE_TOTAL,
	
	//tag it along
	PCM_TYPE_PLAYBACK_MUTE,
	PCM_TYPE_DEVICE
} PCM_TYPE;

typedef enum 
{
	PCM_EVENT_START=0, 
	PCM_EVENT_CLOSE,
	PCM_EVENT_BUFFER_DONE,
	PCM_EVENT_STOP,
	PCM_EVENT_PAUSE, //not supported yet
	PCM_EVENT_RESUME,
	
	//tag it along
	PCM_EVENT_VOLM,
	PCM_EVENT_MUTE,
	PCM_EVENT_DEVICE,
	
	PCM_EVENT_TOTAL
} PCM_EVENT_TYPE;

//event code
#define PCM_EVENT_PLAY_NONE 			 0x00000000
#define PCM_EVENT_PLAY_START 			(1<<PCM_EVENT_START)
#define PCM_EVENT_PLAY_CLOSE 			(1<<PCM_EVENT_CLOSE)
#define PCM_EVENT_PLAY_BUFFER_DONE		(1<<PCM_EVENT_BUFFER_DONE)
#define PCM_EVENT_PLAY_STOP				(1<<PCM_EVENT_STOP)
#define PCM_EVENT_PLAY_PAUSE 			(1<<PCM_EVENT_PAUSE)
#define PCM_EVENT_PLAY_RESUME			(1<<PCM_EVENT_RESUME)

#define PCM_EVENT_RECD_NONE 			 0x00000000
#define PCM_EVENT_RECD_START 			(1<<PCM_EVENT_START)
#define PCM_EVENT_RECD_CLOSE 			(1<<PCM_EVENT_CLOSE)
#define PCM_EVENT_RECD_BUFFER_DONE		(1<<PCM_EVENT_BUFFER_DONE)
#define PCM_EVENT_RECD_STOP				(1<<PCM_EVENT_STOP)
#define PCM_EVENT_RECD_PAUSE 			(1<<PCM_EVENT_PAUSE)
#define PCM_EVENT_RECD_RESUME			(1<<PCM_EVENT_RESUME)

#define PCM_EVENT_PLAY_VOLM		(1<<PCM_EVENT_VOLM)
#define PCM_EVENT_RECD_VOLM		(1<<PCM_EVENT_VOLM)
#define PCM_EVENT_PLAY_MUTE		(1<<PCM_EVENT_MUTE)

typedef struct bcm_alsa_sample
{
	struct snd_pcm_substream *pcm_substream;
	int rate;
	int buffer_bytes;
	int period_bytes;
	int pcm_ptr;
	int task_index;
	int pcm_playback_volume;
	int pcm_playback_mute;
} bcm_alsa_sample_t;

typedef struct bcm_alsa_ops {
	int (*init)(void *user_data);
	int (*exit)(void *user_data);

	int (*play_start)(void *user_data, int index);
	int (*play_substream_start)(void *user_data, int index, bcm_alsa_sample_t *sample);
	int (*play_substream_stop)(void *user_data, int index);
	int (*play_stop)(void *user_data, int index);

	int (*rec_start)(void *user_data, int index);
	int (*rec_substream_start)(void *user_data, int index);
	int (*rec_substream_stop)(void *user_data, int index);
	int (*rec_stop)(void *user_data, int index);

	int (*param_changed)(void *user_data, int index, int event);

	int (*substream_suspend)(void *user_data, int index);
	int (*substream_suspended)(void *user_data, int index);
} bcm_alsa_ops_t;

#endif

/* ---- Function Prototypes --------------------------------------- */

#if defined( __KERNEL__ )

int bcm_alsa_connected(size_t user_data_len, bcm_alsa_ops_t *user_ops);
int bcm_alsa_disconnected(void);
int bcm_alsa_suspend(void);
int bcm_alsa_resume(void);
void bcm_alsa_pcm_int_handler(int index);

#endif

#endif
