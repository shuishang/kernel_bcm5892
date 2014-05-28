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

#ifndef __BCM_ALSA_INTERNAL_H__
#define __BCM_ALSA_INTERNAL_H__

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>
#include <sound/info.h>

#include <linux/broadcom/bcm_alsa.h>

//#define DEBUG_ON

#if defined(DEBUG_ON)
#   define DEBUG(args...) if (debug) printk(args)
#else
#   define DEBUG(args...) 
#endif

#define assert(expr) \
   if (!(expr)) { \
      printk("Assertion failed! %s,%s,%s,line=%d\n", \
             #expr,__FILE__,__func__,__LINE__); \
   }

#define AUDIO_MAX_OUTPUT_VOLUME 100
#define PCM_PLAYBACK_VOLUME_DEFAULT 100
#define PCM_CAPTURE_VOLUME_DEFAULT  100

typedef struct bcm_alsa_chip
{
	struct snd_card *card;
	//struct snd_pcm *pcm;
    bcm_alsa_sample_t sample[BCM_ALSA_SUBSTREAM_NUM];

	spinlock_t lock;
	int device_in_use; //only one device can be used at one time
	
	int task_index[BCM_ALSA_SUBSTREAM_NUM];

    bcm_alsa_ops_t *user_ops;
    void *user_data;
} bcm_alsa_chip_t;

#define BCM_ALSA_WQ_LENGTH 8
typedef struct {
	int index[BCM_ALSA_WQ_LENGTH];
	int cmd[BCM_ALSA_WQ_LENGTH];
	struct work_struct wq;
	int active;
} bcm_alsa_wq_t;


//variables
extern int debug;
extern bcm_alsa_chip_t *g_bcm_alsa_chip;
extern bcm_alsa_wq_t bcm_alsa_wq;

//functions
extern int __devinit bcm_alsa_pcm_new(struct snd_card *card);
extern int __devinit bcm_alsa_ctl_new(struct snd_card *card);

#endif
