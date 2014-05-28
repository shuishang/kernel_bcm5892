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

#ifndef __BRCM_ALSA_H__
#define __BRCM_ALSA_H__


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
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

/*#define DEBUG_ON */

#if defined(DEBUG_ON)
/*#define DEBUG(args...)  if (debug) snd_printk(args) */
#define DEBUG(args...)  if (debug) printk(args)
#else
#define DEBUG(args...)
#endif


#define AUDIO_MAX_OUTPUT_VOLUME 100

typedef enum
{
	PCM_TYPE_PLAYBACK=0,
	PCM_TYPE_CAPTURE,
	PCM_TYPE_TOTAL,

	/*tag it along */
	PCM_TYPE_PLAYBACK_MUTE
} PCM_TYPE;

typedef enum
{
	DAC_TYPE_NONE                   = 0,
	DAC_TYPE_INTERNAL_DAC           = 1,
	DAC_TYPE_EXTERNAL_CODEC         = 2,
} DAC_TYPE;

/*event code */
#define PCM_EVENT_PLAY_NONE  0x00000000
#define PCM_EVENT_PLAY_START (1<<PCM_TYPE_PLAYBACK)
#define PCM_EVENT_RECD_START (1<<PCM_TYPE_CAPTURE)

#define PCM_EVENT_PLAY_VOLM  (1<<PCM_TYPE_PLAYBACK)
#define PCM_EVENT_RECD_VOLM  (1<<PCM_TYPE_CAPTURE)
#define PCM_EVENT_PLAY_MUTE  (1<<PCM_TYPE_PLAYBACK_MUTE)


/* main context */
typedef struct brcm_alsa_chip
{
	struct snd_card *card;

	atomic_t  aEnabled[PCM_TYPE_TOTAL];    /* If enabled or not */

	struct snd_pcm_substream *substream[PCM_TYPE_TOTAL];
	int rate[PCM_TYPE_TOTAL];
	int channels[PCM_TYPE_TOTAL];      /* number of channels */

	int buffer_len[PCM_TYPE_TOTAL];    /* size of buffer */
	int buffer_start_vir[PCM_TYPE_TOTAL];  /* start pointer of buffer, virtual address */
	int buffer_start_phy[PCM_TYPE_TOTAL];  /* start pointer of buffer, physical address  */
	int buffer_pos[PCM_TYPE_TOTAL];    /* position of current pointer */
	int period_bytes[PCM_TYPE_TOTAL];

	u32       *pBuf_CPU; /* the dma buffer */
	dma_addr_t pBuf_DMA;
	u32        nBuf_Len;


	DAC_TYPE  tDac; /* to identify which dac/codec we are using */

	int pcm_param_changed;
	int pcm_playback_volume;
	int pcm_capture_volume;
	int pcm_playback_mute;

	spinlock_t lock;
} brcm_alsa_chip_t;


/*variables */
extern int debug;
extern brcm_alsa_chip_t *g_brcm_alsa_chip;


/*functions */
extern int __devinit brcm_alsa_pcm_new(struct snd_card *card);
extern int __devinit brcm_alsa_ctl_new(struct snd_card *card);
extern void snd_brcm_int_handler(int index);
extern void brcm_alsa_feed_data( void );


#endif
