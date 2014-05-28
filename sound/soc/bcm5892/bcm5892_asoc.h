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
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

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


/*event code */
#define PCM_EVENT_PLAY_NONE  0x00000000
#define PCM_EVENT_PLAY_START (1<<PCM_TYPE_PLAYBACK)
#define PCM_EVENT_RECD_START (1<<PCM_TYPE_CAPTURE)

#define PCM_EVENT_PLAY_VOLM  (1<<PCM_TYPE_PLAYBACK)
#define PCM_EVENT_RECD_VOLM  (1<<PCM_TYPE_CAPTURE)
#define PCM_EVENT_PLAY_MUTE  (1<<PCM_TYPE_PLAYBACK_MUTE)


/* main context */
typedef struct _bcm5892_asoc_ctx_t
{
	atomic_t  aEnabled;    /* If enabled or not */

	int rate;
	int channels;      /* number of channels */

	int buffer_len;    /* size of buffer */
	int buffer_start_vir;  /* start pointer of buffer, virtual address */
	int buffer_start_phy;  /* start pointer of buffer, physical address  */
	int buffer_pos;    /* position of current pointer */
	int period_bytes;

#if BCM5892_ASOC_DMA
    int                  dma_channel;
#else
    int                  fifo_fill_task_running;
    struct task_struct * fifo_fill_task_id;
    struct semaphore     fifo_fill_sema;
    int16_t            * fifo_fill_data;
    int32_t              fifo_fill_len;
#endif

	int pcm_param_changed;
	int pcm_playback_volume;
	int pcm_capture_volume;
	int pcm_playback_mute;

	spinlock_t lock;
} bcm5892_asoc_ctx_t;


/*variables */
extern int debug;

/*functions */
extern int __devinit brcm_alsa_ctl_new(struct snd_card *card);
int bcm5892_asoc_datahandling_setup(void *cb);
void bcm5892_asoc_datahandling_free(void);
void bcm5892_asoc_int_handler(struct snd_pcm_substream *substream);
void bcm5892_asoc_send_data( void );


#endif
