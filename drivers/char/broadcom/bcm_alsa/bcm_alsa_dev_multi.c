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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/version.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

#include "bcm_alsa_internal.h"

//  Module declarations.
//
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom AP sound interface");
MODULE_LICENSE("GPL v2");
MODULE_SUPPORTED_DEVICE("{{ALSA,Broadcom AP soundcard}}");

//global
bcm_alsa_chip_t *g_bcm_alsa_chip=NULL;
int debug = 1;

//
static char *id = NULL;
static int enable = 1;
static int index = 0;

//
module_param(index, int, 0444);
MODULE_PARM_DESC(index, "Index value for Broadcom soundcard.");
module_param(id, charp, 0444);
MODULE_PARM_DESC(id, "ID string for Broadcom soundcard.");
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Enable the Broadcom soundcard.");
module_param(debug, int, 0444);
MODULE_PARM_DESC(debug, "debug value for Broadcom soundcard.");


static void bcm_alsa_proc_read(struct snd_info_entry *entry, struct snd_info_buffer *buffer)
{
    snd_iprintf(buffer, "bcm alsa proc read: TODO\n");

}

int bcm_alsa_connected(size_t user_data_len, bcm_alsa_ops_t *user_ops)
{
    struct snd_card *card;
    struct snd_info_entry *entry;
    int err;

    DEBUG("\n %lx:probe \n",jiffies);

    err = -ENODEV;
    if (!enable)
        return err;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
    err = snd_card_create( SNDRV_DEFAULT_IDX1,
                           SNDRV_DEFAULT_STR1,
                           THIS_MODULE,
                           sizeof(bcm_alsa_chip_t) + user_data_len,
                           &card );
#else
    err = -ENOMEM;  
    card = snd_card_new( SNDRV_DEFAULT_IDX1,
                         SNDRV_DEFAULT_STR1, 
                         THIS_MODULE,
                         sizeof(bcm_alsa_chip_t) + user_data_len );
#endif
    if (!card)
        goto err;

    g_bcm_alsa_chip = (bcm_alsa_chip_t*)card->private_data;
    memset(g_bcm_alsa_chip,0,sizeof(bcm_alsa_chip_t) + user_data_len);

    g_bcm_alsa_chip->card = card;
    g_bcm_alsa_chip->device_in_use = PCM_DEVICE_SPEAKER;
    spin_lock_init(&g_bcm_alsa_chip->lock); 

    g_bcm_alsa_chip->user_data = &g_bcm_alsa_chip[1];
    g_bcm_alsa_chip->user_ops = user_ops;

    err = g_bcm_alsa_chip->user_ops->init(g_bcm_alsa_chip->user_data);
    DEBUG("\n %lx:user init done %d\n",jiffies,err);
    if (err)
        goto err;

    //PCM interface
    err = bcm_alsa_pcm_new(card);
    DEBUG("\n %lx:bcm_alsa_pcm_new done %d\n",jiffies,err);
    if (err)
        goto err;

    //CTRL interface
    err = bcm_alsa_ctl_new(card);
    DEBUG("\n %lx:bcm_alsa_ctl_new done %d\n",jiffies,err);
    if (err)
        goto err;


    //TODO: other interface

    strcpy(card->driver, "bcm-alsa");
    strcpy(card->shortname, "Broadcom ALSA");
    sprintf(card->longname, "Broadcom ALSA PCM %i", 0);

    //proc interface
    err = snd_card_proc_new(card,card->shortname,&entry);
    if (err)
        goto err;

    snd_info_set_text_ops(entry, g_bcm_alsa_chip, bcm_alsa_proc_read);


    err = snd_card_register(card);
    if (err==0)
    {
           return 0;
    }

err:
    DEBUG("\n probe failed =%d\n",err);
    if (card)
        snd_card_free(card);

    g_bcm_alsa_chip=NULL;
    return err;
}

EXPORT_SYMBOL(bcm_alsa_connected);

int bcm_alsa_disconnected(void)
{
    g_bcm_alsa_chip->user_ops->exit(g_bcm_alsa_chip->user_data);

    if (g_bcm_alsa_chip->card)
    {
        snd_card_free(g_bcm_alsa_chip->card);
    }
    return 0;
}

EXPORT_SYMBOL(bcm_alsa_disconnected);

int bcm_alsa_suspend(void)
{
    int i = 0;
    DEBUG("ALSA: Kernel Suspend.\n");

    //snd_power_change_state(g_bcm_alsa_chip->card, SNDRV_CTL_POWER_D3hot);
    //snd_pcm_suspend_all(g_bcm_alsa_chip->pcm);
    for (i = 0; i < BCM_ALSA_SUBSTREAM_NUM; i++) {
        if (g_bcm_alsa_chip->user_ops) {
            if (g_bcm_alsa_chip->user_ops->substream_suspend(g_bcm_alsa_chip->user_data,
                                                             i)) {
                DEBUG(KERN_ERR "Failed to suspend substream %d\n", i);
            }
        }
    }
    return 0;
}

EXPORT_SYMBOL( bcm_alsa_suspend );

int bcm_alsa_resume(void)
{
    int i = 0;
    DEBUG("ALSA: Kernel Resume .\n");

    //snd_power_change_state(g_bcm_alsa_chip->card, SNDRV_CTL_POWER_D0);
    for (i = 0; i < BCM_ALSA_SUBSTREAM_NUM; i++) {
        if (g_bcm_alsa_chip->user_ops) {
            if (g_bcm_alsa_chip->user_ops->substream_suspended(g_bcm_alsa_chip->user_data, i)) {
                DEBUG("RESUMING %d", i);
                //startPlayThread(i);
                if(schedule_work(&bcm_alsa_wq.wq) == 0) {
                    DEBUG(KERN_ERR "Playback trigger still in queue! %d\n",
                            bcm_alsa_wq.active);
                    //return -1;
                }
                if (bcm_alsa_wq.active >= BCM_ALSA_WQ_LENGTH) {
                    return -1;
                }
                bcm_alsa_wq.index[bcm_alsa_wq.active] = i;
                bcm_alsa_wq.cmd[bcm_alsa_wq.active] = SNDRV_PCM_TRIGGER_RESUME;
                bcm_alsa_wq.active++;
            } else {
                DEBUG("Not running substream %d\n", i);
            }
        }
    }

    return 0;
}

EXPORT_SYMBOL(bcm_alsa_resume);

static int __devinit bcm_alsa_device_init(void)
{
    printk(KERN_INFO "Broadcom ALSA driver init OK\n" );

    return 0;
}

static void __devexit bcm_alsa_device_exit(void)
{
    printk(KERN_INFO "Broadcom ALSA driver exit OK\n" );
}

module_init(bcm_alsa_device_init);
module_exit(bcm_alsa_device_exit);

