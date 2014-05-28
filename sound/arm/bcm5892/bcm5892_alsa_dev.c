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
#include <linux/i2c.h>
#include <linux/version.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>


#include "bcm5892_alsa.h"
#include "bcm5892_alsa_dac.h"
#include "bcm5892_alsa_codec.h"
#include "bcm5892_alsa_i2s.h"



/* ---- Module declarations ------------------------------------------------- */
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom 5892 sound interface");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,Broadcom 5892 soundcard}}");

module_param(debug, int, 0444);
MODULE_PARM_DESC(debug, "debug value for Broadcom soundcard.");

/* ---- Global ------------------------------------------------- */
brcm_alsa_chip_t *g_brcm_alsa_chip=NULL;
int debug = 1;


/* ---- External Functions ------------------------------------------------- */
extern int bcm5892_codec_i2c_init(void);
extern int bcm5892_codec_i2c_remove(void);


static int __devinit brcm_alsa_probe(struct platform_device *pdev)
{
	struct snd_card *card;
	int err;

	DEBUG("\n %lx:probe \n",jiffies);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
	err = snd_card_create(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
		THIS_MODULE, sizeof(brcm_alsa_chip_t), &card);
#else
    err = -ENOMEM;
	card = snd_card_new(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
		THIS_MODULE, sizeof(brcm_alsa_chip_t));
#endif
	if (!card)
      goto err;

	g_brcm_alsa_chip = (brcm_alsa_chip_t*)card->private_data; /* the memory has already been zero-ed in snd_card_new() */
	g_brcm_alsa_chip->card = card;
 	spin_lock_init(&g_brcm_alsa_chip->lock);

	card->dev = &pdev->dev;
	strncpy(card->driver, pdev->dev.driver->name, sizeof(card->driver));

	/*PCM interface */
	err = brcm_alsa_pcm_new(card);
	if (err)
    	goto err;

	/*CTRL interface */
	err = brcm_alsa_ctl_new(card);
	if (err)
    	goto err;

	/*TODO: other interface */


	strcpy(card->driver, "Broadcom");
	strcpy(card->shortname, "Broadcom ALSA");
	sprintf(card->longname, "Broadcom ALSA PCM %i", 0);


	err = snd_card_register(card);
	if (err==0)
	{
      platform_set_drvdata(pdev, card);
      return 0;
	}

err:
	DEBUG("\n probe failed =%d\n",err);
	if (card)
		snd_card_free(card);

	g_brcm_alsa_chip=NULL;
	return err;
}

static int brcm_alsa_remove(struct platform_device *pdev)
{
	return 0;
}

static int brcm_alsa_suspend(
		struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int brcm_alsa_resume(struct platform_device *pdev)
{
	return 0;
}


typedef struct brcm_alsa_dev
{
	int init;
} brcm_alsa_dev_t;

static brcm_alsa_dev_t brcm_alsa_device_info =
{
	.init = 0,
};

static void brcm_alsa_device_release(struct device *pdev)
{
	DEBUG("\n TO DO:what am i supposed to do\n");
}

static u64 bcm5892_dmamask = ~(u64)0;
struct platform_device brcm_alsa_device =
{
	.name		= "brcm_alsa_device",
	.dev		=
	{
		.platform_data	= &brcm_alsa_device_info,
		.dma_mask           = &bcm5892_dmamask,
		.coherent_dma_mask  = 0xffffffff, /* if not set: "aplay: set_params:910: Unable to install hw params" */
		.release = brcm_alsa_device_release,
	},
	.id		= -1,
};

static struct platform_driver brcm_alsa_driver =
{
	.probe		= brcm_alsa_probe,
	.remove 	= brcm_alsa_remove,
	.suspend	= brcm_alsa_suspend,
	.resume		= brcm_alsa_resume,
	.driver		=
		{
		.name	= "brcm_alsa_device",
		.owner	= THIS_MODULE,
		},
};

static int __devinit brcm_alsa_device_init(void)
{
	int err;

	DEBUG("\n %lx:debg=%d id=%s\n",jiffies,debug,id);

	/* do hardware related init first
	   note this does not relates to dac/codec since that will be decided later
	*/
	err = bcm5892_i2s_init();
	DEBUG("\n %lx:hw init done %d\n",jiffies,err);
	if (err)
	{
		printk(KERN_ERR "Failed to initialize I2S\n");
		return err;
	}

    /* set up the I2C */
	err = bcm5892_codec_i2c_init();
	if (err)
	{
		printk(KERN_ERR "Codec - Failed to initialize I2C\n");
		return err;
	}

	err =  platform_device_register(&brcm_alsa_device);
	DEBUG("\n %lx:device register done %d\n",jiffies,err);
	if (err)
		return err;

	err = platform_driver_register(&brcm_alsa_driver);
	DEBUG("\n %lx:driver register done %d\n",jiffies,err);

	return err;
}

static void __devexit brcm_alsa_device_exit(void)
{

	DEBUG("\n %lx:brcm_alsa_device_exit\n",jiffies);

	/* hw exit */
	bcm5892_i2s_exit();

	bcm5892_dac_config_i2s(0);

	bcm5892_codec_disable();

	bcm5892_codec_i2c_remove();

	/* alsa exit */
	snd_card_free(g_brcm_alsa_chip->card);

	platform_driver_unregister(&brcm_alsa_driver);
	platform_device_unregister(&brcm_alsa_device);

	DEBUG("\n %lx:exit done \n",jiffies);
}

module_init(brcm_alsa_device_init);
module_exit(brcm_alsa_device_exit);
