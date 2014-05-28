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
/*
 * bcm5892_asoc_card_internal.c  --  SoC audio for BCM5892 chip to interface with internal codec
 *
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

extern struct snd_soc_dai          bcm5892_dai_i2s;
extern struct snd_soc_dai          bcm5892_dai_internal;
extern struct snd_soc_platform     bcm5892_asoc_platform;
extern struct snd_soc_codec_device bcm5892_codec_dev_internal;


/* brcm machine dapm widgets */
static const struct snd_soc_dapm_widget bcm5892_internal_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Dac Out", NULL),
	SND_SOC_DAPM_MIC("Dac In", NULL), 
};

/* machine audio_map to bcm-internal codec
 * sink, control, source 
 */
static const struct snd_soc_dapm_route bcm5892_internal_audio_map[] = {
	{"Dac Out", NULL, "DAC_OUT"},
	{"DAC_IN", NULL, "Dac In"},
};


static int bcm5892_dai_internal_init(struct snd_soc_codec *codec)
{
	printk(KERN_INFO "bcm5892_dai_internal_init\n");
	
	/* codec pins 
	snd_soc_dapm_disable_pin(codec, "VOUT");
	snd_soc_dapm_disable_pin(codec, "VINPUT");
	*/

	/* Add brcm specific widgets */
	snd_soc_dapm_new_controls(codec, bcm5892_internal_dapm_widgets, ARRAY_SIZE(bcm5892_internal_dapm_widgets));

	/* Set up brcm specific audio paths */
	snd_soc_dapm_add_routes(codec, bcm5892_internal_audio_map, ARRAY_SIZE(bcm5892_internal_audio_map));

	snd_soc_dapm_sync(codec);
	return 0;
}

/*
 * digital audio interface glue - connects codec <--> CPU
 */
static struct snd_soc_dai_link bcm5892_dailink_internal = {
	.name = "bcm5892-asoc-dai-link",
	.stream_name = "Mono (Internal DAC)",
	.cpu_dai = &bcm5892_dai_i2s,
	.codec_dai = &bcm5892_dai_internal,
	.init = bcm5892_dai_internal_init,
	//.ops = &bcm5892_asoc_link_ops,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)

/* brcm audio machine driver */
static struct snd_soc_machine bcm5892_asoc_internal_machine = {
	.name = "BCM5892 Soc Internal Machine",
	.dai_link = &bcm5892_dailink_internal,
	.num_links = 1,
};

/* brcm audio subsystem */
static struct snd_soc_device bcm5892_asoc_devdata = {
	.machine = &bcm5892_asoc_internal_machine,
	.platform = &bcm5892_asoc_platform,
	.codec_dev = &bcm5892_codec_dev_internal,
};

#else

static struct snd_soc_card bcm5892_asoc_soc_card = {
    .name = "BCM5892 Soc Internal Card",
    .platform =  &bcm5892_asoc_platform,
    .dai_link = &bcm5892_dailink_internal,
    .num_links = 1,
};

/* brcm audio subsystem */
static struct snd_soc_device bcm5892_asoc_devdata = {
    .card = &bcm5892_asoc_soc_card,
    .codec_dev = &bcm5892_codec_dev_internal,
};
#endif // LINUX_VERSION_CODE

static struct platform_device *bcm5892_asoc_device;

static int __init bcm5892_asoc_internal_init(void)
{
	int ret = 0;
	
    // Must use "soc-audio" for it to probe the sound soc device
	bcm5892_asoc_device = platform_device_alloc("soc-audio", 0);
    if (bcm5892_asoc_device) {
	    platform_set_drvdata(bcm5892_asoc_device, &bcm5892_asoc_devdata);
	    bcm5892_asoc_devdata.dev = &bcm5892_asoc_device->dev;
	    
	    ret = platform_device_add(bcm5892_asoc_device);
	    if (ret) {			
			printk(KERN_ERR "Unable to add platform device\n");
		    platform_device_put(bcm5892_asoc_device);
		}
    }
    else {
		printk(KERN_ERR "Platform device allocation failed\n");
        return -ENOMEM;
	}
	
	return ret;
}

static void bcm5892_asoc_internal_exit(void)
{
    if (bcm5892_asoc_device) platform_device_unregister(bcm5892_asoc_device);
}
module_init(bcm5892_asoc_internal_init);
module_exit(bcm5892_asoc_internal_exit);
MODULE_DESCRIPTION("ALSA SoC BCM5892 Internal DAC");
MODULE_LICENSE("GPL");
