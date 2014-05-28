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
 * bcm5892_asoc_codec_internal.c -- BCM5892 ALSA ASoC internal DAC codec driver
 *
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <mach/hardware.h>
#include "bcm5892_asoc_codec_internal.h"
#include <mach/bcm5892_reg.h>

static unsigned int dac_reg_base;

/*
 * Internal DAC DAPM
 */

static struct snd_soc_dapm_widget bcm5892_codec_internal_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC_OUT", "DAC Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_ADC("DAC_IN", "DAC Record", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_INPUT("LIN"),
};

/* audio routing: sink-dapm, snd-control, source-dapm */
static const struct snd_soc_dapm_route bcm5892_codec_internal_audio_map[] = {
    {"DAC_OUT", NULL, "LOUT"},
 	{"LIN", NULL, "DAC_IN"},
};

static int bcm5892_codec_internal_add_widgets(struct snd_soc_codec *codec)
{
	printk(KERN_INFO "bcm5892_codec_internal_add_widgets\n");
	
	snd_soc_dapm_new_controls(codec, bcm5892_codec_internal_dapm_widgets, ARRAY_SIZE(bcm5892_codec_internal_dapm_widgets));

	snd_soc_dapm_add_routes(codec, bcm5892_codec_internal_audio_map, ARRAY_SIZE(bcm5892_codec_internal_audio_map));

	snd_soc_dapm_new_widgets(codec);
	
	return 0;
}

/*
 * Internal DAC DAI OPS
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
static int bcm5892_daiops_internal_prepare(struct snd_pcm_substream *substream)
#else
static int bcm5892_daiops_internal_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *cpu_dai)
#endif
{
	volatile int i;
	
	/* reset DAC first */
	*DACREG_DAC_CONFIG = DAC_SFT_RST;
	
	for (i=0; i < 100; i++);

	/* set DAC to take I2S data */
	*DACREG_DAC_CONFIG = DAC_I2S_ENABLE | DAC_I2S_SAMPLE_RIGHT | DAC_INTERPOLATION_BYP;
	
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
static int bcm5892_daiops_internal_shutdown(struct snd_pcm_substream *substream)
#else
static int bcm5892_daiops_internal_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *cpu_dai)
#endif
{
	*DACREG_DAC_CONFIG &= ~DAC_I2S_ENABLE;
	
	return 0;
}

/*
 * Internal DAC DAI
 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
static struct snd_soc_dai_ops bcm5892_dai_ops_internal = {
	//.startup = bcm5892_asoc_dac_startup,
	.shutdown = bcm5892_daiops_internal_shutdown,
	.prepare = bcm5892_daiops_internal_prepare,
	//.trigger = bcm5892_asoc_dac_trigger,
    //.hw_params = bcm5892_asoc_dac_hw_params,
};
#endif

struct snd_soc_dai bcm5892_dai_internal = {
	.name = "bcm5892-dai-dac",
	.id = 0,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	.type = SND_SOC_DAI_PCM,
#endif
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
        .rates = (SNDRV_PCM_RATE_8000  | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | 
                  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 ),
		.formats = SNDRV_PCM_FMTBIT_U16_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
        .rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000),
		.formats = SNDRV_PCM_FMTBIT_U16_LE,},
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	.ops = {
		//.startup = bcm5892_asoc_dac_startup,
		.shutdown = bcm5892_daiops_internal_shutdown,
		.prepare = bcm5892_daiops_internal_prepare,
		//.trigger = bcm5892_asoc_dac_trigger,
        //.hw_params = bcm5892_asoc_dac_hw_params,
        },
#else
	.ops = &bcm5892_dai_ops_internal,
#endif
};
EXPORT_SYMBOL_GPL(bcm5892_dai_internal);



static int bcm5892_codec_internal_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec  *codec;
	int ret = 0;

	printk(KERN_INFO "bcm5892_codec_internal_probe\n");

	dac_reg_base = IO_ADDRESS (DAC_REG_BASE_ADDR);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	codec->name  = "bcm5892-asoc-internal";
	codec->owner = THIS_MODULE;
	codec->dai   = &bcm5892_dai_internal;
	codec->num_dai = 1;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	socdev->codec = codec;
#else
    socdev->card->codec = codec;
#endif
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets); /* must do these, otherwise crash when load module */
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "bcm5892: failed to create pcms\n");
		kfree(codec);
		return ret;
	}
	
	ret = bcm5892_codec_internal_add_widgets(codec);
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	ret = snd_soc_register_card(socdev);
#else
    ret = snd_soc_init_card(socdev);
#endif
	if (ret < 0) {
		printk(KERN_ERR "bcm5892: failed to register card\n");
		snd_soc_free_pcms(socdev);
		kfree(codec);
	}


	return ret;
}


/* power down chip */
static int bcm5892_codec_internal_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
    struct snd_soc_codec *codec = socdev->codec;
#else
    struct snd_soc_codec *codec = socdev->card->codec;
#endif

	snd_soc_free_pcms(socdev);
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device bcm5892_codec_dev_internal = {
	.probe = 	bcm5892_codec_internal_probe,
	.remove = 	bcm5892_codec_internal_remove,
	//.suspend = 	bcm5892_codec_internal_suspend,
	//.resume =	bcm5892_codec_internal_resume,
};
EXPORT_SYMBOL_GPL(bcm5892_codec_dev_internal);


// Specific codes for kernel 2.6.32.xx
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
static __init int bcm5892_codec_internal_driver_init(void)
{
    return snd_soc_register_dai(&bcm5892_dai_internal);
}
module_init(bcm5892_codec_internal_driver_init);

static __exit void bcm5892_codec_internal_driver_exit(void)
{
    return snd_soc_unregister_dai(&bcm5892_dai_internal);
}
module_exit(bcm5892_codec_internal_driver_exit);
#endif

MODULE_DESCRIPTION("BCM5892 ASOC Internal DAC driver");
MODULE_LICENSE("GPL");
