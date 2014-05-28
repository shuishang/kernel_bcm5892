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
 * bcm5892_asoc_card_wm8750.c  --  SoC audio for BCM5892 chip to interface with wm8750 codec
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
#include <mach/reg_gpio.h>
#include "../../soc/codecs/wm8750.h"

extern struct snd_soc_dai          bcm5892_dai_i2s;
extern struct snd_soc_platform     bcm5892_asoc_platform;
extern struct snd_soc_dai          wm8750_dai; /* this is in wm8750.c */
extern struct snd_soc_codec_device soc_codec_dev_wm8750; /* this is in wm8750.c */

/*
 * dapm widgets
 */
/* brcm machine dapm widgets */
static const struct snd_soc_dapm_widget bcm5892_wm8750_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Speaker", NULL),
};

/* machine audio_map to wm8750 codec
 * sink, control, source 
 */
static const struct snd_soc_dapm_route bcm5892_wm8750_audio_map[] = {
	{ "Ext Speaker", NULL, "LOUT1" },
	{ "Ext Speaker", NULL, "ROUT1" },
};


/*
 * init
 */

static int bcm5892_wm8750_init(struct snd_soc_codec *codec)
{
	int err;
	int i;

#if 1
	/* These endpoints are not being used. */
	snd_soc_dapm_disable_pin(codec, "RINPUT1");
	snd_soc_dapm_disable_pin(codec, "LINPUT2");
	snd_soc_dapm_disable_pin(codec, "RINPUT2");
	snd_soc_dapm_disable_pin(codec, "LINPUT3");
	snd_soc_dapm_disable_pin(codec, "RINPUT3");
	snd_soc_dapm_disable_pin(codec, "OUT3");
	snd_soc_dapm_disable_pin(codec, "MONO1");
#endif

	/* Add specific widgets */
	err = snd_soc_dapm_new_controls(codec, bcm5892_wm8750_dapm_widgets, ARRAY_SIZE(bcm5892_wm8750_dapm_widgets));
	if (err) {
		printk(KERN_ERR "%s: failed to add widgets (%d)\n", __func__, err);
		return err;
	}

	snd_soc_dapm_add_routes(codec, bcm5892_wm8750_audio_map, ARRAY_SIZE(bcm5892_wm8750_audio_map));

//		snd_soc_dapm_enable_pin(codec, "Ext Speaker");

	snd_soc_dapm_sync(codec);

	return 0;
}


/*
 * dailink ops
 */
static int bcm5892_wm8750_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	struct snd_soc_codec *codec = rtd->socdev->codec;
#else
	struct snd_soc_codec *codec = rtd->socdev->card->codec;
#endif
	return 0;
}

static int bcm5892_wm8750_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int clk = 0;
	int ret = 0;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 48000:
	case 96000:
		clk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
		clk = 11289600;
		break;
	}

	/* set codec DAI configuration: normal bit clock and normal frame, codec clock & frame slave */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		printk(KERN_ERR "snd_soc_dai_set_fmt codec_dai failed\n");
		return ret;
	}

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		printk(KERN_ERR "snd_soc_dai_set_fmt cpu_dai failed\n");
		return ret;
	}

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8750_SYSCLK, clk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "snd_soc_dai_set_sysclk failed\n");
		return ret;
	}

	/* set the I2S system clock as input (unused)
	ret = snd_soc_dai_set_sysclk(cpu_dai, BRCM_I2S_SYSCLK, 0, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;
	*/

	return 0;
}

static struct snd_soc_ops bcm5892_asoc_dailink_ops = {
	.startup   = bcm5892_wm8750_startup,
	.hw_params = bcm5892_wm8750_hw_params,
};

/*
 * digital audio interface glue - connects codec <--> CPU
 */
static struct snd_soc_dai_link bcm5892_dailink_wm8750 = {
	.name      = "bcm5892-asoc-dai-link",
	.stream_name = "Stereo (wm8750 DAC)",
	.cpu_dai   = &bcm5892_dai_i2s,
	.codec_dai = &wm8750_dai,
	.ops       = &bcm5892_asoc_dailink_ops,
	.init      = bcm5892_wm8750_init,
};

static struct wm8750_setup_data bcm5892_wm8750_setup = {
//	.i2c_bus = 0,       // i2c-0 controller
	.i2c_address = 0x1a,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)

/* brcm audio machine driver */
static struct snd_soc_machine bcm5892_asoc_wm8750_machine = {
	.name      = "BCM5892 Soc wm8750 Machine",
	.dai_link  = &bcm5892_dailink_wm8750,
	.num_links = 1,
};

/* brcm audio subsystem */
static struct snd_soc_device bcm5892_asoc_devdata = {
	.machine    = &bcm5892_asoc_wm8750_machine,
	.platform   = &bcm5892_asoc_platform,
	.codec_dev  = &soc_codec_dev_wm8750,
	.codec_data = &bcm5892_wm8750_setup,
};

#else

static struct snd_soc_card bcm5892_asoc_soc_card = {
    .name      = "BCM5892 Soc wm8750 Card",
    .platform  =  &bcm5892_asoc_platform,
    .dai_link  = &bcm5892_dailink_wm8750,
    .num_links = 1,
};

/* brcm audio subsystem */
static struct snd_soc_device bcm5892_asoc_devdata = {
    .card       = &bcm5892_asoc_soc_card,
    .codec_dev  = &soc_codec_dev_wm8750,
	.codec_data = &bcm5892_wm8750_setup,
};
#endif // LINUX_VERSION_CODE

static struct platform_device *bcm5892_asoc_device;

static int __init bcm5892_asoc_wm8750_init(void)
{
	int ret = 0;
	
    // Must use "soc-audio" for it to probe the sound soc device
	bcm5892_asoc_device = platform_device_alloc("soc-audio", -1);
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
	
	/* setup GPIO for I2S */
	printk(KERN_INFO "Enabling GPIO for I2S..\n");
	reg_gpio_iotr_set_pin_type( GPIO_AUX_I2S, GPIO_PIN_TYPE_ALTERNATIVE_FUNC0 );

	return ret;
}

static void bcm5892_asoc_wm8750_exit(void)
{
    if (bcm5892_asoc_device) platform_device_unregister(bcm5892_asoc_device);

	/* set back I2S signals to GPIO */
	reg_gpio_iotr_set_pin_type( GPIO_AUX_I2S, GPIO_PIN_TYPE_ALTERNATIVE_DISABLE );
}
module_init(bcm5892_asoc_wm8750_init);
module_exit(bcm5892_asoc_wm8750_exit);
MODULE_DESCRIPTION("ALSA SoC BCM5892 External WM8750");
MODULE_LICENSE("GPL");
