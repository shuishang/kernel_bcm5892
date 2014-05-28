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


/* ---- Include Files ---------------------------------------------------- */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <mach/reg_gpio.h>

#include "bcm5892_alsa.h"
#include "bcm5892_alsa_codec.h"

#define BCM5892_CODEC_I2C_ADDR     0x1A

/* ---- Private Functions -------------------------------------- */
static void bcm5892_codec_reset(void);
static int bcm5892_codec_reg_read(unsigned int offset);
static int bcm5892_codec_reg_write(uint8_t offset, uint16_t val);
static int bcm5892_alc5623_codec_i2c_write_verify(u8 reg, u16 value);

static int bcm5892_WM8750BL_codec_init(void);
static int bcm5892_ALC5623_codec_init(void);

static int bcm5892_ALC5623_codec_power_off(void);
static int bcm5892_WM8750BL_codec_power_off(void);

static int bcm5892_ALC5623_codec_set_volume(int nVol);
static int bcm5892_WM8750BL_codec_set_volume(int nVol);


/* ---- Public Functions ------------------------------------------------- */
int bcm5892_codec_init(void);
int bcm5892_ALC5623_reset(void);
int bcm5892_WM8750BL_reset(void);
int bcm5892_codec_disable(void);
int bcm5892_codec_set_volume(int nVol);
int bcm5892_selected_codec_chip(unsigned char *chip_name);


/* ---- External Functions ------------------------------------------------- */
extern int bcm5892_codec_i2c_read(unsigned char reg);
extern int bcm5892_codec_i2c_write(unsigned char reg, unsigned char value);
extern int bcm5892_alc5623_codec_i2c_write(u8 reg, u16 value);
extern int bcm5892_alc5623_codec_i2c_read(u8 reg);

ExternalCodecName_t ExternalCodecName=Unknown;

/* ---- Functions -------------------------------------------------------- */

int bcm5892_selected_codec_chip(unsigned char *chip_name)
{
	int vendor_id1;
	int vendor_id2;
	if (strcmp(chip_name,"WM8750BL_codec") == 0)
	{
		ExternalCodecName=Wolfson_WM8750BL;
		printk(KERN_ERR "##Assuming Wolfson WM8750BL codec chip##\n");
		return 0;
	}
	if (strcmp(chip_name,"ALC5623_codec") == 0)
	{
		ExternalCodecName=RealTek_ALC5623;
		vendor_id1 = bcm5892_alc5623_codec_i2c_read(RT5623_VENDOR_ID1);
		vendor_id2 = bcm5892_alc5623_codec_i2c_read(RT5623_VENDOR_ID2);
		if ((vendor_id1 == 0x10EC) && (vendor_id2 == 0x2303))
		{
			printk(KERN_ERR "##Confirmed RealTek ALC5623 codec chip##\n");
		}
		else
		{
			printk(KERN_ERR "##Assuming RealTek ALC5623 codec chip##\n");
		}
		return 0;
	}
	printk(KERN_ERR "##Unknown codec chip##\n");
	ExternalCodecName=Unknown;
	return -1;
}
int bcm5892_WM8750BL_reset(void)
{
	int rc;
	if ((rc = bcm5892_codec_reg_write(CODECREG_RESET, 0x0)) != 0)
	{
		printk(KERN_ERR "WM8750BL reset failed");
	}
	return rc;
}
int bcm5892_ALC5623_reset(void)
{
	int rc;
	if ((rc=bcm5892_alc5623_codec_i2c_write(RT5623_RESET,0)) != 0) 
	{
		printk(KERN_ERR "ALC5623 reset failed");
	}
	return rc;
}


static void bcm5892_codec_reset(void)
{
	bcm5892_codec_reg_write(CODECREG_RESET, 0x0);

	bcm5892_codec_reg_write(CODECREG_PWR_MGMT2, 0x0);
}

/* NOTE this codec (Wolfson) can't read reg so this function is not used */
static int bcm5892_codec_reg_read(unsigned int offset)
{
	int rc;
	rc = bcm5892_codec_i2c_read(offset);

	return rc;
}


static int bcm5892_codec_reg_write(uint8_t offset, uint16_t val)
{
	uint8_t data0, data1;
	int     rc;

	data0 = (offset << 1) | ((val >> 8) & 0x01);
	data1 = val & 0x00FF;

	rc = bcm5892_codec_i2c_write(data0, data1);
	if (rc != 0)
	{
		printk(KERN_CRIT "codec register write failed, addr=%d, val = %d\n", offset, val);
	}

	return rc;
}


static int bcm5892_alc5623_codec_i2c_write_verify(u8 reg, u16 value)
{
	int read_value;
	if (bcm5892_alc5623_codec_i2c_write(reg,value) != 0) 
	{
	  printk(KERN_INFO "bcm5892_alc5623_codec_i2c_write_verify Write failed\n");
		return -1;
	}
	read_value = bcm5892_alc5623_codec_i2c_read(reg);
	printk(KERN_INFO "read_value:%x value:%x\n",read_value,value);
	if (read_value != value)
	{
	  printk(KERN_INFO "bcm5892_alc5623_codec_i2c_write_verify Verification failed\n");
		return -1;
	}
	return 0;
}
static int bcm5892_WM8750BL_codec_init(void)
{
   int rc;
        printk(KERN_INFO "Reseting external audio CODEC WM8750BL\n");

	/* Note: I2C should have been setup already */

	/* reset Codec */
	bcm5892_codec_reset();

	/* program Codec registers */

	/*Turn codec on if not already */
	rc = bcm5892_codec_reg_write(CODECREG_PWR_MGMT1, (CODECFIELD_VMIDSEL_PLAYBACK | CODECFIELD_VREF));
	if (rc != 0) return rc;

	/* enable dacs */
	rc = bcm5892_codec_reg_write(CODECREG_PWR_MGMT2, (CODECFIELD_DACL | CODECFIELD_DACR));
	if (rc != 0) return rc;

	/* enable outputs lout1 and rout1 */
	rc = bcm5892_codec_reg_write(CODECREG_PWR_MGMT2, (CODECFIELD_LOUT1 | CODECFIELD_ROUT1)|(CODECFIELD_DACL | CODECFIELD_DACR));
	if (rc != 0) return rc;

	/* unmute */
	rc = bcm5892_codec_reg_write(CODECREG_ADC_DAC_CONTROL, 0x0);
	if (rc != 0) return rc;

	/*enable I2S 16bit */
	rc = bcm5892_codec_reg_write(CODECREG_AUDIO_INTERFACE, CODECFIELD_FORMAT_I2S);
	if (rc != 0) return rc;

	/*enable DAC to output */
	rc = bcm5892_codec_reg_write(CODECREG_LEFT_OUT_MIX1, CODECFIELD_LD2LO);
	if (rc != 0) return rc;

	rc = bcm5892_codec_reg_write(CODECREG_RIGHT_OUT_MIX2, CODECFIELD_RD2RO);
	if (rc != 0) return rc;


	/* set up volume */
	rc = bcm5892_codec_set_volume(g_brcm_alsa_chip->pcm_playback_volume);

	return( rc );
}
static int bcm5892_ALC5623_codec_init(void)
{
	int rc;
        printk(KERN_INFO "Reseting external audio CODEC ALC5623\n");
	if ((rc=bcm5892_alc5623_codec_i2c_write(RT5623_RESET,0)) != 0) 
		return rc;
        printk(KERN_INFO "RT5623_PWR_MANAG_ADD3\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_PWR_MANAG_ADD3,PWR_MAIN_BIAS)) != 0)
		return rc;
        printk(KERN_INFO "RT5623_PWR_MANAG_ADD2\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_PWR_MANAG_ADD2,PWR_VREF)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_PWR_MANAG_ADD1\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_PWR_MANAG_ADD1,PWR_SOFTGEN_EN)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_PWR_MANAG_ADD3\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_PWR_MANAG_ADD3,PWR_HP_R_OUT_VOL|PWR_HP_L_OUT_VOL)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_MISC_CTRL\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_MISC_CTRL,HP_DEPOP_MODE2_EN)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_HP_OUT_VOL\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_HP_OUT_VOL,0x0808)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_STEREO_DAC_VOL\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_STEREO_DAC_VOL,0x0808)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_AUDIO_INTERFACE\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_AUDIO_INTERFACE,0x8000)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_ADD_CTRL_REG\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_ADD_CTRL_REG,0x5f00)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_OUTPUT_MIXER_CTR\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_OUTPUT_MIXER_CTRL      ,0x1740)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_STEREO_AD_DA_CLK_CTRL\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_STEREO_AD_DA_CLK_CTRL,0x166d)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_PWR_MANAG_ADD2\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_PWR_MANAG_ADD2 ,PWR_VREF|PWR_DAC_REF_CIR|PWR_L_DAC_CLK|PWR_R_DAC_CLK|PWR_L_HP_MIXER|PWR_R_HP_MIXER)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_PWR_MANAG_ADD3\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_PWR_MANAG_ADD3,PWR_MAIN_BIAS|PWR_HP_R_OUT_VOL|PWR_HP_L_OUT_VOL|PWR_SPK_OUT)) != 0)
                return rc;
        printk(KERN_INFO "RT5623_PWR_MANAG_ADD1\n");
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_PWR_MANAG_ADD1,PWR_MAIN_I2S_EN|PWR_HP_OUT_ENH_AMP|PWR_HP_OUT_AMP)) != 0)
                return rc;

	/* set up volume */
	rc = bcm5892_codec_set_volume(g_brcm_alsa_chip->pcm_playback_volume);
	return 0;
}

int bcm5892_codec_init(void)
{
	int rc=0;
	switch(ExternalCodecName)
	{
		case 	Wolfson_WM8750BL:
			rc = bcm5892_WM8750BL_codec_init();
			break;
		case	RealTek_ALC5623:
			rc =  bcm5892_ALC5623_codec_init();
			break;
		default:
			printk(KERN_ERR "Unknown device \n");
			return -ENODEV;
	}
	if (rc == 0)
	{
		/* setup GPIO for I2S */
		printk(KERN_INFO "Enabling GPIO for I2S..\n");
		reg_gpio_iotr_set_pin_type( GPIO_AUX_I2S, GPIO_PIN_TYPE_ALTERNATIVE_FUNC0 );
	}
	return rc;
}
static int bcm5892_WM8750BL_codec_set_volume(int nVol)
{
	int rc = -EINVAL;
	if ((nVol >= 0) && (nVol <= 100))
	{
		/*map the nVol to register field value */
		nVol = (CODECFIELD_LROUT1_VOLUME_MAX - CODECFIELD_LROUT1_VOLUME_MIN) * nVol / 100 + CODECFIELD_LROUT1_VOLUME_MIN;

		/*use LOUT1/ROUT1, these are analog output volume */
		rc = bcm5892_codec_reg_write(CODECREG_LOUT1_VOLUME, (nVol));
		if (rc != 0) return rc;
		rc = bcm5892_codec_reg_write(CODECREG_ROUT1_VOLUME, (CODECFIELD_LROUT1_VOLUME_UPDATE | nVol));
		if (rc != 0) return rc;
	}
	return rc;
}
static int bcm5892_ALC5623_codec_set_volume(int nVol)
{
	int rc = -EINVAL;
	u16 codec_volume;
	if ( (nVol >= 0) && (nVol <= 100))
	{
		if ( nVol == 0)
		{
			/* Mute output */
			codec_volume = RT_L_MUTE|RT_R_MUTE;
		}
		else
		{
			codec_volume = ALC5623_LR_HP_VOLUME_MIN - ((ALC5623_LR_HP_VOLUME_MIN * nVol)/100); //Min value db means maximum power
			codec_volume = (codec_volume << 8) | codec_volume;
		}
		printk(KERN_INFO "bcm5892_codec_set_volume to value %x\n",codec_volume);
       		if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_HP_OUT_VOL,codec_volume)) != 0)
	                return rc;
	}
	return rc;
}

/* input: nVol is a percentage number from 0 to 100 */
int bcm5892_codec_set_volume(int nVol)
{
	int rc = 0;
	printk(KERN_INFO "bcm5892_codec_set_volume %d%%\n",nVol);
	switch(ExternalCodecName)
	{
		case 	Wolfson_WM8750BL:
			rc = bcm5892_WM8750BL_codec_set_volume(nVol);
			break;
		case    RealTek_ALC5623:
			rc = bcm5892_ALC5623_codec_set_volume(nVol);
			break;
		default:
			printk(KERN_ERR "Unknown device \n");
			return -ENODEV;
    	}
	return rc;
}

static int bcm5892_ALC5623_codec_power_off(void)
{
	int rc = 0;
        printk(KERN_INFO "%s: RT5623_PWR_MANAG_ADD1=0\n",__FUNCTION__);
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_PWR_MANAG_ADD1 ,0)) != 0)
                return rc;
        printk(KERN_INFO "%s: RT5623_PWR_MANAG_ADD2=0\n",__FUNCTION__);
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_PWR_MANAG_ADD2,0)) != 0)
                return rc;
        printk(KERN_INFO "%s: RT5623_PWR_MANAG_ADD3\n",__FUNCTION__);
        if( (rc=bcm5892_alc5623_codec_i2c_write_verify(RT5623_PWR_MANAG_ADD3,0)) != 0)
                return rc;
	return rc;
}
static int bcm5892_WM8750BL_codec_power_off(void)
{
	int rc = 0;

	rc = bcm5892_codec_reg_write(CODECREG_PWR_MGMT1, 0);
	if (rc != 0) return rc;

	rc = bcm5892_codec_reg_write(CODECREG_PWR_MGMT2, 0);
	if (rc != 0) return rc;
	/* set up volume */
	rc = bcm5892_codec_set_volume(g_brcm_alsa_chip->pcm_playback_volume);

	return rc;
}


int bcm5892_codec_disable(void)
{
	int rc=0;
	printk(KERN_INFO "bcm5892_codec_disable \n");
	switch(ExternalCodecName)
	{
		case    Wolfson_WM8750BL:
			rc = bcm5892_WM8750BL_codec_power_off();
			if (rc != 0) return rc;
			break;
		case    RealTek_ALC5623:
			rc = bcm5892_ALC5623_codec_power_off();
			if (rc != 0) return rc;
			break;
                default:
                        printk(KERN_ERR "Unknown device \n");
                        return -ENODEV;
    	}

	/* set back I2S signals to GPIO */
	reg_gpio_iotr_set_pin_type( GPIO_AUX_I2S, GPIO_PIN_TYPE_ALTERNATIVE_DISABLE );
	return 0;
}
