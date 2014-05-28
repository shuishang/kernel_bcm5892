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



/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
static struct i2c_client *bcm5892_codec_i2c_datap = NULL;

/* ---- Private Function Prototypes -------------------------------------- */
static int bcm5892_codec_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);


/* ---- Public Variables ------------------------------------------------- */
int bcm5892_codec_i2c_init(void);
int bcm5892_codec_i2c_remove(void);

/* ---- Extern */
extern int bcm5892_WM8750BL_reset();
extern int bcm5892_ALC5623_reset();
extern int bcm5892_selected_codec_chip(unsigned char *chip_name);



/* Debug trace */
#define I2C_ENABLE_LOG           1

#if I2C_ENABLE_LOG
#define I2CLOG(f, ...)           printk(KERN_INFO f, ##__VA_ARGS__)
#else
#define I2CLOG(f, ...)
#endif

static const struct i2c_device_id bcm5892_codec_i2c_ids[] = {
	{"wm8750", 0},
	{"ALC5623_codec", 1},
	{}
};
MODULE_DEVICE_TABLE(i2c, bcm5892_codec_i2c_ids);


struct i2c_driver i2c_driver_ext_codec_wolfson =
{
	.driver = {
		.name  = "bcm5892extcodeci2c",
		.owner = THIS_MODULE,
		},
	.probe     = bcm5892_codec_i2c_probe,
/*	.remove    = __devexit_p(bcm5892_codec_i2c_remove), */
	.id_table  = bcm5892_codec_i2c_ids,
};

/* ---- Functions -------------------------------------------------------- */

int bcm5892_codec_i2c_init(void)
{
   int rc;

   printk(KERN_INFO "bcm5892_codec_i2c_init");

   /* set up the I2C */
   rc = i2c_add_driver(&i2c_driver_ext_codec_wolfson);
   if (rc != 0)
   {
      printk("CodecI2C - Failed to initialize I2C\n");
   }
   return( rc );
}


/****************************************************************************
*
*  bcm5892_codec_i2c_read
*
*  reg: address of register to read
*
*  returns: data read (8 bits) or -1 on error
*
***************************************************************************/
int bcm5892_codec_i2c_read(unsigned char reg)
{
   /* Guard against calling in an atomic context */
   /*might_sleep(); */

   if ( bcm5892_codec_i2c_datap == NULL )
   {
      printk( "i2c bus not configured for ext_codec_wolfson\n" );
      return -1;
   }
   return i2c_smbus_read_byte_data(bcm5892_codec_i2c_datap, reg);
}

/****************************************************************************
*
*  bcm5892_codec_i2c_write
*
*  reg: address of register to write
*  value: value to be written
*
*  returns: 0 on success, -1 on error
*
***************************************************************************/
int bcm5892_codec_i2c_write(unsigned char reg, unsigned char value)
{
   /* Guard against calling in an atomic context */
   /*might_sleep(); */

   if ( bcm5892_codec_i2c_datap == NULL )
   {
      printk( "i2c bus not configured for ext_codec_wolfson\n" );
      return -1;
   }
   return i2c_smbus_write_byte_data(bcm5892_codec_i2c_datap, reg, value);
}

/* ALC5623 write routine */
int bcm5892_alc5623_codec_i2c_write(u8 reg, u16 value)
{
   /* Guard against calling in an atomic context */
   /*might_sleep(); */
   u8 	high_byte,low_byte;
   u16 	word;

   /* little endian processing */
   high_byte = (value & 0xff00) >> 8;
   low_byte = (value & 0x00ff);
   word = (low_byte << 8) | high_byte;

   if ( bcm5892_codec_i2c_datap == NULL )
   {
      printk( "i2c bus not configured for ext_codec_alc5623\n" );
      return -1;
   }
   return i2c_smbus_write_word_data(bcm5892_codec_i2c_datap, reg, word);
}
int bcm5892_alc5623_codec_i2c_read(u8 reg)
{
   /* Guard against calling in an atomic context */
   /*might_sleep(); */
   u8 	high_byte,low_byte;
   s32 	value;
   s32 	word;

   if ( bcm5892_codec_i2c_datap == NULL )
   {
      printk( "i2c bus not configured for ext_codec_alc5623\n" );
      return -1;
   }
   if( (value=i2c_smbus_read_word_data(bcm5892_codec_i2c_datap, reg)) < 0)
   {
	return value;
   }
   /* little endian processing */
   high_byte = (value & 0xff00) >> 8;
   low_byte = (value & 0x00ff);
   word = (low_byte << 8) | high_byte;
   return word;
}



/****************************************************************************
*
*  bcm5892_codec_i2c_probe
*
***************************************************************************/
static int bcm5892_codec_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;


	printk(KERN_INFO "bcm5892_codec_i2c_probe");
	printk(KERN_INFO "Probing Audio codec chip %s %d \n",id->name,id->driver_data);
#if 0
	/* make sure the adapter supports I2C (e.g. not SMBus) */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "adapter do not support I2C\n");
		return -EPFNOSUPPORT;
	}
#endif
     if ( bcm5892_codec_i2c_datap == NULL)
     {
	bcm5892_codec_i2c_datap = client;
	i2c_set_clientdata( client, bcm5892_codec_i2c_datap );
        if (strcmp(id->name,"WM8750BL_codec") == 0)
	{
		printk(KERN_INFO "Checking WM8750BL_codec device..\n");
		if (bcm5892_WM8750BL_reset() != 0)
		{
			bcm5892_codec_i2c_datap = NULL;
			i2c_set_clientdata( client, bcm5892_codec_i2c_datap );
			return -ENODEV;
		}
		bcm5892_selected_codec_chip(id->name);
	}
        if (strcmp(id->name,"ALC5623_codec") == 0)
	{
		printk(KERN_INFO "Checking ALC5623_codec device..\n");
		if (bcm5892_ALC5623_reset() != 0)
		{
			bcm5892_codec_i2c_datap = NULL;
			i2c_set_clientdata( client, bcm5892_codec_i2c_datap );
			return -ENODEV;
		}
		bcm5892_selected_codec_chip(id->name);
	}
	dev_info(dev, "example client created\n");

	return 0;
      }
      else
      {
	printk(KERN_INFO "###bcm5892_codec_i2c_probe:Already been probed###\n");
	return -ENODEV;
      }
}


/****************************************************************************
*
*  bcm5892_codec_i2c_detach_client
*
***************************************************************************/
int bcm5892_codec_i2c_remove()
{
	I2CLOG("bcm5892_codec_i2c_remove");

	i2c_del_driver( &i2c_driver_ext_codec_wolfson );

	bcm5892_codec_i2c_datap = NULL;

	return 0;
}

