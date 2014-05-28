/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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

/* ---- Include Files ---------------------------------------------------- */
#include <linux/module.h>              /* For EXPORT_SYMBOL */
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/broadcom/bcmring_display.h>
#include <mach/csp/cap.h>

/* ---- Private Constants and Types -------------------------------------- */
#define SPI_MAX_SPEED           5000000
#define SPI_BITS_PER_WORD       8

/* Number of bytes, transmitted at a time to LCD panel via SPI */
#define LCD_SPI_TX_BYTES_NUM    2
#define LCD_REG_NUM             16

#define LCD_REG_FUNC1_ADDR      0x06
#define LCD_REG_FUNC1_STANDBY   0x70
#define LCD_REG_FUNC1_NORMAL    0x78

static unsigned short lcdRegisters[LCD_REG_NUM] =
{
	0x0016, /* holds pixel brightness value --bbbbbb */
	0x0815, /* holds VCOMDC value --bbbbbb */ /* changed from 0x083f */
	0x0471, /* holds contrast value bbbb---- */
	0x0CA8,
	0x0252, 
	0x0A4C, 
	0x0670, /* FUNC1: holds standby control bit ----b--- */ /* changed from 0x0610 */
	0x0EF0, 
	0x0138, /* FUNC3: holds gamma potential correction data */ /* changed from 0x0100 */
	0x09C0, /* FUNC4: holds gamma potential correction data */ /* changed from 0x0980 */
	0x0500,
	0x0D00,
	0x0300,
	0x0B00,
	0x0700,
	0x0F00
};

/* Saved copy of the SPI device pointer */
static struct spi_device *spiDevicep = NULL;

static atomic_t bus_is_probed;

static int panel_enable(void)
{
	int i, ret;
	unsigned char buf[LCD_SPI_TX_BYTES_NUM];

	/* Write to LCD registers */
	for (i = 0; i < LCD_REG_NUM; i++)
	{
		buf[0]= lcdRegisters[i] >> 8;
		buf[1]= lcdRegisters[i] & 0x00FF;
		ret = spi_write(spiDevicep, buf, LCD_SPI_TX_BYTES_NUM);
		if (ret < 0)
			return ret;
	}
	
	/* Turn off standby mode */
	buf[0]= LCD_REG_FUNC1_ADDR;
	buf[1]= LCD_REG_FUNC1_NORMAL;
	ret = spi_write(spiDevicep, buf, LCD_SPI_TX_BYTES_NUM);

	return ret;
}

static int panel_disable(void)
{
	int ret;
	unsigned char buf[LCD_SPI_TX_BYTES_NUM];

	memset(buf, 0, sizeof(buf));
   
	/* Turn on standby mode */
	buf[0]= LCD_REG_FUNC1_ADDR;
	buf[1]= LCD_REG_FUNC1_STANDBY;
	ret = spi_write(spiDevicep, buf, LCD_SPI_TX_BYTES_NUM);

	return ret;
}

int display_panel_init(struct lcd_panel_operations *panel_ops)
{
   if (!atomic_read(&bus_is_probed))
      return -ENODEV;

	if (!panel_ops)
		return -EINVAL;

	panel_ops->lcd_panel_enable = &panel_enable;
	panel_ops->lcd_panel_disable = &panel_disable;

	return 0;
}
EXPORT_SYMBOL(display_panel_init);

int display_panel_term(struct lcd_panel_operations *panel_ops)
{
   if (!atomic_read(&bus_is_probed))
      return -ENODEV;

	if (!panel_ops)
		return -EINVAL;

	panel_ops->lcd_panel_enable = NULL;
	panel_ops->lcd_panel_disable = NULL;

	return 0;
}
EXPORT_SYMBOL(display_panel_term);


static int __devinit casio_probe(struct spi_device *spi)
{
	int err;

   atomic_set(&bus_is_probed, 0);

   if (cap_isPresent(CAP_CLCD, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "Casio Display: Not supported\n");
      return -ENODEV;
   }
	
	spi->mode = SPI_MODE_3;
	spi->max_speed_hz = SPI_MAX_SPEED;
	spi->bits_per_word = SPI_BITS_PER_WORD;
	
	err = spi_setup(spi);
	if (err) {
		printk(KERN_ERR "Casio Display: spi_setup failed\n");
		return err;
	}
	
	spiDevicep = spi;
   atomic_set(&bus_is_probed, 1);

	return 0;
}

static int __devexit casio_remove(struct spi_device *spi)
{
   atomic_set(&bus_is_probed, 0);

	spiDevicep = NULL;
	return 0;
}

static struct spi_driver casio_driver = {
	.driver = {
		.name = "display-casio-COM41T4148",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = casio_probe,
	.remove = __devexit_p(casio_remove),
};

static __init int casio_init(void)
{
	int err;

	err = spi_register_driver(&casio_driver);
	if (err)
	{
		printk(KERN_ERR "Casio Display: spi_register_driver failed\n");
		return err;
	}
	
	return 0;   
}
module_init(casio_init);

static __exit void casio_exit(void)
{
	spi_unregister_driver(&casio_driver);
}
module_exit(casio_exit);

MODULE_DESCRIPTION("Casio COM41T4148 Display Driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");
