/*
*  Copyright 2009 Broadcom Corporation.  All rights reserved.
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
*  Shim layer code to export GPL-only smbols.
*  To add a symbol, modify this file and include/mach/shm.h
*/

#include <linux/mtd/nand.h>
#include <linux/mtd/mtd.h>
#include <linux/i2c.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/rtc.h>

#include <mach/shm.h>

extern int gpiochip_add(void *chip);

extern void usb_hcd_poll_rh_status(struct usb_hcd *hcd);

extern void usb_root_hub_lost_power(struct usb_device *rhdev);

/**
 *	bcm5892_platform_get_resource -
 *           wrapper function for platform_get_resource
 *	@dev: platform device
 *	@type: resource type
 *	@num: resource index
 */
struct resource *
bcm5892_platform_get_resource(struct platform_device *dev, unsigned int type,
		      unsigned int num)
{
	return platform_get_resource(dev, type, num);
}
EXPORT_SYMBOL(bcm5892_platform_get_resource);


/**
 *	bcm5892_platform_driver_register -
 *           wrapper function for platform_driver_register
 *	@drv: platform driver structure
 */
int bcm5892_platform_driver_register(struct platform_driver *drv)
{
	return platform_driver_register(drv);
}
EXPORT_SYMBOL(bcm5892_platform_driver_register);


/**
 *	bcm5892_platform_driver_unregister
 *           wrapper function for platform_driver_unregister
 *	@drv: platform driver structure
 */
void bcm5892_platform_driver_unregister(struct platform_driver *drv)
{
	return platform_driver_unregister(drv);
}
EXPORT_SYMBOL(bcm5892_platform_driver_unregister);


/**
 *      bcm5892_platform_device_register - add a platform-level device
 *          wrapper function for platform_device_register
 *      @pdev:  platform device we're adding
 *
 */
int bcm5892_platform_device_register(struct platform_device * pdev)
{
        return platform_device_register(pdev);
}
EXPORT_SYMBOL(bcm5892_platform_device_register);


/**
 *	bcm5892_platform_device_unregister -
 *          wrapper function for platform_device_unregister
 *	@pdev:	platform device we're unregistering
 */
void bcm5892_platform_device_unregister(struct platform_device * pdev)
{
	return platform_device_unregister(pdev);
}
EXPORT_SYMBOL(bcm5892_platform_device_unregister);

int bcm5892_nand_scan_ident (struct mtd_info *mtd, int maxchips)
{
	return nand_scan_ident(mtd, maxchips);
}
EXPORT_SYMBOL(bcm5892_nand_scan_ident);

int bcm5892_nand_scan_tail (struct mtd_info *mtd)
{
	return nand_scan_tail (mtd);
}
EXPORT_SYMBOL(bcm5892_nand_scan_tail);

void bcm5892_nand_dev_release (struct mtd_info *mtd)
{
	nand_release (mtd);
}
EXPORT_SYMBOL(bcm5892_nand_dev_release);

void bcm5892_erase_callback(struct erase_info *instr)
{
	mtd_erase_callback(instr);
}
EXPORT_SYMBOL (bcm5892_erase_callback);

int bcm5892_platform_get_irq(struct platform_device *dev, unsigned int num)
{
	return platform_get_irq(dev, num);
}
EXPORT_SYMBOL(bcm5892_platform_get_irq);

/* Note here we use void * instead of gpio_chip * */
int bcm5892_gpiochip_add(void *chip)
{
	return gpiochip_add(chip);
}
EXPORT_SYMBOL(bcm5892_gpiochip_add);


struct mtd_info *bcm5892_get_mtd_device(struct mtd_info *mtd, int num)
{
        return get_mtd_device(mtd, num);
}
EXPORT_SYMBOL(bcm5892_get_mtd_device);

struct mtd_info *bcm5892_get_mtd_device_nm(const char *name)
{
        return get_mtd_device_nm(name);
}
EXPORT_SYMBOL(bcm5892_get_mtd_device_nm);

void bcm5892_put_mtd_device(struct mtd_info *mtd)
{
        return put_mtd_device(mtd);
}
EXPORT_SYMBOL(bcm5892_put_mtd_device);

int bcm5892_device_register(struct device *dev )
{
return device_register(dev);
}
EXPORT_SYMBOL(bcm5892_device_register);

void bcm5892_device_unregister(struct device *dev)
{
	device_unregister(dev);
}

EXPORT_SYMBOL(bcm5892_device_unregister);

int bcm5892_driver_register(struct device_driver *d_dev)
{
	return driver_register(d_dev);
}

EXPORT_SYMBOL(bcm5892_driver_register);

void bcm5892_driver_unregister(struct device_driver *d_dev)
{
	return driver_unregister(d_dev);
}

EXPORT_SYMBOL(bcm5892_driver_unregister);

void bcm5892_usb_hcd_poll_rh_status(struct usb_hcd *hcd)
{
	return usb_hcd_poll_rh_status(hcd);
}

EXPORT_SYMBOL(bcm5892_usb_hcd_poll_rh_status);

void bcm5892_usb_root_hub_lost_power( struct usb_device *rhdev )
{
	usb_root_hub_lost_power( rhdev );
}

EXPORT_SYMBOL(bcm5892_usb_root_hub_lost_power);


void bcm5892_device_remove_file(struct device * dev, struct device_attribute * attr)
{
       device_remove_file(dev, attr);
}

EXPORT_SYMBOL(bcm5892_device_remove_file);

int bcm5892_driver_create_file(struct device_driver * drv, struct driver_attribute * attr)
{
       return driver_create_file( drv, attr);
}

EXPORT_SYMBOL(bcm5892_driver_create_file);

int bcm5892_device_create_file(struct device * dev, struct device_attribute * attr)
{
	return device_create_file(dev, attr);
}

EXPORT_SYMBOL(bcm5892_device_create_file);

void bcm5892_driver_remove_file(struct device_driver * drv, struct driver_attribute * attr)
{
	driver_remove_file(drv, attr);
}

EXPORT_SYMBOL(bcm5892_driver_remove_file);

/* USB(OTG)  */

void bcm5892_usb_hcd_resume_root_hub (struct usb_hcd *hcd)
{
        usb_hcd_resume_root_hub(hcd);
}
EXPORT_SYMBOL(bcm5892_usb_hcd_resume_root_hub);

struct usb_hcd *bcm5892_usb_create_hcd (const struct hc_driver *driver,
                struct device *dev, const char *bus_name)
{
        return usb_create_hcd (driver,dev, bus_name);
}
EXPORT_SYMBOL(bcm5892_usb_create_hcd);

int bcm5892_usb_add_hcd(struct usb_hcd *hcd,
                        unsigned int irqnum,
                        unsigned long irqflags)
{
        return usb_add_hcd(hcd,irqnum,irqflags);
}
EXPORT_SYMBOL(bcm5892_usb_add_hcd);

void bcm5892_usb_remove_hcd(struct usb_hcd *hcd)
{
        usb_remove_hcd(hcd);
}
EXPORT_SYMBOL(bcm5892_usb_remove_hcd);



void bcm5892_usb_put_hcd(struct usb_hcd *hcd)
{
        usb_put_hcd(hcd);
}
EXPORT_SYMBOL(bcm5892_usb_put_hcd);


void bcm5892_usb_hcd_giveback_urb(	struct usb_hcd *hcd,
					struct urb *urb,
					int status)
{
        usb_hcd_giveback_urb(hcd,urb,status);
}
EXPORT_SYMBOL(bcm5892_usb_hcd_giveback_urb);

long bcm5892_usb_calc_bus_time (int speed,
				int is_input,
				int isoc,
				int bytecount)
{
        return usb_calc_bus_time (speed, is_input, isoc, bytecount);
}
EXPORT_SYMBOL(bcm5892_usb_calc_bus_time);

struct usb_device *bcm5892_usb_alloc_dev(struct usb_device *parent,
                                        struct usb_bus *bus, unsigned port)
{
	return usb_alloc_dev(parent,bus, port);
}
EXPORT_SYMBOL(bcm5892_usb_alloc_dev);


/* I2C subsystem shims */
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
int bcm5892_i2c_add_numbered_adapter(struct i2c_adapter *adap)
{
	return i2c_add_numbered_adapter(adap);
}
EXPORT_SYMBOL(bcm5892_i2c_add_numbered_adapter);
#endif


/* WorkQueue shims */
extern struct workqueue_struct * bcm5892_create_singlethread_workqueue(const char *lock_name)
{
	return create_singlethread_workqueue(lock_name);
}
EXPORT_SYMBOL(bcm5892_create_singlethread_workqueue);

extern void bcm5892_flush_workqueue(struct workqueue_struct *pQueue)
{
	flush_workqueue(pQueue);
}
EXPORT_SYMBOL(bcm5892_flush_workqueue);

extern void bcm5892_destroy_workqueue(struct workqueue_struct *pQueue)
{
	destroy_workqueue(pQueue);
}
EXPORT_SYMBOL(bcm5892_destroy_workqueue);

extern int bcm5892_queue_work(struct workqueue_struct *wq, struct work_struct *work)
{
	return queue_work(wq, work);
}
EXPORT_SYMBOL(bcm5892_queue_work);

/* SPI subsystem shims */
struct spi_master *bcm5892_spi_alloc_master(struct device *dev, unsigned size)
{
	return spi_alloc_master(dev, size);
}
EXPORT_SYMBOL(bcm5892_spi_alloc_master);

int bcm5892_spi_register_master(struct spi_master *pMaster)
{
	return spi_register_master(pMaster);
}
EXPORT_SYMBOL(bcm5892_spi_register_master);

struct spi_master *bcm5892_spi_master_get(struct spi_master *master)
{
	return spi_master_get(master);
}
EXPORT_SYMBOL(bcm5892_spi_master_get);

void bcm5892_spi_master_put(struct spi_master *master)
{
	spi_master_put(master);
}
EXPORT_SYMBOL(bcm5892_spi_master_put);

void bcm5892_spi_unregister_master(struct spi_master *pMaster)
{
	spi_unregister_master(pMaster);
}
EXPORT_SYMBOL(bcm5892_spi_unregister_master);

int bcm5892_usb_hcd_check_unlink_urb(	struct usb_hcd *hcd,
				struct urb *urb,
				int status)
{
	return usb_hcd_check_unlink_urb(hcd,urb,status);
}
EXPORT_SYMBOL(bcm5892_usb_hcd_check_unlink_urb);


int bcm5892_sysfs_create_file(struct kobject * kobj, const struct attribute * attr)
{
	return sysfs_create_file(kobj, attr);
}
EXPORT_SYMBOL(bcm5892_sysfs_create_file);

int bcm5892_sysfs_create_power_file(const struct attribute * attr)
{
	return sysfs_create_file(power_kobj, attr);
}
EXPORT_SYMBOL(bcm5892_sysfs_create_power_file);

void bcm5892_sysfs_remove_power_file(const struct attribute * attr)
{
	sysfs_remove_file(power_kobj, attr);
}
EXPORT_SYMBOL(bcm5892_sysfs_remove_power_file);

void bcm5892_suspend_set_ops(struct platform_suspend_ops *ops)
{
	suspend_set_ops(ops);
}
EXPORT_SYMBOL(bcm5892_suspend_set_ops);

void bcm5892_rtc_update_irq(struct rtc_device *rtc, unsigned long num, unsigned long events) 
{
	rtc_update_irq(rtc, num, events) ;
}
EXPORT_SYMBOL(bcm5892_rtc_update_irq);

struct rtc_device *bcm5892_rtc_device_register(const char *name, struct device *dev, const struct rtc_class_ops *ops, struct module *owner){
	return rtc_device_register(name, dev, ops, owner);
}
EXPORT_SYMBOL(bcm5892_rtc_device_register);


void bcm5892_rtc_device_unregister(struct rtc_device *rtc) 
{
	rtc_device_unregister(rtc);
}
EXPORT_SYMBOL(bcm5892_rtc_device_unregister);

struct kobject *bcm5892_get_power_kobj(void) 
{
	return power_kobj;
}
EXPORT_SYMBOL(bcm5892_get_power_kobj);


