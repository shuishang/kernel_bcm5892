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
*/
/*
 * Header for declaring shim layer exports.
 */

#ifndef __SHM_DOT_H_INCLUDED__
#define __SHM_DOT_H_INCLUDED__

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/suspend.h>
#include <linux/rtc.h>
#include <linux/kobject.h>

extern struct resource *
bcm5892_platform_get_resource(struct platform_device *dev, unsigned int type,
		      unsigned int num);
extern int bcm5892_platform_device_register(struct platform_device * pdev);
extern void bcm5892_platform_device_unregister(struct platform_device * pdev);
extern int bcm5892_platform_driver_register(struct platform_driver *drv);
extern void bcm5892_platform_driver_unregister(struct platform_driver *drv);

struct mtd_info;
struct erase_info;
struct usb_hcd;
struct usb_device;
struct usb_bus;
struct hc_driver;
extern int bcm5892_nand_scan_ident (struct mtd_info *mtd, int maxchips);
extern int bcm5892_nand_scan_tail (struct mtd_info *mtd);
extern void bcm5892_nand_dev_release (struct mtd_info *mtd);
extern void bcm5892_erase_callback (struct erase_info *instr);

extern int bcm5892_platform_get_irq(struct platform_device *dev, unsigned int num);
extern int bcm5892_gpiochip_add(void *chip);

extern struct mtd_info *bcm5892_get_mtd_device(struct mtd_info *mtd, int num);
extern struct mtd_info *bcm5892_get_mtd_device_nm(const char *name);
extern void bcm5892_put_mtd_device(struct mtd_info *mtd);

extern void usb_hcd_resume_root_hub (struct usb_hcd *hcd);
extern int bcm5892_device_register(struct device *dev );
extern void bcm5892_device_unregister(struct device *dev );
extern int bcm5892_driver_register(struct device_driver *d_dev );
extern void bcm5892_driver_unregister(struct device_driver *d_dev );
extern void bcm5892_usb_hcd_poll_rh_status(struct usb_hcd *hcd);
extern void bcm5892_usb_root_hub_lost_power( struct usb_device *rhdev );
extern void bcm5892_device_remove_file(struct device * dev, struct device_attribute * attr);
extern int bcm5892_device_create_file(struct device * dev, struct device_attribute * attr);
extern int bcm5892_driver_create_file(struct device_driver * drv, struct driver_attribute * attr);
extern void bcm5892_driver_remove_file(struct device_driver * drv, struct driver_attribute * attr);

extern int bcm5892_sysfs_create_file(struct kobject * kobj, const struct attribute * attr);
extern int bcm5892_sysfs_create_power_file(const struct attribute * attr);
extern void bcm5892_sysfs_remove_power_file(const struct attribute * attr);

/* USB(OTG) */
extern void bcm5892_usb_hcd_resume_root_hub (struct usb_hcd *hcd);

extern struct usb_hcd *usb_create_hcd (	const struct hc_driver *driver,
                			struct device *dev,
					const char *bus_name);
extern struct usb_hcd *bcm5892_usb_create_hcd (	const struct hc_driver *driver,
                				struct device *dev,
						const char *bus_name);

extern int usb_add_hcd(	struct usb_hcd *hcd,
			unsigned int irqnum,
			unsigned long irqflags);
extern int bcm5892_usb_add_hcd(	struct usb_hcd *hcd,
				unsigned int irqnum,
                        	unsigned long irqflags);

extern void usb_remove_hcd(struct usb_hcd *hcd);
extern void bcm5892_usb_remove_hcd(struct usb_hcd *hcd);

extern void usb_put_hcd (struct usb_hcd *hcd);
extern void bcm5892_usb_put_hcd (struct usb_hcd *hcd);

extern void usb_hcd_giveback_urb(	struct usb_hcd *hcd,
					struct urb *urb,
					int status);
extern void bcm5892_usb_hcd_giveback_urb(	struct usb_hcd *hcd,
						struct urb *urb,
						int status);

extern long bcm5892_usb_calc_bus_time (	int speed,
					int is_input,
					int isoc,
					int bytecount);
extern long usb_calc_bus_time (	int speed,
				int is_input,
				int isoc,
				int bytecount);

extern struct usb_device *usb_alloc_dev(struct usb_device *parent,
                                        struct usb_bus *bus, unsigned port);
extern struct usb_device *bcm5892_usb_alloc_dev(struct usb_device *parent,
                                                struct usb_bus *bus, unsigned port);

extern int bcm5892_i2c_add_numbered_adapter(struct i2c_adapter *adap);

extern struct workqueue_struct * bcm5892_create_singlethread_workqueue(const char *lock_name);
extern void bcm5892_flush_workqueue(struct workqueue_struct *pQueue);
extern void bcm5892_destroy_workqueue(struct workqueue_struct *pQueue);
extern int bcm5892_queue_work(struct workqueue_struct *wq, struct work_struct *work);

extern struct spi_master *bcm5892_spi_alloc_master(struct device *dev, unsigned size);
extern int bcm5892_spi_register_master(struct spi_master *pMaster);
extern struct spi_master *bcm5892_spi_master_get(struct spi_master *master);
extern void bcm5892_spi_master_put(struct spi_master *master);
extern void bcm5892_spi_unregister_master(struct spi_master *pMaster);

extern int usb_hcd_check_unlink_urb(struct usb_hcd *hcd, struct urb *urb, int status);
extern int bcm5892_usb_hcd_check_unlink_urb(struct usb_hcd *hcd, struct urb *urb, int status);

extern void bcm5892_suspend_set_ops(struct platform_suspend_ops *ops);

extern void bcm5892_rtc_update_irq(struct rtc_device *rtc, unsigned long num, unsigned long events) ;
extern struct rtc_device *bcm5892_rtc_device_register(const char *name, struct device *dev, const struct rtc_class_ops *ops, struct module *owner);
extern void bcm5892_rtc_device_unregister(struct rtc_device *rtc) ;
extern struct kobject *bcm5892_get_power_kobj(void);

#endif /*#ifndef __SHM_DOT_H_INCLUDED__*/

