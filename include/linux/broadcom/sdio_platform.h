/*****************************************************************************
* Copyright 2001 - 2009 Broadcom Corporation.  All rights reserved.
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

#ifndef _SDIO_PLATFORM_H
#define _SDIO_PLATFORM_H

/*
 * SDIO device type, can be SD/MMC or WiFi
 */
enum sdio_devtype {
   SDIO_DEV_TYPE_SDMMC = 0,
   SDIO_DEV_TYPE_WIFI,
};

/*
 * SDIO WiFi GPIO configuration
 */
struct sdio_wifi_gpio_cfg {
   int reset;
   int shutdown;
   int reg;
   int host_wake;
};

struct sdio_platform_cfg {
   /*
    * For boards without the SDIO pullup registers, data_pullup needs to set
    * to 1
    */
   unsigned int data_pullup;
   
   enum sdio_devtype devtype;

   union {
      struct sdio_wifi_gpio_cfg wifi_gpio;
   } dev_option;
};

#endif  /* SDIO_PLATFORM_H */
