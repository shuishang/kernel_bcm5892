/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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



#ifndef CFG_GLOBAL_DEFINES_H
#define CFG_GLOBAL_DEFINES_H

/* CPU */
#define   ARM11  4

/* CHIP */
#define BCM2153 0x2153
#define BCM2820 0x2820
#define BCM2826 0x2829
#define BCM11107   0x11107
#define BCM11109   0x11109
#define BCM11170   0x11170
#define BCM11110   0x11110
#define BCM11211   0x11211
#define BCM11221   0x11221
#define BCM11195   0x11195
#define BCM11193   0x11193
#define BCM11197   0x11197
#define BCM11172   0x11172
#define BCMRING_ULTRA   0x66611107

/* CFG_GLOBAL_CHIP_FAMILY types */
#define CFG_GLOBAL_CHIP_FAMILY_NONE        0
#define CFG_GLOBAL_CHIP_FAMILY_BCM116X     2
#define CFG_GLOBAL_CHIP_FAMILY_BCMRING     4

/* CFG_GLOBAL_ROOT_FILE_SYSTEM */
#define JFFS2_RFS      1
#define CRAMFS_RFS     2
#define INITRAMFS      3
#define UBIFS_RFS      4

/* CFG_GLOBAL_LCD_SIZE/CFG_GLOBAL_LCD_SIZE */
#define cfg_global_lcd_wvga      1
#define cfg_global_lcd_vga       2
#define cfg_global_lcd_wqvga     3
#define cfg_global_lcd_qvga      4
#define cfg_global_lcd_pcif      5

/* 
 * FIXME: These symbols below should be deprecated as they are 
 * too general a name now that they are compile time constants 
 * in addition to makefile strings. Also see linux/host/splash
 * for more dependencies on these names.
 */
#define wvga           cfg_global_lcd_wvga
#define vga            cfg_global_lcd_vga
#define wqvga          cfg_global_lcd_wqvga
#define qvga           cfg_global_lcd_qvga
#define pcif           cfg_global_lcd_pcif

#define IMAGE_HEADER_SIZE_CHECKSUM    4
#endif

