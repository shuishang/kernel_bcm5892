/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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




/*
*
*****************************************************************************
*
*  bcm_major.h
*
*  PURPOSE:
*
*     This file contains definitions for the various major devices used
*     on the BCM116x board
*
*  NOTES:
*
*****************************************************************************/


#if !defined( BCM_MAJOR_H )
#define BCM_MAJOR_H
#define BCM_LCC_MAJOR         197
#define BCM_ETH_LED_MAJOR     198
#define BCM_BSC_MAJOR         199
#define BCM_CAM_MAJOR         200
#define BCM_CSXIOD_MAJOR      201
#define BCM_MMDMA_MAJOR       202
#define BCM_GE_MAJOR          203
#define BCM_CLCD_MAJOR        205
#define BCM_UDCHP_MAJOR       206
#define BCM_GVR_MAJOR         207
#define BCM_WLAN_DRV_MAJOR    208
#define BCM_GPIO_MAJOR        209
#define BCM_VDEC_MAJOR        210
#define BCM_OTP_MAJOR         211
#define BCM_VC03_MAJOR        212
#define BCM_AMXR_MAJOR        213
#define BCM_SPEAKER_MAJOR     214
#define BCM_PM_MAJOR          215
#define BCM_LED_MAJOR         216
#define BCM_HEADSET_MAJOR     217
#define BCM_RTC_MAJOR         218
#define BCM_IODUMP_MAJOR      219
#define BCM_ENDPT_MAJOR       221
#define BCM_KEYPAD_MAJOR      222
#define BCM_LCD_MAJOR         223
#define BCM_AUXADC_MAJOR      224
#define BCM_PMU_MAJOR         225
#define BCM_OPROFILE_RESERVED 227
#define BCM_VCP_MAJOR         228
#define BCM_RELTIME_MAJOR     229
#define BCM_ESW_MAJOR         230
#define BCM_EPHY_MAJOR        231
#define BCM_VPM_MAJOR         232
#define BCM_HALAUDIO_MAJOR    233
#define BCM_VC02_MAJOR        234
#define BCM_USB_I2C_MAJOR     235
#define BCM_AMXR_USER_MAJOR   236
#define BCM_HAPI_MAJOR        237
#define BCM_ECAN_MAJOR        238
#define BCM_CSX_UTIL_MAJOR    239

/*
 * For now, leave BCM_VCHIQ_MAJOR at 240. Changing this also requires changing
 * VCHIQ_MAJOR defined in the videocore side.
 */

#define BCM_VCHIQ_MAJOR       240

#define BCM_GPS_MAJOR		  245



#endif /* BCM_MAJOR_H */

