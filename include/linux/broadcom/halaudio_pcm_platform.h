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
/**
*
*  @file    halaudio_pcm_platform.h
*
*  @brief   Platform definitions for HAL Audio PCM driver
*
*****************************************************************************/
#if !defined( HALAUDIO_PCM_PLATFORM_H )
#define HALAUDIO_PCM_PLATFORM_H

/* ---- Include Files ---------------------------------------------------- */

/* ---- Constants and Types ---------------------------------------------- */

/* ---- Variable Externs ------------------------------------------------- */

#if defined( __KERNEL__ )

typedef struct halaudio_pcm_bt_gpio
{
   int rst_b;
   int vreg_ctl;
   int wake;
} HALAUDIO_PCM_BT_GPIO;

typedef struct halaudio_pcm_platform_info
{
   /* Number of PCM channels enabled */
   int channels;

   /* If only one PCM channel is enable, select one */
   int channel_select;

   /* Bluetooth GPIO pin assignment */
   HALAUDIO_PCM_BT_GPIO bt_gpio;
} HALAUDIO_PCM_PLATFORM_INFO;

#endif

/* ---- Function Prototypes --------------------------------------- */

#endif /* HALAUDIO_PCM_PLATFORM_H */
