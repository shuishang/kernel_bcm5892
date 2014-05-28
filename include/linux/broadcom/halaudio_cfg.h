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
*  @file    halaudio_cfg.h
*
*  @brief   Board dependant Hal Audio configurations
*
*****************************************************************************/
#if !defined( HALAUDIO_CFG_H )
#define HALAUDIO_CFG_H

/* ---- Include Files ---------------------------------------------------- */
#include <linux/broadcom/halaudio.h>

/* ---- Constants and Types ---------------------------------------------- */
#define HALAUDIO_CFG_NAME_LEN    20

typedef struct halaudio_dev_info
{
   /* codec channel name */
   char codec_name[HALAUDIO_CFG_NAME_LEN];

   /* AMXR port name */
   char mport_name[HALAUDIO_CFG_NAME_LEN];

   /* mic mux */
   HALAUDIO_HWSEL mic_hwsel;

   /* speaker switch */
   HALAUDIO_HWSEL spkr_hwsel;

   int   mic_ana_gain;
   int   mic_dig_gain;
   int   spkr_ana_gain;
   int   spkr_dig_gain;
   int   sidetone_gain;
   int   mic_equ_coefs_len;
   int   mic_equ_coefs[HALAUDIO_EQU_COEFS_MAX_NUM];
   int   sprk_equ_coefs_len;
   int   sprk_equ_coefs[HALAUDIO_EQU_COEFS_MAX_NUM];

} HALAUDIO_DEV_INFO;

typedef struct halaudio_dev_cfg
{
   /* device name */
   char name[HALAUDIO_CFG_NAME_LEN];

   /* to indicate whether the device has AUX or not */
   int has_aux;

   /* main device information */
   HALAUDIO_DEV_INFO info;

   /* AUX information */
   HALAUDIO_DEV_INFO aux_info;
} HALAUDIO_DEV_CFG;

typedef struct halaudio_cfg
{
   /* number of devices */
   int numdev;

   /* pointer to the list of devices */
   HALAUDIO_DEV_CFG *devlist;
} HALAUDIO_CFG;

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes --------------------------------------- */

#endif /* HALAUDIO_CFG_H */
