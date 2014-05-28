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
#if !defined( HALAUDIO_EXTENSIONS_CFG_H )
#define HALAUDIO_EXTENSIONS_CFG_H

/* ---- Include Files ---------------------------------------------------- */
/* ---- Constants and Types ---------------------------------------------- */

typedef struct halaudio_extensions_cfg
{
   /* device name */
   char dev_name[25];

   /* External speaker op-amp enable GPIO pin for speakerphone operation */
   int ext_spkr_pwdn_gpio;

   /* Line-out detection GPIO pin for external speakerphone operation */
   int line_out_det_gpio;
   int line_out_det_gpio_inserted;

} HALAUDIO_EXTENSIONS_CFG;

/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes --------------------------------------- */

#endif /* HALAUDIO_EXTENSIONS_CFG_H */
