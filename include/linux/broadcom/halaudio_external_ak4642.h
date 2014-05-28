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

#ifndef HALAUDIO_EXTERNAL_AK4642_H
#define HALAUDIO_EXTERNAL_AK4642_H

/* ---- Include Files ---------------------------------------------------- */
#include <linux/broadcom/halaudio.h>      /* HAL Audio API */

/* ---- Constants and Types ---------------------------------------------- */
typedef struct
{
   int gpio_codec_pdn;
} HALAUDIO_EXTERNAL_AK4642_CFG;

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */
int  halAudioExternalAK4642_init( void );
int  halAudioExternalAK4642_exit( void );
int  halAudioExternalAK4642_SetFrequency( 
   int freqHz                 /**< (i) Frequency */
);
int halAudioExternalAK4642_GainSetAnalogHardware(
   int             db,        /**< (i) Gain in dB */
   HALAUDIO_DIR    dir,       /**< (i) Selects direction */
   HALAUDIO_HWSEL  hwsel      /**< (i) Selects one of two PGA blocks */
);
int halAudioExternalAK4642_GainGetAnalogHardware(
   HALAUDIO_GAIN_INFO  *info,       /*<< (o) Ptr to gain info structure */
   HALAUDIO_DIR         dir,        /*<< (i) Direction path */
   HALAUDIO_HWSEL       hwsel       /*<< (i) Hardware mux selection */
);
int halAudioExternalAK4642_GainSetDigitalHardware(
   int             db,        /**< (i) Gain in dB */
   HALAUDIO_DIR    dir        /**< (i) Selects direction */
);
int halAudioExternalAK4642_GainGetDigitalHardware(
   HALAUDIO_GAIN_INFO  *info,       /*<< (o) Ptr to gain info structure */
   HALAUDIO_DIR         dir         /*<< (i) Direction path */
);
int halAudioExternalAK4642_SidetoneSet(
   int      db                      /**< (i) Gain in db */
);
int halAudioExternalAK4642_SidetoneGet(
   HALAUDIO_GAIN_INFO  *info        /*<< (o) Ptr to gain info structure */
);
void halAudioExternalAK4642_CodecReset(
   int reset      /**< (i) reset */
);
void halAudioExternalAK4642_PowerEnable(
   int enable     /**< (i) enable */
);

#endif   /* HALAUDIO_EXTERNAL_AK4642_H */

