/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
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
*  @file    apm_drv.h
*
*  @brief   Private definitions for the BCMRING APM driver.
*
****************************************************************************/
#if !defined( APM_DRV_H )
#define APM_DRV_H

/* ---- Include Files ---------------------------------------------------- */

/* ---- Constants and Types ---------------------------------------------- */
#define APM_MAX_MUX_POSITIONS    2     /* Supports mux positions A and B */

/* APM microphone detection channel numbers */
typedef enum apm_micdet_chan
{
   APM_MICDET_A = 0,
   APM_MICDET_B = 1,
   APM_MICDET_MAX_CHANS
}
APM_MICDET_CHAN;

/* Microphone detection callbacks */
typedef void (*APM_MIC_IN_FP)( APM_MICDET_CHAN chan );
typedef void (*APM_MIC_ON_FP)( APM_MICDET_CHAN chan );

typedef struct apm_mic_det_ops
{
   APM_MIC_IN_FP  mic_in;
   APM_MIC_ON_FP  mic_on;
}
APM_MIC_DET_OPS;


/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

/***************************************************************************/
/**
*  Set microphone detection callback operations.
*/
void apmSetMicDetectOps( 
   APM_MIC_DET_OPS *ops       /*<< (i) Ptr to callbacks. If NULL, reset callbacks */
);

/***************************************************************************/
/**
*  Enable microphone detection
*/
int apmEnableMicDetect(
   enum apm_micdet_chan ch,         /**< (i) Channel */
   int                  enable      /**< (i) 1 to enable, 0 to disable */
);

/***************************************************************************/
/**
*  Initialize APM headset detection
*
*  @remarks
*/
int apmHeadsetInit( void );

/***************************************************************************/
/**
*  Destructor for the APM headset detection driver
*
*  @remarks
*/
void apmHeadsetExit( void );

#endif /* APM_DRV_H */

