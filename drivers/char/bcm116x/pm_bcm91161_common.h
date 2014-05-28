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




/*
*
*****************************************************************************
*
*  pm_bcm91161_common.h
*
*  PURPOSE:
*
*     Some common functions amongst BCM91161 Platforms
*
*  NOTES:
*
*****************************************************************************/


#if !defined( _PM_BCM91161_COMMON_H_ )
#define _PM_BCM91161_COMMON_H_

#if defined( __KERNEL__ )

/* ---- Include Files ---------------------------------------------------- */

#include <linux/broadcom/PowerManager.h>

/* ---- Constants and Types ---------------------------------------------- */

/* TODO: The various ADC values should be documented. */
/*
 * I'd like to see the resistor values, the voltage, and the nominal 
 * voltage and value expected to be recorded here. 
 */

#define BCM91161VP_ADC_CHANNEL             5
#define BCM91161VP_V2BOARD_ADCTHRESHOLD    180

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

void pm_bcm91161_platform_extra_init( void );
void pm_bcm91161_wifi_extra_init( void );
void pm_bcm91161_wifi_extra_action(PM_CompPowerLevel powerLevel);
void pm_bcm91161_usb_extra_init( void );
void pm_bcm91161_usb_extra_action(PM_CompPowerLevel powerLevel);
void pm_bcm91161_audio_extra_init( void );

#endif

#endif  /* _PM_BCM91161_COMMON_H_ */

