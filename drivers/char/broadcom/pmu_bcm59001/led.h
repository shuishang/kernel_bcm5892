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
*  led.h
*
*  PURPOSE:
*
*  This file defines the internal LED interface to the Broadcom BCM59001 PMU chip
*
*  NOTES:
*
*****************************************************************************/


#if !defined( BCM59001_LED_H )
#define BCM59001_LED_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/broadcom/pmu_chip.h>

/* ---- Constants and Types ---------------------------------------------- */

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

int led59001_init( void );

#endif  /* BCM59001_LED_H */


