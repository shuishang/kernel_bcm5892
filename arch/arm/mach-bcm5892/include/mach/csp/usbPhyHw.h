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
/****************************************************************************/
/**
*  @file    usbPhyHw.h
*
*  @brief   API definitions for low level USB PHY drivers.
*
*  @note
*/
/****************************************************************************/

#ifndef USBPHYHW_H
#define USBPHYHW_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ---- Include Files ---------------------------------------------------- */

/*
 * The following include will provide any chip-specific definitions needed to
 * implement the API described in this header file, e.g. inline functions.
 */
#include "usbPhyHw_def.h"

/* ---- Public Constants and Types --------------------------------------- */

/* The following are to be defined in usbPhyHw_def.h. */
/* usbPhyHw_PORT_CNT       // Instance count of USB PHY ports */

/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */

/****************************************************************************/
/**
*  @brief   Finalize (terminate) PHY operations
*
*/
/****************************************************************************/
static inline void usbPhyHw_OpsFinis
(
    unsigned num    /* [IN] PHY port number (0-based) */
);

/****************************************************************************/
/**
*  @brief   Initialize PHY operations
*
*  @note    Have to do a subsequent usbPhyHw_Start() after usbPhyHw_Init().
*
*/
/****************************************************************************/
static inline void usbPhyHw_OpsInit
(
    unsigned num    /* [IN] PHY port number (0-based) */
);

/****************************************************************************/
/**
*  @brief   Reset PHY operations
*
*/
/****************************************************************************/
static inline void usbPhyHw_OpsReset
(
    unsigned num    /* [IN] PHY port number (0-based) */
);

/****************************************************************************/
/**
*  @brief   Power on PHY
*
*  @todo The PHY documentation is horrible. Not sure what the various power on/off
*   do (afe, utmi, pll, xtal) or how to use them.
*
*/
/****************************************************************************/
static inline void usbPhyHw_PwrOn
(
    unsigned num    /* [IN] PHY port number (0-based) */
);

/****************************************************************************/
/**
*  @brief   Power off PHY
*
*/
/****************************************************************************/
static inline void usbPhyHw_PwrOff
(
    unsigned num    /* [IN] PHY port number (0-based) */
);

/****************************************************************************/
/**
*  @brief   Start PHY operations
*
*/
/****************************************************************************/
static inline void usbPhyHw_Start
(
    unsigned num    /* [IN] PHY port number (0-based) */
);

/****************************************************************************/
/**
*  @brief   Stop PHY operations
*
*/
/****************************************************************************/
static inline void usbPhyHw_Stop
(
    unsigned num    /* [IN] PHY port number (0-based) */
);

/****************************************************************************/
/**
*  @brief   Prints USB PHY configuration and register info
*
*/
/****************************************************************************/
void usbPhyHw_PrintInfo
(
    unsigned num,                       /* [IN] PHY instance number (0-based) */
    int (*printFP) (const char *, ...)  /* [IN] printf routine to use for output */
);


#ifdef __cplusplus
extern "C"
}
#endif

#endif /* USBPHYHW_H */

