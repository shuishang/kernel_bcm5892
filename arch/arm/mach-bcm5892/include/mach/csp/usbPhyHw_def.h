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

#ifndef USBPHYHW_DEF_H
#define USBPHYHW_DEF_H

#ifndef USBPHYHW_H
#error *** Do not include usbPhyHw_def.h directly. Use usbPhyHw.h instead. ***
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/* ---- Include Files ---------------------------------------------------- */

#include "reg.h"

#include "mm_io.h"
#include "usbPhyHw_reg_bcm_usb20phy.h"

/* ---- Public Constants and Types --------------------------------------- */

#define usbPhyHw_PORT_CNT           usbPhyHw_REG_PORT_CNT   /* Instance count of USB PHY ports */

#define usbPhyHw_REG_P              ((volatile usbPhyHw_REG_t *)(MM_IO_BASE_USB_PHY))

/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */

/* ---- Inline Function Definitions -------------------------------------- */

/*****************************************************************************
* See usbPhyHw.h for API documentation
*****************************************************************************/
static inline void usbPhyHw_OpsFinis( unsigned num )
{
    /* Disconnect port and put it into reset. */
    usbPhyHw_REG_P->utmiCtrl |= ((1 << num) << usbPhyHw_REG_UTMICTRL_PORT_DISCONNECT_SHIFT);
    usbPhyHw_REG_P->utmiCtrl &= ~((1 << num) << usbPhyHw_REG_UTMICTRL_PORT_RESET_DISABLE_SHIFT);
}

/*****************************************************************************
* See usbPhyHw.h for API documentation
*****************************************************************************/
static inline void usbPhyHw_OpsInit( unsigned num )
{
    /*
     * Try and use a register field that is block only (not def'd on a per port
     * basis) and has a reset value other than we want to use during runtime as
     * an indicator as to whether or not the PHY block has been initialized.
     * If this is not possible, then some form of variable is needed to handle this.
     * Use the PLL calibration enable field for now.
     */
    if ( !(usbPhyHw_REG_P->pllCtrl & usbPhyHw_REG_PLLCTRL_CALIB_ENABLE) )
    {
        /*
         * PHY block has not been initialized. Perform the following sequence.
         * - disconnect all ports
         * - put all ports into reset mode
         * - enable digital chirp detect
         * - put the block into reset mode and enable PLL calibration
         * - wait a period of time for PLL to reset
         * - take the block out of reset
         * - wait a period of time for PLL to lock
         */
        usbPhyHw_REG_P->utmiCtrl |=  usbPhyHw_REG_UTMICTRL_PORT_DISCONNECT_MASK;
        usbPhyHw_REG_P->utmiCtrl &= ~usbPhyHw_REG_UTMICTRL_PORT_RESET_DISABLE_MASK;
        usbPhyHw_REG_P->utmiCtrl |=  usbPhyHw_REG_UTMICTRL_CHIRP_DETECT_DIGITAL;
        usbPhyHw_REG_P->pllCtrl  |= (usbPhyHw_REG_PLLCTRL_RESET_ENABLE | usbPhyHw_REG_PLLCTRL_CALIB_ENABLE);
        /* @todo Need to wait for 2048 refclk cycles... */
        usbPhyHw_REG_P->pllCtrl  &= ~usbPhyHw_REG_PLLCTRL_RESET_ENABLE;
        /* @todo Need to wait for 4096 refclk cycles... */
    }

    /*
     * Normal port level initialization. Just take the port out of reset. Also make sure
     * it is powered up.
     */
    usbPhyHw_REG_P->utmiCtrl |= ((1 << num) << usbPhyHw_REG_UTMICTRL_PORT_AFE_PWR_ON_SHIFT);
    usbPhyHw_REG_P->utmiCtrl |= ((1 << num) << usbPhyHw_REG_UTMICTRL_PORT_RESET_DISABLE_SHIFT);
}

/*****************************************************************************
* See usbPhyHw.h for API documentation
*****************************************************************************/
static inline void usbPhyHw_OpsReset( unsigned num )
{
    usbPhyHw_OpsFinis( num );
    usbPhyHw_OpsInit( num );
}

/*****************************************************************************
* See usbPhyHw.h for API documentation
*****************************************************************************/
static inline void usbPhyHw_PwrOff( unsigned num )
{
    usbPhyHw_REG_P->utmiCtrl &= ~((1 << num) << usbPhyHw_REG_UTMICTRL_PORT_AFE_PWR_ON_SHIFT);
}

/*****************************************************************************
* See usbPhyHw.h for API documentation
*****************************************************************************/
static inline void usbPhyHw_PwrOn( unsigned num )
{
    usbPhyHw_REG_P->utmiCtrl |= ((1 << num) << usbPhyHw_REG_UTMICTRL_PORT_AFE_PWR_ON_SHIFT);
}

/*****************************************************************************
* See usbPhyHw.h for API documentation
*****************************************************************************/
static inline void usbPhyHw_Start( unsigned num )
{
    usbPhyHw_REG_P->utmiCtrl &= ~((1 << num) << usbPhyHw_REG_UTMICTRL_PORT_DISCONNECT_SHIFT);
}

/*****************************************************************************
* See usbPhyHw.h for API documentation
*****************************************************************************/
static inline void usbPhyHw_Stop( unsigned num )
{
    usbPhyHw_REG_P->utmiCtrl |= ((1 << num) << usbPhyHw_REG_UTMICTRL_PORT_DISCONNECT_SHIFT);
}


#ifdef __cplusplus
extern "C"
}
#endif

#endif /* USBPHYHW_DEF_H */

