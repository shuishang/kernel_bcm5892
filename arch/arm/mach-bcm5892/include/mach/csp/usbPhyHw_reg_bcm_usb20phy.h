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

#ifndef USBPHYHW_REG_BCM_USB20PHY2_H
#define USBPHYHW_REG_BCM_USB20PHY2_H

/* ---- Include Files ---------------------------------------------------- */

#include "stdint.h"

/* ---- Public Constants and Types --------------------------------------- */

#define usbPhyHw_REG_PORT_CNT           2   /* Instance count of USB PHY ports */

#define usbPhyHw_REG_UTMICTRL_PORT_AFE_PWR_ON_SHIFT         30
#define usbPhyHw_REG_UTMICTRL_PORT_AFE_PWR_ON_MASK          (3 << usbPhyHw_REG_UTMICTRL_PORT_AFE_PWR_ON_SHIFT)
#define usbPhyHw_REG_UTMICTRL_PORT_CLK_ENABLE_SHIFT         28
#define usbPhyHw_REG_UTMICTRL_PORT_CLK_ENABLE_MASK          (3 << usbPhyHw_REG_UTMICTRL_PORT_CLK_ENABLE_SHIFT)
#define usbPhyHw_REG_UTMICTRL_PORT_RESET_DISABLE_SHIFT      26
#define usbPhyHw_REG_UTMICTRL_PORT_RESET_DISABLE_MASK       (3 << usbPhyHw_REG_UTMICTRL_PORT_RESET_DISABLE_SHIFT)
#define usbPhyHw_REG_UTMICTRL_IDDQ_ENABLE                   (1 << 23)
#define usbPhyHw_REG_UTMICTRL_PORT_DISCONNECT_SHIFT         21
#define usbPhyHw_REG_UTMICTRL_PORT_DISCONNECT_MASK          (3 << usbPhyHw_REG_UTMICTRL_PORT_DISCONNECT_SHIFT)
#define usbPhyHw_REG_UTMICTRL_LOOPBK_DIGITAL_ENABLE         (1 << 20)
#define usbPhyHw_REG_UTMICTRL_CHIRP_DETECT_DIGITAL          (1 << 19)
#define usbPhyHw_REG_UTMICTRL_CHIRP_DETECT_ANALOG           0
#define usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_SHIFT         16
#define usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_MASK          (7 << usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_SHIFT)
#define usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_BITS_4_DFLT   (0 << usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_SHIFT)
#define usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_BITS_2        (1 << usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_SHIFT)
#define usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_BITS_3        (2 << usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_SHIFT)
#define usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_BITS_4        (3 << usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_SHIFT)
#define usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_BITS_5        (4 << usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_SHIFT)
#define usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_BITS_6        (5 << usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_SHIFT)
#define usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_BITS_7        (6 << usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_SHIFT)
#define usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_BITS_8        (7 << usbPhyHw_REG_UTMICTRL_SYNC_DETECT_LEN_SHIFT)
#define usbPhyHw_REG_UTMICTRL_GENERIC_SHIFT                 0
#define usbPhyHw_REG_UTMICTRL_GENERIC_MASK                  (0xff << usbPhyHw_REG_UTusbPhyHw_REG_UTMICTRL_GENERIC_SHIFTMICTRL_SYNC_DETECT_LEN_SHIFT)

#define usbPhyHw_REG_PLLCTRL_SUSPEND_ENABLE                 (1 << 10)
#define usbPhyHw_REG_PLLCTRL_RESET_ENABLE                   (1 << 9)
#define usbPhyHw_REG_PLLCTRL_BYPASS_ENABLE                  (1 << 8)
#define usbPhyHw_REG_PLLCTRL_REFCLK_FREQ_SHIFT              6
#define usbPhyHw_REG_PLLCTRL_REFCLK_FREQ_MASK               (3 << usbPhyHw_REG_PLLCTRL_REFCLK_FREQ_SHIFT)
#define usbPhyHw_REG_PLLCTRL_REFCLK_FREQ_MHZ_48             (0 << usbPhyHw_REG_PLLCTRL_REFCLK_FREQ_SHIFT)
#define usbPhyHw_REG_PLLCTRL_REFCLK_FREQ_MHZ_30             (1 << usbPhyHw_REG_PLLCTRL_REFCLK_FREQ_SHIFT)
#define usbPhyHw_REG_PLLCTRL_REFCLK_FREQ_MHZ_28             (2 << usbPhyHw_REG_PLLCTRL_REFCLK_FREQ_SHIFT)
#define usbPhyHw_REG_PLLCTRL_CALIB_ENABLE                   (1 << 5)
#define usbPhyHw_REG_PLLCTRL_PWR_ON                         (1 << 3)
#define usbPhyHw_REG_PLLCTRL_XTAL_PWR_ON                    (1 << 2)
#define usbPhyHw_REG_PLLCTRL_REFCLK_SEL_SHIFT               0
#define usbPhyHw_REG_PLLCTRL_REFCLK_SEL_MASK                (3 << usbPhyHw_REG_PLLCTRL_REFCLK_SEL_SHIFT)
#define usbPhyHw_REG_PLLCTRL_REFCLK_SEL_XTAL_INPUT          (0 << usbPhyHw_REG_PLLCTRL_REFCLK_SEL_SHIFT)
#define usbPhyHw_REG_PLLCTRL_REFCLK_SEL_XTAL_OSCILLATOR     (1 << usbPhyHw_REG_PLLCTRL_REFCLK_SEL_SHIFT)
#define usbPhyHw_REG_PLLCTRL_REFCLK_SEL_INTERNAL_DIFFERNTL  (2 << usbPhyHw_REG_PLLCTRL_REFCLK_SEL_SHIFT)
#define usbPhyHw_REG_PLLCTRL_REFCLK_SEL_INTERNAL_SINGLE_END (2 << usbPhyHw_REG_PLLCTRL_REFCLK_SEL_SHIFT)


typedef struct
{
    uint32_t bertCtrl1;
    uint32_t bertCtrl2;
    uint32_t bertStat1;
    uint32_t bertStat2;
    uint32_t utmiCtrl;
    uint32_t testPortCtrl;
    uint32_t pllCtrl;
    uint32_t testPortIn;
    uint32_t testPortOut;
    uint32_t mdioWrite;
    uint32_t mdioRead;
    uint32_t mdioCtrl;
    uint32_t mdioCfg;
    uint32_t spare;
}
usbPhyHw_REG_t;

/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */


#endif /* USBPHYHW_REG_BCM_USB20PHY2_H */

