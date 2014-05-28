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
*  @file    mm_io.h
*
*  @brief   Memory Map I/O definitions
*
*  @note
*     None
*/
/****************************************************************************/

#ifndef _MM_IO_H
#define _MM_IO_H

/* ---- Include Files ---------------------------------------------------- */
/* ^^#include <mm_addr.h> */
#include "mm_addr.h"

/*
 * BCM5892 Specific IO memory mapping file 
*/

#include <mach/hardware.h>

#if !defined( CSP_SIMULATION )
#include <cfg_global.h>
#endif

/* ---- Public Constants and Types --------------------------------------- */

#if defined( CONFIG_MMU )

/* This macro is referenced in <mach/io.h> */

#ifndef MM_IO_PHYS_TO_VIRT

#ifdef __ASSEMBLY__

#define MM_IO_PHYS_TO_VIRT(phys) (0xF0000000 | (((phys) >> 4) & 0x0F000000) | ((phys) & 0xFFFFFF))

#else

#define MM_IO_PHYS_TO_VIRT(phys)                                           \
( (((phys) >= 0x20000000) && ((phys) < 0x30000000)) ?                      \
/* Phys to Virtual 0x2xxxxxxx => 0xExxxxxxx */                             \
((phys) | 0xC0000000) :                                                    \
/* Explicitly map phys addr 0x0F0000000 to virt addr 0xF0000000. This */   \
/* address range is reserved for VPM external prog and data memories. */   \
( ((phys) == 0x0F000000) ? 0xF0000000 :                                    \
/* Phys to Virtual 0xNyxxxxxx => 0xFNxxxxxx where N != 2 */                \
(0xF0000000 | (((phys) >> 4) & 0x0F000000) | ((phys) & 0xFFFFFF)) ))

#endif

#endif

#ifndef MM_IO_VIRT_TO_PHYS

#ifdef __ASSEMBLY__
#define MM_IO_VIRT_TO_PHYS(virt) ((((virt) & 0x0F000000) << 4) | ((virt) & 0xFFFFFF))
#else
#define MM_IO_VIRT_TO_PHYS(virt) ((((virt) & 0xF0000000) == 0xE0000000) ?  \
/* Virtual to Physical 0xExxxxxxx => 0x2xxxxxxx */                         \
(((virt) & ~0xF0000000) | 0x20000000)  :                                   \
/* Explicitly map phys addr 0x0F0000000 to virt addr 0xF0000000. This */   \
/* address range is reserved for VPM external prog and data memories. */   \
(((virt) == 0xF0000000) ? 0x0F000000 : \
/* Virtual to Physical 0xFNxxxxxx => 0xN0xxxxxx */                         \
((((virt) & 0x0F000000) << 4) | ((virt) & 0xFFFFFF))) )

#endif
#endif

#else

#ifndef MM_IO_PHYS_TO_VIRT
#define MM_IO_PHYS_TO_VIRT(phys)       (phys)
#endif

#ifndef MM_IO_VIRT_TO_PHYS
#define MM_IO_VIRT_TO_PHYS(virt)       (virt)
#endif

#endif

/* Registers in 0xExxxxxxx that should be moved to 0xFxxxxxxx */


#define MM_IO_BASE_FLASHC              MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_FLASHC )
#define MM_IO_BASE_NAND                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_NAND )
#define MM_IO_BASE_UMI                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_UMI )

#define MM_IO_START MM_ADDR_IO_FLASHC  /* Physical beginning of IO mapped memory */
#define MM_IO_BASE  MM_IO_BASE_FLASHC  /* Virtual beginning of IO mapped memory */

#define MM_IO_BASE_BROM                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_BROM )
#define MM_IO_BASE_ARAM                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_ARAM )


#define MM_IO_BASE_DMA0                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_DMA0 )
#define MM_IO_BASE_DMA1                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_DMA1 )


#define MM_IO_BASE_ESW                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_ESW )
#define MM_IO_BASE_CLCD                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_CLCD )
#define MM_IO_BASE_PIF                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_PIF )
#define MM_IO_BASE_VPM_PROG            MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_VPM_PROG )
#define MM_IO_BASE_VPM_DATA            MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_VPM_DATA )

#define MM_IO_BASE_VRAM                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_VRAM )

#define MM_IO_BASE_CHIPC               MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_CHIPC )
#define MM_IO_BASE_DDRC                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_DDRC )
#define MM_IO_BASE_LEDM                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_LEDM )
#define MM_IO_BASE_PWM                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_PWM )
#define MM_IO_BASE_VINTC               MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_VINTC )
#define MM_IO_BASE_GPIO0               MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_GPIO0 )
#define MM_IO_BASE_GPIO1               MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_GPIO1 )
#define MM_IO_BASE_TMR                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_TMR )
#define MM_IO_BASE_WDOG                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_WDOG )
#define MM_IO_BASE_ETM                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_ETM )
#define MM_IO_BASE_HPM                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_HPM )
#define MM_IO_BASE_TZPC                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_TZPC )
#define MM_IO_BASE_MPU                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_MPU )
#define MM_IO_BASE_SPUMP               MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_SPUMP )
#define MM_IO_BASE_PKA                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_PKA )
#define MM_IO_BASE_RNG                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_RNG )
#define MM_IO_BASE_KEYC                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_KEYC )
#define MM_IO_BASE_OTP                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_OTP )
#define MM_IO_BASE_I2S0                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_I2S0 )
#define MM_IO_BASE_I2S1                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_I2S1 )
#define MM_IO_BASE_UARTA               MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_UARTA )
#define MM_IO_BASE_UARTB               MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_UARTB )
#define MM_IO_BASE_I2CH                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_I2CH )
#define MM_IO_BASE_SPIH                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_SPIH )
#define MM_IO_BASE_TSC                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_TSC )
#define MM_IO_BASE_I2CS                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_I2CS )
#define MM_IO_BASE_SPIS                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_SPIS )
#define MM_IO_BASE_SINTC               MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_SINTC )
#define MM_IO_BASE_INTC0               MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_INTC0 )
#define MM_IO_BASE_INTC1               MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_INTC1 )
#define MM_IO_BASE_GE                  MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_GE )


/*
 * #define MM_IO_BASE_USB_CTLR0           MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_USB_CTLR0 )
 * #define MM_IO_BASE_USB_CTLR1           MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_USB_CTLR1 )
 */

/*
 * BCM5892 Specific address conversion
 */

#define MM_IO_BASE_USB_CTLR1           		         IO_ADDRESS(MM_ADDR_IO_USB_CTLR1 )
#define MM_IO_BASE_USBWC_R_usb_phy_control_MEMADDR       IO_ADDRESS(MM_ADDR_USBWC_R_usb_phy_control_MEMADDR) 
#define MM_IO_BASE_USBWC_F_utmi_pwrdwnb_i_MASK            MM_ADDR_USBWC_F_utmi_pwrdwnb_i_MASK
#define MM_IO_BASE_USBWC_F_afe_pwrdwnb_i_MASK    	  MM_ADDR_USBWC_F_afe_pwrdwnb_i_MASK
#define MM_IO_BASE_USBWC_F_soft_resetb_i_MASK    	  MM_ADDR_USBWC_F_soft_resetb_i_MASK
#define MM_IO_BASE_USBWC_R_usb_gen_control_MEMADDR       IO_ADDRESS(MM_ADDR_USBWC_R_usb_gen_control)
#define MM_IO_BASE_GIO2_R_GRPF2_OUT_ENABLE_MEMADDR  	 IO_ADDRESS(MM_ADDR_GIO2_R_GRPF2_OUT_ENABLE)
#define MM_IO_BASE_GIO2_R_GRPF2_DATA_OUT_MEMADDR  	 IO_ADDRESS(MM_ADDR_GIO2_R_GRPF2_DATA_OUT)
#define MM_IO_BASE_GIO1_R_GRPF1_OUT_ENABLE_MEMADDR  	 IO_ADDRESS(MM_ADDR_GIO1_R_GRPF1_OUT_ENABLE)
#define MM_IO_BASE_GIO1_R_GRPF1_DATA_OUT_MEMADDR  	 IO_ADDRESS(MM_ADDR_GIO1_R_GRPF1_DATA_OUT)




/*
 * Addresses Mapping for UDC Global Registers
 */ 

#define MM_IO_BASE_USB1_DEV_CFG_R_DEV			 IO_ADDRESS(MM_ADDR_USB1_DEV_CFG_R_REG)     /*0x400 */
#define MM_IO_BASE_USB1_DEV_CTRL_R_DEV			 IO_ADDRESS(MM_ADDR_USB1_DEV_CTRL_R_REG)    /*0x404 */
#define MM_IO_BASE_USB1_DEV_STS_R_DEV			 IO_ADDRESS(MM_ADDR_USB1_DEV_STS_R_REG)     /*0x408 */
#define MM_IO_BASE_USB1_DEV_INT_R_STATUS		 IO_ADDRESS(MM_ADDR_USB1_DEV_INT_R_STATUS)  /*0x40c */
#define MM_IO_BASE_USB1_DEV_INT_R_MASK			 IO_ADDRESS(MM_ADDR_USB1_DEV_INT_R_MASK)    /*0x410 */
#define MM_IO_BASE_USB1_EP_INT_R_STATUS			 IO_ADDRESS(MM_ADDR_USB1_EP_INT_R_STATUS)   /*0x414 */
#define MM_IO_BASE_USB1_EP_INT_R_MASK			 IO_ADDRESS(MM_ADDR_USB1_EP_INT_R_MASK)     /*0x418 */

/*
 * Addresses Mapping for UDC EP0-OUT Registers
 */ 

#define MM_IO_BASE_USB1_EP0_CTRL_R_REG			 IO_ADDRESS(MM_ADDR_USB1_EP0_CTRL_R_REG) /*0x200 */
#define MM_IO_BASE_USB1_EP_STS_R_STATUS			 IO_ADDRESS(MM_ADDR_USB1_EP_STS_R_STATUS)/*0x204 */
#define MM_IO_BASE_USB1_EP0_PKT_FRAME_NUM_R_REG		 IO_ADDRESS(MM_ADDR_USB1_EP0_PKT_FRAME_NUM_R_REG) /*0x208 */
#define MM_IO_BASE_USB1_EP0_BUF_SIZ_MAX_PKT_SZ_R_REG	 IO_ADDRESS(MM_ADDR_USB1_EP0_BUF_SIZ_MAX_PKT_SZ_R_REG) /*0x20C */
#define MM_IO_BASE_USB1_EP0_SETUP_BUF_PTR__R_REG	 IO_ADDRESS(MM_ADDR_USB1_EP0_SETUP_BUF_PTR_R_REG) /*0x210 */
#define MM_IO_BASE_USB1_EP0_DATA_DESC_PTR__R_REG	 IO_ADDRESS(MM_ADDR_USB1_EP0_DATA_DESC_PTR_R_REG) /*0x214 */


/*
 * Addresses Mapping for UDC EP0-IN Registers
 */ 

#define MM_IO_BASE_USB1_EP0_IN_CTRL_R_REG		 IO_ADDRESS(MM_ADDR_USB1_EP_IN_CTRL_R_REG) /*0x000 */
#define MM_IO_BASE_USB1_EP_IN_R_STATUS			 IO_ADDRESS(MM_ADDR_USB1_EP_IN_R_STATUS) /*0x004 */
#define MM_IO_BASE_USB1_EP0_IN_BUF_SZ_R_REG		 IO_ADDRESS(MM_ADDR_USB1_EP_IN_BUF_SZ_R_REG) /*0x008 */
#define MM_IO_BASE_USB1_EP0_IN_MAX_PKT_SZ_R_REG		 IO_ADDRESS(MM_ADDR_USB1_EP_IN_MAX_PKT_SZ_R_REG) /*0x00C */
#define MM_IO_BASE_USB1_EP0_IN_DATA_DESC_PTR_R_REG		 IO_ADDRESS(MM_ADDR_USB1_EP_IN_DATA_DESC_PTR_R_REG) /*0x014 */



#define MM_IO_BASE_USB_PHY             MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_USB_PHY )
#define MM_IO_BASE_SDIOH0              MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_SDIOH0 )
#define MM_IO_BASE_SDIOH1              MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_SDIOH1 )
#define MM_IO_BASE_APM                 MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_APM )
#define MM_IO_BASE_SPUM                MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_SPUM )



#define MM_IO_BASE_VPM_EXTMEM_RSVD     MM_IO_PHYS_TO_VIRT( MM_ADDR_IO_VPM_EXTMEM_RSVD )

/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */


#endif /* _MM_IO_H */

