/*****************************************************************************
*  Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
*  http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*
*****************************************************************************/


/****************************************************************************/
/**
*  @file    bcm5892_asoc_code_internal.h
*
*  @brief   definitions for low level BCM5892 internal DAC device
*
*  @note
*/
/****************************************************************************/

#ifndef BCM5892_DAC_H
#define BCM5892_DAC_H




/**********************************************************************
 *  Register
 **********************************************************************/
#define DACREG_DAC_CONFIG	    (volatile unsigned int *)(dac_reg_base + 0x0000)

/**********************************************************************
 *  Register Fields
 **********************************************************************/
#define DAC_I2S_SAMPLE_RIGHT     0x80000000  /* bit31, SAMPLE MONO FROM RIGHT */
#define DAC_INTERPOLATION_BYP    0x40000000  /* bit30, INTERPOLATION BYPASS */
#define DAC_SFT_RST              0x20000000  /* bit29, SW RESET */
#define DAC_I2S_ENABLE           0x00000001  /* bit 1, ENABLE I2S DRIVE TO DAC */



#endif
