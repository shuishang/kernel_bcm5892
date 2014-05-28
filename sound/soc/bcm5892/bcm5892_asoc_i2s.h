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


#ifndef BCM5892_I2S_H
#define BCM5892_I2S_H

#include <linux/types.h>


/* Debug trace */
#define I2S_ENABLE_LOG           0

#if I2S_ENABLE_LOG
#define I2SLOG(f, ...)           printk(KERN_INFO f, ##__VA_ARGS__)
#else
#define I2SLOG(f, ...)
#endif


/**********************************************************************
 *  Chip specific
 **********************************************************************/
#define I2S_FIFO_DEPTH   192 /* in half words */

/**********************************************************************
 *  Register
 **********************************************************************/
#define I2SREG_DATXCTRL	    (volatile unsigned int *)(i2s_reg_base + 0x0000) /* 0xd00d8000 */
#define I2SREG_DARXCTRL	    (volatile unsigned int *)(i2s_reg_base + 0x0004)
#define I2SREG_DAFIFO_DATA  (volatile unsigned int *)(i2s_reg_base + 0x0008)
#define I2SREG_DAI2S   	    (volatile unsigned int *)(i2s_reg_base + 0x000C) /* CONTROL REG */
#define I2SREG_DAI2SRX      (volatile unsigned int *)(i2s_reg_base + 0x0010)
#define I2SREG_DADMACTRL    (volatile unsigned int *)(i2s_reg_base + 0x0014)
#define I2SREG_DADEBUG 	    (volatile unsigned int *)(i2s_reg_base + 0x0018)
#define I2SREG_DASTA   	    (volatile unsigned int *)(i2s_reg_base + 0x001C)
#define I2SREG_DADEBUG_TXP  (volatile unsigned int *)(i2s_reg_base + 0x0020)
#define I2SREG_DADEBUG_RXP  (volatile unsigned int *)(i2s_reg_base + 0x0024)

/**********************************************************************
 *  Register Fields
 **********************************************************************/
#define I2S_DATXCTRL_INT_PERIOD  0x00FF00  /* bit 15:8, PLAYBACK INTERRUPT PERIOD */
#define I2S_DATXCTRL_INT_PERIOD_SHIFT   8  /* bit 15:8, PLAYBACK INTERRUPT PERIOD */
#define I2S_DATXCTRL_INT_EN      0x000080  /* bit 7, ENABLE PLAYBACK INTERRUPT */
#define I2S_DATXCTRL_INT_STATUS  0x000040  /* bit 6, PLAYBACK INTERRUPT STATUS */
#define I2S_DATXCTRL_SRST        0x000008  /* bit 3, SOFT RESET */
#define I2S_DATXCTRL_TX_FLUSH    0x000002  /* bit 1, TX FIFO FLUSH   */
#define I2S_DATXCTRL_TX_EN       0x000001  /* bit 0, TX FIFO ENABLE  */

#define I2S_DAIS_TX_SAMPLE_SHIFT 8         /* bit 11:8 */
#define I2S_DAIS_TX_SAMPLE_MASK  0x000F00  /* bit 11:8 */
#define I2S_DAIS_TX_START_RIGHT  0x000040  /* bit 6 */
#define I2S_DAIS_TX_START_LEFT   0x000000  /* bit 6 */
#define I2S_DAIS_TX_EN           0x000020  /* bit 5 TX INTERFACE ENABLE */
#define I2S_DAIS_RX_EN           0x000010  /* bit 4 RX INTERFACE ENABLE */
#define I2S_DAIS_TX_STEREO       0x000000  /* bit 1 PLAY STEREO */
#define I2S_DAIS_TX_MONO         0x000002  /* bit 1 PLAY MONO */

#define I2S_DADMACTRL_TX_EN      0x000080  /* bit 7 Tx En */
#define I2S_DADMACTRL_TX_SIZE_1  0x000000  /* bit 2-0: 0-1sample, 1-4sample, 2-8sample, ... */
#define I2S_DADMACTRL_TX_SIZE_4  0x000001 

#define I2S_DADEBUG_TXFIFO_CNT_MASK 0x0FF  /* bit 7:0 */
#define I2S_GET_TXFIFO_CNT()     (*I2SREG_DADEBUG & I2S_DADEBUG_TXFIFO_CNT_MASK)

#endif
