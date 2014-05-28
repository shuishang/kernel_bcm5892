/*****************************************************************************
* Copyright 2008 - 2011 Broadcom Corporation.  All rights reserved.
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



#ifndef __BCM5892_MAC_DOT_H_INCLUDED__
#define __BCM5892_MAC_DOT_H_INCLUDED__

#undef PDEBUGG
#define PDEBUGG(fmt, args...) /* nothing: it's a placeholder */

#ifdef K_INT_PHY
#define BCM5892_MAC_PHY_INT		K_INT_PHY
#else
#define BCM5892_MAC_PHY_INT		PHY_POLL
#endif

#define BCM5892_ETH0_HWADDR "00:0A:F7:00:01:00" 
#define BCM5892_ETH1_HWADDR "00:0A:F7:00:01:01"

#define IDX_IORES_IRQ 0
#define IDX_IORES_MAC_MEM 0 //1 //Choice of Ordinal Vs Cardinal
#define IDX_IORES_MMI_MEM 1 //2 //Choice of Ordianl Vs Cardinal
#define TX_TIMEOUT  (2*HZ)
#define GS_TIMEOUT  (2*HZ)

/*Cheap and Dirty copy from MS Windows*/
#define UNREFERENCED_PARAMETER(x) x
 
/**********************************************************************
 *  Simple types
 **********************************************************************/
enum bcm5892_speed
{
        bcm5892_speed_none = 0,
        bcm5892_speed_10 = SPEED_10,
        bcm5892_speed_100 = SPEED_100,
        bcm5892_speed_1000 = SPEED_1000,
};

enum bcm5892_duplex
{
        bcm5892_duplex_none = -1,
        bcm5892_duplex_half = DUPLEX_HALF,
        bcm5892_duplex_full = DUPLEX_FULL,
};

enum bcm5892_fc
{
        bcm5892_fc_none,
        bcm5892_fc_disabled,
        bcm5892_fc_frame,
        bcm5892_fc_collision,
        bcm5892_fc_carrier,
};

enum bcm5892_state
{
        bcm5892_state_uninit,
        bcm5892_state_off,
        bcm5892_state_on,
        bcm5892_state_broken,
};

#define BCM5892_ETHERNET_MAC_BASE	0x00086000
#define BCM5892_MAC_HASH_COUNT		0x40
#define BCM5892_CHIPID_ADDR		0x1025004 // Used to Identify BOARD revision A0/B0
#define BCM5892_A0_ID			0x58920100 //A0 Board ID 	
#define BCM5892_B0_ID			0x58920200 //B0 Board ID 	


/**********************************************************************
 *  Register Offsets _Begin
 **********************************************************************/
#define OFFSET_ETH_CTRL		0x0000
#define OFFSET_INTR_MASK	0x0004
#define OFFSET_INTR		0x0008
#define OFFSET_INTR_RAW		0x000C
#define OFFSET_INTR_CLR		0x0010
#define OFFSET_MCADDRF0		0x0020
#define OFFSET_MCADDRF1		0x0024
#define OFFSET_PHYCTRL		0x0040
#define OFFSET_RBUFFSZ		0x0104
#define OFFSET_RBASE		0x0110
#define OFFSET_RBCFG		0x0114
#define OFFSET_RBDPTR		0x0118
#define OFFSET_RSWPTR		0x011C
#define OFFSET_TBASE		0x0210
#define OFFSET_TBCFG		0x0214
#define OFFSET_TBDPTR		0x0218
#define OFFSET_TSWPTR		0x021C
#define OFFSET_MACBP		0x0404
#define OFFSET_MACCFG		0x0408
#define OFFSET_MACADDR0		0x040C
#define OFFSET_MACADDR1		0x0410
#define OFFSET_MAXFRM		0x0414
#define OFFSET_MACPQ		0x0418
#define OFFSET_MACRXFE		0x041C
#define OFFSET_MACRXFF		0x0420
#define OFFSET_MACTXFE		0x0424
#define OFFSET_MACTXFF		0x0428
#define OFFSET_MACMODE		0x0440
#define OFFSET_TXIPG		0x045C
#define OFFSET_TXPCTRL		0x0730
#define OFFSET_TXFIFOF		0x0734
#define OFFSET_RXFIFOSTAT	0x0738
#define OFFSET_TXFIFOSTAT	0x073C
#define OFFSET_TXOCTGOOD	0x0800
#define OFFSET_TXFRMGOOD	0x0804
#define OFFSET_TXOCTTOTAL	0x0808
#define OFFSET_TXFRMTOTAL	0x080C
#define OFFSET_TXBCASTGOOD	0x0810
#define OFFSET_TXMCASTGOOD	0x0814
#define OFFSET_TX64		0x0818
#define OFFSET_TX65_127		0x081C
#define OFFSET_TX128_255	0x0820
#define OFFSET_TX256_511	0x0824
#define OFFSET_TX512_1023	0x0828
#define OFFSET_TX1024_MAX	0x082C
#define OFFSET_TX65_127		0x081C
#define OFFSET_TXJABBER		0x0830
#define OFFSET_TXJUMBO		0x0834
#define OFFSET_TXFRAG		0x0838
#define OFFSET_TXUNDERRUN	0x083C
#define OFFSET_TXCOLTOTAL	0x0840
#define OFFSET_TX1COL		0x0844
#define OFFSET_TXMCOL		0x0848
#define OFFSET_TXEXCOL		0x084C
#define OFFSET_TXLATE		0x0850
#define OFFSET_TXDEFER		0x0854
#define OFFSET_TXNOCRS		0x0858
#define OFFSET_TXPAUSE		0x085C
#define OFFSET_RXOCTGOOD	0x0880
#define OFFSET_RXFRMGOOD	0x0884
#define OFFSET_RXOCTTOTAL	0x0888
#define OFFSET_RXFRMTOTAL	0x088C
#define OFFSET_RXBCASTGOOD	0x0890
#define OFFSET_RXMCASTGOOD	0x0894
#define OFFSET_RX64		0x0898
#define OFFSET_RX65_127		0x089C
#define OFFSET_RX128_255	0x08A0
#define OFFSET_RX256_511	0x08A4
#define OFFSET_RX512_1023	0x08A8
#define OFFSET_RX1024_MAX	0x08AC
#define OFFSET_RXJABBER		0x08B0
#define OFFSET_RXJUMBO		0x08B4
#define OFFSET_RXFRAG		0x08B8
#define OFFSET_RXOVERRUN	0x08BC
#define OFFSET_RXCRCALIGN	0x08C0
#define OFFSET_RXUSIZE		0x08C4
#define OFFSET_RXCRC		0x08C8
#define OFFSET_RXALIGN		0x08CC
#define OFFSET_RXCDERR		0x08D0
#define OFFSET_RXPAUSEFM	0x08D4
#define OFFSET_RXCTRLFM		0x08D8

#define OFFSET_MIIMGT		0x0000
#define OFFSET_MIICMD		0x0004

/**********************************************************************
 *  Register Offsets _End
 **********************************************************************/

/**********************************************************************
 *  Ethernet Sub-System Control (ETH_CTRL) _Begin
 **********************************************************************/

#define BCM5892_ETH_CTRL	 			(0x0000)
#define BCM5892_INTR_MASK	 			(0x0004)
#define BCM5892_INTR_RAW	 			(0x0008)
#define BCM5892_INTR	 				(0x000C)
#define BCM5892_INTR_CLR	 			(0x0010)
#define BCM5892_MC_ADDRF_0	 			(0x0020)
#define BCM5892_MC_ADDRF_1	 			(0x0024)
#define BCM5892_PHY_CTRL	 			(0x0040)

/* Ethernet Control (ETH_CTRL) flags _Begin */

#define BCM5892_ETH_CTRL_LED1_RCV	(0x0000)
#define BCM5892_ETH_CTRL_LED1_XMIT	(0x0100)
#define BCM5892_ETH_CTRL_LED1_LNK	(0x0200)
#define BCM5892_ETH_CTRL_LED1_DUPLEX	(0x0300)
#define BCM5892_ETH_CTRL_LED1_SPEED	(0x0400)
#define BCM5892_ETH_CTRL_LED1_ACT	(0x0500)
#define BCM5892_ETH_CTRL_LED1_COLSN	(0x0600)
#define BCM5892_ETH_CTRL_LED1_LEDERR	(0x0700)

#define BCM5892_ETH_CTRL_LED0_RCV       (0x0000)
#define BCM5892_ETH_CTRL_LED0_XMIT      (0x0010)
#define BCM5892_ETH_CTRL_LED0_LNK       (0x0020)
#define BCM5892_ETH_CTRL_LED0_DUPLEX    (0x0030)
#define BCM5892_ETH_CTRL_LED0_SPEED     (0x0040)
#define BCM5892_ETH_CTRL_LED0_ACT       (0x0050)
#define BCM5892_ETH_CTRL_LED0_COLSN     (0x0060)
#define BCM5892_ETH_CTRL_LED0_LEDERR    (0x0070)

#define BCM5892_ETH_CTRL_MEN			(0x0008)
#define BCM5892_ETH_CTRL_COR			(0x0004)
#define BCM5892_ETH_CTRL_GRS			(0x0002)
#define BCM5892_ETH_CTRL_GTS			(0x0001)

#define BCM5892_ETH_CTRL_BCR			(0x1000)
#define BCM5892_ETH_CTRL_FBP			(0x2000)
#define BCM5892_ETH_CTRL_BE			(0x4000)

/* Ethernet Control (ETH_CTRL) flags _End */

/* Interrupt Mask (INTR_MASK) flags _Begin */

#define BCM5892_INTR_MASK_PHY		(0x010000)
#define BCM5892_INTR_MASK_RMRK		(0x008000)
#define BCM5892_INTR_MASK_TMRK		(0x004000)
#define BCM5892_INTR_MASK_RHLT		(0x002000)
#define BCM5892_INTR_MASK_THLT		(0x001000)
#define BCM5892_INTR_MASK_ROV		(0x000800)
#define BCM5892_INTR_MASK_TUN		(0x000400)
#define BCM5892_INTR_MASK_TEC		(0x000200)
#define BCM5892_INTR_MASK_TLC		(0x000100)
#define BCM5892_INTR_MASK_RXB		(0x000080)
#define BCM5892_INTR_MASK_TXB		(0x000040)
#define BCM5892_INTR_MASK_RXF		(0x000020)
#define BCM5892_INTR_MASK_TXF		(0x000010)
#define BCM5892_INTR_MASK_BERR		(0x000008)
#define BCM5892_INTR_MASK_MIIM		(0x000004)
#define BCM5892_INTR_MASK_GRSC		(0x000002)
#define BCM5892_INTR_MASK_GTSC		(0x000001)
#define BCM5892_INTR_UNMASK_ALL		(BCM5892_INTR_MASK_GTSC|\
					 BCM5892_INTR_MASK_GRSC|\
					 BCM5892_INTR_MASK_BERR|\
					 BCM5892_INTR_MASK_TXF|\
					 BCM5892_INTR_MASK_RXF|\
					 BCM5892_INTR_MASK_TXB|\
					 BCM5892_INTR_MASK_RXB|\
					 BCM5892_INTR_MASK_TLC|\
					 BCM5892_INTR_MASK_TEC|\
					 BCM5892_INTR_MASK_TUN|\
					 BCM5892_INTR_MASK_ROV|\
					 BCM5892_INTR_MASK_THLT|\
					 BCM5892_INTR_MASK_RHLT|\
					 BCM5892_INTR_MASK_TMRK|\
					 BCM5892_INTR_MASK_RMRK|\
					 BCM5892_INTR_MASK_PHY)

#define BCM5892_INTR_POLL_USECS		5
#define BCM5892_INTR_POLL_TIMEOUT	250

#define BCM5892_INTR_MASK_ALL		 0
						

/* Interrupt Mask (INTR_MASK) flags _End */

/* Masked Interrupt  (INTR)  flags _Begin */

#define BCM5892_INTR_PHY                (0x010000)
#define BCM5892_INTR_RMRK               (0x008000)
#define BCM5892_INTR_TMRK               (0x004000)
#define BCM5892_INTR_RHLT               (0x002000)
#define BCM5892_INTR_THLT               (0x001000)
#define BCM5892_INTR_ROV                (0x000800)
#define BCM5892_INTR_TUN                (0x000400)
#define BCM5892_INTR_TEC                (0x000200)
#define BCM5892_INTR_TLC                (0x000100)
#define BCM5892_INTR_RXB                (0x000080)
#define BCM5892_INTR_TXB                (0x000040)
#define BCM5892_INTR_RXF                (0x000020)
#define BCM5892_INTR_TXF                (0x000010)
#define BCM5892_INTR_BERR               (0x000008)
#define BCM5892_INTR_MIIM               (0x000004)
#define BCM5892_INTR_GRSC               (0x000002)
#define BCM5892_INTR_GTSC               (0x000001)

/* Masked Interrupt  (INTR)  flags _End */

/* Raw Interrupt  (INTR_RAW)  flags _Begin */

#define BCM5892_INTR_RAW_PHY                (0x010000)
#define BCM5892_INTR_RAW_RMRK               (0x008000)
#define BCM5892_INTR_RAW_TMRK               (0x004000)
#define BCM5892_INTR_RAW_RHLT               (0x002000)
#define BCM5892_INTR_RAW_THLT               (0x001000)
#define BCM5892_INTR_RAW_ROV                (0x000800)
#define BCM5892_INTR_RAW_TUN                (0x000400)
#define BCM5892_INTR_RAW_TEC                (0x000200)
#define BCM5892_INTR_RAW_TLC                (0x000100)
#define BCM5892_INTR_RAW_RXB                (0x000080)
#define BCM5892_INTR_RAW_TXB                (0x000040)
#define BCM5892_INTR_RAW_RXF                (0x000020)
#define BCM5892_INTR_RAW_TXF                (0x000010)
#define BCM5892_INTR_RAW_BERR               (0x000008)
#define BCM5892_INTR_RAW_MIIM               (0x000004)
#define BCM5892_INTR_RAW_GRSC               (0x000002)
#define BCM5892_INTR_RAW_GTSC               (0x000001)

/* Raw Interrupt (INTR_RAW)  flags _End */

/* Clear Interrupt (INTR_CLR)  flags _Begin */

#define BCM5892_INTR_CLR_PHY                (0x010000)
#define BCM5892_INTR_CLR_RMRK               (0x008000)
#define BCM5892_INTR_CLR_TMRK               (0x004000)
#define BCM5892_INTR_CLR_RHLT               (0x002000)
#define BCM5892_INTR_CLR_THLT               (0x001000)
#define BCM5892_INTR_CLR_ROV                (0x000800)
#define BCM5892_INTR_CLR_TUN                (0x000400)
#define BCM5892_INTR_CLR_TEC                (0x000200)
#define BCM5892_INTR_CLR_TLC                (0x000100)
#define BCM5892_INTR_CLR_RXB                (0x000080)
#define BCM5892_INTR_CLR_TXB                (0x000040)
#define BCM5892_INTR_CLR_RXF                (0x000020)
#define BCM5892_INTR_CLR_TXF                (0x000010)
#define BCM5892_INTR_CLR_BERR               (0x000008)
#define BCM5892_INTR_CLR_MIIM               (0x000004)
#define BCM5892_INTR_CLR_GRSC               (0x000002)
#define BCM5892_INTR_CLR_GTSC               (0x000001)

/* Clear Interrupt (INTR_CLR)  flags _End */

/**********************************************************************
 *  Ethernet Sub-System Control (ETH_CTRL) _End
 **********************************************************************/

/**********************************************************************
 *  Multicast Address Filter. (MCADDRFn) _Begin
 **********************************************************************/

#define BCM5892_ETH_MCADDRF0   			(0x0020)
#define BCM5892_ETH_MCADDRF1   			(0x0024)

/**********************************************************************
 *  Multicast Address Filter. (MCADDRFn) _End
 **********************************************************************/

/**********************************************************************
 *  OnChip PHY Control Flags (PHYCTRL) _Begin
 **********************************************************************/
#define BCM5892_PHY_CTRL_EXT_PHY	    (0x80000000)
#define BCM5892_PHY_CTRL_PSEL               (0x00000020)
#define BCM5892_PHY_CTRL_IDQ                (0x00000010)
#define BCM5892_PHY_CTRL_DET                (0x00000008)
#define BCM5892_PHY_CTRL_PDB                (0x00000004)
#define BCM5892_PHY_CTRL_PDD                (0x00000002)
#define BCM5892_PHY_CTRL_PDP                (0x00000001)
#define BCM5892_PHY_CTRL_RESET_VAL	    0

/**********************************************************************
 *  OnChip PHY Control Flags (PHYCTRL) _End
 **********************************************************************/

/**********************************************************************
 *  DMA Infrastructure Info _Begin
 **********************************************************************/

/* Ethernet Receive DMA */

#define BCM5892_RBUFFSZ	 	(0x0104)
#define BCM5892_RBASE	 	(0x0110)
#define BCM5892_RBCFG	 	(0x0114)
#define BCM5892_RBDPTR	 	(0x0118)
#define BCM5892_RSWPTR	 	(0x011c)

/* Ethernet Transmit DMA */

#define BCM5892_TBASE           (0x0210)
#define BCM5892_TBCFG           (0x0214)
#define BCM5892_TBDPTR          (0x0218)
#define BCM5892_TSWPTR          (0x021c)

/* TX DMA Buffer Descriptor flags _Begin */

#define BCM5892_TX_BD_SOP	            (0x80000000)
#define BCM5892_TX_BD_EOP	            (0x40000000)
#define BCM5892_TX_BD_PAD	            (0x00800000)
#define BCM5892_TX_BD_CAP	            (0x00400000)
#define BCM5892_TX_BD_CRP	            (0x00200000)
#define BCM5892_TX_BD_XDEF	            (0x00080000)
#define BCM5892_TX_BD_XCOL	            (0x00040000)
#define BCM5892_TX_BD_LCOL	            (0x00020000)
#define BCM5892_TX_BD_UN	            (0x00010000)

/* TX DMA Buffer Descriptor flags _End */

/* RX DMA Buffer Descriptor flags _Begin */

#define BCM5892_RX_BD_SOP	            (0x80000000)
#define BCM5892_RX_BD_EOP	            (0x40000000)
#define BCM5892_RX_BD_BC	            (0x00800000)
#define BCM5892_RX_BD_MC	            (0x00400000)
#define BCM5892_RX_BD_NO	            (0x00200000)
#define BCM5892_RX_BD_TC	            (0x00080000)
#define BCM5892_RX_BD_ER	            (0x00080000)
#define BCM5892_RX_BD_OF	            (0x00020000)

/* RX DMA Buffer Descriptor flags _End */

/**********************************************************************
 *  DMA Infrastructure Info _End
 **********************************************************************/

/**********************************************************************
 *  Ethernet MAC Configuration Registers _Begin
 **********************************************************************/

#define BCM5892_MAC_BP					(0x0404)
#define BCM5892_MAC_CFG					(0x0408)
#define BCM5892_MAC_ADDR0				(0x040C)
#define BCM5892_MAC_ADDR1				(0x0410)
#define BCM5892_MAC_MAX_FRM				(0x0414)
#define BCM5892_MAC_PQ					(0x0418)
#define BCM5892_MAC_RXFE				(0x041C)
#define BCM5892_MAC_RXFF				(0x0420)
#define BCM5892_MAC_TXFE				(0x0424)
#define BCM5892_MAC_TXFF				(0x0428)
#define BCM5892_MAC_MODE_SR				(0x0444)
#define BCM5892_MIN_TX_IPG				(0x045C)

#define BCM5892_MAC_MAX_FRM_DEFAULT_VAL			(0x2400)
#define BCM5892_MAC_PQ_RESET_CASE_VAL			0xFFFF
#define BCM5892_MAC_RXFF_RECOMMENDED_VAL		4
#define BCM5892_MAC_TXFE_RECOMMENDED_VAL		6
#define BCM5892_MAC_TXFF_RECOMMENDED_VAL		4
#define BCM5892_MIN_TX_IPG_BEST_CASE			26

/* Ethernet MAC Backoff Control flags _Begin */

#define BCM5892_MAC_BP_IPG_CFG               (0x0028)
#define BCM5892_MAC_BP_BKEN                  (0x0002)
#define BCM5892_MAC_BP_FCEN                  (0x0001)

/* Ethernet MAC Backoff Control flags _End */

/* Ethernet MAC Configuration flags _Begin */

#define BCM5892_MAC_CFG_TXEN                 (0x00000001)
#define BCM5892_MAC_CFG_RXEN                 (0x00000002)
#define BCM5892_MAC_CFG_SPD_100              (0x00000004)
#define BCM5892_MAC_CFG_SPD_10               (0x00000000)
#define BCM5892_MAC_CFG_PROM                 (0x00000010)
#define BCM5892_MAC_CFG_RXPAD                (0x00000020)
#define BCM5892_MAC_CFG_CFWD                 (0x00000040)
#define BCM5892_MAC_CFG_PFWD                 (0x00000080)
#define BCM5892_MAC_CFG_PDIS                 (0x00000100)
#define BCM5892_MAC_CFG_TXAD                 (0x00000200)
#define BCM5892_MAC_CFG_HDEN                 (0x00000400)
#define BCM5892_MAC_CFG_RxOFLO               (0x00001000) /* To Accomodate Rx Overflow in B0 and Production tapeout*/
#define BCM5892_MAC_CFG_SRST                 (0x00002000)
#define BCM5892_MAC_CFG_LLB                  (0x00008000) 
#define BCM5892_MAC_CFG_ACFG                 (0x00400000)
#define BCM5892_MAC_CFG_CFA                  (0x00800000) 
#define BCM5892_MAC_CFG_NOLC                 (0x01000000)
#define BCM5892_MAC_CFG_RLB                  (0x02000000)
#define BCM5892_MAC_CFG_TPD                  (0x10000000)
#define BCM5892_MAC_CFG_TXRX                 (0x20000000)
#define BCM5892_MAC_CFG_RFLT                 (0x40000000)

/* Ethernet MAC Configuration flags _End */


/* Ethernet MAC Misc flags _Begin */

#define BCM5892_MAC_BP_BKEN                  (0x0002)
#define BCM5892_MAC_BP_FCEN                  (0x0001)

#define BCM5892_MAC_MODE_SR_LNK             (0x0020)
#define BCM5892_MAC_MODE_SR_TXP             (0x0010)
#define BCM5892_MAC_MODE_SR_RXP             (0x0008)
#define BCM5892_MAC_MODE_SR_DP              (0x0004)
#define BCM5892_MAC_MODE_SR_SPD_10          (0x0000)
#define BCM5892_MAC_MODE_SR_SPD_100         (0x0001)

#define BCM5892_TX_PAUSE_CTRL_EN           (0x040000)

#define BCM5892_RX_FIFO_STAT_OR            (0x0002)
#define BCM5892_RX_FIFO_STAT_UR            (0x0001)

#define BCM5892_TX_FIFO_STAT_OR            (0x0002)
#define BCM5892_TX_FIFO_STAT_UR            (0x0001)

/* Ethernet MAC Misc flags _End */

/* MMI FLAGS _Begin*/
/*Refer to Pg. 464 of 5892-PR100-eth_subsystem-2.pdf */
#define BCM5892_MIIMGT_MDCDIV_1_65MHZ		0x14
#define BCM5892_MIIMGT_MDCDIV_UNMASK_ALL	0x3F
#define BCM5892_PHY_SB			0x01<<30
#define BCM5892_PHY_DIR(x)			(x)<<28
#define BCM5892_PHY_ADDR(x)			(x)<<23
#define BCM5892_PHY_REG_ADDR(x)		(x)<<18
#define BCM5892_PHY_TA			0x02<<16
#define BCM5892_PHY_GIVEN			2 /*Hard Coded, for internal PHY, confirmed with MATT*/
#define BCM5892_PHY_BUSY_POLL_COUNT		100
#define BCM5892_PHY_POLL_USEC			20
#define BCM5892_PHY_BYP			(0x01<<10)
#define BCM5892_PHY_EXT			(0x01<<9)
#define BCM5892_PHY_BSY			(0x01<<8)
#define BCM5892_PHY_GEN_PRE		(0x01<<7)
#define BCM5892_MDIO_READ		0x02
#define BCM5892_MDIO_WRITE		0x01
/* MMI FLAGS _End*/

/**********************************************************************
 *  Ethernet MAC Configuration Registers _End
 **********************************************************************/

/**********************************************************************
 *  Packet Filtering Constants _Begin
 **********************************************************************/

#define EMAC_RXFILTER_NOTHING      0
#define EMAC_RXFILTER_DIRECT       1
#define EMAC_RXFILTER_BROADCAST    2
#define EMAC_RXFILTER_MULTICAST    3
#define EMAC_RXFILTER_ALL          4

/**********************************************************************
 *  Packet Filtering Constants _End
 **********************************************************************/

/**********************************************************************
 *  Utility Macros & Inline Functions _Begin
 **********************************************************************/

#define BCM5892DMA_NEXTBUF(d,f) ((((d)->f+1) == (d)->bcm5892dma_dscrtable_end) ? \
                          (d)->bcm5892dma_dscrtable : (d)->f+1)


#define NUMCACHEBLKS(x) (((x)+SMP_CACHE_BYTES-1)/SMP_CACHE_BYTES)

#define BCM5892_MAX_TXDESCR       0x100
#define BCM5892_MAX_RXDESCR       0x100
#define BCM5892_RBUFFSZ_SHIFT	  0x06

#define BCM5892_RBUFF_SET_BIT31   31  

#define ETHER_ALIGN     2
#define ETHER_ADDR_LEN  6
#define ENET_PACKET_SIZE        1518

#define DMA_RX	0
#define DMA_TX	1

#define ALIGN64 0x40

#define BCM5892_RFBWMRK 0
#define BCM5892_TFBWMRK 0

static inline void* get_aligned(void *x, int power2)
{
	u32 retVal = 0;

	retVal = ((u32)x + power2) & (~(power2 - 1));
	return (void*)retVal;
}

/**********************************************************************
 *  Utility Macros & Inline Functions _End
 **********************************************************************/

/**********************************************************************
 *  Structure definitions _Begin
 **********************************************************************/

/*  DMA Descriptor Structure _Begin */
struct bcm5892dmadscr
{
	uint32_t  flags;
	uint32_t  pData;
};
/*  DMA Descriptor Structure _End */

struct eth_mib_stats; /* Ethernet MIB Statistics Structure, declared later in this file */

/*  DMA Controller structure _Begin */
struct bcm5892macdma
{
	/*
	 * This stuff is used to identify the channel and the registers
	 * associated with it.
	 */
	struct bcm5892mac_softc	*bcm5892dma_eth;     /* back pointer to associated
							MAC */

	/*int			bcm5892dma_channel;*//* channel number */
	int			bcm5892dma_txdir;    /* direction (1=transmit) */
	int			bcm5892dma_maxdescr; /* total # of descriptors
									in ring */
	volatile void __iomem	*bcm5892dma_buffsz;  /* Rx Buffer Size = n*64*/
	volatile void __iomem	*bcm5892dma_dscrbase;
					   	     /* descriptor base address */
	volatile void __iomem	*bcm5892dma_bcfg;    /* Buff Desc Ring Config*/
	volatile void __iomem	*bcm5892dma_bdptr;   /* current desc address */
	volatile void __iomem	*bcm5892dma_swptr;   /* Recently released desc */

	/*
	 * This stuff is for maintenance of the ring
	 */

	void			*bcm5892dma_dscrtable_unaligned;
	struct bcm5892dmadscr	*bcm5892dma_dscrtable;
						     /* base of descriptor table */
	struct bcm5892dmadscr	*bcm5892dma_dscrtable_end;
						     /* end of descriptor table */
	struct sk_buff		**bcm5892dma_ctxtable;
						     /* context table, one
							per descr */
 	dma_addr_t              bcm5892dma_dscrtable_phys;
						     /* and also the phys addr */
	struct bcm5892dmadscr	*bcm5892dma_addptr;  /* next dscr for sw to add */	
	struct bcm5892dmadscr	*bcm5892dma_remptr;  /* next dscr for sw
							to remove */
 };
/*  DMA Controller structure _End */

/*  Driver Context Structure _Begin */
struct bcm5892mac_softc
{	
	/*
	 * Linux-specific things
	 */

	struct	net_device	*bcm5892_dev;		/* pointer to linux device */
	struct	napi_struct	napi;
	struct	phy_device	*bcm5892phy_dev;	/* the associated PHY device */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#ifdef USE_MII_BUS_POINTER
	struct	mii_bus		*mii_bus;        	/* the MII bus */
#else
	struct	mii_bus		mii_bus;        	/* the MII bus */
#endif
#else
	struct	mii_bus		mii_bus;        	/* the MII bus */
#endif
	struct  mii_if_info	mii_info;		/* the MII Info */
	int     phy_irq[PHY_MAX_ADDR];			/* We expect at least 2 PHYS*/
							/* Internal and External */

	spinlock_t	bcm5892_lock;			/* spin lock */
        int	bcm5892_devflags;			/* Current device flags
							   (netdevice->flags)*/
	int	bcm5892_buffersize;			/* The skbuff size */ 
 
	enum bcm5892_state	current_state;		/* current state */

	int	runt;					/* Runt packet flag */
	int	rx_overflow;				/* RX overflow flag */

	#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
	struct net_device_stats bcm5892_stats;
	#endif


	/*
         * Controller-specific things
         */

	void __iomem	*bcm5892_macbase;	/* MAC's base address */

	volatile void __iomem	*bcm5892_ethctrl;	/* Ethernet Subsystem Control
							   (ETH_CTRL) */
	volatile void __iomem	*bcm5892_intrmask;	/* Interrupt Mask
							   (INTR_MASK) */
	volatile void __iomem	*bcm5892_maskedintr;	/* The Masked Interrupt Register 
							   (INTR) */
	volatile void __iomem	*bcm5892_intr_raw;	/* Raw Interrupt Register
							   (INTR_RAW) */
	volatile void __iomem	*bcm5892_intr_clr;	/* Interrupt Clear Register 
							   (INTR_CLR) */

	volatile void __iomem	*bcm5892_mcastaddrf0;	/* Multicast Address Filter Register 0
							   (MCADDRF0) */
	volatile void __iomem	*bcm5892_mcastaddrf1;	/* Multicast Address Filter Register 1
							   (MCADDRF0) */

	volatile void __iomem	*bcm5892_phyctrl;	/* On Chip PHY Control (PHYCTRL) */

	volatile void __iomem	*bcm5892_rbufsize;	/* DMA Rx Buffer Size (RBUFFSZ) */
	volatile void __iomem	*bcm5892_rbase;		/* DMA Rx Buffer Descriptor Base Address 
							   (RBASE) */
	volatile void __iomem	*bcm5892_rcfg;		/* DMA Rx Buffer Ring Config
							   (RBCFG) */
	volatile void __iomem	*bcm5892_rbdptr;	/* Current DMA Rx Buffer Desc Ptr 
							   (RBDPTR) */
	volatile void __iomem	*bcm5892_rswptr;	/* Recieve Software Pointer
							   (RSWPTR) */

	volatile void __iomem	*bcm5892_tbase;		/* DMA Tx Buffer Descriptor Base Address 
							   (TBASE) */
	volatile void __iomem	*bcm5892_tcfg;		/* DMA Tx Buffer Ring Config
							   (TBCFG) */
	volatile void __iomem	*bcm5892_tbdptr;	/* Current DMA Tx Buffer Desc Ptr 
							   (TBDPTR) */
	volatile void __iomem	*bcm5892_tswptr;	/* Recieve Software Pointer
							   (TSWPTR) */

	volatile void __iomem	*bcm5892_mac_bp;	/* Ethernet MAC Half-Duplex Backoff
							   Control (MACBP) */
	volatile void __iomem	*bcm5892_mac_cfg;	/* Ethernet MAC Configuration 
							   (MACCFG) */
	volatile void __iomem	*bcm5892_macaddr0;	/* Eth Mac Addr Lo (MACADDR0) [31:0] */
	volatile void __iomem	*bcm5892_macaddr1;	/* Eth Mac Addr Hi (MACADDR1) [47:32] */
	volatile void __iomem	*bcm5892_maxfrm;	/* Eth Max Frame Length (MAXFRM)*/
	volatile void __iomem	*bcm5892_pq;		/* Pause Quanta (MACPQ) */
	volatile void __iomem	*bcm5892_rxfe;		/* Eth Rx FIFO Empty Watermark (MACRXFE) */
	volatile void __iomem	*bcm5892_rxff;		/* Eth Rx FIFO Frame Valid (MACRXFF) */
	volatile void __iomem	*bcm5892_txfe;		/* Eth Tx FIFO Empty Watermark (MACTXFE)*/
	volatile void __iomem	*bcm5892_txff;		/* Eth Tx FIFO Frame Valid (MACTXFF) */
	volatile void __iomem	*bcm5892_macmode;	/* Eth Mac Status Register (MACMODE) */
	volatile void __iomem	*bcm5892_txipg;		/* Min Txmit IPG Length (TXIPG) */


	volatile void __iomem	*bcm5892_txpctrl;	/* Transmit Pause Control (TXPCTRL) */
	volatile void __iomem	*bcm5892_txfifof;	/* Transmit FIFO Flush (TXFIFOF) */
	volatile void __iomem	*bcm5892_rxfifostat;	/* Recieve FIFO Status (RXFIFOSTAT) */
	volatile void __iomem	*bcm5892_txfifostat;	/* Transmit FIFO Status (TXFIFOSTAT) */

	void __iomem	*bcm5892_mmibase;	/* MDIO Base Register (MII/MMI Base) */
	volatile void __iomem	*bcm5892_miimgt;	/* MII Management Control (MIIMGT Register) */
	volatile void __iomem	*bcm5892_miicmd;	/* MII Command Register (MIICMD) */
#ifdef BCM5892_DEBUG
	volatile void __iomem	*bcm5892_gio2base;	/* GPIO Base register */
	volatile void __iomem	*bcm5892_expmntal;	/* Experimental */
#endif

	struct eth_mib_stats	*bcm5892_mib_stats;	/* Ethernet MIB Statistics */
	enum bcm5892_speed	current_speed;		/* current speed */
	enum bcm5892_duplex	current_duplex;		/* current duplex */
	enum bcm5892_fc		current_fc;         	/* cur. flow control setting */
	int			current_pause;		/* current pause setting */
	int			current_link;		/* current link state */

	unsigned char           bcm5892_hwaddr[ETHER_ADDR_LEN];
							/* The MAC Address */

 	struct bcm5892macdma	bcm5892_rxdma;		/* RX_DMA */
 	struct bcm5892macdma	bcm5892_txdma;		/* TX_DMA */
	int			rx_hw_checksum;		/* Checksum Method ? */
	int                     sbe_idx;		/* The Index of MAC, just in case ... */
};

/*  Driver Context Structure _End */

/*  Ethernet Mib Statistics structure _Begin */
struct eth_mib_stats
{
	volatile void __iomem	*txoctgood;		/* Total Transmitted Octets in Good 
							   Packets (TXOCTGOOD) */
	volatile void __iomem	*txfrmgood;		/* Total of Successfully Transmitted 
							   Frames (TXFRMGOOD) */
	volatile void __iomem	*txfrmtotal;		/* Total of Transmitted Frames 
							   (TXFRMTOTAL) */
	volatile void __iomem	*txbcastgood;		/* Total of Good Broadcast Frames
							   Transmitted (BCASTGOOD) */
	volatile void __iomem	*txmcastgood;		/* Total of Good Multicast Frames
							   Transmitted (MCASTGOOD) */
	volatile void __iomem	*tx64;			/* 64B Length Frames Transmitted
							   (TX64) */
	volatile void __iomem	*tx65_127;		/* 65B-127B Length Frames Transmitted
							   (TX65_127) */
	volatile void __iomem	*tx128_255;		/* 127B-255B Length Frames Transmitted
							   (TX128_255) */
	volatile void __iomem	*tx256_511;		/* 256B-511B Length Frames Transmitted
							   (TX256_511) */
	volatile void __iomem	*tx512_1023;		/* 512B-1023B Length Frames Transmitted
							   (TX512_1023) */
	volatile void __iomem	*tx1024_max;		/* 1024B-MAXFRM Length Frames Transmitted
							   (TX1024_MAX) */
	volatile void __iomem	*txjabber;		/* Frames > MAXFRM with Bad CRC Transmitted
							   (TXJABBER) */
	volatile void __iomem	*txjumbo;		/* Frames > MAXFRM with Good CRC Transmitted
							   (TXJUMBO) */
	volatile void __iomem	*txfrag;		/* Frames < 64B with Bad CRC Transmitted
							   (TXFRAG) */
	volatile void __iomem	*txunderrun;		/* Frames truncated due to FIFO underrun
							   (TXUNDERRUN) */
	volatile void __iomem	*txcoltotal;		/* Collisions seen by Transmitter 
							   (TXCOLTOTAL) */
	volatile void __iomem	*tx1col;		/* Frames Transmitted with only one 
							   collision (TX1COL) */
	volatile void __iomem	*txmcol;		/* Frames Transmitted with > 1  
							   collision (TXMCOL) */
	volatile void __iomem	*txexcol;		/* Frames Aborted after Excessive [16]
							   collision (TXEXCOL) */
	volatile void __iomem	*txlate;		/* Frames that Experienced Late
							   collision (TXLATE) */
	volatile void __iomem	*txdefer;		/* Frames Transmission Delayed due to
							   Busy Network (TXLATE) */
	volatile void __iomem	*txcrs;			/* Loss of CRS while transmitting
							   (TXNOCRS) */
	volatile void __iomem	*txpause;		/* Total Pause Frames Transmitted
							   (TXPAUSE) */

	volatile void __iomem	*rxoctgood;		/* Total Received Octets in Good 
							   Packets (RXOCTGOOD) */
	volatile void __iomem	*rxfrmgood;		/* Total of Successfully Received 
							   Frames (RXFRMGOOD) */
	volatile void __iomem	*rxfrmtotal;		/* Total of Received Frames 
							   (RXFRMTOTAL) */
	volatile void __iomem	*rxbcastgood;		/* Total of Good Broadcast Frames
							   Received (BCASTGOOD) */
	volatile void __iomem	*rxmcastgood;		/* Total of Good Multicast Frames
							   Received (MCASTGOOD) */
	volatile void __iomem	*rx64;			/* 64B Length Frames Received
							   (RX64) */
	volatile void __iomem	*rx65_127;		/* 65B-127B Length Frames Received
							   (RX65_127) */
	volatile void __iomem	*rx128_255;		/* 127B-255B Length Frames Received
							   (RX128_255) */
	volatile void __iomem	*rx256_511;		/* 256B-511B Length Frames Received
							   (RX256_511) */
	volatile void __iomem	*rx512_1023;		/* 512B-1023B Length Frames Received
							   (RX512_1023) */
	volatile void __iomem	*rx1024_max;		/* 1024B-MAXFRM Length Frames Received
							   (RX1024_MAX) */
	volatile void __iomem	*rxjabber;		/* Frames > MAXFRM with Bad CRC Received
							   (RXJABBER) */
	volatile void __iomem	*rxjumbo;		/* Frames > MAXFRM with Good CRC Received
							   (RXJUMBO) */
	volatile void __iomem	*rxfrag;		/* Frames < 64B with Bad CRC Received
							   (RXFRAG) */
	volatile void __iomem	*rxoverrun;		/* Frames truncated due to FIFO overrun
							   (RXOVERRUN) */
	volatile void __iomem	*rxcrcalign;		/* Received Frames with Bad CRC or 
							   Alignment Error (RXCRCALIGN) */
	volatile void __iomem	*rxusize;		/* Frames < 64B with Good CRC Received
							   (RXUSIZE) */
	volatile void __iomem	*rxcrc;			/* Received Frames with Bad CRC but 
							   Even Alignment (RXCRC) */
	volatile void __iomem	*rxalign;		/* Received Frames with Bad CRC and 
							   Odd Alignment (RXALIGN) */
	volatile void __iomem	*rxcderr;		/* Received Frames with Code Error
							   Events (RXCDERR) */
	volatile void __iomem	*rxpausefm;		/* Pause Frames Received (RXPAUSEFM) */
	volatile void __iomem	*rxctrlfm;		/* Control Frames (Non-Pause) Received 
							   (RXCTRLFM) */
};
/*  Ethernet Mib Statistics structure _End */

/**********************************************************************
 *  Structure definitions _End
 **********************************************************************/



#endif /*#ifndef __BCM5892_MAC_DOT_H_INCLUDED__*/
