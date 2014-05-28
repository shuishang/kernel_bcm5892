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


/*
 *  Broadcom BCM5892 Project
 *  File: dma_cpuapi.h
 *  Description: the DMA header file
 *
 *
 *****************************************************************************/



#ifndef __DMAC_H
#define __DMAC_H

#include "bcm5892_sw.h"
#include "bcm5892_reg.h"
#include "regaccess.h"

#define ddr_vmem_addr(addr) (addr & 0x00ffffff)

#ifndef LLI_ORIGINAL
#define LLI_ORIGINAL 1
#endif

#ifdef ARMCC

#ifdef INSIDEARM
#include "arm_console.h"
#endif

#ifndef INSIDEARM
#include "mboxutils.h"
#include "mbox_ids.h"

#endif

#endif

#ifdef ARMCC
#ifndef INSIDEARM
#ifdef FC_SCENARIO_TEST
volatile uint8_t dma0_chan0_sema_key=1;
volatile uint8_t dma0_chan1_sema_key=1;
volatile uint8_t dma1_chan0_sema_key=1;
volatile uint8_t dma1_chan1_sema_key=1;
#endif
void dma_get_sema_key(uint32_t nDMAC, uint32_t nChannel) 
{
#ifdef FC_SCENARIO_TEST
	volatile uint8_t *key;
	if((nDMAC == 0) && (nChannel == 0)) {
		key = &dma0_chan0_sema_key;
	} else if((nDMAC == 0) && (nChannel == 1)) {
		key = &dma0_chan1_sema_key;
	} else if((nDMAC == 1) && (nChannel == 0)) {
		key = &dma1_chan0_sema_key;
	} else if((nDMAC == 1) && (nChannel == 1)) {
		key = &dma1_chan1_sema_key;
	} else {
		fatal_log("DMA_GET_SEMA_KEY: Invalid Parameters!\n");
	}
	while(*key == 0) {timeout_ns(100);}
	/*print_log("DMA_GET_SEMA_KEY, DMAC%d, Chan=%d\n", nDMAC, nChannel); */
	*key = 0;
#endif
}
void dma_set_sema_key(uint32_t nDMAC, uint32_t nChannel) 
{
#ifdef FC_SCENARIO_TEST
	volatile uint8_t *key;
	if((nDMAC == 0) && (nChannel == 0)) {
		key = &dma0_chan0_sema_key;
	} else if((nDMAC == 0) && (nChannel == 1)) {
		key = &dma0_chan1_sema_key;
	} else if((nDMAC == 1) && (nChannel == 0)) {
		key = &dma1_chan0_sema_key;
	} else if((nDMAC == 1) && (nChannel == 1)) {
		key = &dma1_chan1_sema_key;
	} else {
		fatal_log("DMA_GET_SEMA_KEY: Invalid Parameters!\n");
	}
	*key = 1;
	/*print_log("DMA_SET_SEMA_KEY, DMAC%d, Chan=%d\n", nDMAC, nChannel); */
#endif
}
#endif
#endif

/* transfer width, also used as function parameters */
#define DMA_TRANSFER_WIDTH_BYTE    0 /* 8 bit   */
#define DMA_TRANSFER_WIDTH_HWORD   1 /* 16 bits */
#define DMA_TRANSFER_WIDTH_WORD    2 /* 32 bits */

/* burst size, also used as function parameters */
#define DMA_BURST_SIZE_1           0
#define DMA_BURST_SIZE_4           1
#define DMA_BURST_SIZE_8           2
#define DMA_BURST_SIZE_16          3
#define DMA_BURST_SIZE_32          4
#define DMA_BURST_SIZE_64          5
#define DMA_BURST_SIZE_128         6
#define DMA_BURST_SIZE_256         7

/* flow control and transfer type values, values refer to PL081 TRM P3-22 Table 3-24 */
#define DMA_MEM2MEM_DMAC_CTL       0 /* PrimeCell DMAC as the flow controller */
#define DMA_MEM2PERI_DMAC_CTL      1
#define DMA_PERI2MEM_DMAC_CTL      2
#define DMA_PERI2PERI_DMAC_CTL     3
#define DMA_PERI2PERI_DST_CTL      4 /* destination periphral as flow controller */
#define DMA_MEM2PERI_PERI_CTL      5
#define DMA_PERI2MEM_PERI_CTL      6
#define DMA_PERI2PERI_SRC_CTL      7 /* source periphral as flow controller */


/* PL081 DMA peripheral IDs */
#define DMAC_PERIPHERAL_MEMORY         24 /* not a real ID used by HW */
#define DMAC1_PERIPHERAL_SMAU_IN         0 
#define DMAC1_PERIPHERAL_SMAU_OUT        1
#define DMAC1_PERIPHERAL_SmartCard0_Tx    2 
#define DMAC1_PERIPHERAL_SmartCard0_Rx    3
#define DMAC1_PERIPHERAL_SPI0_Tx          4
#define DMAC1_PERIPHERAL_SPI0_Rx         5
#define DMAC1_PERIPHERAL_SPI1_Tx          6
#define DMAC1_PERIPHERAL_SPI1_Rx         7
#define DMAC1_PERIPHERAL_SmartCard1_Tx    8 
#define DMAC1_PERIPHERAL_SmartCard1_Rx    9

/* PL080 DMA peripheral IDs */
#define DMAC0_PERIPHERAL_MEMORY     24 /* not a real ID used by HW */
#define DMAC0_PERIPHERAL_UART0_Tx 0   
#define DMAC0_PERIPHERAL_UART0_Rx 1   
#define DMAC0_PERIPHERAL_UART1_Tx 2   
#define DMAC0_PERIPHERAL_UART1_Rx 3   
#define DMAC0_PERIPHERAL_UART2_Tx 4   
#define DMAC0_PERIPHERAL_UART2_Rx 5   
#define DMAC0_PERIPHERAL_UART3_Tx 6   
#define DMAC0_PERIPHERAL_UART3_Rx 7   
#define DMAC0_PERIPHERAL_SPI0_Tx 8
#define DMAC0_PERIPHERAL_SPI0_Rx 9
#define DMAC0_PERIPHERAL_SPI1_Tx 10
#define DMAC0_PERIPHERAL_SPI1_Rx 11
#define DMAC0_PERIPHERAL_SPI2_Tx 12   
#define DMAC0_PERIPHERAL_SPI2_Rx 13   
#define DMAC0_PERIPHERAL_SPI3_Tx 14   
#define DMAC0_PERIPHERAL_SPI3_Rx 15   
#define DMAC0_PERIPHERAL_SmartCard0_Tx 16
#define DMAC0_PERIPHERAL_SmartCard0_Rx 17 
#define DMAC0_PERIPHERAL_I2S_Tx 18   
#define DMAC0_PERIPHERAL_I2S_Rx 19   
#define DMAC0_PERIPHERAL_TP_ESPI_TX 20
#define DMAC0_PERIPHERAL_TP_ESPI_RX 21
#define DMAC0_PERIPHERAL_SmartCard1_Tx 22
#define DMAC0_PERIPHERAL_SmartCard1_Rx 23 

#ifdef MBOX_UMC
#define DMADATA_START   ((DMA_SRAM_START & 0x00ffffff) | 0x38000000)
#else
#define DMADATA_START   DMA_SRAM_START
#endif

#define SMC_VMEM_BASE   0x00000000

#define MAX_DMA_PKT_ARMCC            40

/*
 * the definition of DMA CONTROL register fields
 * the upper layer will need to set all of this and pass in
 * this definition only works in little endian
 */
typedef struct _control_reg
{
	uint32_t
	    TransferSize : 12, /*how many to dma */
	    SBSize       :  3, /* src  burst size */
	    DBSize       :  3, /*dest burst size */
	    SWidth       :  3, /* src  width */
	    DWidth       :  3, /* dest width */
            SM	         :  1, /*Source AHB Master for Pl080, reserved for Pl081 */
            DM	         :  1, /*Destination AHB Master for Pl080, reserved for Pl081 */
	    SI           :  1, /*src incr */
	    DI           :  1, /* dest incr */
	    Prot         :  3, /* protection */
	    TCINT        :  1; /* terminal count interrupt */
} DMA_CTRL_REG;


#define DMAC_MAX_TRANSFER_SIZE 0x1000 /* the max transfer size in one dma buffer without chaining */

/* the LLI structure will be used as data in the actual dma transfer */
typedef struct _LLI
{
	uint32_t nSrcAddr;
	uint32_t nDstAddr;          /* byte aligned */
	uint32_t nLLIAddr;          /* word aligned */ /* linked list item */
	uint32_t regCCtrl;          /* channel control register */
} LLI_t;


/*
 * the dma controller channel structure
 * use uint8_t to save space
 */
typedef struct _dmac
{
  uint8_t  nDMAC;         /* the dma controller number, 0 or 1  //DMA0 is PL080/DMA1 is PL081 */
  uint8_t  nChannel;      /* the channel number used,  DMA0- 0..7 and DMA1 0..1 */
  uint8_t  nSrcPeripheral, nDstPeripheral; /* DMA Peripheral ID, defined above */
  uint16_t flags;         /* endianess */
  uint16_t nFlowControl;  /* transfer type and flow controll, P3-22 */
  uint32_t nDMACRegBase;  /* base register address of DMA controller */
  uint32_t nNumPackets;   /* total items of LLI */
  uint32_t nLLI_index;
  LLI_t   *LLI;
  uint8_t chnlHalt;       /* 0: enable dma req; 1: ignore further src DMA req */
  uint8_t chnlActive;     /* 0: no data in fifo, 1: has data in fifo */
  uint8_t chnlLock;       /* 1: enable locked xfer */
  uint8_t ITC_Mask;       /* termincal count int mask */
  uint8_t IE_Mask;        /* interrupt error mask */
  LLI_t   lliTb[MAX_DMA_PKT_ARMCC];
} dmac_t;

#define DMAC_FLAGS_BIGENDIAN           0x0001
#define DMAC_FLAGS_BIGENDIAN           0x0001
#define DMAC_FLAGS_POOLING             0x0008
#define DMAC_FLAGS_VALID               0x0010 /* if this is a valid channel, set after init */

/* flow controller definition, used for function parameter */
#define DMAC_FC_DMAC                   0x0001 /* DMAC is the flow controller */
#define DMAC_FC_PERI                   0x0002 /* Peripheral is the flow controller */
#define DMAC_FC_SRCPERI                0x0003 /* Src peripheral is the flow controller, used in peripheral to peripheral case */
#define DMAC_FC_DSTPERI                0x0004 /* Dst peripheral is the flow controller, used in peripheral to peripheral case */

#define DMAC_GET_LLI_NUM(dmac)         (dmac.nLLI_index)

#define DMAC0_CHANNELS_BUSY               0xFF
#define DMAC0_CHANNEL_ANY                 0x8
#define DMAC1_CHANNELS_BUSY               0x3
#define DMAC1_CHANNEL_ANY                 0x2
#define DMAC_CHANNEL_0                   0x0
#define DMAC_CHANNEL_1                   0x1
#define DMAC_CHANNEL_2                   0x2
#define DMAC_CHANNEL_3                   0x3
#define DMAC_CHANNEL_4                   0x4
#define DMAC_CHANNEL_5                   0x5
#define DMAC_CHANNEL_6                   0x6
#define DMAC_CHANNEL_7                   0x7

/* function prototypes */
CLS_STATUS dma_init_old(dmac_t *dmac, LLI_t *LLI, uint32_t nNumPackets, uint32_t nSrcPeripheral,
				  uint32_t nDstPeripheral, uint32_t nFC, uint32_t bBigEndian);
CLS_STATUS dma_add_packet(void *dma_handle, uint32_t nSrcAddr, uint32_t nDstAddr, DMA_CTRL_REG nCtrlRegVal);
CLS_STATUS dma_activate(void *dma_handle, uint32_t nDMAC, uint32_t bPooling);
CLS_STATUS dma_status(void *dma_handle);
void dma_cleanup(void *dma_handle);
void dma_halt(void *dma_handle);



/* ======== 5880 DMA API Routines ====== */
#ifdef ARMCC
    #ifdef INSIDEARM
        CLS_STATUS DMA_Init(dmac_t *dmac, LLI_t *LLI, uint32_t nNumPackets, uint32_t nSrcPeripheral, uint32_t nDstPeripheral, uint32_t nFC, uint32_t bBigEndian);
        CLS_STATUS DMA_AddPacketDes(void *dma_handle, uint32_t nSrcAddr, uint32_t nDstAddr, DMA_CTRL_REG nCtrlRegVal, int memLoc);
        CLS_STATUS DMA_Config(void *dma_handle, uint32_t nDMAC, uint32_t nChannel, uint32_t bPolling);
        CLS_STATUS DMA_IsChannelBusy(void *dma_handle);
        CLS_STATUS DMA_IsChannelActive(void *dma_handle);
        CLS_STATUS DMA_ChannelOff(void *dma_handle);
        CLS_STATUS DMA_ChannelHalt(void *dma_handle);
        CLS_STATUS DMA_ChannelResume(void *dma_handle);
        CLS_STATUS DMA_ChannelOn(void *dma_handle);
        CLS_STATUS DMA_InterruptClear(void *dma_handle, uint8_t itcClr, uint8_t ieClr);
        CLS_STATUS DMA_InterruptStatus(void *dma_handle, uint8_t *itcStatus, uint8_t *ieStatus);
        CLS_STATUS DMA_InterruptStatus_opt(void *dma_handle, uint8_t *itcStatus, uint8_t *ieStatus);
        CLS_STATUS DMA_Config_ChannelOn(void *dma_handle, uint32_t nDMAC, uint32_t nChannel, uint32_t bPolling);
	CLS_STATUS DMA_Full_Transfer(uint32_t nSrcPeripheral,uint32_t nDstPeripheral,uint32_t nFC,uint32_t nSrcAddr,uint32_t nDstAddr,DMA_CTRL_REG nCtrlRegVal,uint32_t nDMAC,uint8_t * nChannel, uint32_t bBigEndian, uint32_t xferSize);


	/* ISR Function also in ARM compile */
	void dma1_isr(void);
	void dma0_isr(void);
	/*void bblrtc_isr(void); */
   #else

#define DMA_Init(dmac, LLI, nNumPackets, nSrcPeripheral, nDstPeripheral, nFC, bBigEndian) \
DMA_Init_Mbox(dmac, LLI, nNumPackets, nSrcPeripheral, nDstPeripheral, nFC, bBigEndian) 

CLS_STATUS DMA_Init_Mbox(dmac_t *dmac, LLI_t *LLI, uint32_t nNumPackets, uint32_t nSrcPeripheral, uint32_t nDstPeripheral, uint32_t nFC, uint32_t bBigEndian)
{ \
             uint32_t args[7]; \
             CLS_STATUS   status; \
             byte *ptr; \
	     ptr = (byte*) dmac; \
             mboxmem->mem_write(ddr_vmem_addr(DMADATA_START), ptr,sizeof(dmac_t));   \
	     print_log("DMA_Init: dmac=%x, LLI=%x, NumPtks=%d\n", dmac, LLI, nNumPackets); \
             args[0] = (int) (DMADATA_START); \
	     args[1] = (int) LLI; \
	     args[2] = nNumPackets; \
	     args[3] = nSrcPeripheral; \
	     args[4] = nDstPeripheral; \
	     args[5] = nFC; \
	     args[6] = bBigEndian; \
             status = call_mbox_from_tb(DMA_INIT_ID, args, 7); \
	     mboxmem->mem_read(ddr_vmem_addr(DMADATA_START), ptr, sizeof(dmac_t));   \
	     return status; \
           }


#define DMA_AddPacketDes(dma_handle, nSrcAddr,nDstAddr, nCtrlRegVal, memLoc) \
DMA_AddPacketDes_Mbox(dma_handle, nSrcAddr,nDstAddr, nCtrlRegVal, memLoc) 

#ifdef BIGENDIAN 
CLS_STATUS DMA_AddPacketDes_Mbox(void *dma_handle, uint32_t nSrcAddr, uint32_t nDstAddr, DMA_CTRL_REG nCtrlRegVal, int memLoc)
{ \
             uint32_t args[5]; \
             CLS_STATUS   status; \
             byte *ptr; \
  	     print_log("DMA_Add: dma_handle=%x, nSrc=%x, nDst=%x, CtrlReg=%x\n", dma_handle, nSrcAddr, nDstAddr, nCtrlRegVal); \
	     ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(DMADATA_START), ptr,sizeof(dmac_t));   \
             args[0] = (int) (DMADATA_START); \
	     args[1] = nSrcAddr; \
	     args[2] = nDstAddr; \
	     args[3] =  (nCtrlRegVal.TransferSize << 20) |  (nCtrlRegVal.SBSize << 17) | (nCtrlRegVal.DBSize << 14) |  (nCtrlRegVal.SWidth << 11) | (nCtrlRegVal.DWidth << 8) | (nCtrlRegVal.SM << 7) |  (nCtrlRegVal.DM << 6) | (nCtrlRegVal.SI << 5) |  (nCtrlRegVal.DI << 4) | (nCtrlRegVal.Prot << 1) | (nCtrlRegVal.TCINT); \
	     args[4] = memLoc; \
             status = call_mbox_from_tb(DMA_ADDPACKETDES_ID, args, 5); \
	     mboxmem->mem_read(ddr_vmem_addr(DMADATA_START), ptr, sizeof(dmac_t));   \
	     return status; \
  }
#else 

CLS_STATUS DMA_AddPacketDes_Mbox(void *dma_handle, uint32_t nSrcAddr, uint32_t nDstAddr, DMA_CTRL_REG nCtrlRegVal, int memLoc)
{ \
             uint32_t args[5]; \
             CLS_STATUS   status; \
             byte *ptr; \
             print_log("DMA_Add: dma_handle=%x, nSrc=%x, nDst=%x, CtrlReg=%x\n", dma_handle, nSrcAddr, nDstAddr, nCtrlRegVal); \
             ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(DMADATA_START), ptr,sizeof(dmac_t));   \
             args[0] = (int) (DMADATA_START); \
             args[1] = nSrcAddr; \
             args[2] = nDstAddr; \
             args[3] = *((int *)&nCtrlRegVal);        \
             args[4] = memLoc; \
             status = call_mbox_from_tb(DMA_ADDPACKETDES_ID, args, 5); \
             mboxmem->mem_read(ddr_vmem_addr(DMADATA_START), ptr, sizeof(dmac_t));   \
             return status; \
  }
#endif 
#define DMA_Config(dma_handle, nDMAC, nChannel, bPolling)  DMA_Config_Mbox(dma_handle, nDMAC, nChannel, bPolling) 


CLS_STATUS DMA_Config_Mbox(void *dma_handle, uint32_t nDMAC, uint32_t nChannel, uint32_t bPolling)
{ \
             uint32_t args[4]; \
             CLS_STATUS   status; \
             byte *ptr; \
	     uint32_t memaddr = DMADATA_START; \
	     ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(memaddr), ptr,sizeof(dmac_t));   \
             args[0] = (int) (memaddr); \
	     args[1] = nDMAC; \
	     args[2] = nChannel; \
	     args[3] = bPolling; \
             status = call_mbox_from_tb(DMA_CONFIG_ID, args, 4); \
	     mboxmem->mem_read(ddr_vmem_addr(memaddr), ptr, sizeof(dmac_t));   \
	     return status; \
  }

#define DMA_ChannelOn(dma_handle)   DMA_ChannelOn_Mbox(dma_handle)

CLS_STATUS DMA_ChannelOn_Mbox(void *dma_handle) 
{ \
             uint32_t args[1]; \
             CLS_STATUS   status; \
             byte *ptr; \
	     uint32_t memaddr = DMADATA_START;  \
             ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(memaddr), ptr,sizeof(dmac_t));   \
             args[0] = (int) ( memaddr); \
             status = call_mbox_from_tb(DMA_CHANNELON_ID, args, 1); \
             mboxmem->mem_read(ddr_vmem_addr(memaddr), ptr, sizeof(dmac_t));   \
             return status; \
}

#define DMA_IsChannelBusy(dma_handle)   DMA_IsChannelBusy_Mbox(dma_handle) 

CLS_STATUS DMA_IsChannelBusy_Mbox(void *dma_handle)
{ \
             uint32_t args[1]; \
             CLS_STATUS   status; \
             byte *ptr; \
	     uint32_t memaddr = DMADATA_START; \
	     ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(memaddr), ptr,sizeof(dmac_t));   \
             args[0] = (int) ( memaddr); \
             status = call_mbox_from_tb(DMA_ISCHANNELBUSY_ID, args, 1); \
	     mboxmem->mem_read(ddr_vmem_addr(memaddr), ptr, sizeof(dmac_t));   \
	     return status; \
}

#define DMA_IsChannelActive(dma_handle) DMA_IsChannelActive_Mbox(dma_handle) 
CLS_STATUS DMA_IsChannelActive(void *dma_handle)
{ \
             uint32_t args[1]; \
             byte *ptr; \
             CLS_STATUS   status; \
  	     uint32_t memaddr = DMADATA_START; \
	     ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(memaddr), ptr,sizeof(dmac_t));   \
             args[0] = (int) ( memaddr); \
             status = call_mbox_from_tb(DMA_ISCHANNELACTIVE_ID, args, 1); \
	     mboxmem->mem_read(ddr_vmem_addr(memaddr), ptr, sizeof(dmac_t));   \
	     return status; \
}

#define DMA_ChannelOff(dma_handle) DMA_ChannelOff_Mbox(dma_handle) 
CLS_STATUS DMA_ChannelOff_Mbox(void *dma_handle)
{ \
             uint32_t args[1]; \
             CLS_STATUS   status; \
             byte *ptr; \
	     uint32_t memaddr = DMADATA_START; \
	     ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(memaddr), ptr,sizeof(dmac_t));   \
             args[0] = (int) ( memaddr); \
             status = call_mbox_from_tb(DMA_CHANNELOFF_ID, args, 1); \
	     mboxmem->mem_read(ddr_vmem_addr(memaddr), ptr, sizeof(dmac_t));   \
	     return status; \
}

#define DMA_ChannelHalt(dma_handle) DMA_ChannelHalt_Mbox(dma_handle) 
CLS_STATUS DMA_ChannelHalt_Mbox(void *dma_handle)
{ \
             uint32_t args[1]; \
             CLS_STATUS   status; \
             byte *ptr; \
	     uint32_t memaddr = DMADATA_START; \
	     ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(memaddr), ptr,sizeof(dmac_t));   \
             args[0] = (int) ( memaddr); \
             status = call_mbox_from_tb(DMA_CHANNELHALT_ID, args, 1); \
	     mboxmem->mem_read(ddr_vmem_addr(memaddr), ptr, sizeof(dmac_t));   \
	     return status; \
}

#define DMA_ChannelResume(dma_handle) DMA_ChannelResume_Mbox(dma_handle) 
CLS_STATUS DMA_ChannelResume_Mbox(void *dma_handle)
{ \
             uint32_t args[1]; \
             CLS_STATUS   status; \
             byte *ptr; \
	     uint32_t memaddr = DMADATA_START; \
	     ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(memaddr), ptr,sizeof(dmac_t));   \
             args[0] = (int) (memaddr); \
             status = call_mbox_from_tb(DMA_CHANNELRESUME_ID, args, 1); \
	     mboxmem->mem_read(ddr_vmem_addr(memaddr), ptr, sizeof(dmac_t));   \
	     return status; \
}

#define DMA_InterruptClear(dma_handle,itcClr,ieClr)  DMA_InterruptClear_Mbox(dma_handle,itcClr,ieClr)
CLS_STATUS DMA_InterruptClear_Mbox(void *dma_handle, uint8_t itcClr, uint8_t ieClr)
{ \
             uint32_t args[3]; \
             CLS_STATUS   status; \
             byte *ptr; \
	     uint32_t memaddr = DMADATA_START;  \
	     ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(memaddr), ptr,sizeof(dmac_t));   \
             args[0] = (int) ( memaddr); \
	     args[1] = itcClr; \
	     args[2] = ieClr; \
             status = call_mbox_from_tb(DMA_INTERRUPTCLEAR_ID, args, 3); \
	     mboxmem->mem_read(ddr_vmem_addr(memaddr), ptr, sizeof(dmac_t));   \
	     return status; \
}

#define DMA_InterruptStatus(dma_handle, itcStatus, ieStatus)  DMA_InterruptStatus_Mbox(dma_handle, itcStatus, ieStatus)
CLS_STATUS DMA_InterruptStatus_Mbox(void *dma_handle, uint8_t *itcStatus, uint8_t *ieStatus)
{ \
             uint32_t args[3]; \
             uint8_t temp; \
             uint8_t temp2; \
	     byte *ptr;
             CLS_STATUS   status; \
	     uint32_t memaddr = DMADATA_START;   \
	     ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(memaddr), ptr,sizeof(dmac_t));   \
             args[0] = (int) (memaddr); \
	     args[1] = (int) ( memaddr+ sizeof(dmac_t)); \
	     args[2] = (int) ( memaddr+ sizeof(dmac_t) + 1); \
             status = call_mbox_from_tb(DMA_INTERRUPTSTATUS_ID, args, 3); \
	     mboxmem->mem_read(ddr_vmem_addr(memaddr), ptr, sizeof(dmac_t));   \
             *itcStatus = temp = mboxmem->mem_read8(( memaddr+ sizeof(dmac_t)) & 0xffff);
             *ieStatus = temp2 = mboxmem->mem_read8(( memaddr+ sizeof(dmac_t) + 1) & 0xffff);
	     print_log("DMA_InterruptStatus_Mbox: itcStatus=  %x; ieStatus = %x\n", temp, temp2);
	     return status; \
}

#define DMA_InterruptStatus_opt(dma_handle, itcStatus, ieStatus)  DMA_InterruptStatus_opt_Mbox(dma_handle, itcStatus, ieStatus)

CLS_STATUS DMA_InterruptStatus_opt_Mbox(void *dma_handle, uint8_t *itcStatus, uint8_t *ieStatus)
{ \
             uint32_t args[3]; \
             CLS_STATUS   status; \
             byte *ptr, *ptr1, *ptr2;                           \
             uint32_t memaddr = DMADATA_START;  \
             ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(memaddr), ptr,sizeof(dmac_t));   \
             ptr1 = (byte*) itcStatus; \
             ptr2 = (byte*) ieStatus; \
             args[0] = (int) ( memaddr); \
             args[1] = (int) ( memaddr+ sizeof(dmac_t)); \
             args[2] = (int) ( memaddr+ sizeof(dmac_t) + 4); \
             status = call_mbox_from_tb(DMA_INTERRUPTSTATUS_OPT_ID, args, 3); \
             mboxmem->mem_read(ddr_vmem_addr(memaddr), ptr, sizeof(dmac_t));   \
             mboxmem->mem_read(ddr_vmem_addr(memaddr+ sizeof(dmac_t)),ptr1, 1);  \
             mboxmem->mem_read(ddr_vmem_addr(memaddr+ sizeof(dmac_t) +4),ptr2, 1); \
             return status; \
}

#define DMA_Config_ChannelOn(dma_handle, nDMAC, nChannel, bPolling)  \
DMA_Config_ChannelOn_Mbox(dma_handle, nDMAC, nChannel, bPolling) 
CLS_STATUS DMA_Config_ChannelOn_Mbox(void *dma_handle, uint32_t nDMAC, uint32_t nChannel, uint32_t bPolling)
{ \
             uint32_t args[4]; \
             CLS_STATUS   status; \
             byte *ptr; \
	     uint32_t memaddr = DMADATA_START;  \
	     print_log("ConfigMBOX: dma_handle=%x, nDMAC=%x, nChannel=%x\n", dma_handle, nDMAC, nChannel); \
	     ptr = (byte*) dma_handle; mboxmem->mem_write(ddr_vmem_addr(memaddr), ptr,sizeof(dmac_t));   \
             args[0] = (int) ( memaddr); \
	     args[1] = nDMAC; \
	     args[2] = nChannel; \
	     args[3] = bPolling; \
             status = call_mbox_from_tb(DMA_CONFIG_CHANNELON_ID, args, 4); \
	     mboxmem->mem_read(ddr_vmem_addr(memaddr), ptr, sizeof(dmac_t));   \
	     return status; \
  }

#define DMA_Full_Transfer(nSrcPeripheral,nDstPeripheral,nFC,nSrcAddr,nDstAddr,nCtrlRegVal,nDMAC,nChannel,bBigEndian, xferSize) \
DMA_Full_Transfer_Mbox(nSrcPeripheral,nDstPeripheral,nFC,nSrcAddr,nDstAddr,nCtrlRegVal,nDMAC,nChannel,bBigEndian, xferSize)

CLS_STATUS DMA_Full_Transfer_Mbox(uint32_t nSrcPeripheral,uint32_t nDstPeripheral,uint32_t nFC,uint32_t nSrcAddr,uint32_t nDstAddr, DMA_CTRL_REG nCtrlRegVal,uint32_t nDMAC,uint8_t * nChannel, uint32_t bBigEndian, uint32_t xferSize )
{ \
             uint32_t args[10]; \
             CLS_STATUS status; \
             args[0] = nSrcPeripheral; args[1] = nDstPeripheral; args[2] = nFC; args[3] = nSrcAddr; \
             args[4] = nDstAddr; args[5] = *((int *)&nCtrlRegVal); args[6] = nDMAC; args[7] = DMADATA_START; args[8] = bBigEndian; args[9] = xferSize; \
             status = call_mbox_from_tb(DMA_FULL_TRANSFER_ID, args, 10); \
             *nChannel =  mboxmem->mem_read8((DMADATA_START & 0xfffff)); \
             return status; \
}

    #endif
#else
        CLS_STATUS DMA_Init(dmac_t *dmac, LLI_t *LLI, uint32_t nNumPackets, uint32_t nSrcPeripheral,
				  uint32_t nDstPeripheral, uint32_t nFC, uint32_t bBigEndian);
        CLS_STATUS DMA_AddPacketDes(void *dma_handle, uint32_t nSrcAddr, uint32_t nDstAddr, DMA_CTRL_REG nCtrlRegVal, int memLoc);
        CLS_STATUS DMA_Config(void *dma_handle, uint32_t nDMAC, uint32_t nChannel, uint32_t bPolling);
        CLS_STATUS DMA_IsChannelBusy(void *dma_handle);
        CLS_STATUS DMA_IsChannelActive(void *dma_handle);
        CLS_STATUS DMA_ChannelOff(void *dma_handle);
        CLS_STATUS DMA_ChannelHalt(void *dma_handle);
        CLS_STATUS DMA_ChannelResume(void *dma_handle);

        CLS_STATUS DMA_ChannelOn(void *dma_handle);
        CLS_STATUS DMA_InterruptClear(void *dma_handle, uint8_t itcClr, uint8_t ieClr);
        CLS_STATUS DMA_InterruptStatus(void *dma_handle, uint8_t *itcStatus, uint8_t *ieStatus);
        CLS_STATUS DMA_InterruptStatus_opt(void *dma_handle, uint8_t *itcStatus, uint8_t *ieStatus);

        /*
         *CLS_STATUS DMA_PeripheralTxEnable(uint32_t peripheralID,uint8_t burstSize,uint8_t chainCount);
         *CLS_STATUS DMA_PeripheralRxEnable(uint32_t peripheralID,uint8_t burstSize,uint8_t mode);
         *CLS_STATUS DMA_PeripheralDMAOnError(uint32_t peripheralID);
         *CLS_STATUS DMA_PeripheralDMAStatus(uint32_t peripheralID,uint8_t *txStatus, uint8_t *rxStatus);
         */
        CLS_STATUS DMA_Config_ChannelOn(void *dma_handle, uint32_t nDMAC, uint32_t nChannel, uint32_t bPolling);
	CLS_STATUS DMA_Full_Transfer(uint32_t nSrcPeripheral,uint32_t nDstPeripheral,uint32_t nFC,uint32_t nSrcAddr,uint32_t nDstAddr,DMA_CTRL_REG nCtrlRegVal,uint32_t nDMAC,uint8_t * nChannel, uint32_t bBigEndian, uint32_t xferSize);

#endif

#endif /* __DMAC_H__ */
