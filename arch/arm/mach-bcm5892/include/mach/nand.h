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


#ifndef __BCM5892_NAND_DOT_H_INCLUDED__
#define __BCM5892_NAND_DOT_H_INCLUDED__


/* Driver state values. */

#define STATE_READY	0
#define STATE_BUSY	1

/*
 * ---------------------------------------------------
 * UMC Configuration values _Begin
 * ---------------------------------------------------
 */
#define INTERFACE_0 0
#define INTERFACE_1 1

#define CS_0 0
#define CS_1 1
#define CS_2 2

#define TYPE_NODEVICE      0
#define TYPE_SRAM_NONMUXED 1
#define TYPE_NAND          2
#define TYPE_SRAM_MUXED    3

#define CS_NAND 0
#define CS_SRAM 1

#define WIDTH_8  0
#define WIDTH_16 1
#define WIDTH_32 2

#define CHIPS_1 0
#define CHIPS_2 1
#define CHIPS_3 2
#define CHIPS_4 3

/* Dev IDs*/
#define MICRON_1G_16BIT	0xB1
 #define MICRON_2G_8BIT 0xDA
#define STMICRO_2G_8BIT	0xAA
#define STMICRO_1G_8BIT	0xA1
#define SAMSUNG_64MB_8BIT	0x76
#define SAMSUNG_32MB_8BIT	0x75

#define BL_1  0x0
#define BL_4  0x1
#define BL_8  0x2
#define BL_16 0x3
#define BL_32 0x4

#define ECC_ABORT_INT_DIS 0x0
#define ECC_ABORT_INT_EN 0x1

#define ECC_PAGE_512  0x1
#define ECC_PAGE_1024 0x2
#define ECC_PAGE_2048 0x3

#define ECC_BYPASS    0x0
#define ECC_APBRD     0x1
#define ECC_ENABLE    0x2

#define ECC_RD_512    0x0
#define ECC_RD_END    0x1

#define ECC_NOJUMP     0x0
#define ECC_JUMP_COL   0x1
#define ECC_JUMP_FULL  0x2

#define ECC_COL_ADDR_MT 0x410

/* ECC value reg bitmasks */
#define ECC_FAIL		0x10000000
#define ECC_CORRECTABLE	0x08000000

/*
 * ---------------------------------------------------
 * NOR status read definitions
 * ---------------------------------------------------
 */
#define FLASH_STATUS_DQ7 0x80
#define FLASH_STATUS_DQ6 0x40
#define FLASH_STATUS_DQ5 0x20

/*
 * ---------------------------------------------------
 * NAND Vendors
 * ---------------------------------------------------
 * ---------------------------------------------------
 * Timing register fields
 * ---------------------------------------------------
 */
#define T_TR_MASK      0x7
#define T_PC_MASK      0x7
#define T_WP_MASK      0x7
#define T_CEOE_MASK    0x7
#define T_WC_MASK      0xf
#define T_RC_MASK      0xf

#define UMC_F_t_tr_3_R         17
#define UMC_F_t_pc_3_R         14
#define UMC_F_t_wp_3_R         11
#define UMC_F_t_ceoe_3_R       8
#define UMC_F_t_wc_0_R         4
#define UMC_F_t_rc_1_R         0

/*
 * ---------------------------------------------------
 * NAND Commands
 * ---------------------------------------------------
 */
#define NAND_PAGE_PRG_S_CMD 0x80
#define NAND_READ_ID_S_CMD  0x90
#define NAND_OTP_PRG_S_CMD  0xA0
#define NAND_PAGE_PRG_E_CMD 0x10
#define NAND_PAGE_PRG_C_CMD 0x15
#define NAND_PAGE_RD_S_CMD  0x00
#define NAND_PAGE_RD_E_CMD  0x30
#define NAND_BLK_ERS_S_CMD  0x60
#define NAND_BLK_ERS_E_CMD  0xD0
#define NAND_RAND_RD_S_CMD  0x05
#define NAND_RAND_RD_E_CMD  0xE0
#define NAND_RAND_WR_S_CMD  0x85
#define NAND_RESET_S_CMD    0xFF
/*
 * ---------------------------------------------------
 * device id definitions
 * ---------------------------------------------------
 */

#define DTYPE_IN_ID  (0xF00000)
#define DEV_NOR      (0xA00000)
#define DEV_NAND     (0xB00000)
#define DEV_SRAM     (0xC00000)

#define CRS_8        (0x0A0000)
#define CRS_16       (0x0B0000)

#define VENDOR_IN_ID (0x00F000)
#define NAND_SAMSUNG (0x00A000)
#define NAND_MICRON  (0x00B000)

#define NOR_64M      (0x000A00)
#define NOR_16M      (0x000B00)

#define OPV_33V      (0x0000A0)
#define OPV_18V      (0x0000B0)

#define SIZE_8M      (0x000001)
#define SIZE_64M     (0x000002)
#define SIZE_128M    (0x000003)
#define SIZE_256M    (0x000004)
#define SIZE_512M    (0x000005)
#define SIZE_1024M   (0x000006)
#define SIZE_256K    (0x000007)


#define S29_GL512  (DEV_NOR  | NOR_64M | OPV_33V)
#define S29_WS512  (DEV_NOR  | NOR_64M | OPV_18V)
#define S29_GL128  (DEV_NOR  | NOR_16M | OPV_33V)
#define S29_NS128  (DEV_NOR  | NOR_16M | OPV_18V)
#define K9_F1G8    (DEV_NAND | NAND_SAMSUNG | SIZE_128M | OPV_18V)
#define K9_K2G8    (DEV_NAND | NAND_SAMSUNG | SIZE_128M | OPV_33V)

#define MT29_F1G8  (DEV_NAND | NAND_MICRON  | SIZE_128M  | OPV_18V | CRS_8)
#define MT29_F8G8  (DEV_NAND | NAND_MICRON  | SIZE_1024M | OPV_33V | CRS_8)

#define MT29_F2G16 (DEV_NAND | NAND_MICRON  | SIZE_128M  | OPV_33V | CRS_16)
#define MT29_F4G16 (DEV_NAND | NAND_MICRON  | SIZE_256M  | OPV_33V | CRS_16)
#define MT29_F2G8  (DEV_NAND | NAND_MICRON  | SIZE_256M  | OPV_33V | CRS_8 )
#define MT29_F4G8  (DEV_NAND | NAND_MICRON  | SIZE_512M  | OPV_33V | CRS_8 )

#define SRAM_IDT    (DEV_SRAM | OPV_33V)
#define SRAM_MICRON (DEV_SRAM | OPV_18V)

#define DEVICE_NONE (0)
#define DEVICE_ANY  (1)

struct cs {
    uint32_t device_type;
    uint32_t width;
    uint32_t device_id;
} ;

typedef struct umc_cs_cfg {
    struct cs cs0, cs1, cs2;
} umc_cs_cfg;

/*
 * ---------------------------------------------------
 * UMC Configuration values _End
 * ---------------------------------------------------
 */

#define IDX_CS0 0
#define IDX_CS1 1
#define IDX_UMC 2
#define BCM5892_MAX_NAND_DEVICE 2


#ifndef AHB_BIT32
#define AHB_BIT32 (1<<5)
#endif

#ifndef AHB_BIT16
#define AHB_BIT16 (1<<4)
#endif

#ifndef AHB_BIT8
#define AHB_BIT8 (1<<3)
#endif

#define ECC_COL_ADDR_MT 0x410

#define BCM5892_READ_PAGE_SIZE  (8192) 
#define BCM5892_WRITE_PAGE_SIZE (2048)

#define DMU_UMC_PWR_ENABLE      DMU_F_dmu_pwd_umc_R+32
#define CHIP_SELECT 1

#if 1
#define CPU_READ_SINGLE(addr,size)  ((size == AHB_BIT8) ?  *((volatile uint8_t *)addr) :\
                                      ((size == AHB_BIT16) ? *((volatile uint16_t *)addr) :\
                                      *((volatile uint32_t *)addr)))

/*#define CPU_READ_BURST(addr, data, size) memcpy((void *)data, (void *)addr, size) */

#define CPU_RMW_OR_SINGLE(addr,data,size)  if(size == AHB_BIT8) *((volatile uint8_t *)addr) |= data; \
                                           else if(size == AHB_BIT16) *((volatile uint16_t *)addr) |= data; \
                                           else  *((volatile uint32_t *)addr) |= data


#define CPU_RMW_AND_SINGLE(addr,data,size) if(size == AHB_BIT8) *((volatile uint8_t *)addr) &= data; \
                                           else if(size == AHB_BIT16)  *((volatile uint16_t *)addr) &= data ; \
                                           else  *((volatile uint32_t *)addr) &= data

#define CPU_WRITE_SINGLE(addr,data,size)  if(size == AHB_BIT8) *((volatile uint8_t *)addr) = data ;\
                                          else if(size == AHB_BIT16) *((volatile uint16_t *)addr) = data ; \
                                          else *((volatile uint32_t *)addr) = data

#else

#define CPU_READ_SINGLE(addr,size)  (size == AHB_BIT8) ?  __raw_readb(addr) :\
                                      ((size == AHB_BIT16) ? __raw_readw(addr) :\
                                       __raw_readl(addr))

/*#define CPU_READ_BURST(addr, data, size) memcpy((void *)data, (void *)addr, size) */

#define CPU_RMW_OR_SINGLE(addr,data,size)  if(size == AHB_BIT8) {data |= __raw_readb(addr); __raw_writeb(data, addr);} \
						else if(size == AHB_BIT16) {data |= __raw_readw(addr); __raw_writew(data, addr);} \
							else {data |= __raw_readl(addr); __raw_writel(data, addr);} 


#define CPU_RMW_AND_SINGLE(addr,data,size) if(size == AHB_BIT8) *((volatile uint8_t *)addr) &= data; \
                                           else if(size == AHB_BIT16)  *((volatile uint16_t *)addr) &= data ; \
                                           else  *((volatile uint32_t *)addr) &= data

#define CPU_WRITE_SINGLE(addr,data,size)  if(size == AHB_BIT8) *((volatile uint8_t *)addr) = data ;\
                                          else if(size == AHB_BIT16) *((volatile uint16_t *)addr) = data ; \
                                          else *((volatile uint32_t *)addr) = data

#endif

struct bcm5892_nand_softc
{		
	volatile void __iomem *nand_cs0_base;
	volatile void __iomem *nand_cs1_base;

	u32 nand_umc_base;

  	u32 pUMC_R_umc_status_reg; 
  	u32 pUMC_R_umc_memif_cfg; 
  	u32 pUMC_R_umc_memc_cfg;   
  	u32 pUMC_R_umc_memc_clr;
  	u32 pUMC_R_umc_direct_cmd; 
  	u32 pUMC_R_umc_set_cycles; 
  	u32 pUMC_R_umc_set_opmode;
  	u32 pUMC_R_umc_set_opmode0_cs1; 
  	u32 pUMC_R_umc_addr_mask;  
  	u32 pUMC_R_umc_addr_match; 
  	u32 pUMC_R_umc_conf;      

  	u32 pUMC_R_umc_ecc_status; 
  	u32 pUMC_R_umc_ecc_memcfg;  
  	u32 pUMC_R_umc_ecc_memcmd1;
  	u32 pUMC_R_umc_ecc_memcmd2;
  	u32 pUMC_R_umc_ecc_addr0;  
  	u32 pUMC_R_umc_ecc_addr1;  
  	u32 pUMC_R_umc_ecc_value0; 
  	u32 pUMC_R_umc_ecc_value1; 
  	u32 pUMC_R_umc_ecc_value2;
  	u32 pUMC_R_umc_ecc_value3;
  	
	u32 pUMC_R_umc_ecc_value9;
	u32 pUMC_R_umc_ecc_value10;
	u32 pUMC_R_umc_ecc_value11;
	u32 pUMC_R_umc_ecc_value12;

  	u32 pUMC_R_umc_msu_scr_control_reg;          
  	u32 pUMC_R_umc_msu_addr_swz_ctrl_lo_reg;    
  	u32 pUMC_R_umc_msu_addr_swz_ctrl_hi_reg;   
  	u32 pUMC_R_umc_msu_addr_key_reg;          
  	u32 pUMC_R_umc_msu_addr_scr_msk_reg; 
  	u32 pUMC_R_umc_msu_data_swz_ctrl_lo_reg;  
  	u32 pUMC_R_umc_msu_data_swz_ctrl_hi_reg;
  	u32 pUMC_R_umc_msu_data_key_reg;      
  	u32 pUMC_R_umc_msu_addr_scr_base_reg; 
  	u32 pUMC_R_umc_msu_addr_scr_base_msk_reg;  

	u32 pUMC_R_umc_memc_cfg_MEMADDR; /*for enabling interrupt */

	struct mtd_info bcm5892_mtd_info;
	struct nand_chip bcm5892_nand_chip;
	int read_id;

	int (*nand_read)(struct mtd_info *, loff_t, size_t, size_t *, u_char *);
	int (*nand_write)(struct mtd_info *, loff_t, size_t, size_t *, const u_char *);
	int (*nand_erase)(struct mtd_info *, struct erase_info *);


	int (*nand_read_oob)(struct mtd_info *, loff_t, struct mtd_oob_ops *);
	void (*nand_readid_cmd)(struct bcm5892_nand_softc *, uint32_t, uint8_t, uint32_t *);
	void (*umc_nand_random_data_rd)(struct mtd_info *, uint32_t, uint32_t, uint8_t, uint8_t,uint8_t *, uint32_t);
	uint32_t (*umc_nand_random_page_read_end)( struct bcm5892_nand_softc *, uint32_t);

	void (*umc_nand_random_data_wr)(uint32_t, uint32_t, uint8_t, uint8_t,
					uint8_t *, uint32_t, 
					struct bcm5892_nand_softc *);
	void (*umc_nand_random_page_prg_end)( struct bcm5892_nand_softc *, uint32_t, uint32_t);

	int	state;

#ifdef BCM5892_DEBUG	
	unsigned int dbg_val;
#endif /*#ifdef BCM5892_DEBUG*/
};

#endif /*#ifndef __BCM5892_NAND_DOT_H_INCLUDED__*/

