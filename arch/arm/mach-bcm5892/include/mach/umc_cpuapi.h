/*****************************************************************************
* Copyright 2007 - 2009 Broadcom Corporation.  All rights reserved.
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
 *                                                         /\
 *                                                  _     /  \     _
 * ________________________________________________/ \   /    \   / \_
 *                                                    \_/      \_/
 *  Broadcom Claudius Project
 *
 *  File:    umc_cpuapi.h
 *
 *  Author : Praveen Sathyadevan
 *           praveenr@broadcom.com
 *           +91-80-2518-4205 (x84205)
 *
 *  Description: 
 *  Version: $Id $
 *
*****************************************************************************/

#include "bcm5892_sw.h"
#include "bcm5892_reg.h"
#include "regaccess.h"
#include "umc_reg.h"

#ifdef ARMCC
      #ifdef INSIDEARM
             #include "arm_console.h"
      #else
            #include "mboxutils.h"
     #endif
#endif

#ifndef _UMC_CPUAPI_H_
#define _UMC_CPUAPI_H_

#define VIC_UMC_INTR_ID 29
#define VIC_UMC_INTR_EN (0x1 << VIC_UMC_INTR_ID)

#define EN_DATA_SCR (0x1)
#define EN_ADDR_SCR (0x1)
#define DATA_SCR_DIS (0x0)
#define ADDR_SCR_DIS (0x0)


/*
 * ---------------------------------------------------
 * NAND ECC_STATUS fields
 * ---------------------------------------------------
 */
#define UMC_NAND_ECC_INT_BLK0 0x1
#define UMC_NAND_ECC_INT_BLK1 0x2
#define UMC_NAND_ECC_INT_BLK2 0x4
#define UMC_NAND_ECC_INT_BLK3 0x8

#define UMC_NAND_ECC_FAIL_BLK0 (1 << 15)
#define UMC_NAND_ECC_FAIL_BLK1 (1 << 16)
#define UMC_NAND_ECC_FAIL_BLK2 (1 << 17)
#define UMC_NAND_ECC_FAIL_BLK2 (1 << 17)

#define UMC_NAND_ECC_CORRECTABLE_BLK0 (1 << 20)
#define UMC_NAND_ECC_CORRECTABLE_BLK1 (1 << 21)
#define UMC_NAND_ECC_CORRECTABLE_BLK2 (1 << 22)
#define UMC_NAND_ECC_CORRECTABLE_BLK3 (1 << 23)

/*
 * ---------------------------------------------------
 * Timing register fields
 * ---------------------------------------------------
 */
#define T_WE_MASK      0x7
#define T_TR_MASK      0x7
#define T_PC_MASK      0x7
#define T_WP_MASK      0x7
#define T_CEOE_MASK    0x7
#define T_WC_MASK      0xf
#define T_RC_MASK      0xf

#define UMC_F_t_we_3_R         20
#define UMC_F_t_tr_3_R         17
#define UMC_F_t_pc_3_R         14
#define UMC_F_t_wp_3_R         11
#define UMC_F_t_ceoe_3_R       8
#define UMC_F_t_wc_0_R         4
#define UMC_F_t_rc_1_R         0


/*
 * ---------------------------------------------------
 * UMC Configuration values
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
 */


/*
 * ---------------------------------------------------
 * NAND Commands
 * ---------------------------------------------------
 */
#define NAND_PAGE_PRG_S_CMD 0x80
#define NAND_PAGE_PRG_E_CMD 0x10
#define NAND_PAGE_RD_S_CMD  0x00
#define NAND_PAGE_RD_E_CMD  0x30
#define NAND_BLK_ERS_S_CMD  0x60
#define NAND_BLK_ERS_E_CMD  0xD0
#define NAND_RAND_RD_S_CMD  0x05
#define NAND_RAND_RD_E_CMD  0xE0
#define NAND_RAND_WR_S_CMD  0x85

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
 * ---------------------------------------------------------------------------
 * Function prototypes
 * ---------------------------------------------------------------------------
 */
#ifdef ARMCC
  #ifdef INSIDEARM
     void umc_sram_mt45w8mw16_cycles(void);
     void umc_nor_config_k8a2815etb_7c(void) ;
     void umc_nor_config_s29gl512p_11(void) ;
     void umc_nor_config_mt45w8mw16(void);
     void umc_nor_cycles_s29gl512p_11(void);
     CLS_STATUS umc_config_nand16(void);
     
     
     CLS_STATUS umc_set_cycles(uint8_t t_WE, uint8_t t_TR, uint8_t t_PC, uint8_t t_WP, uint8_t t_CEOE, uint8_t t_WC, uint8_t t_RC); 
     CLS_STATUS umc_memif_cfg(uint8_t type0, uint8_t chip0, uint8_t width0, uint8_t type1, uint8_t chip1, uint8_t width1 );
     CLS_STATUS umc_nor_write(uint32_t wrAddr, uint8_t* wrBuf, uint32_t byteLength, uint8_t  ahbMode);
     CLS_STATUS umc_config_cs(uint8_t cs, uint8_t interface, uint8_t set_adv, uint8_t set_sync_mode, 
                              uint8_t wr_bl, uint8_t rd_bl, uint8_t width) ;
     CLS_STATUS umc_ecc_config(uint8_t ecc_jump, uint8_t ecc_read_end, 
                               uint8_t ecc_mode, uint8_t pagesize);
     CLS_STATUS umc_int_enable(uint8_t ecc_int1, uint8_t ecc_int0, uint8_t int_en1, uint8_t int_en0);
     CLS_STATUS umc_config_cs_type(uint8_t cs2_type, uint8_t cs1_type, uint8_t cs0_type);
     
     
     CLS_STATUS umc_nand_page_prg_start(uint32_t cs, uint32_t address, uint8_t width,uint8_t addr_cycles);
     CLS_STATUS umc_nand_page_prg_end(uint32_t cs, uint32_t data);
     CLS_STATUS umc_nand_page_rd_cmd(uint32_t cs, uint32_t address, uint8_t width, uint8_t no_addr_cycles , uint32_t no_of_col_addr_cycles, uint32_t pagesize_in_bytes ) ;
     CLS_STATUS umc_smallnand_page_rd_cmd(uint32_t cs, uint32_t address, uint8_t width, uint8_t no_addr_cycles );
     CLS_STATUS umc_smallnand_page_rd(uint32_t cs, uint32_t page_addr, uint8_t width, uint8_t no_addr_cycles ,
                             uint8_t* dst_addr, uint32_t bytesize);
     CLS_STATUS umc_nand_blk_erase(uint32_t cs, uint32_t address);
     void       umc_nand_word_wr(uint8_t cs, uint32_t data);
     uint32_t   umc_nand_wrod_rd(uint8_t cs);
     CLS_STATUS umc_nand_random_data_rd_cmd(uint32_t cs, uint32_t address, uint8_t width, uint8_t no_addr_cycles );
     CLS_STATUS umc_nand_page_prg( uint8_t cs,  uint32_t page_addr, uint8_t width, uint8_t no_addr_cycles ,
                                  uint8_t* src_addr, uint32_t bytesize);
     CLS_STATUS umc_nand_page_rd(uint32_t cs, uint32_t page_addr, uint8_t width, uint8_t no_addr_cycles ,
                uint8_t* dst_addr, uint32_t bytesize, uint32_t no_of_col_addr_cycles, uint32_t pagesize_in_bytes) ;
     uint32_t umc_nand_page_rd_end(uint32_t cs ) ;
     CLS_STATUS umc_nand_random_data_rd(uint32_t cs, uint32_t col_addr, uint8_t width, uint8_t no_addr_cycles,
                                        uint8_t* dst_addr, uint32_t bytesize);
     CLS_STATUS umc_nand_random_data_wr(uint32_t cs, uint32_t col_addr, uint8_t width, uint8_t no_addr_cycles,
                                   uint8_t* src_addr, uint32_t bytesize);

     uint32_t  umc_wait_for_intr(uint8_t intr_id);
     uint32_t umc_ecc_status(void);

     CLS_STATUS umc_set_cs_baseaddr(uint32_t cs2_base, uint32_t cs1_base, uint32_t cs0_base);
     void umc_isr(void);
     void umc_int_cnt_latch(void);
     void umc_nand_cycles(uint32_t device_id);
     CLS_STATUS umc_low_power_enter(void);
     CLS_STATUS umc_low_power_exit(void);
     CLS_STATUS cls_scrambler_en( uint32_t data_scr_en, uint32_t addr_scr_en) ;
     CLS_STATUS cls_scrambler_bypass ( uint32_t bypass, uint32_t bypass_lock);
     CLS_STATUS cls_scrambler_setup(
                            uint32_t addr_swl_lo, uint32_t addr_swl_hi,
                            uint32_t data_swl_lo, uint32_t data_swl_hi,
                            uint32_t addr_key   , uint32_t data_key,
                            uint32_t addr_scr_xor_mask,
                            uint32_t scr_addr_base, uint32_t scr_addr_mask
                          );
     CLS_STATUS umc_nand_set_cycles( uint8_t t_RR,
                                uint8_t t_AR,
                                uint8_t t_CLR,
                                uint8_t t_WP,
                                uint8_t t_REA,
                                uint8_t t_WC,
                                uint8_t t_RC
                              );
     void umc_set_nand_onfi_timing_mode(uint32_t timmode, uint32_t freq) ;
  #else
     #define umc_nand_blk_erase(...) call_mbox(3, UMC_NAND_BLK_ERASE_ID, __VA_ARGS__);
     #define umc_ecc_config(...) call_mbox(5, UMC_ECC_CONFIG_ID, __VA_ARGS__);
     #define umc_isr() call_mbox(1, UMC_ISR_ID);
     #define umc_int_cnt_latch() call_mbox(1, UMC_INT_CNT_LATCH_ID);
     #define umc_nand_wrod_rd(...) call_mbox(2, UMC_NAND_WROD_RD_ID, __VA_ARGS__);
     #define umc_config_cs_type(...) call_mbox(4, UMC_CONFIG_CS_TYPE_ID, __VA_ARGS__);
     #define cls_scrambler_en(...) call_mbox(3, CLS_SCRAMBLER_EN_ID, __VA_ARGS__);
     #define umc_low_power_exit() call_mbox(1, UMC_LOW_POWER_EXIT_ID);
     #define umc_wait_for_intr(...) call_mbox(2, UMC_WAIT_FOR_INTR_ID, __VA_ARGS__);
     #define umc_nand_word_wr(...) call_mbox(3, UMC_NAND_WORD_WR_ID, __VA_ARGS__);
     #define cls_scrambler_bypass(...) call_mbox(3, CLS_SCRAMBLER_BYPASS_ID, __VA_ARGS__);
     #define umc_nand_cycles(...) call_mbox(2, UMC_NAND_CYCLES_ID, __VA_ARGS__);
     #define umc_nand_page_prg_start(...) call_mbox(5, UMC_NAND_PAGE_PRG_START_ID, __VA_ARGS__);
     #define cls_scrambler_setup(...) call_mbox(10, CLS_SCRAMBLER_SETUP_ID, __VA_ARGS__);
     #define umc_nand_page_prg(...) call_mbox(7, UMC_NAND_PAGE_PRG_ID, __VA_ARGS__);
     #define umc_nand_page_rd(...) call_mbox(9, UMC_NAND_PAGE_RD_ID, __VA_ARGS__);
     #define umc_config_cs(...) call_mbox(8, UMC_CONFIG_CS_ID, __VA_ARGS__);
     #define umc_nand_page_rd_cmd(...) call_mbox(7, UMC_NAND_PAGE_RD_CMD_ID, __VA_ARGS__);
     #define umc_low_power_enter() call_mbox(1, UMC_LOW_POWER_ENTER_ID);
     #define umc_nor_write(...) call_mbox(5, UMC_NOR_WRITE_ID, __VA_ARGS__);
     #define umc_nor_config_s29gl512p_11() call_mbox(1, UMC_NOR_CONFIG_S29GL512P_11_ID);
     #define umc_nand_random_data_rd_cmd(...) call_mbox(5, UMC_NAND_RANDOM_DATA_RD_CMD_ID, __VA_ARGS__);
     #define umc_nand_page_rd_end(...) call_mbox(2, UMC_NAND_PAGE_RD_END_ID, __VA_ARGS__);
     #define umc_set_cs_baseaddr(...) call_mbox(4, UMC_SET_CS_BASEADDR_ID, __VA_ARGS__);
     #define umc_sram_mt45w8mw16_cycles() call_mbox(1, UMC_SRAM_MT45W8MW16_CYCLES_ID);
     #define umc_ecc_status() call_mbox(1, UMC_ECC_STATUS_ID);
     #define umc_int_enable(...) call_mbox(5, UMC_INT_ENABLE_ID, __VA_ARGS__);
     #define umc_nand_page_prg_end(...) call_mbox(3, UMC_NAND_PAGE_PRG_END_ID, __VA_ARGS__);
     #define umc_config_nand16() call_mbox(1, UMC_CONFIG_NAND16_ID);
     #define umc_set_cycles(...) call_mbox(8, UMC_SET_CYCLES_ID, __VA_ARGS__);
     #define umc_nor_config_mt45w8mw16() call_mbox(1, UMC_NOR_CONFIG_MT45W8MW16_ID);
     #define umc_memif_cfg(...) call_mbox(7, UMC_MEMIF_CFG_ID, __VA_ARGS__);
     #define umc_nor_cycles_s29gl512p_11() call_mbox(1, UMC_NOR_CYCLES_S29GL512P_11_ID);
     #define umc_nand_random_data_rd(...) call_mbox(7, UMC_NAND_RANDOM_DATA_RD_ID, __VA_ARGS__);
     #define umc_nand_random_data_wr(...) call_mbox(7, UMC_NAND_RANDOM_DATA_WR_ID, __VA_ARGS__);
     #define umc_nor_config_k8a2815etb_7c() call_mbox(1, UMC_NOR_CONFIG_K8A2815ETB_7C_ID);
  #endif
#else 
     void umc_sram_mt45w8mw16_cycles(void);
     void umc_nor_config_k8a2815etb_7c(void) ;
     void umc_nor_config_s29gl512p_11(void) ;
     void umc_nor_config_mt45w8mw16(void);
     void umc_nor_cycles_s29gl512p_11(void);
     CLS_STATUS umc_config_nand16(void);
     
     CLS_STATUS umc_set_cycles( uint8_t t_TR, uint8_t t_PC, uint8_t t_WP, uint8_t t_CEOE, uint8_t t_WC, uint8_t t_RC); 
/*     CLS_STATUS umc_set_cycles(uint8_t t_WE, uint8_t t_TR, uint8_t t_PC, uint8_t t_WP, uint8_t t_CEOE, uint8_t t_WC, uint8_t t_RC); */
     CLS_STATUS umc_memif_cfg(uint8_t type0, uint8_t chip0, uint8_t width0, uint8_t type1, uint8_t chip1, uint8_t width1 );
     CLS_STATUS umc_nor_write(uint32_t wrAddr, uint8_t* wrBuf, uint32_t byteLength, uint8_t  ahbMode);
     CLS_STATUS umc_config_cs(uint8_t cs, uint8_t interface, uint8_t set_adv, uint8_t set_sync_mode, 
                              uint8_t wr_bl, uint8_t rd_bl, uint8_t width) ;
     CLS_STATUS umc_ecc_config(uint8_t ecc_jump, uint8_t ecc_read_end, 
                               uint8_t ecc_mode, uint8_t pagesize);
     CLS_STATUS umc_int_enable(uint8_t ecc_int1, uint8_t ecc_int0, uint8_t int_en1, uint8_t int_en0);
     CLS_STATUS umc_config_cs_type(uint8_t cs2_type, uint8_t cs1_type, uint8_t cs0_type);
     
     CLS_STATUS umc_nand_page_prg_start(uint32_t cs, uint32_t address, uint8_t width,uint8_t addr_cycles);
     CLS_STATUS umc_nand_page_prg_end(uint32_t cs, uint32_t data);
     CLS_STATUS umc_nand_page_rd_cmd(uint32_t cs, uint32_t address, uint8_t width, uint8_t no_addr_cycles , uint32_t no_of_col_addr_cycles, uint32_t pagesize_in_bytes ) ;
     CLS_STATUS umc_smallnand_page_rd_cmd(uint32_t cs, uint32_t address, uint8_t width, uint8_t no_addr_cycles );
     CLS_STATUS umc_smallnand_page_rd(uint32_t cs, uint32_t page_addr, uint8_t width, uint8_t no_addr_cycles ,
                             uint8_t* dst_addr, uint32_t bytesize);
     CLS_STATUS umc_nand_blk_erase(uint32_t cs, uint32_t address);
     void       umc_nand_word_wr(uint8_t cs, uint32_t data);
     uint32_t   umc_nand_wrod_rd(uint8_t cs);
     CLS_STATUS umc_nand_random_data_rd_cmd(uint32_t cs, uint32_t address, uint8_t width, uint8_t no_addr_cycles );
     CLS_STATUS umc_nand_page_prg( uint8_t cs,  uint32_t page_addr, uint8_t width, uint8_t no_addr_cycles ,
                                  uint8_t* src_addr, uint32_t bytesize);
     CLS_STATUS umc_nand_page_rd(uint32_t cs, uint32_t page_addr, uint8_t width, uint8_t no_addr_cycles ,
                uint8_t* dst_addr, uint32_t bytesize, uint32_t no_of_col_addr_cycles, uint32_t pagesize_in_bytes) ;
     CLS_STATUS umc_nand_random_data_rd(uint32_t cs, uint32_t col_addr, uint8_t width, uint8_t no_addr_cycles,
                                        uint8_t* dst_addr, uint32_t bytesize);
     CLS_STATUS umc_nand_random_data_wr(uint32_t cs, uint32_t col_addr, uint8_t width, uint8_t no_addr_cycles,
                                   uint8_t* src_addr, uint32_t bytesize);
     uint32_t  umc_wait_for_intr(uint8_t intr_id);
     void umc_isr(void);
     uint32_t umc_ecc_status(void);
     CLS_STATUS umc_set_cs_baseaddr(uint32_t cs2_base, uint32_t cs1_base, uint32_t cs0_base);
     void umc_int_cnt_latch(void);
     uint32_t umc_nand_page_rd_end(uint32_t cs ) ;
     void umc_nand_cycles(uint32_t device_id);
     CLS_STATUS umc_low_power_enter(void);
     CLS_STATUS umc_low_power_exit(void);
     CLS_STATUS cls_scrambler_en( uint32_t data_scr_en, uint32_t addr_scr_en) ;
     CLS_STATUS cls_scrambler_bypass ( uint32_t bypass, uint32_t bypass_lock);
     CLS_STATUS cls_scrambler_setup(
                            uint32_t addr_swl_lo, uint32_t addr_swl_hi,
                            uint32_t data_swl_lo, uint32_t data_swl_hi,
                            uint32_t addr_key   , uint32_t data_key,
                            uint32_t addr_scr_xor_mask,
                            uint32_t scr_addr_base, uint32_t scr_addr_mask
                          );
     void umc_set_nand_onfi_timing_mode(uint32_t timmode, uint32_t freq) ;
#endif

#endif /* _UMC_CPUAPI_H_ */
/*
 * ---------------------------------------------------------------------------
 * End of File
 * ---------------------------------------------------------------------------
 */
