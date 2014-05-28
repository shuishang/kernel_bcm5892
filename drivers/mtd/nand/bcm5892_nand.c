/*****************************************************************************
* Copyright 2009 - 2011 Broadcom Corporation.  All rights reserved.
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


#include <linux/module.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/concat.h>
#include <linux/amba/bus.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <mach/umc_reg.h>
#include <mach/bcm5892_reg.h>
#include <mach/bcm5892_sw.h> 
#include <mach/nand.h>
#include <mach/shm.h>

/* module params */
static int dev_width = 16;
module_param(dev_width, int, 0644);

static int debug = 0;
module_param(debug, int, 0644);

/*Intialization & structures*/
#define NAND_PART_NUM 2
#define CHIP_SEL	1

#define GET_SC(mtd)	(struct bcm5892_nand_softc *)(container_of(mtd, 	\
				      struct bcm5892_nand_softc, bcm5892_mtd_info))
#define PAGE_ADDRESS(x) (x * (1 << pg_addr_shift))
#define PDEBUG(fmt, args...) if (debug) printk( KERN_INFO "BCM5892_NAND: " fmt, ## args)


static struct mtd_partition BCM5892_nand_partition [] = {
{
	.name		= "NAND-block1",
	.size		= 0x1000000,	// 16MB
	.offset		=  0x0,
},           
{
	.name		= "yaffs2",
	.size		= 0x1E00000,	// 30MB
	.offset		= 0x1000000,
},
{
	.name		= "NAND-block2",
	.size		= MTDPART_SIZ_FULL,
	.offset		= MTDPART_OFS_NXTBLK,
}
};

static const char module_id[] = "bcm5892 nand";

/* Bad block stuff */
static uint8_t bcm5892_bbt_pattern_2048[] = { 'B', 'b', 't', '0' };
static uint8_t bcm5892_mirror_pattern_2048[] = { '1', 't', 'b', 'B' };

/* small page device */
static uint8_t bcm5892_bbt_pattern_512[] = { 'B' };
static uint8_t bcm5892_mirror_pattern_512[] = { 'b' };

static struct nand_bbt_descr bcm5892_bbt_main_descr_2048 = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	8,
	.len = 4,
	.veroffs = 12,
	.maxblocks = 4,
	.pattern = bcm5892_bbt_pattern_2048
};

static struct nand_bbt_descr bcm5892_bbt_mirr_descr_2048 = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	8,
	.len = 4,
	.veroffs = 12,
	.maxblocks = 4,
	.pattern = bcm5892_mirror_pattern_2048
};

static struct nand_bbt_descr bcm5892_bbt_main_descr_512 = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	4,
	.len = 1,
	.veroffs = 5,
	.maxblocks = 4,
	.pattern = bcm5892_bbt_pattern_512
};

static struct nand_bbt_descr bcm5892_bbt_mirr_descr_512= {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	4,
	.len = 1,
	.veroffs = 5,
	.maxblocks = 4,
	.pattern = bcm5892_mirror_pattern_512
};

/* we store ECC bytes at offset 32 in spare area */
static struct nand_ecclayout bcm5892_oobinfo_2048 = {
	.eccbytes = 16,
	.eccpos = {32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47},
	.oobfree = {
		 { .offset = 48,
		   .length = 16}}
};

/* we store ECC bytes at offset 8 in spare area */
static struct nand_ecclayout bcm5892_oobinfo_512 = {
	.eccbytes = 4,
	.eccpos = {8, 9, 10, 11},
	.oobfree = {
		 { .offset = 12,
		   .length = 4}}
};

/*addres cycles info */
static uint16_t rd_addr_cyc;
static uint16_t prg_addr_cyc;
static uint16_t rand_op_addr_cyc;
static uint16_t pg_addr_shift;

static void umc_memif_cfg(struct bcm5892_nand_softc *sc, uint8_t type0, uint8_t chip0, uint8_t width0, uint8_t type1, uint8_t chip1, uint8_t width1 );
static void umc_config_cs(struct bcm5892_nand_softc *sc, uint8_t cs,uint8_t interface, uint8_t set_adv, uint8_t set_sync_mode,uint8_t wr_bl, uint8_t rd_bl, uint8_t width);


// ----------------------------------------------------------------------------
// umc_int_enable
//
// function to enable/disable the UMC Interrupts
// ----------------------------------------------------------------------------
static void umc_int_enable(struct bcm5892_nand_softc *sc, uint8_t ecc_int1, 
				 uint8_t ecc_int0, uint8_t int_en1, uint8_t int_en0) 
{
	uint32_t val = 0x0;
	val = (ecc_int1 << 6) |
		(ecc_int0 << 5) |
		(int_en1  << 1) |
		(int_en0  << 0) ;
	writel (val, sc->pUMC_R_umc_memc_cfg_MEMADDR);
}

static uint32_t umc_nand_word_rd (struct bcm5892_nand_softc *sc, uint8_t cs) 
{
	uint32_t addr;
	uint8_t  no_addr_cycles = 0x1;
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);
	addr = addr | ( no_addr_cycles << 21)
                | ( NAND_PAGE_RD_E_CMD << 11)
                | ( 1 << 19)     // Data Phase
		;
//	printk("0000000\n");
	return  readl (addr);
}

static void umc_smallnand_erase (struct bcm5892_nand_softc *sc, uint32_t cs, 
				 uint16_t width, uint32_t address) 
{
	uint8_t st_cmd;
	uint8_t end_cmd;
	uint32_t addr;
	uint32_t device_addr1 = address;
	uint8_t  no_addr_cycles = 0x3;
	uint8_t dev_ad_byte[5];

	st_cmd  = NAND_BLK_ERS_S_CMD;
	end_cmd = NAND_BLK_ERS_E_CMD;

	dev_ad_byte[0] = ( width == 8) ? (address >> 9) & 0xff : (address >> 9) & 0x07; // CA[10:8]
	dev_ad_byte[1] = ( width == 8) ? (address >> 17) & 0xff : (address >> 16) & 0xff; // {BA[1:0],PA[5:0]}
	dev_ad_byte[2] = ( width == 8) ? (address >> 25) & 0x01 : (address >> 24) & 0xff;
	dev_ad_byte[3] = 0;

	device_addr1 = (dev_ad_byte[3] << 24) |
		(dev_ad_byte[2] << 16) |
		(dev_ad_byte[1] << 8) |
		(dev_ad_byte[0] ) ;
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);

	addr = addr | (st_cmd << 3)
                | (end_cmd << 11)
                | ( no_addr_cycles << 21)
                | ( 1 << 20)              //  End Command Valid = 1
		;
	writel (device_addr1, addr);
}     


// ----------------------------------------------------------------------------
// nand blk erase
// ----------------------------------------------------------------------------
static void umc_nand_blk_erase( struct bcm5892_nand_softc *sc, uint32_t cs, 
			       uint32_t address) 
{
	uint8_t st_cmd;
	uint8_t end_cmd;
	uint32_t addr;
	uint32_t device_addr1 = address;
	uint8_t  no_addr_cycles = 0x3;

	st_cmd  = NAND_BLK_ERS_S_CMD;
	end_cmd = NAND_BLK_ERS_E_CMD;

	addr = (u32) (cs ? sc->nand_cs1_base:sc->nand_cs0_base);

	addr = addr | (st_cmd << 3)
                | (end_cmd << 11)
                | ( no_addr_cycles << 21)
                | ( 1 << 20)              //  End Command Valid = 1
		;
	writel (device_addr1, addr);
}     

/* Read the ID*/
static void umc_nand_readid_cmd(struct bcm5892_nand_softc *sc, uint32_t cs, 
				uint8_t no_addr_cycles, uint32_t *read_id ) 
{
	uint8_t st_cmd;
	uint8_t end_cmd;
	uint32_t addr;
	uint16_t dev_id;
	struct nand_chip *nand = &sc->bcm5892_nand_chip;
	uint32_t data1, data2, val, val1, val2;
	
	st_cmd  = NAND_READ_ID_S_CMD;
	end_cmd = 0x0;
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);
	addr = addr | (st_cmd << 3)
                | (end_cmd << 11)
                | ( no_addr_cycles << 21)
                | ( 0 << 20)              //  End Command Valid = 0
		;

	writel (0, addr);

        data1 = umc_nand_word_rd(sc,cs);
	val1 = data1 & 0xff;
	val1 |= (data1 >> 8);
	data2 = umc_nand_word_rd(sc, cs);
	val2 = (data2 & 0xff) << 16;
	val2 |= ((data2 & 0xff0000) << 8);
	val = val1 | val2;

        dev_id = (val >> 8) & 0xff;
	sc->read_id = *read_id = val;

	/* set up addr cycles based on device */
	rand_op_addr_cyc = 2;	
	switch (dev_id) {
	case MICRON_1G_16BIT:
		rd_addr_cyc	= 4;
		prg_addr_cyc	= 4;
		pg_addr_shift	= 16;
		nand->options	|= NAND_BUSWIDTH_16;		
		break;

	case MICRON_2G_8BIT:
		printk("MICRON_2G_8BIT config param lee\n");
/*
		rd_addr_cyc	= 5;
		prg_addr_cyc	= 5;
		rand_op_addr_cyc = 5;	
		pg_addr_shift	= 12;
		dev_width = 8;		//add by lee
//		nand->options	|= NAND_BUSWIDTH_16; //del by lee		
*/
		rd_addr_cyc	= 5;
		prg_addr_cyc	= 5;
		pg_addr_shift	= 12;
		dev_width = 8;
		break;

		break;
	case STMICRO_2G_8BIT:
		rd_addr_cyc	= 5;
		prg_addr_cyc	= 5;
		pg_addr_shift	= 12;
		dev_width = 8;
		break;

	case STMICRO_1G_8BIT:
		rd_addr_cyc	= 4;
		prg_addr_cyc	= 4;
		dev_width = 8;
		break;

	case SAMSUNG_64MB_8BIT:
		rd_addr_cyc	= 4;
		prg_addr_cyc	= 4;
		pg_addr_shift	= 9;
		rand_op_addr_cyc = 4;
		dev_width = 8;
		break;

	case SAMSUNG_32MB_8BIT:
		rd_addr_cyc	= 3;
		prg_addr_cyc	= 3;
		pg_addr_shift	= 9;
		rand_op_addr_cyc = 3;
		dev_width = 8;
		break;
	default:
		printk(KERN_ERR "Unrecognized NAND device!! (0x%x)\n",  *read_id);
		break;
	}

	if(dev_width == 8) { /* reconfigure for 8-bit device */
		umc_memif_cfg(sc, TYPE_SRAM_NONMUXED,CHIPS_2, WIDTH_8, 
		      TYPE_NAND, CHIPS_1, WIDTH_8);
		umc_config_cs(sc, CHIP_SELECT,INTERFACE_1,0,0,0,0,WIDTH_8);
	}


}

/*---------------------------------------------------------------
 * Name    : cls_dmu_enable_block(uint32_t enable_mask)
 * Purpose : Sets power on enabling on a per-block basis
 *              (clears bit in dmu_pwd register)
 * Input   : bit vector for the blocks to be enabled
 * Return  : void.
 *--------------------------------------------------------------*/
static void  cls_dmu_block_enable(uint32_t enable)
{
	uint32_t num;

	if (enable < 32) {
		num = (enable & 0x1f);
		CPU_RMW_AND_SINGLE(IO_ADDRESS(DMU_R_dmu_pwd_blk1_MEMADDR),~(0x1<<num),  AHB_BIT32);
	} else if ((enable > 32) && (enable <64)) {
		num = (enable & 0x1f);
		CPU_RMW_AND_SINGLE(IO_ADDRESS(DMU_R_dmu_pwd_blk2_MEMADDR), ~(0x1<<num), AHB_BIT32);
	} else {
		printk(KERN_ERR "ERROR! Unknown block enable \n");
	}

	return;
}

// ----------------------------------------------------------------------------
// function to set the memif cfg register in umc
// ----------------------------------------------------------------------------
static void umc_memif_cfg(struct bcm5892_nand_softc *sc, uint8_t type0, uint8_t chip0, uint8_t width0, uint8_t type1, uint8_t chip1, uint8_t width1 )

{
	uint32_t val = 0;

	val = ( width1 << 12 ) |
		( chip1  << 10 ) |
		( type1  <<  8 ) |
		( width0 <<  4 ) |
		( chip0  <<  2 ) |
		( type0  <<  0 ) ;
	writel (val, sc->pUMC_R_umc_memif_cfg);
}

static int umc_config_cs_type(struct bcm5892_nand_softc *sc, uint8_t cs2_type, uint8_t cs1_type, uint8_t cs0_type)
{
	uint32_t val;
	uint8_t  typecs = 0;

	uint32_t addrmatch = 0x0;
	uint32_t base_cs0 = 0x0;
	uint32_t base_cs1 = 0x0;
	uint32_t base_cs2 = 0x0;
	
	base_cs0 = ( cs0_type == CS_NAND) ? START_NAND_CS0 : START_NOR_FLASH0;
	base_cs1 = ( cs1_type == CS_NAND) ? START_NAND_CS1 : START_NOR_FLASH1;
	base_cs2 = (u32) START_NOR_FLASH2;
	PDEBUG(" base_cs0=%x, base_cs1=%x,base_cs2=%x\n", base_cs0,base_cs1,base_cs2);
	
	addrmatch = (base_cs0 >> 24 ) |
                (base_cs1 >> 16 ) |
                (base_cs2 >>  8 ) ;
	
	PDEBUG("UMC : Address match = 0x%x\n",addrmatch);
	writel (addrmatch, sc->pUMC_R_umc_addr_match);
	writel (0xffffffff, sc->pUMC_R_umc_addr_mask);
	
	typecs = (cs2_type << 2) |
		(cs1_type << 1) |
		cs0_type;
	
	val = readl (sc->pUMC_R_umc_conf);
 
	// 19 : 16 When high, a corresponding SRAM/NOR chip-select is
	// configured into ADmux mode
	// Reset value is 0xf.
	// This has to be made 0 for claudius

	// +
	
	// 3:0 has to be changed to the input cs_type
	
	// To Overcome the Bug in UMC APB Interface Read
	
	val = 0x30ff440f;
	val = val & 0xfff0fff0;
	val = val | (typecs & 0xf);

	writel (val,sc->pUMC_R_umc_conf);
	if (val == readl (sc->pUMC_R_umc_conf))
		return CLS_STATUS_OK;
	else
		return CLS_STATUS_UMC_CONFIG_SET_ERROR;
}

// ----------------------------------------------------------------------------
// umc_ecc_config
//
// Arguments :
//            1 : ecc_jump
//            2 : ecc_read_end
//            3 : ecc_mode
//            4 : page size
// ----------------------------------------------------------------------------
static void umc_ecc_config(struct bcm5892_nand_softc *sc, uint8_t ecc_jump,
			  uint8_t ecc_read_end, uint8_t ecc_mode, uint8_t pagesize)
{
	uint32_t value ;
	uint8_t ecc_ignore_eight = 0;

	writel (ECC_COL_ADDR_MT, (u32)sc->pUMC_R_umc_ecc_addr0);

	/* 
	 * For devices with small page size, the ECC logic should use the first cycle 
	 * for column address.
	 */
	if(pagesize == ECC_PAGE_512) 
		ecc_ignore_eight = 1;

	value =
		(pagesize  << 11)   |
		(1 << 10)   |
		(ecc_ignore_eight << 7) |
		((ecc_jump & 0x3) << 5)     |
		((ecc_read_end & 0x1) << 4) |
		((ecc_mode & 0x3) << 2)     |
		(pagesize & 0x3) ;
	
	writel (value, sc->pUMC_R_umc_ecc_memcfg);
}


// ----------------------------------------------------------------------------
// function name   : umc_config_cs
// Return Value    : CLS_STATUS
// Arguments       : 1. cs_number  (0,1,2,3)
//                 : 2. interface number  (0 or 1) // Nand MUST only BE at Interface 1
//                 : 3. set_adv value
//                 : 3. sram sync mode (1)  or Async Mode (0)
//                 : 4. write burst length
//                 : 5. read burst length
//                 : 6. memory width
// Note            : set_cycles function should be called before this fucntion
// ----------------------------------------------------------------------------
static void umc_config_cs(struct bcm5892_nand_softc *sc, uint8_t cs,
                         uint8_t interface, uint8_t set_adv, uint8_t set_sync_mode,
                         uint8_t wr_bl, uint8_t rd_bl, uint8_t width) 
{

	uint32_t val;

	val = readl (sc->pUMC_R_umc_set_opmode);
	
	
	val &= 0xfffff000;

	val =  val | (set_adv << 11)
		| ((wr_bl & 0x7) << 7)
		| (set_sync_mode << 6)
		| ((rd_bl & 0x7) << 3)
		| (set_sync_mode << 2)
		| (width & 0x3) ;
	
	writel (val, sc->pUMC_R_umc_set_opmode);
	
	
	// Direct Command
	// 25:23  :: 2 : CS2 on interface 0
	// 22:21  :: 2 : Update Regs
	//    20  :: 0 : Set cre 0
	// 19:00  :: 0 : dont care
	
	PDEBUG("cs=%x,interface=%x\n",cs, interface);
   
	val = ((cs & 0x3) + ( interface * 4)) <<  23;
	val = val | (2 << 21);  // Update Reg Command
	val = val & 0xffefffff; // Bit 20 as ZERO :set_cre
	writel (val, sc->pUMC_R_umc_direct_cmd);
}

static CLS_STATUS sv_umc_nand_set_cycles(struct bcm5892_nand_softc *sc,  uint8_t t_RR, uint8_t t_TR, uint8_t t_PC, uint8_t t_WP, uint8_t t_CEOE, uint8_t t_WC, uint8_t t_RC)
{
	uint32_t val = 0;

	val = ( ((t_RR&T_TR_MASK)<<20) |
		((t_TR&T_TR_MASK)<<UMC_F_t_tr_3_R) |
		((t_PC&T_PC_MASK)<<UMC_F_t_pc_3_R) |
		((t_WP&T_WP_MASK)<<UMC_F_t_wp_3_R) |
		((t_CEOE&T_CEOE_MASK)<<UMC_F_t_ceoe_3_R) |
		((t_WC&T_WC_MASK)<<UMC_F_t_wc_0_R) |
		((t_RC&T_RC_MASK)<<UMC_F_t_rc_1_R)
		);

	writel (val, sc->pUMC_R_umc_set_cycles);

	return CLS_STATUS_OK;
}

// ----------------------------------------------------------------------------
// nand page read end command
// ----------------------------------------------------------------------------
static uint32_t umc_nand_page_rd_end( struct bcm5892_nand_softc *sc, uint32_t cs ) {
	uint32_t addr;

	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);
	addr = addr | 0x00298400;
	return  readl (addr);
}

// ----------------------------------------------------------------------------
// nand random read command
// ----------------------------------------------------------------------------
static void  umc_random_data_rd_cmd( struct bcm5892_nand_softc * sc,  uint32_t cs, uint32_t address, uint8_t width, uint8_t no_addr_cycles ) {
	uint8_t st_cmd;
	uint8_t end_cmd;
	uint32_t addr;
	uint32_t device_addr1 = address;
	
	uint8_t  dev_ad_byte[4];
	
	dev_ad_byte[0] = ( width == 8) ? (address      ) & 0xff : (address >>  1) & 0xff; // CA[7:0]
	dev_ad_byte[1] = ( width == 8) ? (address >>  8) & 0x0f : (address >>  9) & 0x07; // CA[10:8]
	dev_ad_byte[2] = ( width == 8) ? (address >> 16) & 0xff : (address >> 16) & 0xff; // {BA[1:0],PA[5:0]}
	dev_ad_byte[3] = ( width == 8) ? (address >> 24) & 0xff : (address >> 24) & 0xff;
	dev_ad_byte[4] = ( width == 8) ? (address >> 28) & 0x03 : (address >> 26) & 0x03;

	device_addr1 = (dev_ad_byte[3] << 24) |
		(dev_ad_byte[2] << 16) |
		(dev_ad_byte[1] <<  8) |
		(dev_ad_byte[0]      ) ;

	st_cmd  = NAND_RAND_RD_S_CMD;
	end_cmd = NAND_RAND_RD_E_CMD;
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);   
	addr = addr | (st_cmd << 3)
		| (end_cmd << 11)
		| ( no_addr_cycles << 21)
		| ( 1 << 20)              //  End Command Valid = 1
		;
	
	writel (device_addr1, addr);
}

static void  umc_smallnand_rand_rd_cmd( struct bcm5892_nand_softc * sc,  
					       uint32_t cs, uint32_t address, uint8_t width,
					       uint8_t no_addr_cycles ) 
{
	uint8_t st_cmd;
	uint32_t addr;
	uint32_t device_addr1 = address;
	uint32_t device_addr2 = 0x0;
	uint8_t  dev_ad_byte[5];
    

	dev_ad_byte[0] = ( width == 8) ? (address      ) & 0xff : (address >>  1) & 0xff; // CA[7:0]
	dev_ad_byte[1] = ( width == 8) ? (address >>  9) & 0xff : (address >>  9) & 0x07; // CA[10:8]
	dev_ad_byte[2] = ( width == 8) ? (address >> 17) & 0xff : (address >> 16) & 0xff; // {BA[1:0],PA[5:0]}
	dev_ad_byte[3] = ( width == 8) ? (address >> 25) & 0x01 : (address >> 24) & 0xff;
	
	
	device_addr1 = (dev_ad_byte[3] << 24) |
		(dev_ad_byte[2] << 16) |
		(dev_ad_byte[1] <<  8) |
		(dev_ad_byte[0]      ) ;
	
	device_addr2 = 0x0 ;

	st_cmd = 0x50;	
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);
	addr = addr | (st_cmd << 3)
                | ( no_addr_cycles << 21)
		| ( 0 << 20);              //  End Command Valid = 0


	PDEBUG ("NAND Page Read Command Issue : 0x%x : 0x%x 0x%x\n", addr,device_addr1, device_addr2);

	writel (device_addr1, addr);
	if (no_addr_cycles == 5)
		writel(device_addr2, addr);

}

static void umc_smallnand_page_rd_cmd( struct bcm5892_nand_softc *sc, uint32_t cs, 
				       uint32_t address, uint8_t width, uint8_t no_addr_cycles ) 
{
	uint8_t st_cmd;
	uint32_t addr;
	uint32_t device_addr1 = address;
	uint32_t device_addr2 = 0x0;
	uint8_t  dev_ad_byte[5];
    
	st_cmd = ( address & 0x100 ) ? 0x01 : 0x00;
	
	dev_ad_byte[0] = ( width == 8) ? (address      ) & 0xff : (address >>  1) & 0xff; // CA[7:0]
	dev_ad_byte[1] = ( width == 8) ? (address >>  9) & 0xff : (address >>  9) & 0x07; // CA[10:8]
	dev_ad_byte[2] = ( width == 8) ? (address >> 17) & 0xff : (address >> 16) & 0xff; // {BA[1:0],PA[5:0]}
	dev_ad_byte[3] = ( width == 8) ? (address >> 25) & 0x01 : (address >> 24) & 0xff;
	
	
	device_addr1 = (dev_ad_byte[3] << 24) |
		(dev_ad_byte[2] << 16) |
		(dev_ad_byte[1] <<  8) |
		(dev_ad_byte[0]      ) ;
	
	device_addr2 = 0x0 ;
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);
	addr = addr | (st_cmd << 3) 
		| (0 << 11)
                | ( no_addr_cycles << 21)
		| ( 0 << 20);              //  End Command Valid = 0


	PDEBUG ("NAND Page Read Command Issue : 0x%x : 0x%x 0x%x\n", address,device_addr1, device_addr2);

	writel (device_addr1, addr);
	if (no_addr_cycles == 5)
		writel (device_addr2, addr);

}

// ----------------------------------------------------------------------------
// nand page read start
// ----------------------------------------------------------------------------
static void umc_nand_page_rd_cmd( struct bcm5892_nand_softc *sc, uint32_t cs, 
				  uint32_t address, uint8_t width, uint8_t no_addr_cycles ) 
{
	uint8_t st_cmd;
	uint8_t end_cmd;
	uint32_t addr;
	uint32_t device_addr1 = address;
	uint32_t device_addr2 = 0x0;
	uint8_t  dev_ad_byte[5] = { 0 };
	uint32_t manuf_id = sc->read_id  & 0xff;

	if (manuf_id == 0x2C) { // for Micron 1Gb
		dev_ad_byte[0] = ( width == 8) ? 
			(address) & 0xff : (address >>  1) & 0xff; // CA[7:0]
		dev_ad_byte[1] = ( width == 8) ? 
			(address >>  8) & 0x0f : (address >>  9) & 0x07; // CA[10:8]
		dev_ad_byte[2] = ( width == 8) ? 
			(address >> 16) & 0xff : (address >> 16) & 0xff; // {BA[1:0],PA[5:0]}
		dev_ad_byte[3] = ( width == 8) ? 
			(address >> 24) & 0xff : (address >> 24) & 0xff;
		dev_ad_byte[4] = ( width == 8) ? 
			(address >> 28) & 0x03 : (address >> 26) & 0x03;
	} else if (manuf_id == 0x20) { // STMicro 
		dev_ad_byte[0] = ( width == 8) ? 
			(address) & 0xff : (address >>  1) & 0xff; // CA[7:0]
		dev_ad_byte[1] = ( width == 8) ? 
			(address >>  8) & 0x0f : (address >>  9) & 0x07; // CA[10:8]
		dev_ad_byte[2] = ( width == 8) ? 
			(address >> 12) & 0xff : (address >> 12) & 0xff; // {BA[1:0],PA[5:0]}
		dev_ad_byte[3] = ( width == 8) ? 
			(address >> 20) & 0xff : (address >> 20) & 0xff;
		dev_ad_byte[4] = ( width == 8) ? 
			(address >> 28) & 0x01 : (address >> 28) & 0x03;

	}else if(manuf_id == 0xda){ //for Micron 2Gb    add by lee

          	dev_ad_byte[0] = ( width == 8) ? (address) & 0xff :
                               (address >>  1) & 0xff; // CA[7:0]
          	dev_ad_byte[1] = ( width == 8) ? (address >>  8) & 0x0f :
                               (address >>  9) & 0x07; // CA[10:8]
          	dev_ad_byte[2] = ( width == 8) ? (address >> 12) & 0xff :
                               (address >> 12) & 0xff; // {BA[1:0],PA[5:0]}
          	dev_ad_byte[3] = ( width == 8) ? (address >> 20) & 0xff :
                               (address >> 20) & 0xff;
          	dev_ad_byte[4] = ( width == 8) ? (address >> 28) & 0x03 :
                               (address >> 28) & 0x03;
		printk("manuf_id==0xda read  add by lee\n");

/*
		dev_ad_byte[0] = ( width == 8) ? 
			(address) & 0xff : (address >>  1) & 0xff; // CA[7:0]
		dev_ad_byte[1] = ( width == 8) ? 
			(address >>  8) & 0x0f : (address >>  9) & 0x07; // CA[10:8]
		dev_ad_byte[2] = ( width == 8) ? 
			(address >> 12) & 0xff : (address >> 12) & 0xff; // {BA[1:0],PA[5:0]}
		dev_ad_byte[3] = ( width == 8) ? 
			(address >> 20) & 0xff : (address >> 20) & 0xff;
		dev_ad_byte[4] = ( width == 8) ? 
			(address >> 28) & 0x01 : (address >> 28) & 0x03;

*/
        }


	device_addr1 = (dev_ad_byte[3] << 24) |
		(dev_ad_byte[2] << 16) |
		(dev_ad_byte[1] <<  8) |
		(dev_ad_byte[0]      ) ;
	
	device_addr2 = 0x0 | dev_ad_byte[4];
	
	st_cmd  = NAND_PAGE_RD_S_CMD;
	end_cmd = NAND_PAGE_RD_E_CMD;
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);
	
	addr = addr | (st_cmd << 3)
                | (end_cmd << 11)
                | ( no_addr_cycles << 21)
                | ( 1 << 20)              //  End Command Valid = 1
		;
	
	writel (device_addr1,addr);

	PDEBUG ("NAND Page Read Command Issue :  0x%x : 0x%x 0x%x\n", address,device_addr1, device_addr2);
//	printk ("NAND Page Read Command Issue :  0x%x : 0x%x 0x%x\n", address,device_addr1, device_addr2);

	if (no_addr_cycles == 5)
		writel (device_addr2, addr);
	
}
				  
// ----------------------------------------------------------------------------
// nand page read
// ----------------------------------------------------------------------------
static void umc_nand_page_rd(struct bcm5892_nand_softc *sc, uint32_t cs,
				   uint32_t page_addr, uint8_t width, 
				   uint8_t no_addr_cycles ,uint8_t* dst_addr, 
				   uint32_t bytesize) 
{
	int i=0;
	uint32_t  data,dst_ptr;
	uint32_t timeout;
	uint32_t num_of_words =0;

	dst_ptr = (uint32_t) dst_addr;
	umc_nand_page_rd_cmd(sc, cs, page_addr, width, no_addr_cycles);

	timeout = 0x3000;
#define INT_STATUS ((*(volatile uint32_t *)sc->pUMC_R_umc_status_reg)&(1<<6))

	while( (!INT_STATUS) && (timeout !=0x1))
	{
		timeout--;
	}		

	if(!INT_STATUS)
		printk(KERN_ERR "%s: %d Critical Error: HW Read operation FAILED!!!\n",
		       __func__, __LINE__);

//	printk("---------------1----------------------------\n");
	num_of_words = bytesize/4;
	for ( i = 0 ; i <  num_of_words; i++) 
	{
		data = umc_nand_word_rd(sc,cs);
	//	printk("0000111000\n");
		*((uint32_t*)dst_ptr) = data;
		dst_ptr += 4;

	}

//	printk("---------------2----------------------------\n");
	data = umc_nand_page_rd_end( sc, cs);
	writel ( data, dst_ptr);
//	printk("--------------------------------------------\n");
}

// ----------------------------------------------------------------------------
// nand page program start command
// ----------------------------------------------------------------------------
static void umc_nand_page_prg_start( struct bcm5892_nand_softc *sc, 
					   uint32_t cs, uint32_t address, uint8_t width,
					   uint8_t no_addr_cycles ) 
{
	uint8_t st_cmd;
	uint8_t end_cmd;
	uint32_t addr;
	uint32_t device_addr1 = address;
	uint32_t device_addr2 = 0x0;
	uint8_t  dev_ad_byte[5] = { 0 };
	uint32_t manuf_id = sc->read_id  & 0xff;

	if (manuf_id == 0x20) { // STMicro  
		dev_ad_byte[0] = ( width == 8) ? (address) & 0xff : 
			(address >>  1) & 0xff; // CA[7:0]
		dev_ad_byte[1] = ( width == 8) ? (address >>  8) & 0x0f : 
			(address >>  9) & 0x07; // CA[10:8]
		dev_ad_byte[2] = ( width == 8) ? (address >> 12) & 0xff : 
			(address >> 12) & 0xff; // {BA[1:0],PA[5:0]}
		dev_ad_byte[3] = ( width == 8) ? (address >> 20) & 0xff : 
			(address >> 20) & 0xff;
		dev_ad_byte[4] = (width == 8) ? (address >> 28) & 0x01 : 
					(address >> 28) & 0x03;

	} else if (manuf_id == 0x2C) { // Micron 
		dev_ad_byte[0] = ( width == 8) ? (address      ) & 0xff : 
			(address >>  1) & 0xff; // CA[7:0]
		dev_ad_byte[1] = ( width == 8) ? (address >>  8) & 0x0f : 
			(address >>  9) & 0x07; // CA[10:8]
		dev_ad_byte[2] = ( width == 8) ? (address >> 16) & 0xff : 
			(address >> 16) & 0xff; // {BA[1:0],PA[5:0]}
		dev_ad_byte[3] = ( width == 8) ? (address >> 24) & 0xff : 
			(address >> 24) & 0xff;
		dev_ad_byte[4] = ( width == 8) ? (address >> 28) & 0x03 : 
			(address >> 26) & 0x03;
	}else if(manuf_id == 0xda){// Micron  2G       //add by lee

        	dev_ad_byte[0] = ( width == 8) ? (address) & 0xff :     //
                           (address >>  1) & 0xff; // CA[7:0]
        	dev_ad_byte[1] = ( width == 8) ? (address >>  8) & 0x0f :  //
                           (address >>  9) & 0x07; // CA[10:8]
        	dev_ad_byte[2] = ( width == 8) ? (address >> 12) & 0xff :
                           (address >> 12) & 0xff; // {BA[1:0],PA[5:0]}
        	dev_ad_byte[3] = ( width == 8) ? (address >> 20) & 0xff :
                           (address >> 20) & 0xff;
        	dev_ad_byte[4] = ( width == 8) ? (address >> 28) & 0x03 :
                           (address >> 28) & 0x03;

/*
		dev_ad_byte[0] = ( width == 8) ? (address) & 0xff : 
			(address >>  1) & 0xff; // CA[7:0]
		dev_ad_byte[1] = ( width == 8) ? (address >>  8) & 0x0f : 
			(address >>  9) & 0x07; // CA[10:8]
		dev_ad_byte[2] = ( width == 8) ? (address >> 12) & 0xff : 
			(address >> 12) & 0xff; // {BA[1:0],PA[5:0]}
		dev_ad_byte[3] = ( width == 8) ? (address >> 20) & 0xff : 
			(address >> 20) & 0xff;
		dev_ad_byte[4] = (width == 8) ? (address >> 28) & 0x01 : 
					(address >> 28) & 0x03;
*/
        }

	device_addr1 = (dev_ad_byte[3] << 24) |
		(dev_ad_byte[2] << 16) |
		(dev_ad_byte[1] <<  8) |
		(dev_ad_byte[0]      ) ;
	
	PDEBUG("DEVICE ADDR1:-%x\n",device_addr1);

	device_addr2 = 0x0 | dev_ad_byte[4];
	st_cmd  = NAND_PAGE_PRG_S_CMD;
	end_cmd = NAND_PAGE_PRG_E_CMD;
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);
	addr = addr | (st_cmd << 3)
		| (end_cmd << 11)
		| ( no_addr_cycles << 21) 
		;
	/*Need to check this line*/
	writel (device_addr1, addr);
	if (no_addr_cycles == 5 ) 
		writel (device_addr2, addr);

}

static void umc_nand_word_wr(  struct bcm5892_nand_softc *sc, uint8_t cs, 
			       uint32_t data) 
{
	uint32_t addr;
	uint8_t  no_addr_cycles = 0x1;
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);
	addr = addr  | (no_addr_cycles << 21)
		| ( 1 << 19)     // Data Phase
		;
	writel (data, addr);
}

static void umc_nand_byte_wr(struct bcm5892_nand_softc *sc, uint8_t cs, 
			     uint8_t data) 
{
	uint32_t addr;
	uint8_t  no_addr_cycles = 0x1;
	
	addr = (u32) (cs ? sc->nand_cs1_base : sc->nand_cs0_base);
	addr = addr | ( no_addr_cycles << 21)
                | ( 1 << 19)     // Data Phase
		;
	writeb (data,addr);
}

static void umc_nand_page_prg_end(struct bcm5892_nand_softc *sc, uint32_t cs, uint32_t data) 
{
	uint32_t addr;
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);
	addr = addr | 0x00388400;
	writel (data, addr);
}

static void umc_smallnand_page_prg_start(struct bcm5892_nand_softc *sc, uint32_t cs, 
					 uint32_t address, uint8_t width, 
					 uint8_t no_addr_cycles ) 
{
	uint8_t st_cmd;
	uint8_t end_cmd;
	uint32_t addr;
	uint32_t device_addr1 = address;
	uint32_t device_addr2 = 0x0;
	uint8_t  dev_ad_byte[5];
	
	dev_ad_byte[0] = ( width == 8) ? (address      ) & 0xff : (address >>  1) & 0xff; // CA[7:0]
	dev_ad_byte[1] = ( width == 8) ? (address >>  9) & 0xff : (address >>  9) & 0x07; // CA[10:8]
	dev_ad_byte[2] = ( width == 8) ? (address >> 17) & 0xff : (address >> 12) & 0xff; // {BA[1:0],PA[5:0]}
	dev_ad_byte[3] = ( width == 8) ? (address >> 25) & 0x01: (address >> 20) & 0xff;
	dev_ad_byte[4] = ( width == 8) ? (address >> 28) & 0x03 : (address >> 28) & 0x03;
    

	device_addr1 = (dev_ad_byte[3] << 24) |
		(dev_ad_byte[2] << 16) |
		(dev_ad_byte[1] <<  8) |
		(dev_ad_byte[0]      ) ;
	device_addr2 = 0x0 | dev_ad_byte[4];

	/* 
	 * we have to do this for first page only, but since we are in a loop,
	 * i can't figure out which page is first page.
	 */
	addr = (u32) (cs ? sc->nand_cs1_base: sc->nand_cs0_base);
	st_cmd = 0x00;
	addr = addr | (st_cmd << 3) 
		| (0 << 20)
		| (0 << 11);
	writel (0, addr);

	st_cmd  = NAND_PAGE_PRG_S_CMD;
	end_cmd = NAND_PAGE_PRG_E_CMD;
	addr = (u32) (cs ? sc->nand_cs1_base: sc->nand_cs0_base);
	addr = addr | (st_cmd << 3) 
	  | (end_cmd << 11)
	  | ( no_addr_cycles << 21);
	  
	writel (device_addr1, addr);

	if (no_addr_cycles == 5) 
		writel (device_addr2, addr);

}

static void umc_smallnand_page_prg(struct bcm5892_nand_softc *sc, uint32_t cs, uint32_t page_addr, 
				   uint8_t width, uint8_t no_addr_cycles ,
				   uint8_t* src_addr, uint32_t bytesize) 
{
	int i;
	uint32_t src_ptr, data;
     
	src_ptr = (uint32_t) src_addr;
	umc_smallnand_page_prg_start(sc, cs, page_addr, width, no_addr_cycles);
	src_ptr = (uint32_t)(src_addr);
	for ( i = 1 ; i <  (bytesize / 4 ); i++) {
		
		data = readl (src_ptr);
	
		src_ptr += 4;
		umc_nand_word_wr(sc,cs, data);
	}
	data =  readl (src_ptr);
	umc_nand_page_prg_end(sc,cs, data);
}

static void umc_nand_status_cmd( struct bcm5892_nand_softc *sc, uint8_t cs)
{
	uint32_t addr ;
	uint8_t st_cmd;
	uint8_t end_cmd;
	uint8_t  no_addr_cycles = 0;

	st_cmd = 0x70; // Command to Check Busy status
	end_cmd = 0 ;

	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);
	addr = addr | ( st_cmd << 3   )
		| ( no_addr_cycles << 21 )
		;
	writel(0, addr);
	udelay(30);
}

static int block_checkbad(struct mtd_info *mtd, loff_t off)
{
	struct nand_chip *chip;
	u16 bad;
	int ret = 0;;
	
	chip = mtd->priv;
	if (!chip->bbt){ 	/* If no BBT */
		int page, column;

		page = (int)(off >> chip->page_shift) & chip->pagemask;
		column =  chip->badblockpos;

		if (chip->options & NAND_BUSWIDTH_16) {
			printk("chip->cmdfunc(mtd, NAND_CMD_READOOB, chip->badblockpos & 0xFE,page)\n");
			chip->cmdfunc(mtd, NAND_CMD_READOOB, chip->badblockpos & 0xFE,
				      page);
			bad = chip->read_word(mtd);
			if (chip->badblockpos & 0x1)
				bad >>= 8;
			if ((bad & 0xFF) != 0xff)
				ret = 1;
		} else {
			printk("chip->cmdfunc(mtd, NAND_CMD_READOOB, chip->badblockpos, page)\n");
			chip->cmdfunc(mtd, NAND_CMD_READOOB, chip->badblockpos, page);
			if (chip->read_byte(mtd) != 0xff)
				ret = 1;
		}

		goto out;
	}

	ret = nand_isbad_bbt(mtd, off, 1);

out:	return ret;

}		

static int bcm_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int start_blk;
	int end_blk;
	int page, pages_per_block;
	int ret;
	uint32_t size = instr->len;
//	loff_t offset;
	uint32_t offset = instr->addr;

	struct bcm5892_nand_softc *sc;
	struct nand_chip *chip = mtd->priv;

	/* Start address must align on block boundary */
	if (instr->addr & ((1 << chip->phys_erase_shift) - 1)) {
		DEBUG(MTD_DEBUG_LEVEL0, "nand_erase: Unaligned address\n");
		return -EINVAL;
	}

	/* Length must align on block boundary */
	if (instr->len & ((1 << chip->phys_erase_shift) - 1)) {
		DEBUG(MTD_DEBUG_LEVEL0, "nand_erase: "
		      "Length not block aligned\n");
		return -EINVAL;
	}

	/* Do not allow erase past end of device */
	if ((offset + size) > mtd->size) {
		PDEBUG( "nand_erase: Erase past end of device\n");
		return -EINVAL;
	}

	sc = container_of(mtd, struct bcm5892_nand_softc, bcm5892_mtd_info);

	/* Shift to get first page */
	page = (int)(instr->addr >> chip->page_shift);

	/* Calculate pages in each block */
	pages_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);

	/* Check, if it is write protected */
	umc_nand_status_cmd(GET_SC(mtd), CHIP_SEL);
	if (! (chip->read_byte(mtd) & NAND_STATUS_WP)) {
		DEBUG(MTD_DEBUG_LEVEL0, "nand_erase: "
		      "Device is write protected!!!\n");
		instr->state = MTD_ERASE_FAILED;
		goto erase_exit;
	}

	/*
	 * Invalidate the page cache, if we erase the block which
	 * contains the current cached page
	 */
	if (page <= chip->pagebuf && chip->pagebuf <
	    (page + pages_per_block))
		chip->pagebuf = -1;

	instr->fail_addr = 0xffffffff;
	instr->state = MTD_ERASING;

	/* get block number */
	start_blk = offset >> chip->phys_erase_shift ;
	end_blk = (offset + size) >> chip->phys_erase_shift;
	if (offset & ((1 << chip->phys_erase_shift) -1))
		end_blk++;

	PDEBUG("offset = %x, size=%x, start_blk= %x, end_blk=%x\n", offset, size, 
	       start_blk, end_blk);

	do
	  {
		  /*
		   *  we do not erase bad blocks!
		   */
		  if (block_checkbad(mtd, offset)) {
			  printk(KERN_WARNING "bcm_nand_erase: "
				 "attempt to erase bad block  0x%08x offset:%0x8 \n", 
				 start_blk, offset);
			  instr->state = MTD_ERASE_FAILED;
			  goto erase_exit;
		  }

		  if ((sc->read_id & 0xff) == 0xEC) 
			  /* samsung */
			  umc_smallnand_erase (sc, CHIP_SEL, dev_width, offset);
		  else 
			  /* for Micron 1Gbx16, Numonyx */
			  umc_nand_blk_erase(sc,CHIP_SELECT, start_blk <<6);  // for Micron 1Gbx16, Nomonyx
		  mdelay(5);
		  chip->waitfunc(mtd, chip);
		  start_blk++;
		  offset += (1 << chip->phys_erase_shift);
	  } while(start_blk < end_blk);

	/* success */
	instr->state = MTD_ERASE_DONE;

erase_exit:
	ret = instr->state == MTD_ERASE_DONE ? 0 : -EIO;	
	/* Do call back function */
	if (!ret)
		bcm5892_erase_callback (instr);

	return ret;
}

static void  umc_random_data_wr(uint32_t cs, uint32_t col_addr, uint8_t width, 
				uint8_t no_addr_cycles, uint8_t* src_addr, 
				uint32_t bytesize, struct bcm5892_nand_softc *sc ) ;

static void bcm5892_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	return;

}

static void umc_smallnand_rand_data_wr_cmd( struct bcm5892_nand_softc *sc, 
					  uint32_t cs, uint32_t address, 
					  uint8_t width, uint8_t no_addr_cycles ) 
{
	uint8_t st_cmd;
	uint8_t end_cmd;
	uint32_t addr;
	uint32_t device_addr1 = address;
	
	uint8_t  dev_ad_byte[4];

	dev_ad_byte[0] = ( width == 8) ? (address      ) & 0xff : (address >>  1) & 0xff; // CA[7:0]
	dev_ad_byte[1] = ( width == 8) ? (address >>  8) & 0x0f : (address >>  9) & 0x07; // CA[10:8]
	dev_ad_byte[2] = ( width == 8) ? (address >> 16) & 0xff : (address >> 16) & 0xff; // {BA[1:0],PA[5:0]}
	dev_ad_byte[3] = ( width == 8) ? (address >> 24) & 0xff : (address >> 24) & 0xff;
	dev_ad_byte[4] = ( width == 8) ? (address >> 28) & 0x03 : (address >> 26) & 0x03;
    
	device_addr1 = (dev_ad_byte[3] << 24) |
		(dev_ad_byte[2] << 16) |
		(dev_ad_byte[1] <<  8) |
		(dev_ad_byte[0]      ) ;
	
	st_cmd  = NAND_RAND_WR_S_CMD;
	end_cmd = 0x0;
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);  	 
	addr = addr | (st_cmd << 3)
                | (end_cmd << 11)
                | ( no_addr_cycles << 21)
                | ( 0 << 20)              //  End Command Valid = 0
		;

	writel (device_addr1, addr);
}

// ----------------------------------------------------------------------------
// nand random program command
// ----------------------------------------------------------------------------
 static void umc_nand_random_data_wr_cmd( struct bcm5892_nand_softc *sc, 
					  uint32_t cs, uint32_t address, 
					  uint8_t width, uint8_t no_addr_cycles ) 
{
	uint8_t st_cmd;
	uint8_t end_cmd;
	uint32_t addr;
	uint32_t device_addr1 = address;
	
	uint8_t  dev_ad_byte[4];

	dev_ad_byte[0] = ( width == 8) ? (address      ) & 0xff : (address >>  1) & 0xff; // CA[7:0]
	dev_ad_byte[1] = ( width == 8) ? (address >>  8) & 0x0f : (address >>  9) & 0x07; // CA[10:8]
	dev_ad_byte[2] = ( width == 8) ? (address >> 16) & 0xff : (address >> 16) & 0xff; // {BA[1:0],PA[5:0]}
	dev_ad_byte[3] = ( width == 8) ? (address >> 24) & 0xff : (address >> 24) & 0xff;
	dev_ad_byte[4] = ( width == 8) ? (address >> 28) & 0x03 : (address >> 26) & 0x03;
    
	device_addr1 = (dev_ad_byte[3] << 24) |
		(dev_ad_byte[2] << 16) |
		(dev_ad_byte[1] <<  8) |
		(dev_ad_byte[0]      ) ;
	
	st_cmd  = NAND_RAND_WR_S_CMD;
	end_cmd = 0x0;
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);  	 
	addr = addr | (st_cmd << 3)
                | (end_cmd << 11)
                | ( no_addr_cycles << 21)
                | ( 0 << 20)              //  End Command Valid = 0
		;

	writel (device_addr1, addr);
	
}

// ----------------------------------------------------------------------------
// nand random data write
// ----------------------------------------------------------------------------
static void  umc_random_data_wr(uint32_t cs, uint32_t col_addr, uint8_t width, 
				uint8_t no_addr_cycles, uint8_t* src_addr, 
				uint32_t bytesize, struct bcm5892_nand_softc *sc ) 
{
	int i;
	uint32_t src_ptr, data;
	int no_of_four, not_four_bytes;
	uint8_t *byte_ptr;
	
	src_ptr = (uint32_t) src_addr;

	if (bytesize == 16)
		umc_smallnand_rand_data_wr_cmd (sc, CHIP_SEL, col_addr, dev_width,
						no_addr_cycles);
	else
		umc_nand_random_data_wr_cmd(sc, CHIP_SEL, col_addr, dev_width , 
					    no_addr_cycles);
	
	no_of_four = (bytesize / 4);
	not_four_bytes = bytesize % 4;
	for ( i = 0 ; i <  (no_of_four - 1); i++) {
		data = readl (src_ptr);
		src_ptr += 4;
		umc_nand_word_wr(sc,cs, data);
	}

	if ( not_four_bytes) {
		byte_ptr = src_addr;
		for (i=0; i< not_four_bytes;  i++) {
			umc_nand_byte_wr(sc, cs, *byte_ptr++);
		}
	}

	data =  readl (src_ptr);
	umc_nand_page_prg_end(sc, CHIP_SEL, data);
}

static void bcm5892_select_chip(struct mtd_info *mtd, int chipnr)
{
	/* do nothing for now */
	   
}

static int bcm5892_calculate (struct mtd_info *mtd, const uint8_t *dat,
			      uint8_t *ecc_code)
{
	return 0;
}

static int bcm5892_correct (struct mtd_info *mtd, uint8_t *dat, uint8_t *read_ecc,
			    uint8_t *calc_ecc)
{
	return 0;
}

static void bcm5892_hwctl (struct mtd_info *mtd, int mode)
{
	return;
}

static u16 bcm5892_read_word(struct mtd_info *mtd)
{
	uint32_t addr;
	int cs = 1;
	struct bcm5892_nand_softc *sc = GET_SC(mtd);
	int no_addr_cycles = 1;
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);  	 

	addr = addr | ( no_addr_cycles << 21)
		| ( NAND_PAGE_RD_E_CMD << 11)
		| ( 1 << 19)     // Data Phase
		;

	return readw(addr);
}

static uint8_t bcm5892_read_byte(struct mtd_info *mtd)
{
	uint32_t addr;
	int cs = 1;
	struct bcm5892_nand_softc *sc = GET_SC(mtd);
	
	addr = (u32)(cs ? sc->nand_cs1_base:sc->nand_cs0_base);  	 

	addr = addr | ( 1 << 21)
		| ( 1 << 19)     // Data Phase
		;

	return (uint8_t) readw(addr);
}

static void bcm5892_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int  i;
	uint32_t dst_ptr,data;

	dst_ptr = (uint32_t) buf;
	for ( i = 0 ; i <  (len / 4 ); i++) {
		data = umc_nand_word_rd (GET_SC(mtd), CHIP_SEL);
		*((uint32_t*)dst_ptr) = data;
		dst_ptr += 4;
	}
}	

static void bcm5892_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	int i;
	uint32_t data;
	struct bcm5892_nand_softc *sc = GET_SC(mtd);
	
	for ( i = 0 ; i < (len / 4 ); i++) {
		
		data =  *((uint32_t*)buf);
		buf += 4;
		umc_nand_word_wr (sc, CHIP_SEL, data);
	}
}
	
static int bcm5892_wait(struct mtd_info *mtd, struct nand_chip *chip)
{
	uint32_t addr ;
	uint8_t st_cmd, cs = 1;
	uint8_t end_cmd;
	uint8_t  no_addr_cycles = 0;
	uint32_t status = 0;

	st_cmd = 0x70; // Command to Check Busy status
	end_cmd = 0 ;

	addr = (u32)(cs ? GET_SC(mtd)->nand_cs1_base:GET_SC(mtd)->nand_cs0_base);
	addr = addr | ( st_cmd << 3   )
		| ( no_addr_cycles << 21 )
		;
	do {
		writel (0, addr);
		status = chip->read_byte(mtd);
	} while (status & 0x1) ;

	return status;
}

static int bcm5892_write_oob(struct mtd_info *mtd, 
			       struct nand_chip *chip, int page)
{
	uint8_t *oob_buf = chip->oob_poi;
	int oob_len = mtd->oobsize;
	int page_sz = mtd->writesize;

	if (mtd->writesize == 512) {
		uint8_t *buf = kmalloc (page_sz + oob_len, GFP_KERNEL);
		memset (buf, 0xff, page_sz);
		memcpy (buf + page_sz, oob_buf, oob_len );
		umc_smallnand_page_prg (GET_SC(mtd), CHIP_SEL, PAGE_ADDRESS(page),
					dev_width, prg_addr_cyc, buf, page_sz + oob_len);
		kfree (buf);
		udelay(700);

	} else {
		umc_nand_page_prg_start (GET_SC(mtd), CHIP_SEL, PAGE_ADDRESS(page), 
					 dev_width,  prg_addr_cyc);
		umc_random_data_wr (CHIP_SEL, page_sz, dev_width, rand_op_addr_cyc, oob_buf,
				    oob_len,  GET_SC(mtd));
	}

	return 0;
}

/* read and write routines for HW ecc 
   for 512 byte page , ECC is stored at offset 8 in 16-byte OOB
   for 2K byte page, ECC is stored at offset 32 in 64-byte OOB. 
   that's why the mtd->oobsize/2 below. 
*/
static int bcm5892_read_page_hwecc (struct mtd_info *mtd, struct nand_chip *chip,
				    uint8_t *buf)
{
	int i;
	int eccsteps = chip->ecc.steps;
	uint32_t  ecc_val, ecc_reg;
	uint32_t  ecc_val9, ecc_reg9;

	chip->read_buf(mtd, buf, mtd->writesize);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize/2); 

	/* correction code */
	if (mtd->writesize == 512)
		ecc_reg  = (uint32_t) (GET_SC(mtd)->pUMC_R_umc_ecc_value0);
	else
		ecc_reg  = (uint32_t) (GET_SC(mtd)->pUMC_R_umc_ecc_value9);
	for (i = 0; i < eccsteps;  i++) {
		ecc_val = readl (ecc_reg);

		if (ecc_val & ECC_FAIL) {
			printk (KERN_INFO "bcm5892_read_page_hwecc: Correctable 0x%08x\n",
				(uint32_t)(ecc_val & ECC_CORRECTABLE));
			if (ecc_val & ECC_CORRECTABLE) {
				int bytepos, bitpos;
				uint32_t src_addr, src_data;

				bitpos = ecc_val & 0x7; 
				bytepos = (ecc_val >> 3) & 0x1FF;
				src_addr = (uint32_t) (buf + (i * 511) + (bytepos -1));
				src_data = *((uint8_t *) src_addr) ^ (1 << bitpos);
				*(uint8_t *)src_addr = src_data;

				PDEBUG("Corrected ECC error \n");
				mtd->ecc_stats.corrected++;
			}
			else {
				printk(KERN_NOTICE "Uncorrectable ECC error!!!!  :%x \n", ecc_val);
				mtd->ecc_stats.failed++;
			}
		}
		ecc_reg += 4;	/* next ecc val reg */		

	} /* for */

	return 0;
}

static void bcm5892_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				   const uint8_t *buf)
{
	uint32_t *end_ptr;

	/* 
	   What we want to do is write the 2K page with ECC_last, then do random 
	   write to program the spare area and issue the prg end command with no 
	   extra block. This doesn't seem to work on Micron 1Gb, so we currentlly do 
	   ECC_last and prg_end after 2K + 32 bytes (extra block size).
	*/
	chip->write_buf(mtd, buf, mtd->writesize);
	chip->write_buf(mtd, chip->oob_poi, (mtd->oobsize/2) - 4);

	/* the last word is programmed in the final data phase with prg end */
	end_ptr = (uint32_t *) (chip->oob_poi + (mtd->oobsize/2) - 4);
	umc_nand_page_prg_end(GET_SC(mtd), CHIP_SEL, *end_ptr);
	udelay (700);
}

static void bcm5892_cmdfunc(struct mtd_info *mtd,unsigned command,int column, int page)
{
	int ret = 0;
	struct bcm5892_nand_softc *sc = GET_SC(mtd);
	int timeout = 0;
	uint32_t data  = 0;

	sc->state = STATE_BUSY;

	/* Emulate NAND_CMD_READOOB on large-page chips */
	if (command == NAND_CMD_READOOB) {
		if (mtd->writesize > 512) {
			uint8_t buf[2048];

			memset (buf, 0xff, sizeof(buf));
			column += mtd->writesize;
	//		printk("---------123----------------\n");
			umc_nand_page_rd (GET_SC(mtd), CHIP_SEL, PAGE_ADDRESS(page), 
					  dev_width, rd_addr_cyc, buf, mtd->writesize);
		//	printk("---------456----------------\n");
			umc_random_data_rd_cmd(GET_SC(mtd), CHIP_SEL, column, dev_width , 
						   rand_op_addr_cyc);
			udelay(25);
		    } else {
			umc_smallnand_rand_rd_cmd (GET_SC(mtd), CHIP_SEL, 
						   PAGE_ADDRESS(page), dev_width, rand_op_addr_cyc);
			udelay(15);
		}

	}

	switch(command)
	{
	case NAND_CMD_READ0:
	case NAND_CMD_READ1:
		if (mtd->writesize == 512) {

		//	printk(KERN_ERR "Write_sz= %d\n",mtd->writesize);	
			umc_smallnand_page_rd_cmd (sc, CHIP_SEL, PAGE_ADDRESS(page),
						   dev_width, rd_addr_cyc);
			printk("lee write----->mtd->writesize=512\n");
		}
		else
		{
			umc_nand_page_rd_cmd (sc, CHIP_SEL, PAGE_ADDRESS(page), dev_width, rd_addr_cyc);

	//		umc_nand_page_rd_cmd (sc, CHIP_SEL, PAGE_ADDRESS(page), 
	//				      dev_width, 2);
		}	
		udelay(25); 
		timeout = 0x3000;
#define INT_STATUS ((*(volatile uint32_t *)sc->pUMC_R_umc_status_reg)&(1<<6))

		while( (!INT_STATUS) && (timeout !=0x1))
		{
			timeout--;
		}		
		if(!INT_STATUS)
			printk(KERN_ERR "%s: %d Critical Error: HW Read operation FAILED!!!\n",
			       __func__, __LINE__);
		break;
	case NAND_CMD_SEQIN:
		if (mtd->writesize == 512) 
			umc_smallnand_page_prg_start (sc, CHIP_SEL, PAGE_ADDRESS(page),
						      dev_width, prg_addr_cyc);
		else
			umc_nand_page_prg_start(sc, CHIP_SEL, PAGE_ADDRESS(page), dev_width, 
						prg_addr_cyc);		
		break;
	case NAND_CMD_READSTART:
		ret = umc_nand_page_rd_end(sc, CHIP_SEL);
		break;
	case NAND_CMD_PAGEPROG:
		umc_nand_page_prg_end(GET_SC(mtd), CHIP_SEL, data);	 
		udelay(700); 
		break;
	case NAND_CMD_STATUS:
		umc_nand_status_cmd(sc, CHIP_SEL);
		break;
	default:
		return;
	}// end of switch		
	sc->state = STATE_READY;
}

static void flash_init (struct bcm5892_nand_softc* sc)
{
	//Enable the UMC block in the DMU
	cls_dmu_block_enable(DMU_UMC_PWR_ENABLE);
	
	//Configure the UMC
	if (dev_width == 16)
		umc_memif_cfg(sc, TYPE_SRAM_NONMUXED,CHIPS_2, WIDTH_16, 
			      TYPE_NAND, CHIPS_1, WIDTH_16);
	else if (dev_width == 8)
		umc_memif_cfg(sc, TYPE_SRAM_NONMUXED,CHIPS_2, WIDTH_8, 
		      TYPE_NAND, CHIPS_1, WIDTH_8);

	// device types on cs : cs2 : SRAM/NOR cs1:SRAM/NOR cs0:NAND
	umc_config_cs_type(sc, CS_SRAM, CS_NAND, CS_SRAM );

	if (dev_width == 16)
		umc_config_cs (sc, CHIP_SELECT,INTERFACE_1,0,0,0,0,WIDTH_16);
	else if (dev_width == 8)
		umc_config_cs (sc, CHIP_SELECT,INTERFACE_1,0,0,0,0,WIDTH_8);


	 umc_int_enable(sc, 1, 1, 1, 1); 
}

static int bcm5892_nand_init_chip (struct bcm5892_nand_softc *sc)
{
	int  retval = ~0;
	struct mtd_info *mtd;
	struct nand_chip *nand;
	int manuf_id;

	mtd = &sc->bcm5892_mtd_info;
	nand = &sc->bcm5892_nand_chip;

	mtd->name = NULL;
	mtd->priv = nand;

	/* h/w init */
	flash_init (sc);

	nand->IO_ADDR_W = nand->IO_ADDR_R = 
		(void __iomem*)sc->nand_cs1_base;

	nand->cmdfunc	= bcm5892_cmdfunc;
	nand->read_buf = bcm5892_read_buf;
	nand->write_buf = bcm5892_write_buf;
	nand->read_word = bcm5892_read_word;
	nand->read_byte = bcm5892_read_byte;
	nand->select_chip = bcm5892_select_chip;
	nand->waitfunc = bcm5892_wait;

	PDEBUG("Doing nand_scan_ident\n");
	retval = bcm5892_nand_scan_ident (mtd, 1);
	
	if (retval)
		goto out;
	
	if (!retval) {
		if (!mtd->name)
			mtd->name = (char *)"bcm5892_nand_chip_on_umc";
	}

	manuf_id = sc->read_id & 0xff;
	if (manuf_id == 0x2c)		// Micron
		sv_umc_nand_set_cycles(sc,2,1,1,3,2,5,7);
        else if(manuf_id == 0xda)       // Micron 2G            ----->add by lee

                sv_umc_nand_set_cycles(sc,2,1,1,3,1,4,6);
                //sv_umc_nand_set_cycles(sc,2,1,1,3,2,5,7);
	 //	sv_umc_nand_set_cycles(sc,3,2,2,4,3,6,7);
	else if (manuf_id == 0x20)	// ST Micro
		sv_umc_nand_set_cycles(sc,3,2,2,4,3,6,7);
	else if ((manuf_id == 0xEC) && (sc->read_id ==0x76))	// Samsung
		sv_umc_nand_set_cycles(sc,2,1,1,3,2,5,6);
	else if ((manuf_id == 0xEC) && (sc->read_id == 0x75))	// Samsung
		sv_umc_nand_set_cycles(sc,2,1,1,2,2,4,5);

	nand->ecc.mode = NAND_ECC_HW;

	nand->cmd_ctrl	= bcm5892_hwcontrol;
	nand->ecc.write_oob = bcm5892_write_oob;

	if (nand->ecc.mode == NAND_ECC_HW) {
		nand->ecc.bytes = 3;
		nand->ecc.size = 512;
		nand->ecc.read_page = bcm5892_read_page_hwecc;
		nand->ecc.write_page = bcm5892_write_page_hwecc;
		nand->ecc.calculate = bcm5892_calculate;
		nand->ecc.correct = bcm5892_correct;;
		nand->ecc.hwctl = bcm5892_hwctl;
		if (mtd->writesize == 512) 
			umc_ecc_config(sc, ECC_NOJUMP, ECC_RD_END, ECC_ENABLE, 
				       ECC_PAGE_512); 
		else if (mtd->writesize == 2048) 
			umc_ecc_config(sc, ECC_NOJUMP, ECC_RD_END, ECC_ENABLE, 
				       ECC_PAGE_2048); 
	}

	nand->options |= NAND_USE_FLASH_BBT; 
	/* setup bbt descr */
	if (mtd->writesize == 512) {
		nand->bbt_td = &bcm5892_bbt_main_descr_512;
		nand->bbt_md = &bcm5892_bbt_mirr_descr_512;
		nand->ecc.layout = &bcm5892_oobinfo_512;
	} else {
		nand->bbt_td = &bcm5892_bbt_main_descr_2048;
		nand->bbt_md = &bcm5892_bbt_mirr_descr_2048;
		nand->ecc.layout = &bcm5892_oobinfo_2048;
	}

	nand->options |= NAND_NO_SUBPAGE_WRITE;

	/* final init */
	retval = bcm5892_nand_scan_tail(mtd);

out:
	return retval;
}


/*
 * This function allocates basic resources for UMC controller
 *
 * @param	 dev - pointer to the struct amba device 
 *
 * @return 	Int (0-Success, Negative value-fail)
*/

static int bcm5892_nand_drv_probe(struct amba_device *dev, void *id)
{
	int retval = 0;
	volatile void __iomem *nand_cs1_base = NULL;
	struct bcm5892_nand_softc *sc = NULL;

	PDEBUG("Entered bcm5892_nand_drv_probe function\n");
	if (! dev) {
		PDEBUG("FATAL: NULL Device Pointer for PROBE\n");
		return -EINVAL;
	}

	nand_cs1_base = ioremap(dev->res.start, (END_NAND_CS1 - START_NAND_CS1 + 1));
	if (!nand_cs1_base) {
		PDEBUG("ioremap failed, for NAND CHIP 0\n");
       	 	retval = -ENOMEM;
       		goto fail;
	}

	sc = kzalloc(sizeof(struct bcm5892_nand_softc), GFP_KERNEL);
	if (!sc) {
		PDEBUG("kmalloc  failed, \n");
       	 	retval = -ENOMEM;
       		goto fail;
	}

	amba_set_drvdata (dev, sc);

	PDEBUG("Start initializing Soft context\n");
	
	sc->nand_cs1_base = nand_cs1_base;
	sc->nand_umc_base = IO_ADDRESS (UMC_REG_BASE_ADDR);

	sc->pUMC_R_umc_status_reg =  			(sc->nand_umc_base + 0x00);
	sc->pUMC_R_umc_memif_cfg =   			(sc->nand_umc_base + 0x04);
	sc->pUMC_R_umc_memc_cfg =    		(sc->nand_umc_base + 0x08);
	sc->pUMC_R_umc_memc_clr =    			(sc->nand_umc_base + 0x0c);
	sc->pUMC_R_umc_direct_cmd =  		(sc->nand_umc_base + 0x10);
	sc->pUMC_R_umc_set_cycles =  			(sc->nand_umc_base + 0x14);
	sc->pUMC_R_umc_set_opmode = 		(sc->nand_umc_base + 0x18);
	sc->pUMC_R_umc_set_opmode0_cs1 =  		(sc->nand_umc_base + 0x124);
	sc->pUMC_R_umc_addr_mask =   		(sc->nand_umc_base + 0xa00);
	sc->pUMC_R_umc_addr_match =  		(sc->nand_umc_base + 0xa04);
	sc->pUMC_R_umc_conf =        			(sc->nand_umc_base + 0xa08);
	sc->pUMC_R_umc_ecc_status =   		(sc->nand_umc_base + 0x400);
	sc->pUMC_R_umc_ecc_memcfg =   		(sc->nand_umc_base + 0x404);
	sc->pUMC_R_umc_ecc_memcmd1 =  		(sc->nand_umc_base + 0x408);
	sc->pUMC_R_umc_ecc_memcmd2 =  		(sc->nand_umc_base + 0x40c);
	sc->pUMC_R_umc_ecc_addr0 =    		(sc->nand_umc_base + 0x410);
	sc->pUMC_R_umc_ecc_addr1 =    		(sc->nand_umc_base + 0x414);
	sc->pUMC_R_umc_ecc_value0 =   		(sc->nand_umc_base + 0x418);
	sc->pUMC_R_umc_ecc_value1 =   		(sc->nand_umc_base + 0x41c);
	sc->pUMC_R_umc_ecc_value2 =   		(sc->nand_umc_base + 0x420);
	sc->pUMC_R_umc_ecc_value3 =   		(sc->nand_umc_base + 0x424);
	sc->pUMC_R_umc_msu_scr_control_reg =            (sc->nand_umc_base + 0x0b00);
	sc->pUMC_R_umc_msu_addr_swz_ctrl_lo_reg =     (sc->nand_umc_base + 0x0b04);
	sc->pUMC_R_umc_msu_addr_swz_ctrl_hi_reg =     (sc->nand_umc_base + 0x0b08);
	sc->pUMC_R_umc_msu_addr_key_reg =               (sc->nand_umc_base + 0x0b0c);
	sc->pUMC_R_umc_msu_addr_scr_msk_reg =         (sc->nand_umc_base + 0x0b10);
	sc->pUMC_R_umc_msu_data_swz_ctrl_lo_reg =     (sc->nand_umc_base + 0x0b14);
	sc->pUMC_R_umc_msu_data_swz_ctrl_hi_reg =     (sc->nand_umc_base + 0x0b18);
	sc->pUMC_R_umc_msu_data_key_reg =               (sc->nand_umc_base + 0x0b1c);
	sc->pUMC_R_umc_msu_addr_scr_base_reg =        (sc->nand_umc_base + 0x0b20);
	sc->pUMC_R_umc_msu_addr_scr_base_msk_reg = (sc->nand_umc_base + 0x0b24);
	sc->pUMC_R_umc_memc_cfg_MEMADDR = 	(sc->nand_umc_base+0x0008);

	sc->pUMC_R_umc_ecc_value9 = 		(sc->nand_umc_base + 0x1A0);	
	sc->pUMC_R_umc_ecc_value10 = 	(sc->nand_umc_base + 0x1A4);	
	sc->pUMC_R_umc_ecc_value11 = 	(sc->nand_umc_base + 0x100);	
	sc->pUMC_R_umc_ecc_value12 = 	(sc->nand_umc_base + 0x104);	

	/* we have our own erase and read ID routines*/
	sc->nand_erase  = bcm_nand_erase; 	
	sc->nand_readid_cmd = umc_nand_readid_cmd;

	retval = bcm5892_nand_init_chip(sc);
	if (retval) {
		PDEBUG("nand init failed\n");
       	 	retval = -ENXIO;
		PDEBUG("BCM5892:- Init Failed, Freeing bcm5892_soft Context \n");
		goto fail;
	}
       
#ifdef CONFIG_MTD_PARTITIONS
	retval = add_mtd_partitions(&sc->bcm5892_mtd_info,BCM5892_nand_partition,
				    ARRAY_SIZE(BCM5892_nand_partition));
	if (retval) {
		PDEBUG(KERN_ERR "BCM5892 NAND :- Add MTD Partitions Failed\n");
		goto fail;
	}
#else

	retval = add_mtd_device(sc->bcm5892_mtd_info);
	if (retval) {
		PDEBUG("BCM5892 NAND :- Add MTD Device Failed- Exiting Probe\n");
		goto fail;
	}

#endif

	  PDEBUG ("BCM5892 NAND:- NAND pertitions added successfully\n");
	  return retval;

fail:
	if (nand_cs1_base) {
		iounmap((void*)nand_cs1_base);
		nand_cs1_base = NULL;
		if (sc) 
			kfree (sc);
	}
	PDEBUG("PROBE FAILED, No NAND Chip Detected !!!!! \n");	
	return retval;
}

/**
 *This function performs cleanup
*/
static int bcm5892_nand_drv_remove (struct amba_device *dev)
{
	struct bcm5892_nand_softc *sc = amba_get_drvdata (dev);

	PDEBUG("bcm5892_nand_drv_remove called!!!\n");

	amba_set_drvdata (dev, NULL);
	if (unlikely(!sc)) {
		printk(KERN_ERR "%s: called %s without private data!!",
		       module_id, __func__);
		return -EINVAL;
	}
	
	bcm5892_nand_dev_release (&sc->bcm5892_mtd_info);
	iounmap (sc->nand_cs1_base);
	kfree (sc);

	return 0;
}
/*
#ifdef CONFIG_PM
static int bcm5892_nand_suspend(struct amba_device *dev, pm_message_t state)
{
	struct bcm5892_nand_softc *sc = amba_get_drvdata (dev);


	printk("=====================1==============\n");
	printk("=====================1==============\n");
	printk("=====================1==============\n");
	printk("=====================1==============\n");
        if (sc->state != STATE_READY) {
                printk(KERN_ERR "%s: driver busy\n", module_id);
                return -EAGAIN;
        }

        return 0;
}

static int bcm5892_nand_resume(struct amba_device *dev)
{
	printk("====================================\n");
	printk("====================================\n");
	printk("====================================\n");
	printk("====================================\n");
        return 0;
}
#else
#define bcm5892_nand_suspend     NULL
#define bcm5892_nand_resume      NULL
#endif
*/
#define bcm5892_nand_suspend     NULL
#define bcm5892_nand_resume      NULL
static struct amba_id bcm5892nand_ids[] __initdata = {
	{
		.id	= 0x00041353,
		.mask	= 0x000fffff,
	},
	{ 0, 0 },
};

static struct amba_driver bcm5892nand_driver = {
	.drv = {
		.name	= "bcm5892-nand",
	},
	.id_table	= bcm5892nand_ids,
	.probe		= bcm5892_nand_drv_probe,
	.remove		= bcm5892_nand_drv_remove,
	.suspend	= bcm5892_nand_suspend,
	.resume		= bcm5892_nand_resume,
};

static int __init bcm5892_nand_init (void)
{
	int retval = 0;

	PDEBUG(" bcm5892_nand_init***** called\n");

	retval = amba_driver_register (&bcm5892nand_driver);
	if (retval < 0) 
		PDEBUG("AMBA driver registration failed\n");
	
	return retval;
}

/**
 *	This function is called when the driver is removed from the kernel with 
 *	the rmmod command.
 *	The driver unregisters itself with its bus driver. 
 */

static void __exit bcm5892_nand_cleanup (void)
{
	amba_driver_unregister(&bcm5892nand_driver);
}

MODULE_AUTHOR("Broadcom Corp");
MODULE_DESCRIPTION("PL353 NAND driverr");
MODULE_LICENSE("GPL");

/* Intialization of the Modules*/
module_init (bcm5892_nand_init);
module_exit (bcm5892_nand_cleanup);

