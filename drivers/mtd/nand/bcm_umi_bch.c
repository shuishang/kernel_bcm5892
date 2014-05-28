/*****************************************************************************
* Copyright 2004 - 2009 Broadcom Corporation.  All rights reserved.
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

/* ---- Include Files ---------------------------------------------------- */
#include <linux/version.h>
#include "nand_bcm_umi.h"

/* ---- External Variable Declarations ----------------------------------- */
/* ---- External Function Prototypes ------------------------------------- */
/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

/* ---- Private Function Prototypes -------------------------------------- */
/* We need a patch to get the page numbers passed in for the write function... */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int bcm_umi_bch_read_page_hwecc(struct mtd_info *mtd,
				       struct nand_chip *chip, uint8_t * buf, int page);
static void bcm_umi_bch_write_page_hwecc(struct mtd_info *mtd,
	struct nand_chip *chip, const uint8_t *buf, int page);
#else
static int bcm_umi_bch_read_page_hwecc(struct mtd_info *mtd,
				       struct nand_chip *chip, uint8_t * buf);
static void bcm_umi_bch_write_page_hwecc(struct mtd_info *mtd,
	struct nand_chip *chip, const uint8_t *buf);
#endif

/* ---- Private Variables ------------------------------------------------ */

/*
** nand_hw_eccoob
** New oob placement block for use with hardware ecc generation.
*/
static struct nand_ecclayout nand_hw_eccoob_512 = {
	/* Reserve 5 for BI indicator */
	.oobfree = {
#if (NAND_ECC_NUM_BYTES > 3)
		    {.offset = 0, .length = 2}
#else
		    {.offset = 0, .length = 5},
		    {.offset = 6, .length = 7}
#endif
		    }
};

/*
** We treat the OOB for a 2K page as if it were 4 512 byte oobs,
** except the BI is at byte 0.
*/
static struct nand_ecclayout nand_hw_eccoob_2048 = {
	/* Reserve 0 as BI indicator */
	.oobfree = {
#if (NAND_ECC_NUM_BYTES > 10)
		    {.offset = 1, .length = 2},
#elif (NAND_ECC_NUM_BYTES > 7)
		    {.offset = 1, .length = 5},
		    {.offset = 16, .length = 6},
		    {.offset = 32, .length = 6},
		    {.offset = 48, .length = 6}
#else
		    {.offset = 1, .length = 8},
		    {.offset = 16, .length = 9},
		    {.offset = 32, .length = 9},
		    {.offset = 48, .length = 9}
#endif
		    }
};

/* We treat the OOB for a 4K page as if it were 8 512 byte oobs,
 * except the BI is at byte 0. */
static struct nand_ecclayout nand_hw_eccoob_4096 = {
	/* Reserve 0 as BI indicator */
	.oobfree = {
#if (NAND_ECC_NUM_BYTES > 10)
		    {.offset = 1, .length = 2},
		    {.offset = 16, .length = 3},
		    {.offset = 32, .length = 3},
		    {.offset = 48, .length = 3},
		    {.offset = 64, .length = 3},
		    {.offset = 80, .length = 3},
		    {.offset = 96, .length = 3},
		    {.offset = 112, .length = 3}
#else
		    {.offset = 1, .length = 5},
		    {.offset = 16, .length = 6},
		    {.offset = 32, .length = 6},
		    {.offset = 48, .length = 6},
		    {.offset = 64, .length = 6},
		    {.offset = 80, .length = 6},
		    {.offset = 96, .length = 6},
		    {.offset = 112, .length = 6}
#endif
		    }
};


#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
/* We treat the OOB for a 4K page as if it were 16 512 byte oobs,
 * except the BI is at byte 0. */
static struct nand_ecclayout nand_hw_eccoob_8192 = {
	/* Reserve 0 as BI indicator */
	.oobfree = {
#if (NAND_ECC_NUM_BYTES > 10)
		    {.offset = 1, .length = 2},
		    {.offset = 16, .length = 3},
		    {.offset = 32, .length = 3},
		    {.offset = 48, .length = 3},
		    {.offset = 64, .length = 3},
		    {.offset = 80, .length = 3},
		    {.offset = 96, .length = 3},
		    {.offset = 112, .length = 3}
			 /* Maximum 8 entries allowed as per mtd-abi.h */
#else
		    {.offset = 1, .length = 5},
		    {.offset = 16, .length = 6},
		    {.offset = 32, .length = 6},
		    {.offset = 48, .length = 6},
		    {.offset = 64, .length = 6},
		    {.offset = 80, .length = 6},
		    {.offset = 96, .length = 6},
		    {.offset = 112, .length = 6}
			 /* Maximum 8 entries allowed as per mtd-abi.h */
#endif
		    }
};

/* Internal ECC to nand chip - no ecc bytes defined here */
static struct nand_ecclayout nand_hw_eccoob_8192_internal_ecc = {
	/* Reserve 0 as BI indicator */
	.oobfree = {
		    {.offset = 1, .length = 31} 
		    }
};
#endif

/****************************************************************************
*
*  bcm_umi_bch_read_page_hwecc - hardware ecc based page read function
*  @mtd:	mtd info structure
*  @chip:	nand chip info structure
*  @buf:	buffer to store read data
*
***************************************************************************/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int bcm_umi_bch_read_page_hwecc(struct mtd_info *mtd,
				       struct nand_chip *chip, uint8_t * buf, int page)
#else
static int bcm_umi_bch_read_page_hwecc(struct mtd_info *mtd,
				       struct nand_chip *chip, uint8_t * buf)
#endif
{
	int sectorIdx = 0;
	int eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps;
	uint8_t *datap = buf;
	uint8_t eccCalc[NAND_ECC_NUM_BYTES];
	int sectorOobSize = mtd->oobsize / eccsteps;
	int stat;
	int writesize = mtd->writesize;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
	//printk("%s: mtd->writesize=%d mtd->oobsize=%d chip->options=0x%x page=0x%x\n",  __func__, mtd->writesize, mtd->oobsize, chip->options, page);
	if (chip->options & NAND_INTERNAL_ECC)
	{
	   bcm_umi_nand_read_buf(mtd, datap, writesize);
	   return 0;
	}
#endif
	
	for (sectorIdx = 0; sectorIdx < eccsteps;
			sectorIdx++, datap += eccsize) {
		if (sectorIdx > 0) {
			/* Seek to page location within sector */
			printf("chip->cmdfunc(mtd, NAND_CMD_RNDOUT, sectorIdx * eccsize,-1)\n");
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT, sectorIdx * eccsize,
				      -1);
		}

		/* Enable hardware ECC before reading the buf */
		nand_bcm_umi_bch_enable_read_hwecc();

		/* Read in data */
		bcm_umi_nand_read_buf(mtd, datap, eccsize);

		/* Pause hardware ECC after reading the buf */
		nand_bcm_umi_bch_pause_read_ecc_calc();

		/* Read the OOB ECC */
		if (writesize > 512) {
			/* 
			 * For 512 byte page devices, we are already at the oob 
			 * location and don't need a READOOB command. Also  
			 * RNDOUT isn't available in general on 512 byte devices.
			 */
			printf(chip->cmdfunc(mtd, NAND_CMD_RNDOUT,writesize + sectorIdx * sectorOobSize, -1)\n);
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT,
						writesize + sectorIdx * sectorOobSize, -1);
		}
		nand_bcm_umi_bch_read_oobEcc(writesize, eccCalc,
					     NAND_ECC_NUM_BYTES,
					     chip->oob_poi +
					     sectorIdx * sectorOobSize);

		/* Correct any ECC detected errors */
		stat =
		    nand_bcm_umi_bch_correct_page(datap, eccsize, eccCalc,
						  NAND_ECC_NUM_BYTES);

		/* Update Stats */
		if (stat < 0) {
#if defined(NAND_BCM_UMI_DEBUG) || 0
#if NAND_BCM_UMI_EXTERN_GPAGE
			printk(KERN_WARNING "%s uncorr_err sectorIdx=%d page=0x%x\n", __func__, sectorIdx, gPage);
#else
			printk(KERN_WARNING "%s uncorr_err sectorIdx=%d\n", __func__, sectorIdx);
#endif
			printk(KERN_WARNING
			       "%s data %02x %02x %02x %02x %02x %02x %02x %02x\n",
			       __func__, datap[0], datap[1], datap[2], datap[3],
			       datap[4], datap[5], datap[6], datap[7]);
			printk(KERN_WARNING
			       "%s ecc  %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			       __func__, eccCalc[0], eccCalc[1], eccCalc[2],
			       eccCalc[3], eccCalc[4], eccCalc[5], eccCalc[6],
			       eccCalc[7], eccCalc[8], eccCalc[9], eccCalc[10],
			       eccCalc[11], eccCalc[12]);
#if NAND_BCM_UMI_EXTERN_GPAGE
			KNLLOG("%s uncorr_err sectorIdx=%d page=0x%x\n", sectorIdx, gPage);
#else
			KNLLOG("%s uncorr_err sectorIdx=%d\n", sectorIdx);
#endif
			KNLLOG("%s data %08x %08x\n",
			       datap[0]<<24|datap[1]<<16|datap[2]<<8|datap[3], 
			       datap[4]<<24|datap[5]<<16|datap[6]<<8|datap[7]);
			KNLLOG("%s ecc  %08x %08x %08x %02x\n",
			       eccCalc[0]<<24|eccCalc[1]<<16|eccCalc[2]<<8|eccCalc[3], 
					 eccCalc[4]<<24|eccCalc[5]<<16|eccCalc[6]<<8|eccCalc[7], 
					 eccCalc[8]<<24|eccCalc[9]<<16|eccCalc[10]<<8|eccCalc[11], 
			       eccCalc[12]);
			//BUG();
#endif
			mtd->ecc_stats.failed++;
		} else {
			if (stat > 0) {
 
				/* 
				 * Do not report small numbers of corrected bits since UBI wear leveling
				 * has problems with MLC flash triggering scrubbing/torturing in a loop.
				 * Note that this statistic is for 1 sector in a page, but if any sector
				 * shows high error counts, then the whole page (and block) is suspect.
				 * Run higher level ubi torture testing on highly degraded sectors. 
				 * The error correction capability is NAND_ECC_NUM_BYTES*8/13, so for
				 * NAND_ECC_NUM_BYTES=13, this implies 8 bits of error correction.
				 * Set the threshold to 3/4 here so that 7 or 8 bits are considered unclean. 
				 * This can be changed if desired. Keep these as hard numbers for now, but 
				 * this could be a macro. But since it's only used in this one place I want 
				 * it to be clear.
				 */
				if (stat > ((NAND_ECC_NUM_BYTES*8/13) * 3/4))
				{
					mtd->ecc_stats.corrected += stat;	
				}
				//mtd->ecc_stats.corrected += stat;
				
#if defined(NAND_BCM_UMI_DEBUG) || 0
#if NAND_BCM_UMI_EXTERN_GPAGE
				printk(KERN_INFO "%s %d correctable_errors detected page=0x%x\n", __func__, stat, gPage);
				KNLLOG("%d correctable_errors detected page=0x%x\n", 
						 stat, gPage);
#else
				printk(KERN_INFO "%s %d correctable_errors detected\n", __func__, stat);
#endif
				KNLLOG("%d correctable_errors detected\n", stat);
#endif
			}
		}
	}
	return 0;
}

/****************************************************************************
*
*  bcm_umi_bch_write_page_hwecc - hardware ecc based page write function
*  @mtd:	mtd info structure
*  @chip:	nand chip info structure
*  @buf:	data buffer
*
***************************************************************************/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static void bcm_umi_bch_write_page_hwecc(struct mtd_info *mtd,
	struct nand_chip *chip, const uint8_t *buf, int page)
#else
static void bcm_umi_bch_write_page_hwecc(struct mtd_info *mtd,
	struct nand_chip *chip, const uint8_t *buf)
#endif
{
	int sectorIdx = 0;
	int eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps;
	const uint8_t *datap = buf;
	uint8_t *oobp = chip->oob_poi;
	int sectorOobSize = mtd->oobsize / eccsteps;
	int writesize = mtd->writesize;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
	//printk("%s: page=0x%x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",  __func__, page, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	if (chip->options & NAND_INTERNAL_ECC)
	{
		/* See comment block above for read function. Same applies here as well. */
		bcm_umi_nand_write_buf(mtd, datap, writesize);
		bcm_umi_nand_write_buf(mtd, chip->oob_poi, mtd->oobsize);
		return;
	}
#endif

	for (sectorIdx = 0; sectorIdx < eccsteps;
	     sectorIdx++, datap += eccsize, oobp += sectorOobSize) {
		/* Enable hardware ECC before writing the buf */
		nand_bcm_umi_bch_enable_write_hwecc();
		bcm_umi_nand_write_buf(mtd, datap, eccsize);
		nand_bcm_umi_bch_write_oobEcc(writesize, oobp, NAND_ECC_NUM_BYTES);
	}

	bcm_umi_nand_write_buf(mtd, chip->oob_poi, mtd->oobsize);
}
