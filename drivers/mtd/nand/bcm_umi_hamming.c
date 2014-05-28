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
#include <linux/mtd/nand_ecc512.h>
/* ---- External Variable Declarations ----------------------------------- */
/* ---- External Function Prototypes ------------------------------------- */
/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

/* ---- Private Function Prototypes -------------------------------------- */
static int bcm_umi_hamming_get_hw_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code);
static void bcm_umi_hamming_enable_hwecc(struct mtd_info *mtd, int mode);

/* ---- Private Variables ------------------------------------------------ */

/*
** nand_hw_eccoob
** New oob placement block for use with hardware ecc generation.
*/
static struct nand_ecclayout nand_hw_eccoob_512 = {
	.eccbytes	= 3,
	.eccpos		= {6, 7, 8 },
    /*
    ** Reserve 0/1 and 10/11 as BI indicators for 16-bit flash
    ** Reserve 5 for 8-bit BI
    ** 6/7/8 are for ecc so this is all that's left
    */
	.oobfree	= {
        { .offset = 2,      .length = 3},
        { .offset = 9,      .length = 1},
        { .offset = 12,     .length = 4 }}
};

/*
** We treat the OOB for a 2K page as if it were 4 512 byte oobs, except that the ECC offset if 8 rather than 6.
*/
static struct nand_ecclayout nand_hw_eccoob_2048 = {
	.eccbytes	= 12,
	.eccpos		= {8, 9, 10, 24, 25, 26, 40, 41, 42, 56, 57, 58 },
    /*
    ** Reserve 0/1 as BI indicators for 8/16-bit flash
    ** 8/9/10 are for ecc so this is all that's left
    */
	.oobfree	= {
        { .offset = 2,      .length = 6},
        { .offset = 11,     .length = 13 },
        { .offset = 27,     .length = 13 },
        { .offset = 43,     .length = 13 },
        { .offset = 59,     .length = 5 }}
};

/*
** We treat the OOB for a 4K page as if it were 8 512 byte oobs, except that the ECC offset if 8 rather than 6.
*/
static struct nand_ecclayout nand_hw_eccoob_4096 = {
	.eccbytes	= 24,
	.eccpos		= {8, 9, 10, 24, 25, 26, 40, 41, 42, 56, 57, 58, 72, 73, 74, 88, 89, 90, 104, 105, 106, 120, 121, 122 },
	/*
	** Reserve 0/1 as BI indicators for 8/16-bit flash
	** 8/9/10 are for ecc so this is all that's left
	*/
	.oobfree	= {
		{ .offset = 2,      .length = 6},
		{ .offset = 11,     .length = 13 },
		{ .offset = 27,     .length = 13 },
		{ .offset = 43,     .length = 13 },
		{ .offset = 59,     .length = 13 },
		{ .offset = 75,     .length = 13 },
		{ .offset = 91,     .length = 13 },
		{ .offset = 107,    .length = 13 }}
	/*	{ .offset = 123,    .length = 5 }}    It turns out nand_ecclayout only has space for 8 entries */
};

/* ---- Private Functions ------------------------------------------------ */

/****************************************************************************
*
*  bcm_umi_hamming_get_hw_ecc
*
*   Used to get the hardware ECC.
*
***************************************************************************/

static int bcm_umi_hamming_get_hw_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
   unsigned long ecc = REG_UMI_NAND_ECC_DATA;
   ecc_code[2] = (ecc >> 16) & 0xff;
   ecc_code[1] = (ecc >>  8) & 0xff;
   ecc_code[0] = (ecc >>  0) & 0xff;

   (void)mtd;
   (void)dat;

   return 0;
}

/****************************************************************************
*
*  bcm_umi_hamming_enable_hwecc
*
*   Used to turn on hardware ECC.
*
***************************************************************************/

static void bcm_umi_hamming_enable_hwecc(struct mtd_info *mtd, int mode)
{
   (void)mtd;
   (void)mode;
   nand_bcm_umi_hamming_enable_hwecc();
}

/* ==== Public Functions ================================================= */

