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
#include <mach/reg_umi.h>
#include "nand_bcm_umi.h"
#ifdef BOOT0_BUILD
#include <uart.h>
#endif

/* ---- External Variable Declarations ----------------------------------- */
/* ---- External Function Prototypes ------------------------------------- */
/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Functions ------------------------------------------------ */

#if NAND_ECC_BCH
/****************************************************************************
*  nand_bch_ecc_flip_bit - Routine to flip an errored bit
*
*  PURPOSE:
*     This is a helper routine that flips the bit (0 -> 1 or 1 -> 0) of the
*     errored bit specified
*
*  PARAMETERS:
*     datap - Container that holds the 512 byte data
*     errorLocation - Location of the bit that needs to be flipped
*
*  RETURNS:
*     None
****************************************************************************/
static void nand_bcm_umi_bch_ecc_flip_bit(uint8_t *datap, int datalen, int errorLocation)
{
	int locWithinAByte = (errorLocation & REG_UMI_BCH_ERR_LOC_BYTE) >> 0;
	int locWithinAWord = (errorLocation & REG_UMI_BCH_ERR_LOC_WORD) >> 3;
	int locWithinAPage = (errorLocation & REG_UMI_BCH_ERR_LOC_PAGE) >> 5;

	uint8_t errorByte = 0;
	uint8_t byteMask = 1 << locWithinAByte;
   int offset;

	/* BCH uses big endian, need to change the location
	 * bits to little endian */
	locWithinAWord = 3 - locWithinAWord;
	offset = locWithinAPage * sizeof(uint32_t) + locWithinAWord;
   if (offset >= datalen)
   {
      /* bad bit in ECC bytes */
		KNLLOG("Ignoring bad bits in ECC at offset:0x%x\n", offset);
      return;
   }

	errorByte = datap[offset];

#ifdef BOOT0_BUILD
	puthexs("\nECC Correct Offset: ",
		locWithinAPage * sizeof(uint32_t) + locWithinAWord);
	puthexs(" errorByte:", errorByte);
	puthex8(" Bit: ", locWithinAByte);
#endif

	KNLLOG("ECC Correct Offset:0x%x errorByte=0x%x, errorBit=0x%x byteMask=0x%x\n", 
			 offset, errorByte, locWithinAByte, byteMask);
	 
	if (errorByte & byteMask) {
		/* bit needs to be cleared */
		errorByte &= ~byteMask;
	} else {
		/* bit needs to be set */
		errorByte |= byteMask;
	}

	KNLLOG("corrected errorByte=0x%x\n", errorByte);
	
	/* write back the value with the fixed bit */
	datap[offset] = errorByte;
}

/****************************************************************************
*  nand_correct_page_bch - Routine to correct bit errors when reading NAND
*
*  PURPOSE:
*     This routine reads the BCH registers to determine if there are any bit
*     errors during the read of the last 512 bytes of data + ECC bytes.  If
*     errors exists, the routine fixes it.
*
*  PARAMETERS:
*     datap - Container that holds the 512 byte data
*     len - data ecc is over length
*     readEccData - ecc data read into a buffer
*     numEccBytes - ecc length
*
*  RETURNS:
*     0 or greater = Number of errors corrected
*                    (No errors are found or errors have been fixed)
*    -1 = Error(s) cannot be fixed
****************************************************************************/
int nand_bcm_umi_bch_correct_page(uint8_t *datap, int datalen, uint8_t *readEccData,
				  int numEccBytes)
{
	int numErrors;
	int errorLocation;
	int idx;
	uint32_t regValue;
	uint32_t *longwordp;

	/* wait for read ECC to be valid */
	regValue = nand_bcm_umi_bch_poll_read_ecc_calc();

	/*
	 * read the control status register to determine if there
	 * are error'ed bits
	 * see if errors are correctible
	 */
	if ((regValue & REG_UMI_BCH_CTRL_STATUS_UNCORR_ERR) > 0) {
		int i;

		for (i = 0; i < numEccBytes; i++) {
			if (readEccData[i] != 0xff) {
				/* errors cannot be fixed, return -1 */
				return -1;
			}
		}
		/* 
		 * If bits in an erased page are stuck low and 
		 * the ECC is all FF, then we should report
		 * the stuck low bits as uncorrectible errors
		 * as the ECC is unprogrammed. Later, if the
		 * page is still in use (block not marked bad
		 * by filesystem code) and programmed, the ECC 
		 * can then recover stuck low bits. For stuck 
		 * high bits, this check won't help, but that
		 * will be detected upon initial programming 
		 * and again handled by the file system.
		 */
		numErrors = 0;
		longwordp = (uint32_t *)datap;
		for (i = 0; i < (int)(datalen/sizeof(uint32_t)); i++, longwordp++) {
			if (*longwordp != 0xffffffff) {
			  int j;
			  KNLLOG("ECC at all FF, word[%d]=0x%08x\n", i, *longwordp); 
			  for (j = 0; j < 32; j++) {
				  if ((*longwordp & (1<<j)) == 0) {
						  numErrors++;
				  }
			  }
			  *longwordp = 0xffffffff; /* Fake correction */
			}
		}
		if (numErrors)
		{
#if NAND_BCM_UMI_EXTERN_GPAGE
			KNLLOG("ECC at all FF, numErrors=%d page=0x%x\n", numErrors, gPage);
#else
			KNLLOG("ECC at all FF, numErrors=%d\n", numErrors);
#endif 
		}
		return numErrors;
	}

	if ((regValue & REG_UMI_BCH_CTRL_STATUS_CORR_ERR) == 0) {
		/* no errors */
		return 0;
	}

	/*
	 * Fix errored bits by doing the following:
	 * 1. Read the number of errors in the control and status register
	 * 2. Read the error location registers that corresponds to the number
	 *    of errors reported
	 * 3. Invert the bit in the data
	 */
	numErrors = (regValue & REG_UMI_BCH_CTRL_STATUS_NB_CORR_ERROR) >> 20;

#if NAND_BCM_UMI_EXTERN_GPAGE
	KNLLOG("numErrors=%d page=0x%x\n", numErrors, gPage);
#else
	KNLLOG("numErrors=%d\n", numErrors);
#endif
	
	for (idx = 0; idx < numErrors; idx++) {
		errorLocation =
		    REG_UMI_BCH_ERR_LOC_ADDR(idx) & REG_UMI_BCH_ERR_LOC_MASK;

		/* Flip bit */
		nand_bcm_umi_bch_ecc_flip_bit(datap, datalen, errorLocation);
	}
	/* Errors corrected */
	return numErrors;
}
#endif
