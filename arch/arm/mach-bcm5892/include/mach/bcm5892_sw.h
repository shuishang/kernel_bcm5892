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




#ifndef _BCM5892_H_
#define _BCM5892_H_



	#define PACKED _packed
#if 0
#ifndef __cplusplus
typedef unsigned char bool;
#endif
#endif

#define BZERO(a, size) memset(a, 0, size) /* memset() has the same prototype in ARM & GHS */

/**********************************************************************
 *  Constants
 **********************************************************************/

#ifndef NULL
#define NULL 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/**********************************************************************
 *  Basic types
 **********************************************************************/


/* One of the followiong two options must be defined */
/*#define BYTE_ORDER LITTLE_ENDIAN*/
/*#define BYTE_ORDER BIG_ENDIAN*/


#define BITLEN2BYTELEN(len) (((len)+7) >> 3)
#define BYTELEN2BITLEN(len) (len << 3)

#define WORDLEN2BYTELEN(len) (len << 2)
#define BYTELEN2WORDLEN(len) (len >> 2)


/* Time out check for dead loops, just a place holder for now */
#define CLS_CHECK_TIMEOUT() {}


/**********************************************************************
 *  Status Codes
 **********************************************************************/

typedef enum bcm_status_code {
    CLS_STATUS_OK                           =  0,
    CLS_STATUS_OK_NEED_DMA                  =  1,

    CLS_EXCEPTION_UNDEFINED                 = 10, /* THIS SET OF VALUE REPEATED IN SYSTEM.S */
    CLS_EXCEPTION_SWI,
    CLS_EXCEPTION_PRE_ABT,
    CLS_EXCEPTION_DATA_ABT,
    CLS_EXCEPTION_RESEVED,
    CLS_EXCEPTION_IRQ,
    CLS_EXCEPTION_FIQ,
    CLS_STATUS_MBIST_FAIL_PKE,
    CLS_STATUS_MBIST_FAIL_IDTCM,
    CLS_STATUS_MBIST_FAIL_SMAU_SCRATCH,           /* FOR SBI */

    CLS_EXCEPTION_OPEN_UNDEFINED            = 40, /* THIS SET OF VALUE REPEATED IN SYSTEM.S */
    CLS_EXCEPTION_OPEN_SWI,
    CLS_EXCEPTION_OPEN_PRE_ABT,
    CLS_EXCEPTION_OPEN_DATA_ABT,
    CLS_EXCEPTION_OPEN_RESEVED,
    CLS_EXCEPTION_OPEN_IRQ,
    CLS_EXCEPTION_OPEN_FIQ,

    CLS_STATUS_NO_DEVICE                    = 50,
    CLS_STATUS_DEVICE_BUSY,
    CLS_STATUS_DEVICE_FAILED,
    CLS_STATUS_NO_MEMORY,
    CLS_STATUS_NO_RESOURCES,
    CLS_STATUS_NO_BUFFERS,
    CLS_STATUS_PARAMETER_INVALID,
    CLS_STATUS_TIMED_OUT,
    CLS_STATUS_UNAUTHORIZED,
    CLS_STATUS_SSMC_FLASH_CFI_FAIL          = 70,
    CLS_STATUS_SSMC_FLASH_AUTOSELECT_FAIL,
    CLS_STATUS_SSMC_FLASH_ERASE_FAIL,
    CLS_STATUS_SSMC_FLASH_PROGRAM_FAIL,

    CLS_STATUS_BSC_DEVICE_NO_ACK            = 80,
    CLS_STATUS_BSC_DEVICE_INCOMPLETE,
    CLS_STATUS_BSC_TIMEOUT,
    CLS_STATUS_BBL_BUSY                     = 90,
    CLS_STATUS_CRYPTO_ERROR                 = 100,
    CLS_STATUS_CRYPTO_NEED2CALLS,
    CLS_STATUS_CRYPTO_PLAINTEXT_TOOLONG,
    CLS_STATUS_CRYPTO_AUTH_FAILED,
    CLS_STATUS_CRYPTO_ENCR_UNSUPPORTED,
    CLS_STATUS_CRYPTO_AUTH_UNSUPPORTED,
    CLS_STATUS_CRYPTO_MATH_UNSUPPORTED,
    CLS_STATUS_CRYPTO_WRONG_STEP,
    CLS_STATUS_CRYPTO_KEY_SIZE_MISMATCH,
    CLS_STATUS_CRYPTO_AES_LEN_NOTALIGN,
    CLS_STATUS_CRYPTO_AES_OFFSET_NOTALIGN,
    CLS_STATUS_CRYPTO_AUTH_OFFSET_NOTALIGN,
    CLS_STATUS_CRYPTO_DEVICE_NO_OPERATION,
    CLS_STATUS_CRYPTO_NOT_DONE,
    CLS_STATUS_CRYPTO_RNG_NO_ENOUGH_BITS,
    CLS_STATUS_CRYPTO_RNG_RAW_COMPARE_FAIL,       /* generate rng is same as what saved last time */
    CLS_STATUS_CRYPTO_RNG_SHA_COMPARE_FAIL,       /* rng after hash is same as what saved last time */
    CLS_STATUS_CRYPTO_TIMEOUT,
    CLS_STATUS_CRYPTO_AUTH_LENGTH_NOTALIGN,       /* for sha init/update */
    CLS_STATUS_RSA_PKCS1_LENGTH_NOT_MATCH   = 150, /* PKCS1 related error codes */
    CLS_STATUS_RSA_PKCS1_SIG_VERIFY_FAIL,
    CLS_STATUS_RSA_PKCS1_ENCODED_MSG_TOO_SHORT,
    CLS_STATUS_SELFTEST_FAIL                = 200,
    CLS_STATUS_SELFTEST_3DES_FAIL,
    CLS_STATUS_SELFTEST_DES_FAIL,
    CLS_STATUS_SELFTEST_AES_FAIL,
    CLS_STATUS_SELFTEST_SHA1_FAIL,
    CLS_STATUS_SELFTEST_DH_FAIL,
    CLS_STATUS_SELFTEST_RSA_FAIL,
    CLS_STATUS_SELFTEST_DSA_FAIL,
    CLS_STATUS_SELFTEST_DSA_PAIRWISE_FAIL,
    CLS_STATUS_SELFTEST_MATH_FAIL,
    CLS_STATUS_SELFTEST_RNG_FAIL,
    CLS_STATUS_SELFTEST_MODEXP_FAIL,
    CLS_STATUS_DMA_NO_FREE_CHANNEL          = 250,
    CLS_STATUS_DMA_INVALID_CHANNEL,
    CLS_STATUS_DMA_CHANNEL_BUSY,
    CLS_STATUS_DMA_CHANNEL_ACTIVE,       /* channel halt, but still has data in fifo */
    CLS_STATUS_DMA_CHANNEL_FREE,
    CLS_STATUS_DMA_BUFLIST_FILLED,
    CLS_STATUS_DMA_ERROR,
    CLS_STATUS_DMA_NOT_DONE,
    CLS_STATUS_SMAU_SCRATCH_BUSY            = 300,
    CLS_STATUS_SMAU_SCRATCH_NOT_DONE,
    CLS_STATUS_SMAU_SCRATCH_ERROR,
    CLS_STATUS_SPI_FLASH_PROTECTED          = 350,
    CLS_STATUS_SPI_WRITE_NOTSUPPORT,
    CLS_STATUS_SPI_READ_FIFO_EMPTY,
    CLS_STATUS_SPI_WRITE_FIFO_FULL,
    CLS_STATUS_SPI_DATA_STORE_NULL,
    CLS_STATUS_USB_DEVICE_NOTSUPPORT        = 360, /* SSMC ADDR PINs set to host mode, so device mode not support */
    CLS_STATUS_USB_DMA_CHAIN_INVALID,
    CLS_STATUS_USB_DMA_TX_DESC_ERR,
    CLS_STATUS_USB_DMA_TX_BUFF_ERR,
    CLS_STATUS_USB_DMA_TX_UNKNOWN_STATUS,
    CLS_STATUS_USB_DMA_RX_NOTDONE,
    CLS_STATUS_USB_SUSPENDED,
    CLS_STATUS_USB_REQUEST_UNKNOWN,
    CLS_STATUS_USB_REQUEST_WRONG_STRING,
    CLS_STATUS_USB_TIMEDOUT,
    CLS_STATUS_USB_CONTRLINIT_FAIL_CONFIG,
    CLS_STATUS_USB_READ_INT_TIMEDOUT,
    CLS_STATUS_USB_WRITE_INT_TIMEDOUT,
    CLS_STATUS_USB_DMA_DONE_TIMEDOUT,
    CLS_STATUS_USB_CONTROL_WAIT_TIMEDOUT,
    CLS_STATUS_USB_MASS_STORAGE_PROCESS_ERROR,

    CLS_STATUS_USB_OFFSET_GT_READ_OFFSET,
    CLS_STATUS_OTP6T_NOT_PROGRAMMED         = 400,
    CLS_STATUS_OTP6T_PROG_FAIL,
    CLS_STATUS_OTP6T_HASH_NOTMATCH,
    CLS_STATUS_OTP6T_CLRCHECK_FAIL,
    CLS_STATUS_OTP6T_INVALID_VALUES,               /* too much programming error in OTP */
    CLS_STATUS_OTP6T_READ_AFTER_PROGRAM_FAIL,      /* the read operation after programming failed */
    CLS_STATUS_OTP6T_READOUT_HASH_NOTMATCH,        /* the hash of readout otp != original otp hash */
    CLS_STATUS_OTP6T_INVALID,                      /* either not programmed or hash not match */
    CLS_STATUS_OTP6T_MACHX_FAIL,                   /* marchx algorithm (mbist) failed */
    CLS_STATUS_OTP6T_CUSTID_FAIL_FAILBIT,          /* customer id read fail because of failure bit set */
    CLS_STATUS_OTP6T_CUSTID_FAIL_EQUAL,            /* customer id read fail because of equal number of 1 and 0 */
    CLS_STATUS_OTP6T_CUSTID_FAIL_PROGRAMMING,      /* customer id programming fail */
    /* LOTP  */
    CLS_STATUS_LOTP_PROG_OVERFLOW,		   /* program row overflow */
    CLS_STATUS_LOTP_FAIL_PROG_FAILED,		   /* min 2 fail bits program failed */
    CLS_STATUS_LOTP_UNPROG,
    CLS_STATUS_LOTP_RD_OVERFLOW,		   /* out of bounds overflow, while reading mfg keys array */
    CLS_STATUS_LOTP_RD_CRC_FAIL,		   /* reading mfg keys array yields crc error. */
    CLS_STATUS_LOTP_PROG_SUCCESS,		   /* successful read. no crc error */
    CLS_STATUS_LOTP_ILENGTH_SMALL,		   /* Length passed in is smaller than the one programmed */
    CLS_STATUS_OTP2T_BUSY                   = 430, /* hardware is reading now */
    CLS_STATUS_OTP2T_READ_EMPTY,
    CLS_STATUS_OTP2T_PROG_FAIL,
    CLS_STATUS_BTROM_HASH_NOTMATCH_P1       = 450, /* BOOTROM */
    CLS_STATUS_BTROM_UNKNOWN_BTINTERFACE,
    CLS_STATUS_BTROM_SYM_VERIFY_NOTALLOW,
    CLS_STATUS_BTROM_UNKNOWN_AUTH_METHOD,
    CLS_STATUS_BTROM_HASH_NOTMATCH_P2       = 480, /* BOOTROM part2 */

    CLS_STATUS_SBI_MAGIC_NOTMATCH           = 600,
    CLS_STATUS_SBI_INVALID_HEADER,
    CLS_STATUS_SBI_SIZE_TOO_BIG_IN_ABI,  /* the sbi size field in abi is too big */
    CLS_STATUS_SBI_SIZE_TOO_BIG_SBI,     /* the auth offset field (e.g. sbi length) is too big */
    CLS_STATUS_SBI_NO_KCR_PUB,
    CLS_STATUS_SBI_WRONG_KCR_PUB,
    CLS_STATUS_SBI_CHAIN_OF_TRUST_FAIL,

    CLS_STATUS_SBI_HASH_NOTMATCH,  		/* auth fail: local hmac-sha1 key */
    CLS_STATUS_SBI_DSASIG_NOTMATCH,		/* auth fail: DSA */
    CLS_STATUS_SBI_CUSTOMERID_MISMATCH,	/* customer ID mismatch in M-SBI */
    CLS_STATUS_SBI_AES_SELFTEST_FAILURE,
    CLS_STATUS_SBI_SECURECACHEKEYS_GENERATION_FAILURE, /* secure cache keys failure */
    CLS_STATUS_SBI_SYMMETRIC_ABI_FLASH_REWRITE_FAILURE,
    CLS_STAUS_SBI_SYMMETRIC_ABI_SIGNATURE_VERIFY_FAILURE,
    CLS_STATUS_SBI_SYMMMETRIC_ABI_HASH_GEN_FAILURE,
    CLS_STATUS_SBI_SYMMETRIC_ABI_HASH_FLASH_WRITE_FAILURE,
    CLS_STATUS_SBI_APPLICATION_ENCRYPT_FAILURE,
    CLS_STATUS_SBI_APPLICATION_FLASH_WRITE_FAILURE,
    CLS_STATUS_SBI_APPLICATION_SIG_VERIFY_FAILURE,
    CLS_STATUS_SBI_APPLICATION_SECURE_CACHE_KEYS_ENC_FAILIRE,
    CLS_STATUS_SBI_APPLICATION_SECURE_CACHE_KEYS_WRITE_FAILIRE,
    CLS_STATUS_SBI_KDI_KDC_PUB_FLASH_WRITE_FAILURE,
    
    
    CLS_STATUS_SC_NO_ATR                   = 700, /* Either no smart card, or synchronous card which we don't support */
    CLS_STATUS_SC_ATR_TCK_FAIL,         /* check sum of ATR failed */
    CLS_STATUS_SC_ATR_TOO_MANY_SETS,    /* too much sets of data in the ATR response */
    CLS_STATUS_SC_RX_TIMEOUT,           /* wait for Rx byte, timed out */
    CLS_STATUS_SC_TX_TIMEOUT,           /* wait for Tx byte, timed out (normally does not happen) */
    CLS_STATUS_SC_T0_BUF_TOO_SHORT,     /* send out buf too short */
    CLS_STATUS_SC_T0_BUF_WRONG_LEN,     /* send out buf contains a wrong length */
    CLS_STATUS_SC_T0_WRONG_INS,
    CLS_STATUS_SC_PTS_FAIL,
    CLS_STATUS_SC_CARD_NOT_SUPPORTED,

        /*** SC TEST SPECIFIC STATUS : added by Rohit ***/
        CLS_STATUS_SC_RX_MISCOMPARE,	     /* Rx byte not same as Tx byte from SCI */

    CLS_STATUS_SMC_AHBMODE_INVALID       = 750,
    CLS_STATUS_SMC_AMD_FLASH_PRM_ERROR,

    CLS_STATUS_SDM_DATA_LEN_ERROR        = 850,
    CLS_STATUS_SDM_OP_HAS_ERROR,         /* NormalInterruptStatusRegister.D15 = 1 */
    CLS_STATUS_SDM_CMD_COMPLETE,             /* NormalInterruptStatusRegister.D0 = 1 */
    CLS_STATUS_SDM_DATA_XFER_COMPLETE,        /* NormalInterruptStatusRegister.D1 = 1 */
    CLS_STATUS_SDM_BLK_GAP_EVENT,         /* NormalInterruptStatusRegister.D2 = 1 */
    CLS_STATUS_SDM_DMA_INT,               /* NormalInterruptStatusRegister.D3 = 1 */
    CLS_STATUS_SDM_BUF_WRITE_READY,       /* NormalInterruptStatusRegister.D4 = 1 */
    CLS_STATUS_SDM_BUF_READ_READY,        /* NormalInterruptStatusRegister.D5 = 1 */
    CLS_STATUS_SDM_BUF_READ_ENABLE,       /* PresentStateRegister.D11 = 1 */
    CLS_STATUS_SDM_BUF_WRITE_ENABLE,      /* PresentStateRegister.D10 = 1 */
    CLS_STATUS_SDM_READ_DATA_COUNT_MISMATCH,
    CLS_STATUS_SDM_WRITE_DATA_COUNT_MISMATCH,
    CLS_STATUS_RESPONSE_HAS_ERROR,
    CLS_STATUS_SDM_ADMA2_DES_ERROR,
    CLS_STATUS_SDM_SDMA_TRANS_ERROR,
    CLS_STATUS_SDM_ADMA2_TRANS_ERROR,
    CLS_STATUS_SDM_SYNC_ABORT_ERROR,
    CLS_STATUS_SDM_WAKEUP_ERROR,
    CLS_STATUS_SDM_TRANS_SUPENDED,

    CLS_STATUS_UMC_NOR_PROGRAM_ERROR    = 950,
    CLS_STATUS_UMC_CONFIG_SET_ERROR,
 
    CLS_STATUS_VIC_INVALID_PRIORITY	= 960,
    CLS_STATUS_VIC_INVALID_INT, 
} CLS_STATUS;

#endif  /* _BCM5892_H_ */

