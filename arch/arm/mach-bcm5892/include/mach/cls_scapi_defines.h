/*****************************************************************************
*  Copyright 2008 Broadcom Corporation.  All rights reserved.
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
 
/*
 *  Broadcom Claudius Project
 *  File: cls_scapi_defines.h
 *  Description: Crypto apis.
 *
 *
 *****************************************************************************/
#ifndef _CLS_SCAPI_DEFINES_H_
#define _CLS_SCAPI_DEFINES_H_

#include "cls_bulk_hw.h"

/**********************************************************************
 *  SCAPI defines
 **********************************************************************/

typedef struct _cls_bulk_cipher_cmd {
	uint32_t cipher_mode  : 3,    /* encrypt or decrypt */
	         encr_alg     : 4,    /* encryption algorithm */
	         encr_mode    : 3,
	         auth_alg     : 4,    /* authentication algorithm */
	         auth_mode    : 3,
	         cipher_order : 3,    /* encrypt first or auth first */
#ifdef DV_ONLY 
		 sim_copy     : 1,
		 rsvd         : 11;
#else
		 rsvd         : 12;
#endif
} CLS_BULK_CIPHER_CMD;

typedef enum cls_encr_algs {
	CLS_ENCR_ALG_NONE      =  0,
    CLS_ENCR_ALG_AES_128,
    CLS_ENCR_ALG_AES_192,
    CLS_ENCR_ALG_AES_256,
    CLS_ENCR_ALG_DES,
    CLS_ENCR_ALG_3DES,
} CLS_ENCR_ALG;

typedef enum cls_encr_mode {
	CLS_ENCR_MODE_NONE     = 0,
	CLS_ENCR_MODE_CBC,
	CLS_ENCR_MODE_ECB,
	CLS_ENCR_MODE_CTR,
	CLS_ENCR_MODE_CCM = 5,
	CLS_ENCR_MODE_CMAC
} CLS_ENCR_MODE;

typedef enum cls_auth_algs {
    CLS_AUTH_ALG_NONE      =  0,
    CLS_AUTH_ALG_HMAC_SHA1,
    CLS_AUTH_ALG_SHA1,
    CLS_AUTH_ALG_DES_MAC,
    CLS_AUTH_ALG_3DES_MAC,
    CLS_AUTH_ALG_SHA256,
    CLS_AUTH_ALG_HMAC_SHA256
} CLS_AUTH_ALG;

typedef enum cls_auth_mode {
    CLS_AUTH_MODE_ALL      =  0,
    CLS_AUTH_MODE_INIT,
    CLS_AUTH_MODE_UPDATE,
    CLS_AUTH_MODE_FINAL
} CLS_AUTH_MODE;

typedef enum cls_math_modes {
    CLS_CRYPTO_MATH_MODADD =  0,
    CLS_CRYPTO_MATH_MODSUB,
    CLS_CRYPTO_MATH_MODMUL,
    CLS_CRYPTO_MATH_MODEXP,
    CLS_CRYPTO_MATH_MODREM,
    CLS_CRYPTO_MATH_MODINV
} CLS_CRYPTO_MATH;

typedef enum cls_cipher_modes {
	CLS_CIPHER_MODE_NULL   =  0,
	CLS_CIPHER_MODE_ENCRYPT,
	CLS_CIPHER_MODE_DECRYPT,
	CLS_CIPHER_MODE_AUTHONLY
} CLS_CIPHER_MODE;

typedef enum cls_cipher_order {
	CLS_CIPHER_ORDER_NULL  =  0,
	CLS_CIPHER_ORDER_AUTH_CRYPT,
	CLS_CIPHER_ORDER_CRYPT_AUTH
} CLS_CIPHER_ORDER;


#define BITS_PER_BYTE		     8
/* sizes. all sizes (not specified ) are in bytes */
#define DEFAULT_RAW_RNG_SIZE_WORD    5
#define MIN_RAW_RNG_SIZE_WORD        5 /* 20 bytes, 160 bit */
#define MAX_RAW_RNG_SIZE_WORD        16/* 64 bytes, 512 bit */

#define MAX_CRYPT_KEY_LENGTH         256
#define DES_BLOCK_SIZE               8
#define AES_IV_SIZE                  16
#define SHA1_HASH_SIZE               20
#define SHA1_BLOCK_SIZE              20 /* 160 bit block */
#define SHA256_BLOCK_SIZE            64 /* 512 bit block */
#define SHA256_HASH_SIZE             32
#define SHA256_KEY_SIZE              32

#define DSA_SIZE_SIGNATURE           40
#define DSA_SIZE_V                   20
#define DSA_SIZE_P_DEFAULT           128 /* 1024 bit. p: 512 bit to 1024 bit prime */
#define DSA_SIZE_PUB_DEFAULT         128
#define DSA_SIZE_G                   128
#define DSA_SIZE_Q                   20
#define DSA_SIZE_X                   20
#define DSA_SIZE_RANDOM              20
#define DSA_SIZE_HASH                20
#define DSA_SIZE_R                   20
#define DSA_SIZE_S                   20

#define RSA_SIZE_MODULUS_DEFAULT     512 /* maximum RSA modulus size */
#define RSA_SIZE_SIGNATURE_DEFAULT   RSA_SIZE_MODULUS_DEFAULT

#define RSA_MAX_PUB_EXP_BITS 	(17)
#define RSA_MAX_PUB_EXP_BYTES	((RSA_MAX_PUB_EXP_BITS + BITS_PER_BYTE - 1) / BITS_PER_BYTE)
#define RSA_DEFAULT_PUB_EXP	(0x10001)
#define RSA_DEFAULT_PUB_EXP_WORD	((sizeof( uint32_t ) - RSA_MAX_PUB_EXP_BYTES) * BITS_PER_BYTE)
	/* 	The above constant is useful for initializing a word with the exponent when
		you only wish to use the first (i.e., upper) RSA_MAX_PUB_EXP_BYTES bytes of the
		word. */


#define DH_SIZE_MODULUS_DEFAULT      256 /* default DH modulus size */

/**********************************************************************
 *  SCAPI Structures
 *  It also includes DMA structures. So we word align.
 **********************************************************************/


typedef void scapi_callback (uint32_t status, void *handle, void *arg);

typedef struct cls_bulk_cipher_context_s {

	CLS_BULK_CIPHER_CMD cmd;

	struct _size {
		uint32_t    encr_key     : 16,   /* size in bytes */
		            auth_key     : 16;   /* is that possible Auth Key size > 16bit ??? */
	} size;

    uint8_t     crypto_key[MAX_CRYPT_KEY_LENGTH]; /* crypto key will always been copied here */

	uint8_t    *auth_key;

	uint32_t	auth_total_len;          /* for SHA init/update/final, remember the total length among calls */

	uint32_t    smau_header[SMAU_HEADER_NUM_WORDS]; /* number of words in smau client crypto header */
	uint32_t    smau_header_swapped[SMAU_HEADER_NUM_WORDS]; /* swapped endianness of the header */
	uint32_t    flags;

	/* DMA structures */
	dmac_t      dma_handle_in;  /* from memory to SMAU */
	LLI_t       LLI_in[SMAU_CLIENT_IN_MAX_DMA_PACKETS];

	dmac_t      dma_handle_out; /* from SMAU to memory */
	LLI_t       LLI_out[SMAU_CLIENT_OUT_MAX_DMA_PACKETS];

        /* 3DES key schedule. Whole context will be initialized to 0. */
        uint32_t    KnL[32];
	uint32_t    KnR[32];
	uint32_t    Kn3[32];

	/* callback functions for bulk crypto */
	scapi_callback *callback;
	void     *arg;

} cls_bulk_cipher_context;


#define CLS_BULK_FLAGS_SWAP_IN      0x0001 /* swap endianness for input, this will require us to swap the smau header */
#define CLS_BULK_FLAGS_SWAP_OUT     0x0002 /* swap endianness for output */
#define CLS_BULK_FLAGS_SWAP_BYTE    0x0004 /* swap in bytes or 16bit words */
#define CLS_BULK_FLAGS_BIGENDIAN    0x0008 /* swap in bytes or 16bit words */

/* fips rng context */
typedef struct _fips_rng_context
{
	uint32_t raw_rng;                            /* rng directly from rng core, we compare/save one word (minimum is 16bit) */
	uint32_t sha_rng[DEFAULT_RAW_RNG_SIZE_WORD]; /* rng after sha1 */
} fips_rng_context;

/* RNG types */
#define CLS_RNG_TYPE_DSA_X                        0x0001 /* FIPS 186-2 original standard */
#define CLS_RNG_TYPE_DSA_K                        0x0002
#define CLS_RNG_TYPE_DSA_X_CHG                    0x0004 /* FIPS 186-2 change notice dated 10/05/2001 */
#define CLS_RNG_TYPE_DSA_K_CHG                    0x0008  
#define CLS_RNG_TYPE_OTHER                        0x0010 /* General Purpose RNG, "mod q" is omitted */
#define CLS_RNG_TYPE_SKIP_CONTINUOUS_CHK          0x0020
#define CLS_RNG_TYPE_MASK_ORG                     (CLS_RNG_TYPE_DSA_X | CLS_RNG_TYPE_DSA_K) 
#define CLS_RNG_TYPE_MASK_CHG                     (CLS_RNG_TYPE_DSA_X_CHG | CLS_RNG_TYPE_DSA_K_CHG) 
#define CLS_RNG_TYPE_MASK_K                       (CLS_RNG_TYPE_DSA_K | CLS_RNG_TYPE_DSA_K_CHG) 

/*
 * crypto context, we need it here to get the size
 * crypto context space is allocated by upper layer
 */
typedef struct _crypto_lib_handle
{
	uint32_t        cmd;
	uint32_t        busy;
	uint8_t        *result;
	uint32_t        result_len;
	uint32_t       *presult_len; /* tell the caller the result length */
	scapi_callback *callback;    /* callback functions for async calls */
	void           *arg;

	/* context pointers */
	/* These two context can exist at the same time, so can not share one pointer */
	cls_bulk_cipher_context *bulkctx; /* points to current active async bulk cipher context */
	fips_rng_context        *rngctx;  /* pionts to the fips rng context */
} crypto_lib_handle;

#define NUM_CRYPTO_CTX       3  /* PKE/MATH, BULK, RNG */
#define CLS_CRYPTO_LIB_HANDLE_SIZE (NUM_CRYPTO_CTX*sizeof(crypto_lib_handle))
/* crypto devices bitmask */
#define CRYPTO_DEVICE_BULK    0x01
#define CRYPTO_DEVICE_PKEMATH 0x02
#define CRYPTO_DEVICE_RNG     0x04

#define HMAC_IPAD    0x36
#define HMAC_OPAD    0x5C

#define SHA_INIT_STATE_NUM_WORDS 5
#define SHA_INIT_STATE0          0x67452301
#define SHA_INIT_STATE1          0xEFCDAB89
#define SHA_INIT_STATE2          0x98BADCFE
#define SHA_INIT_STATE3          0x10325476
#define SHA_INIT_STATE4          0xC3D2E1F0


/* this structure is used for DES-CBC */
struct _des_block{
        union {
                uint8_t  char_block[DES_BLOCK_SIZE];
                uint32_t long_block[2];
        } block;
};

/* crypto context device index */
#define CRYPTO_INDEX_BULK    0
#define CRYPTO_INDEX_PKEMATH 1
#define CRYPTO_INDEX_RNG     2


#define CHECK_DEVICE_BUSY(nDevice) \
                { \
                        if (callback != NULL && pHandle -> busy == TRUE) \
                                return CLS_STATUS_DEVICE_BUSY; \
                }

#define TIME_DELAY_USEC_PKE 5000000          /* 5000ms = 5000,000 us */

/* for PKCS#1 EMSA_PKCS1_v1-5 encoding */
#define EMSA_PKCS1_V15_PS_LENGTH        8
#define EMSA_PKCS1_V15_PS_BYTE          0xff

#define SHA1_DER_PREAMBLE_LENGTH     15 /* refer to PKCS#1 v2.1 P38 section 9.2 */
#define SHA256_DER_PREAMBLE_LENGTH   19 /* refer to PKCS#1 v2.1 P38 section 9.2 */

#endif
