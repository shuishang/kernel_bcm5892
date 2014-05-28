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
 *  File: cls_scapi.h
 *  Description: Crypto apis.
 *
 *
 *****************************************************************************/
#ifndef _CLS_SCAPI_H_
#define _CLS_SCAPI_H_

#include "cls_bulk_hw.h"
/*#include "sha256.h" */
#include "cls_scapi_defines.h"

CLS_STATUS cls_smu_mem_init(void);
CLS_STATUS cls_smu_test_config(int mode);
CLS_STATUS cls_smu_test_sweep(int num_xfers, uint32_t start_addr);

CLS_STATUS cls_smu_config_win(int win, uint32_t base, uint32_t size, uint32_t hmac_base, int spi_mapped, int auth_bypass, int enc_bypass);
CLS_STATUS cls_bulk_cipher_all( CLS_BULK_CIPHER_CMD *cmd, uint32_t *enc_key, uint32_t *auth_key, uint32_t *iv, uint8_t *src_addr, uint8_t *dst_addr, uint32_t size, uint32_t swap);
CLS_STATUS cls_force_vld_dty_cam(void);

/**********************************************************************
 *  SCAPI Function Prototypes
 **********************************************************************/

CLS_STATUS cls_bulk_cipher_single (
                    CLS_BULK_CIPHER_CMD cmd, uint32_t memAddrSrc, uint32_t numWords,
                    uint32_t memAddrDst, int memBrst4, int smauBrst4);

void cls_crypto_init (crypto_lib_handle *pHandle);

void cls_bulk_cipher_swap_endianness (crypto_lib_handle *pHandle, cls_bulk_cipher_context *ctx,
                        uint32_t bSwapIn, uint32_t bSwapOut, uint32_t bSwapByte);

CLS_STATUS cls_bulk_cipher_init (
                    crypto_lib_handle *pHandle, CLS_BULK_CIPHER_CMD cmd,
                    uint8_t *encr_key, uint8_t *auth_key, uint32_t auth_key_len, cls_bulk_cipher_context *ctx);

CLS_STATUS cls_bulk_cipher_start (
                    crypto_lib_handle *pHandle, uint8_t *data_in, uint8_t *iv,
                    uint32_t crypto_len, uint16_t crypto_offset,
                    uint32_t auth_len,   uint16_t auth_offset,   uint8_t *output,
                    cls_bulk_cipher_context *ctx);

CLS_STATUS cls_bulk_cipher_dma_start (crypto_lib_handle *pHandle, cls_bulk_cipher_context *ctx,
                        /*uint32_t nDMAC,*/ scapi_callback callback, void *arg);

uint32_t cls_check_crypto_completion (crypto_lib_handle *pHandle, uint32_t device_bitmask);


CLS_STATUS cls_get_hmac_sha1_hash(crypto_lib_handle *pHandle, uint8_t *pMsg, uint32_t nMsgLen,
                                                        uint8_t *pKey, uint32_t nKeyLen, uint8_t *output, uint32_t bSwap);

CLS_STATUS cls_get_hmac_sha256_hash(crypto_lib_handle *pHandle, uint8_t *pMsg, uint32_t nMsgLen,
                                                        uint8_t *pKey, uint32_t nKeyLen, uint8_t *output, uint32_t bSwap);

CLS_STATUS cls_fips_selftest_hmac_sha1(crypto_lib_handle *pHandle);
CLS_STATUS cls_fips_selftest_hmac_sha256(crypto_lib_handle *pHandle);
CLS_STATUS cls_fips_selftest_aes_cbc(void);

CLS_STATUS cls_fips_selftest_dsa_verify(crypto_lib_handle *pHandle);
CLS_STATUS cls_fips_selftest_dsa_pairwise(crypto_lib_handle *pHandle, uint8_t *privkey, uint8_t *pubkey);
CLS_STATUS cls_fips_selftest_pkcs1_rsassa_v15_sig_verify(crypto_lib_handle *pHandle, uint32_t size, bool use_sha256);
CLS_STATUS cls_fips_selftest_modexp(crypto_lib_handle *pHandle);

/* Added for SW team 08/06/2007 */

CLS_STATUS cls_diffie_hellman_generate (
                    crypto_lib_handle *pHandle,
                    uint8_t *x,      uint32_t x_bitlen,
                    uint32_t xvalid,
                    uint8_t *y,      uint32_t *y_bytelen,
                    uint8_t *g,      uint32_t g_bitlen,
                    uint8_t *m,      uint32_t m_bitlen,
                    scapi_callback callback, void *arg);


CLS_STATUS cls_diffie_hellman_shared_secret (
                                         crypto_lib_handle *pHandle,
                     uint8_t *x, uint32_t x_bitlen,
                     uint8_t *y, uint32_t y_bitlen,
                     uint8_t *m, uint32_t m_bitlen,
                     uint8_t *k, uint32_t *k_bytelen,
                     scapi_callback callback, void *arg);


CLS_STATUS cls_rsa_mod_exp (
                     crypto_lib_handle *pHandle,
                     uint8_t *x, uint32_t x_bitlen,
                     uint8_t *m, uint32_t m_bitlen,
                     uint8_t *e, uint32_t e_bitlen,
                     uint8_t *y, uint32_t *y_bytelen,
                     scapi_callback callback, void *arg);

CLS_STATUS cls_rsa_mod_exp_crt (
                    crypto_lib_handle *pHandle,
                    uint8_t *x,    uint32_t x_bitlen,
                    uint8_t *edq,
                    uint8_t *q,    uint32_t q_bitlen,
                    uint8_t *edp,
                    uint8_t *p,    uint32_t p_bitlen,
                    uint8_t *pinv,
                    uint8_t *y,    uint32_t *y_bytelen,
                    scapi_callback callback, void *arg);

CLS_STATUS cls_dsa_sign (
                                  crypto_lib_handle *pHandle,
                  uint8_t *hash,
                  uint8_t *random,
                  uint8_t *p,      uint32_t p_bitlen,
                  uint8_t *q,
                  uint8_t *g,
                  uint8_t *x,
                  uint8_t *rs,     uint32_t *rs_bytelen,
                  scapi_callback   callback, void *arg);

CLS_STATUS cls_dsa_verify (
                    crypto_lib_handle *pHandle,
                    uint8_t *hash,
                    uint8_t *p,    uint32_t p_bitlen,
                    uint8_t *q,
                    uint8_t *g,
                    uint8_t *y,
                    uint8_t *r,    uint8_t  *s,
                    uint8_t *v,    uint32_t *v_bytelen,
                    scapi_callback callback, void *arg);

CLS_STATUS cls_ecp_diffie_hellman_generate (
                  crypto_lib_handle *pHandle,
                  uint8_t type,
                  uint8_t *p,      uint32_t p_bitlen,
                  uint8_t *a,
                  uint8_t *b,
                  uint8_t *n,
                  uint8_t *Gx,
                  uint8_t *Gy,
                  uint8_t *d, 
                  uint32_t dvalid,
                  uint8_t *Qx,
                  uint8_t *Qy,
                  scapi_callback callback, void *arg);
 
CLS_STATUS cls_ecp_diffie_hellman_shared_secret (
                  crypto_lib_handle *pHandle,
                  uint8_t type,
                  uint8_t *p,      uint32_t p_bitlen,
                  uint8_t *a,
                  uint8_t *b,
                  uint8_t *n,
                  uint8_t *d,
                  uint8_t *Qx,
                  uint8_t *Qy,
                  uint8_t *Kx,
                  uint8_t *Ky,
                  scapi_callback callback, void *arg);

CLS_STATUS cls_ecp_ecdsa_sign (
                  crypto_lib_handle *pHandle,
                  uint8_t *hash,
                  uint8_t type,
                  uint8_t *p,      uint32_t p_bitlen,
                  uint8_t *a,
                  uint8_t *b,
                  uint8_t *n,
                  uint8_t *Gx,
                  uint8_t *Gy,
                  uint8_t *k,
                  uint32_t kvalid,
                  uint8_t *d,
                  uint8_t *r,
                  uint8_t *s,
                  scapi_callback   callback, void *arg);

CLS_STATUS cls_ecp_ecdsa_verify (
                  crypto_lib_handle *pHandle,
                  uint8_t *hash,
                  uint8_t type,
                  uint8_t *p,      uint32_t p_bitlen,
                  uint8_t *a,
                  uint8_t *b,
                  uint8_t *n,
                  uint8_t *Gx,
                  uint8_t *Gy,
                  uint8_t *Qx,
                  uint8_t *Qy,
                  uint8_t *Qz,
                  uint8_t *r,    uint8_t  *s,
                  uint8_t *v,
                  scapi_callback callback, void *arg);

CLS_STATUS cls_rsassa_pkcs1_v15_verify(crypto_lib_handle *pHandle,
                                        uint32_t nLen, uint8_t *n, /* modulus */
                                        uint32_t eLen, uint8_t *e, /* exponent */
                                        uint32_t MLen, uint8_t *M, /* message */
                                        uint32_t SLen, uint8_t *S,  /* signature */
                                        bool use_sha256); /* Use SHA256 as digest
												  algorithm for PKCS encoding */

CLS_STATUS cls_math_accelerate (
                    crypto_lib_handle *pHandle,
                    uint32_t         cmd,
                    uint8_t *modN,   uint32_t  modN_bitlen,
                    uint8_t *paramA, uint32_t  paramA_bitlen,
                    uint8_t *paramB, uint32_t  paramB_bitlen,
                    uint8_t *result, uint32_t  *result_bytelen,
                    scapi_callback callback, void *arg);

CLS_STATUS cls_dsa_key_generate (crypto_lib_handle *pHandle,
                                 uint8_t *p,
                                 uint32_t p_bitlen,
                                 uint8_t *q,
                                 uint32_t q_bitlen,
                                 uint8_t *g,
                                 uint8_t *privkey,
                                 uint8_t *pubkey);

CLS_STATUS cls_rng_fips_init (
                    crypto_lib_handle *pHandle, fips_rng_context *rngctx, uint32_t bSkipGen);

CLS_STATUS cls_rng_fips_generate(crypto_lib_handle *pHandle, uint32_t *raw_rng, uint32_t b,
                                 uint32_t m, uint32_t *q, uint32_t type, uint32_t *result);


CLS_STATUS cls_fips_selftest_modinv(crypto_lib_handle *pHandle);

CLS_STATUS cls_fips_rng_selftest (crypto_lib_handle *pHandle);
void dpa_stall_enable(void);
CLS_STATUS cls_rng_raw_generate (crypto_lib_handle *pHandle,uint8_t *result,
				 uint32_t num_words, scapi_callback callback, 
                                                                   void *arg);

#endif

