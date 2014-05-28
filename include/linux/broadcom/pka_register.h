/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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
*
*****************************************************************************
*
*  pka_register.h
*
*  PURPOSE:
*
*  This file defines the registration interface to the PKA driver.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_PKA_REGISTER_H )
#define LINUX_PKA_REGISTER_H

/* ---- Include Files ---------------------------------------------------- */
#include <linux/broadcom/pka.h>

/* ---- Constants and Types ---------------------------------------------- */

/* A structure of plugin functions that the core pka.ko will register with pka_driver.ko */
typedef struct
{

q_status_t (*ctx_init) (q_lip_ctx_t *ctx,         	/* QLIP context pointer */
                       uint32_t *ctxDataMemPtr,  	/* QLIP data memory pointer */
                       uint32_t ctxDataMemSize,  	/* QLIP data memory size (in words) */
                       q_status_t (*yield)(void));  /* User supplied yield function pointer or NULL */

q_status_t (*init) (q_lip_ctx_t *ctx,    /* QLIP context pointer */
                   q_lint_t *z,      /* pointer to q_lint to be initialized */
                   q_size_t size);       /* maximum data word size */

q_status_t (*import) (q_lip_ctx_t *ctx,   /* QLIP context pointer */
                     q_lint_t *z,        /* destination q_lint pointer */
                     q_size_t size,      /* data size (in word) */
                     int order,          /* import order: 1 = BIG_NUM, -1 = normal */
                     int endian,         /* endianness: 0 = native, 1 = big, -1 = little */
                     const void *data);   /* source data pointer */

q_status_t (*rsa_enc) (q_lip_ctx_t *ctx, q_lint_t *c, q_rsa_key_t *rsa, q_lint_t *m);
/*!< RSA encryption function
    \param ctx QLIP context pointer
    \param c cipher text
    \param rsa RSA public key structure
    \param m clear text
*/
q_status_t (*export) (void *data, int *size, int order, int endian, q_lint_t *a);
/*!< export the value of a long integer to a data array
    \param pointer to the data array
    \param size the size of the data
    \param order the long integer can be initialized in NORMAL (1) or BIGNUM order (-1)
    \param endian the Endianness to be applied
    \param a long integer exported
*/

q_status_t (*free) (q_lip_ctx_t *ctx, q_lint_t *z);
/*!< free memory allocated from the long integer from QLIP context
    \param ctx QLIP context pointer
    \param z the long integer
*/

q_status_t (*rsa_crt) (q_lip_ctx_t *ctx, q_lint_t *m, q_rsa_crt_key_t*rsa, q_lint_t *c);
/*!< RSA CRT decryption function
    \param ctx QLIP context pointer
    \param m clear text
    \param rsa RSA CRT private key structure
    \param c cipher text
*/

q_status_t (*dsa_sign) (q_lip_ctx_t *ctx, q_signature_t *rs, q_dsa_param_t *dsa, q_lint_t *d, q_lint_t *h, q_lint_t *k);
/*!< DSA digital signature signing function
    \param ctx QLIP context pointer
    \param rs generated DSA signature
    \param dsa DSA parameters
    \param d signing private key
    \param h hashed message to be signed
    \param k random
*/

q_status_t (*dsa_verify) (q_lip_ctx_t *ctx, q_lint_t *v, q_dsa_param_t *dsa, q_lint_t *y, q_lint_t *h, q_signature_t *rs);
/*!< DSA digital signature verification function
    \param ctx QLIP context pointer
    \param v computed signature verification value to be compared with rs->s
    \param dsa DSA parameters
    \param y DSA public key
    \param h hashed message to be signed
    \param rs generated DSA signature
*/

q_status_t (*cmp) (q_lint_t *a, q_lint_t *b);
/*!< signed long integer comparison (a>b) return 1; (a<b) return -1; (a==b) return 0;
    \param a source
    \param b source
*/

q_status_t (*pka_zeroize_mem) (q_lip_ctx_t *ctx);
/*!< Zeroize PKA internal memory using a data loading sequence
    \param ctx QLIP context pointer
*/

} pka_plugin_t;


/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */
int pka_driver_register_plugin(pka_plugin_t* p);
int pka_driver_deregister_plugin(void);

#endif /* LINUX_PKA_REGISTER_H */
