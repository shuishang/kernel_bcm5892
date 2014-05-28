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
*  pka.h
*
*  PURPOSE:
*
*  This file defines the interface to the PKA driver.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_PKA_H )
#define LINUX_PKA_H

/* ---- Include Files ---------------------------------------------------- */
/* ---- Constants and Types ---------------------------------------------- */
/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

/*!
******************************************************************
*   QLIP main header file for basic long integer structures, functions  (q_lip.h)
******************************************************************
*/

/* QLIP status */
#define Q_SUCCESS                            0
#define Q_ERR_CTX_MEM_SIZE_LOW              -1
#define Q_ERR_CTX_ERR                       -2
#define Q_ERR_CTX_OVERFLOW                  -3
#define Q_ERR_MEM_DEALLOC                   -4
#define Q_ERR_DST_OVERFLOW                  -5
#define Q_ERR_DIV_BY_0                      -6
#define Q_ERR_NO_MODINV                     -7

#define Q_ERR_EC_PT_NOT_AFFINE              -10
#define Q_ERR_PKA_HW_ERR                    -100
#define Q_ERR_PPSEL_FAILED                  -200

#define Q_ERR_GENERIC                       -1000

/* Type definition */
#ifndef _LINUX_TYPES_H
#ifndef _STDINT_H
#ifndef int32_t
typedef int int32_t;
#endif

#ifndef uint32_t
typedef unsigned int        uint32_t;
#endif

#ifndef uint16_t
typedef unsigned short int  uint16_t;
#endif

#ifndef uint8_t
typedef unsigned char       uint8_t;
#endif

#endif /* _STDINT_H */
#endif /* _LINUX_TYPES_H */

typedef uint32_t            q_limb_t;
typedef q_limb_t*           q_limb_ptr_t;
typedef uint32_t            q_size_t;
typedef int32_t             q_status_t;

/* the long integer */
typedef struct q_lint {
  q_limb_t *limb;     /*!< pointer to long integer value */
  int size;           /*!< the size of the long integer in words */
  int alloc;          /*!< the memory allocated for the long integer */
  int neg;            /*!< sign flag of the long integer */
} q_lint_t; /*!< data structure of the long integer */

typedef struct q_mont {
  q_lint_t n;
  q_lint_t np;
  q_lint_t rr;
  int br;
} q_mont_t; /*!< data structure of the Montgomery fields */

typedef struct q_lip_ctx {
  q_status_t status;         /*!< QLIP status */
  q_status_t (*q_yield)(void);   /*!< QLIP yield function pointer */
  uint32_t CurMemLmt;        /*!< QLIP context data memory size in words */
  uint32_t CurMemPtr;        /*!< Pointer that points to the next unused QLIP context address */
  uint32_t *CtxMem;          /*!< Pointer to the start of QLIP context data memory */
} q_lip_ctx_t; /*!< data structure of the QLIP context */

typedef struct q_signature {
  q_lint_t r;
  q_lint_t s;
} q_signature_t; /*!< data structure of the DSA or EC-DSA signature */

/*!
******************************************************************
*   header file for DSA functions  (q_dsa.h)
******************************************************************
*/

typedef struct q_dsa_param {
  q_lint_t p;
  q_lint_t q;
  q_lint_t g;
} q_dsa_param_t; /*!< data structure of the DSA parameter */


/*!
******************************************************************
*   Header file for RSA functions  (q_rsa.h)
*   RSA encryption can be implemented using modexp function. For
*   embedded system with limited ROM size, RSA encryption function is
*   optional.
*
*   ppsel function is included here because it is used for RSA key generation.
******************************************************************
*/

typedef struct q_rsa_key {
  q_lint_t n;
  q_lint_t e;
} q_rsa_key_t; /*!< data structure of the RSA public key */

typedef struct q_rsa_crt_key {
  q_lint_t p;
  q_lint_t q;
  q_lint_t dp;
  q_lint_t dq;
  q_lint_t qinv;
} q_rsa_crt_key_t; /*!< data structure of the RSA CRT private key */



#endif /* LINUX_PKA_H */
