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
 *  File: cls_bulk_hw.h
 *  Description: Bulk accelaration defines.
 *
 *  Version: $Id: $
 *
 *****************************************************************************/
#ifndef _CLS_BULK_HW_H_
#define _CLS_BULK_HW_H_

/* 20 bytes header */
#define SMAU_HEADER_NUM_WORDS       5
#define SMAU_HEADER_INDEX_FLAGS     0
#define SMAU_HEADER_INDEX_PADLEN    1
#define SMAU_HEADER_INDEX_OFFSETS   2
#define SMAU_HEADER_INDEX_AESLEN    3
#define SMAU_HEADER_INDEX_AUTHLEN   4


/* 1st word */
#define SMAU_HEADER_AES_ENABLE      0x00000001
#define SMAU_HEADER_AUTH_ENABLE     0x00000002
#define SMAU_HEADER_AUTH_FIRST      0x00000004
#define SMAU_HEADER_AES_ENCRYPT     0x00000008

#define SMAU_HEADER_AES_CBC         0x00000000
#define SMAU_HEADER_AES_CTR         0x00000010
#define SMAU_HEADER_AES_ECB         0x00000020
#define SMAU_HEADER_AES_CMAC        0x00000060
#define SMAU_HEADER_AES_CCM         0x00000050

#define SMAU_HEADER_AES_KEYSIZE_128 0x00000000 /* 16 bytes */
#define SMAU_HEADER_AES_KEYSIZE_192 0x00000100 /* 24 bytes */
#define SMAU_HEADER_AES_KEYSIZE_256 0x00000200 /* 32 bytes */

#define SMAU_HEADER_AUTH_SHA2	    0x00000800 /* sha256 */
#define SMAU_HEADER_SHA256	    0x00000800 /* sha256 */

#define SMAU_HEADER_AUTH_CMD_HCTX   0x00001000 /* generate inner/outer hash context for hmac */
#define SMAU_HEADER_AUTH_CMD_HASH   0x00002000 /* simple sha1 hash, the SHA1_xxx below defines the hash type */
#define SMAU_HEADER_AUTH_CMD_HMAC   0x00004000 /* HMAC, starts from the key and default state */
#define SMAU_HEADER_AUTH_CMD_FMAC   0x00008000 /* HMAC, starts from inner/outer hash context  */

#define SMAU_HEADER_SHA_INIT       0x00010000 /* SHA1-INIT */
#define SMAU_HEADER_SHA_UPDATE     0x00020000 /* SHA1-UPDATE */
#define SMAU_HEADER_SHA_FINISH     0x00040000 /* SHA1-FINISH */
#define SMAU_HEADER_SHA_ALL        0x00080000 /* ALL OF 3 - will be added by HW */

#define SMAU_HEADER_CHECK_KEY_LEN   0x00100000 /* only valid with CMD_HMAC/FMAC */
                                               /* when set, auth_keysize is valid. when cleared, default key len is 20 bytes */
#define SMAU_HEADER_LEN_OVERRIDE    0x00200000 /* only valid with CMD_HASH & SHA1_FINISH */
                                               /* when set, pad_len overrides internal len counter */
#define SMAU_HEADER_OUTPUT_SUPPRESS 0x00400000 /* do not output plaintext in auth operation */

#define SMAU_HEADER_AUTH_KEYSIZE_SHIFT 24      /* bits 31-24. Must be divisible by 4 */

/* 2nd word is the pad length */
/* 3rd word: auth_offset | aes_offset, they all must be divisible by 4 */
/* 4th word: [15:0] aes_length , includes 128 bit IV, [31:16] auth_length */

#define SMAU_CLIENT_IN_MAX_DMA_PACKETS  60 /*6*/  /* 6 packets: SMAU header, AES key, SHA1 key (twice), IV/ctr, data */
#define SMAU_CLIENT_OUT_MAX_DMA_PACKETS 2  /* 2 packets: out data, and auth out */

#endif

