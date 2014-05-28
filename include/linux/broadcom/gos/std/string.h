/*****************************************************************************
* Copyright 2009 Broadcom Corporation.  All rights reserved.
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
#if !defined( STD_STRING_H )
#define STD_STRING_H

/* ---- Include Files ---------------------------------------------------- */
#include <linux/broadcom/gos/gos_basic_types.h>

/* ---- Constants and Types ---------------------------------------------- */
#ifdef memset
#undef memset
#endif

#define memset memset_kernel

#ifdef strcpy
#undef strcpy
#endif

#define strcpy strcpy_kernel

#ifdef strncpy
#undef strncpy
#endif

#define strncpy strncpy_kernel

#ifdef strcmp
#undef strcmp
#endif

#define strcmp strcmp_kernel

#ifdef strncmp
#undef strncmp
#endif

#define strncmp strncmp_kernel

#ifdef strlen
#undef strlen
#endif

#define strlen strlen_kernel

#ifdef memcpy
#undef memcpy
#endif

#define memcpy memcpy_kernel

#ifdef memcmp
#undef memcmp
#endif

#define memcmp memcmp_kernel

#ifdef memmove
#undef memmove
#endif

#define memmove memmove_kernel

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */
extern void* memset_kernel(void *s, int c, size_t n);

extern char *strcpy_kernel(char *dst,const char *src);

extern int strcmp_kernel(const char *src1, const char *src2);

extern int strncmp_kernel(const char *src1, const char *src2, size_t s);

extern char *strncpy_kernel(char *dst,const char *src, size_t s);

extern int strlen_kernel(const char *s);

extern void* memcpy_kernel(void *dst, const void *src, size_t n);

extern int memcmp_kernel(const void *src1, const void *src2, size_t n);

extern void* memmove_kernel(void *to, const void *from, size_t s);

#endif   /* STD_STRING_H */
