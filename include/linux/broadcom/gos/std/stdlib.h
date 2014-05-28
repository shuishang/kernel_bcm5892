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
#if !defined( STD_STDLIB_H )
#define STD_STDLIB_H

/* ---- Include Files ---------------------------------------------------- */
#include <linux/broadcom/gos/gos.h>

/* ---- Constants and Types ---------------------------------------------- */
#ifdef atoi
#undef atoi
#endif

#define atoi(str) atoi_kernel(str)

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */
extern int atoi(const char * str);

extern int  rand( void );
extern void srand( unsigned int seed );

#endif   /* STD_STDLIB_H */