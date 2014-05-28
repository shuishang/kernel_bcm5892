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
#if !defined( STD_ARPA_INET_H )
#define STD_ARPA_INET_H

/* ---- Include Files ---------------------------------------------------- */
#include <linux/broadcom/gos/gos_basic_types.h>

/* ---- Constants and Types ---------------------------------------------- */
#ifdef htonl
#undef htonl
#endif

#ifdef htons
#undef htons
#endif

#ifdef ntohl
#undef ntohl
#endif

#ifdef ntohs
#undef ntohs
#endif

#define htonl htonl_kernel
#define htons htons_kernel
#define ntohl ntohl_kernel
#define ntohs ntohs_kernel

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */
extern uint32_t htonl_kernel(uint32_t hl);
extern uint16_t htons_kernel(uint16_t hs);
extern uint32_t ntohl_kernel(uint32_t nl);
extern uint16_t ntohs_kernel(uint16_t ns);

#endif   /* STD_ARPA_INET_H */