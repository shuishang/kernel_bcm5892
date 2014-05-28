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
/**
*
*  @file    gos_basic_types.h
*
*  @brief   Contains the Generic OS abstraction definitions for 
*           standard data type and constants.
*
*****************************************************************************/
#if !defined( GOS_BASIC_TYPES_H )
#define GOS_BASIC_TYPES_H

/* ---- Include Files ---------------------------------------------------- */
/* Definition of ISO C standard integer such as int8_t and uint8_t */
#ifdef __KERNEL__
   #include <linux/types.h>
#else
   #include <stdint.h>
   #include <stddef.h>
#endif

/* Definition of ISO C and POSIX errno */
#ifdef __KERNEL__
   #include <linux/errno.h>
#else
   #include <errno.h>
#endif

#define useconds_t suseconds_t

#ifdef __cplusplus
extern "C"
{
#endif

/* ---- Constants and Types ---------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif   /* GOS_BASIC_TYPES_H */
