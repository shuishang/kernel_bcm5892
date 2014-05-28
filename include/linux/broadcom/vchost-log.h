/*****************************************************************************
* Copyright 2001 - 2009 Broadcom Corporation.  All rights reserved.
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

/****************************************************************************
*
*  vchost-log.h
*
*  PURPOSE:
*
*       This file describes some functions which are used for logging.
*
*****************************************************************************/

#if !defined( LINUX_BROADCOM_VCHOST_LOG_H )
#define LINUX_BROADCOM_VCHOST_LOG_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/kernel.h>
#include <stdarg.h>

/* ---- Constants and Types ---------------------------------------------- */

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

void vchost_vlog( const char *function, int logType, const char *fmt, va_list args );
void vchost_log( const char *function, int logType, const char *fmt, ... );
void vchost_log_dump_mem( const char *function, int logType, uint32_t addr, const void *mem, size_t numBytes );

void vc_dump_mem( const char *label, uint32_t addr, const void *voidMem, size_t numBytes );

// From vceb-host-interface-pif.c
void vchost_set_interface_for_run( void );

#endif  /* LINUX_BROADCOM_VCHOST_LOG_H */

