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
*  vchiq.h
*
*  PURPOSE:
*
*       This file describes low-level macros and variables, typically
*       used for debugging purposes, with the vchiq stack.
*
*****************************************************************************/

#if !defined( LINUX_BROADCOM_VCHIQ_H )
#define LINUX_BROADCOM_VCHIQ_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/kernel.h>
#include <stdarg.h>

/* ---- Constants and Types ---------------------------------------------- */

/* ---- Variable Externs ------------------------------------------------- */

#if defined( __KERNEL__ )
#   define VC_DEBUG_ENABLED    1       /* Debug flag for kernel mode */
#else
#   define VC_DEBUG_ENABLED    1       /* Debug flag for user mode */
#endif

#if VC_DEBUG_ENABLED

extern  int gVcDebugIrqTrace;
extern  int gVcDebugGpioIrqTrace;

#   if defined( __KERNEL__ )
        
#       include <linux/broadcom/knllog.h>

        /**
         *  @brief Debug print macro.  Printing is conditional based on
         *         the 'flag' argument which in turn is linked to a
         *         debug-... proc entry--see \ref procIface
         *  
         *  VC_DEBUG is the preferred macro to use. It uses a bitmask to
         *  allow printk and/or KNLLOG (or maybe other) forms of logging
         *  based on the value of the debug flag.
         *  
         *  VC_DEBUG2 will disappear
         *  
         *  VC_PRINTK should be used where the options passed in are
         *  potentially too complex for KNLLOG to be used, and only
         *  printk is appropriate
         *  
         *  VC_KNLLOG will only do a KNLLOG and not a printk
         */
#       define VC_DEBUG(flag, fmt, args...)     \
            do                                  \
            {                                   \
                if ( gVcDebug##flag )           \
                {                               \
                    vchost_log( __FUNCTION__, gVcDebug##flag, fmt, ## args ); \
                }                               \
            } while (0)

#       define VC_DEBUG2(flag, fmt, args...) if ( gVcDebug##flag ) printk( fmt, ##args ) /* for kernel mode */
#       define VC_PRINTK(flag, fmt, args...) if ( gVcDebug##flag ) printk( fmt, ##args ) /* for kernel mode */
#       define VC_KNLLOG(flag, fmt, args...) if ( gVcDebug##flag ) KNLLOG( fmt, ## args ) /* for kernel mode */
#       define VC_DUMP_MEM(flag, addr, mem, numBytes ) if ( gVcDebug##flag ) vchost_log_dump_mem( __FUNCTION__, gVcDebug##flag, addr, mem, numBytes );

#   else
        /**
         *  @brief Debug print macro.  Printing is conditional based on
         *         the 'flag' argument which in turn is linked to a
         *         debug-... proc entry--see \ref procIface
         */
#       define VC_DEBUG(flag, fmt, args...)  if ( gVcDebug##flag ) printf( "%s: " fmt, __FUNCTION__, ## args )
#       define VC_DEBUG2(flag, fmt, args...) if ( gVcDebug##flag ) printf( fmt, ##args )
#       define VC_PRINTK(flag, fmt, args...) if ( gVcDebug##flag ) printf( "%s: " fmt, __FUNCTION__, ##args )
#       define VC_KNLLOG(flag, fmt, args...) if ( gVcDebug##flag ) printf( fmt, ## args )
#       define VC_DUMP_MEM(flag, addr, mem, numBytes )

#   endif
#else

#   define VC_DEBUG(flag, fmt, args...)
#   define VC_DEBUG2(flag, fmt, args...)

#endif /* VC_DEBUG_ENABLED */

/* ---- Function Prototypes ---------------------------------------------- */

extern void (*vchiq_proc_logging_filter_changed)( uint32_t mask );

void vchost_vlog( const char *function, int logType, const char *fmt, va_list args );
void vchost_log( const char *function, int logType, const char *fmt, ... );
void vchost_log_dump_mem( const char *function, int logType, uint32_t addr, const void *mem, size_t numBytes );

void vc_dump_mem( const char *label, uint32_t addr, const void *voidMem, size_t numBytes );

void vchost_set_interface_for_run( void );

#endif  /* LINUX_BROADCOM_VCHIQ_H */

