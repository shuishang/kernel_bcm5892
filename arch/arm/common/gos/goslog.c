/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
*  @file    goslog.c
*
*  @brief   This file implements the Generic OS (gos) logging abstraction.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/types.h>                     /* For standard types */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/preempt.h>
#include <linux/broadcom/knllog.h>
#include <linux/broadcom/vchiq.h>
#include <linux/broadcom/gos/gos.h>              /* GOS API */

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
#define MAX_LABEL_LENGTH 50
#define MAX_LOG_LENGTH 200

/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Function Prototypes -------------------------------------- */

static inline int LogToConsole( int flags )
{
    return ( flags & GOS_LOG_CONSOLE_FLAG ) != 0;
}

#ifdef CONFIG_BCM_KNLLOG_SUPPORT
static inline int LogToKnlLog( int flags )
{
    return ( flags & GOS_LOG_KNLLOG_FLAG ) != 0;
}
static inline int LogToKnlLogProfiling( int flags )
{
    return ( flags & GOS_LOG_PROFILING_FLAG ) != 0;
}
#endif

/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  Logging function used for reporting debug prints
*
*  @return Nothing
*/
static void gos_vlog(
   const char *cat1,                /**< (i) First Log label */
   const char *cat2,                /**< (i) Second Log label */
   int         flags,               /**< (i) Conditional flags */
   const char *fmt,                 /**< (i) Printf format string */
   va_list     args                 /**< (i) Variable arguments */
)
{
   const char *label;

   if ( LogToConsole( flags ))
   {
      char vlog[MAX_LOG_LENGTH];
      char label_buffer[MAX_LABEL_LENGTH] = "";
      int len;

      if ( cat1 )
      {
         label = cat1;
         if ( cat2)
         {
            snprintf( label_buffer, MAX_LABEL_LENGTH, "%s:%s", cat1, cat2 );
            label_buffer[MAX_LABEL_LENGTH - 1] = '\0';
            label = label_buffer;
         }
      }
      else if ( cat2 )
      {
         label = cat2;
      }
      else
      {
         label = "";
      }

      vsnprintf( vlog, MAX_LOG_LENGTH, fmt, args );
      vlog[MAX_LOG_LENGTH - 1] = '\0';

      len = strlen(vlog);
      if( vlog[len-1] == '\n' ) vlog[len-1] = '\0';

      printk( KERN_WARNING "%s: %s\n", label, vlog );
   }

#ifdef CONFIG_BCM_KNLLOG_SUPPORT
   if( cat2 ) label = cat2;
   else if( cat1 ) label = cat1;
   else label = "";

   if ( LogToKnlLog( flags ))
   {
      knllog_ventry( label, fmt, args );
   }
   if ( LogToKnlLogProfiling( flags ) && ( gKnllogIrqSchedEnable & KNLLOG_PROFILING ) )
   {
      knllog_ventry( label, fmt, args );
   }
#endif
}

/***************************************************************************/
/**
*  Used to dump memory bytes to the console
*
*  @return Nothing
*/
void gos_dump_mem(
   const char    *label,            /**< (i) Label prefix */
   unsigned int   addr,             /**< (i) Numerical address */
   const void    *voidMem,          /**< (i) Pointer to memory */
   size_t         numBytes          /**< (i) Number of bytes to dump */
)
{
   const uint8_t  *mem = (uint8_t *)voidMem;
   size_t          offset;
   char            lineBuf[ 100 ];
   char           *s;

   while ( numBytes > 0 )
   {
      s = lineBuf;

      for ( offset = 0; offset < 16; offset++ )
      {
         if ( offset < numBytes )
         {
            s += sprintf( s, "%02x ", mem[ offset ]);
         }
         else
         {
            s += sprintf( s, "   " );
         }
      }

      for ( offset = 0; offset < 16; offset++ )
      {
         if ( offset < numBytes )
         {
            uint8_t ch = mem[ offset ];

            if (( ch < ' ' ) || ( ch > '~' ))
            {
               ch = '.';
            }
            *s++ = (char)ch;
         }
      }
      *s++ = '\0';

      printk( KERN_WARNING "%s %08x: %s\n", label, addr, lineBuf );

      addr += 16;
      mem += 16;
      if ( numBytes > 16 )
      {
         numBytes -= 16;
      }
      else
      {
         numBytes = 0;
      }
   }
}

/***************************************************************************/
/**
*  Logging routine like printf. Logging is conditional based on the
*  'flags' argument.  The 'flags' argument is a bitmask to allow printk
*  and/or KNLLOG (or maybe other) forms of logging.
*
*  @return Nothing
*/
void gosLog(
   const char    *cat1,  /**< (i) Name of 1st category for logging */
   const char    *cat2,  /**< (i) Name of 2nd category for logging (e.g. function name)*/
   int            flags, /**< (i) Conditional logging flags */
   const char    *fmt,   /**< (i) Printf format string */
   ...
)
{
   va_list args;

   va_start( args, fmt );
   gos_vlog( cat1, cat2, flags, fmt, args );
   va_end( args );
}
EXPORT_SYMBOL( gosLog );

/***************************************************************************/
/**
*  Logging routine to dump memory. Logging is conditional based on the
*  'flags' argument. The 'flags' argument is a bitmask to allow printk
*  and/or KNLLOG (or maybe other) forms of logging.
*
*  @return Nothing
*/
void gosLogDumpMem(
   const char    *cat1,  /**< (i) Name of 1st category for logging */
   const char    *cat2,  /**< (i) Name of 2nd category for logging (e.g. function name)*/
   int            flags, /**< (i) Conditional logging flags */
   unsigned int   addr,  /**< (i) Numerical address of memory to dump */
   const void    *mem,   /**< (i) Pointer to memory location */
   unsigned int   bytes  /**< (i) Number of bytes to dump */
)
{
   char label[MAX_LABEL_LENGTH];

   snprintf( label, MAX_LABEL_LENGTH, "%s:%s", cat1, cat2 );
   label[MAX_LABEL_LENGTH - 1] = '\0';

   if ( LogToConsole( flags ))
   {
      gos_dump_mem( label, addr, mem, bytes );
   }

#ifdef CONFIG_BCM_KNLLOG_SUPPORT
   if ( LogToKnlLog( flags ))
   {
      knllog_dump_mem( label, addr, mem, bytes );
   }
#endif
}
EXPORT_SYMBOL( gosLogDumpMem );
