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
*  @file    gos.h
*
*  @brief   Contains the Generic OS abstraction definitions.
*
*****************************************************************************/
#if !defined( GOS_H )
#define GOS_H

/* ---- Include Files ---------------------------------------------------- */
#include <linux/broadcom/gos/gos_basic_types.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ---- Constants and Types ---------------------------------------------- */

/* Semaphore object */
typedef void * GOS_SEM;

/* Interrupt state used for nestable interrupt save and restore */
typedef unsigned long GOS_ISR_STATE;

/* Thread ID */
typedef void * GOS_THRD_ID;

/* Thread priorities */
typedef enum gos_thrd_prio
{
   GOS_THRD_PRIO_LOW = 0,        /**< Low priority thread */
   GOS_THRD_PRIO_MED_LOW,        /**< Medium-low priority thread */
   GOS_THRD_PRIO_MED,            /**< Medium priority thread */
   GOS_THRD_PRIO_MED_HIGH,       /**< Medium-high priority thread */
   GOS_THRD_PRIO_HIGH,           /**< High priority thread */
}
GOS_THRD_PRIO;

typedef enum gos_mem_type
{
   GOS_MEM_TYPE_UNSPECIFIED,     /**< GOS decides the memory type to use */
   GOS_MEM_TYPE_NOT_VIRTUAL,     /**< Do not use virtual memory */
   GOS_MEM_TYPE_VIRTUAL          /**< Use virtual memory */
}
GOS_MEM_TYPE;

/**
 * Debug print macro.  Printing is conditional based on the 'flags'
 * argument which in turn may be linked to a debug proc entry. The
 * flags arg is a bitmask to allow console print (like printk or
 * printf) and/or KNLLOG (or maybe other) forms of logging.
 */
#define GOS_DEBUG(flags, fmt, args...)   \
   do { if ( flags ) gosLog( NULL, __FUNCTION__, flags, fmt, ## args ); } while ( 0 )

/**
 * Debug memory dump macro. Dumping is conditional based on the 'flags'
 * argument which in turn may be linked to a debug proc entry. The
 * flags arg is a bitmask to allow console print (like printk or
 * printf) and/or KNLLOG (or maybe other) forms of logging.
 */
#define GOS_DUMP_MEM(flags, addr, mem, bytes)  \
   do { if ( flags ) gosLogDumpMem( "", __FUNCTION__, flags, addr, mem, bytes ); } while ( 0 )

/* Logging flags */
#define GOS_LOG_CONSOLE_FLAG        1
#define GOS_LOG_KNLLOG_FLAG         2
#define GOS_LOG_PROFILING_FLAG      4

/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

/***************************************************************************/
/**
*  Allocates a counting semaphore object
*
*  @return
*     0        Success, semp contains valid semaphore object
*     -ve      On general failure
*/
int gosSemAlloc(
   const char *name,                /**< (i) Name to assign to the semaphore */
   int         initval,             /**< (i) Initial value of the semaphore */
   GOS_SEM    *semp                 /**< (o) Place to return new semaphore */
);

/***************************************************************************/
/**
*  Frees a previously created counting semaphore object
*
*  @return
*     0        Success
*     -ve      On general failure
*/
int gosSemFree(
   GOS_SEM     sem                  /**< (i) Semaphore to free */
);

/***************************************************************************/
/**
*  Increments the count of the specified semaphore object.
*
*  @return
*     0        Success
*     -ve      On general failure
*/
int gosSemGive(
   GOS_SEM     sem                  /**< (i) Semaphore to give */
);

/***************************************************************************/
/**
*  Decrements the count of the specified semaphore object. The calling
*  thread may block.
*
*  @return
*     0        Success
*     -ve      On general failure
*/
int gosSemTake(
   GOS_SEM     sem                  /**< (i) Semaphore to take */
);

/***************************************************************************/
/**
*  Decrements the count of the specified semaphore object. The calling
*  thread may block with a timeout.
*
*  @return
*     0        Success
*     -ve      On general failure
*/
int gosSemTimedTake(
   GOS_SEM     sem,          /**< (i) Semaphore to take */
   int         timeout       /**< (i) Timeout (in ms) for semaphore take */
);

/* Mutex operations */
#define gosMutexAlloc(name, mutexp)    gosSemAlloc( name, 1, mutexp )
#define gosMutexFree                   gosSemFree
#define gosMutexGive                   gosSemGive
#define gosMutexTake                   gosSemTake

/***************************************************************************/
/**
*  Nestable interrupts disable
*
*  @return Interrupt state used to restore interrupts with
*/
GOS_ISR_STATE gosInterruptsSave( void );

/***************************************************************************/
/**
*  Nestable interrupts enable
*
*  @return Nothing
*/
void gosInterruptsRestore(
   GOS_ISR_STATE state              /**< (i) Interrupt state to restore */
);

/***************************************************************************/
/**
*  Blocks and sleeps for the specified period in milli-seconds
*
*  @return
*     0        Success
*     -ve      On general failure
*/
int gosSleepMs(
   unsigned int msec                /**< (i) Period in ms to sleep */
);

/***************************************************************************/
/**
*  Create a processing thread.
*
*  @return
*     0        Success
*     -ve      On general failure
*/
int gosThreadCreate(
   const char     *name,            /**< (i) Name to assign to the new thread */
   GOS_THRD_PRIO   priority,        /**< (i) Thread priority */
   int (*funcp)( void *arg ),       /**< (i) Thread processing function */
   void           *arg,             /**< (i) Argument to pass to processing thread */
   GOS_THRD_ID    *threadidp        /**< (o) Place to store the ID of the new thread */
);

/***************************************************************************/
/**
*  Destroy a processing thread. This function may block until the
*  child thread exits.
*
*  @return
*     0        Success
*     -ve      On general failure
*/
int gosThreadDestroy(
   GOS_THRD_ID    threadid          /**< (i) Thread to destroy */
);

/***************************************************************************/
/**
*  Retrieve system time in milliseconds since system startup.
*
*  @return system time in ms since system startup.
*/
unsigned int gosElapsedTime_ms( void );

/***************************************************************************/
/**
*  Retrieve the system time in seconds and usecs elapsed since system startup.
*
*  @return  0 always
*
*  @remarks
*/
int gosElapsedTime_msec_usec(
   uint32_t *seconds,   /**< (o) place to store the seconds */
   uint32_t *useconds   /**< (o) place to store the usecs */
);

/***************************************************************************/
/**
*  Allocate memory
*
*  @return
*     ptr     Valid pointer to allocated memory
*     NULL    Insufficient memory
*
*  @remark
*     This routine may block and may not return physically contiguous
*     memory.
*/
void *gosMemAlloc(
   unsigned int bytes               /**< (i) Size in bytes */
);

/***************************************************************************/
/**
*  Allocate memory, specifying virtual or non-virtual memory
*
*  @return
*     ptr     Valid pointer to allocated memory
*     NULL    Insufficient memory
*
*  @remark
*     This routine may block and may not return physically contiguous
*     memory.
*/
void *gosMemAllocType(
   unsigned int bytes,              /**< (i) Size in bytes */
   GOS_MEM_TYPE type                /**< (i) Type of memory */
);

/***************************************************************************/
/**
*  Free previously allocated memory
*
*  @return
*     0        Success
*     -ve      Failed to free memory
*/
int gosMemFree(
   void          *memp              /**< (i) Memory to free */
);

/***************************************************************************/
/**
*  Logging routine like printf. Logging is conditional based on the
*  'flags' argument.  The 'flags' argument is a bitmask to allow printk
*  and/or KNLLOG (or maybe other) forms of logging.
*
*  @return Nothing
*
*  @remarks  If fmt does not finish with a newline character then one will
*            automatically be added.
*/
void gosLog(
   const char    *cat1,  /**< (i) Name of 1st category for logging */
   const char    *cat2,  /**< (i) Name of 2nd category for logging (e.g. function name)*/
   int            flags, /**< (i) Conditional logging flags */
   const char    *fmt,   /**< (i) Printf format string */
   ...
);

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
);

#ifdef __cplusplus
}
#endif

#endif   /* GOS_H */
