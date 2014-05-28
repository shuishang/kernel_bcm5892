/*****************************************************************************
* Copyright 2003 - 2008 Broadcom Corporation.  All rights reserved.
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

#ifndef BCMRING_VPM_H
#define BCMRING_VPM_H

/*
 * ---- Include Files ---------------------------------------- 
 */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#endif
/*
 * ---- Constants and Types ---------------------------------- 
 */
typedef int VPM_LCC_CHANNEL_HANDLE;
typedef struct
{
   int16_t  priority;
   uint16_t channel;
   uint16_t moduleId;
   uint16_t pad;
} VPM_LCC_JOB;

typedef struct
{
   VPM_LCC_CHANNEL_HANDLE   handle;
   int32_t                  priority;
   VPM_LCC_JOB             *lccJobp;
   int32_t                  headerSizeInBytes;
   uint16_t                *buffer;
   int32_t                  bufferSizeInBytes;
} VPM_LCC_SUBMIT_JOB_PARAM;

typedef struct
{
   VPM_LCC_CHANNEL_HANDLE   handle;
   VPM_LCC_JOB             *lccJobp;
   uint32_t                *numBytesp;
} VPM_LCC_RETRIEVE_JOB_PARAM;

/*
 * ---- Variable Externs ------------------------------------- 
 * ---- Function Prototypes ---------------------------------- 
 */

/*
 * IOCTLs for user applications
 */

#define VPM_MAGIC                   'V'
#define VPM_LCC_CMD_OPEN_CHANNEL    0x80
#define VPM_LCC_CMD_CLOSE_CHANNEL   0x81
#define VPM_LCC_CMD_SUBMIT_JOB      0x82
#define VPM_LCC_CMD_RETRIEVE_JOB    0x83
#define VPM_LCC_CMD_FREE_JOB        0x84
#define VPM_LCC_CMD_RELEASE_ALL_JOB 0x85

#define VPM_LCC_IOCTL_OPEN_CHANNEL     _IOW(VPM_MAGIC, VPM_LCC_CMD_OPEN_CHANNEL, uint16_t)
#define VPM_LCC_IOCTL_CLOSE_CHANNEL    _IOW(VPM_MAGIC, VPM_LCC_CMD_CLOSE_CHANNEL, VPM_LCC_CHANNEL_HANDLE)
#define VPM_LCC_IOCTL_SUBMIT_JOB       _IOW(VPM_MAGIC, VPM_LCC_CMD_SUBMIT_JOB, VPM_LCC_SUBMIT_JOB_PARAM)
#define VPM_LCC_IOCTL_RETRIEVE_JOB     _IOWR(VPM_MAGIC, VPM_LCC_CMD_RETRIEVE_JOB, VPM_LCC_RETRIEVE_JOB_PARAM)
#define VPM_LCC_IOCTL_FREE_JOB         _IOW(VPM_MAGIC, VPM_LCC_CMD_FREE_JOB, VPM_LCC_CHANNEL_HANDLE)
#define VPM_LCC_IOCTL_RELEASE_ALL_JOB  _IOW(VPM_MAGIC, VPM_LCC_CMD_RELEASE_ALL_JOB, VPM_LCC_CHANNEL_HANDLE)

#if defined( __KERNEL__ )

int vpmLoadAndRun( char *filename );
int vpmLoadBinaryFile( char *fileName );
void vpmShutdown( void );
void vpmRequest( void );
void vpmRelease( void );
void vpmRun( void );
void vpmHalt( void );
void vpmRequestAndRun( void );
void vpmHaltAndRelease( void );

typedef VPM_LCC_CHANNEL_HANDLE (*VPM_LCC_OPEN_CHANNEL)( uint16_t moduleId);
typedef int (*VPM_LCC_CLOSE_CHANNEL)( VPM_LCC_CHANNEL_HANDLE handle);
typedef int (*VPM_LCC_SUBMIT_JOB)(
   VPM_LCC_CHANNEL_HANDLE   handle,
   int32_t                  priority,
   VPM_LCC_JOB             *lccJobp,
   int32_t                  headerSizeInBytes,
   uint16_t                *buffer,
   int32_t                  bufferSizeInBytes
);
typedef int (*VPM_LCC_RETRIEVE_JOB)(
   VPM_LCC_CHANNEL_HANDLE   handle,
   VPM_LCC_JOB            **lccJobpp,
   uint32_t                *numBytesp
);
typedef void (*VPM_LCC_FREE_JOB)( VPM_LCC_CHANNEL_HANDLE handle);
typedef void (*VPM_LCC_RELEASE_ALL_JOB)( VPM_LCC_CHANNEL_HANDLE handle);

struct vpmLccFuncs {
   VPM_LCC_OPEN_CHANNEL     openChannel;
   VPM_LCC_CLOSE_CHANNEL    closeChannel;
   VPM_LCC_SUBMIT_JOB       submitJob;
   VPM_LCC_RETRIEVE_JOB     retrieveJob;
   VPM_LCC_FREE_JOB         freeJob;
   VPM_LCC_RELEASE_ALL_JOB  releaseAllJob;
};

int vpmRegisterLccClient( struct vpmLccFuncs *funcsp );
int vpmUnregisterLccClient( struct vpmLccFuncs *funcsp );

#endif /* __KERNEL__ */

#endif /* BCMRING_VPM_H */ 

