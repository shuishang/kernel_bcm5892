/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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




/*
*
*****************************************************************************
*
*  speaker.h
*
*  PURPOSE:
*
*  This file defines the interface to the SPEAKER driver.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_SPEAKER_H )
#define LINUX_SPEAKER_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/ioctl.h>

typedef enum
{
    SPEAKER_UNPLUGGED,      /* speaker unplugged */
    SPEAKER_TOGGLE          /* speaker plugged in */

} speaker_state;

typedef enum
{
    SPEAKER_NULL = 0,       /* NULL event */
    SPEAKER_REMOVED,        /* speaker unplugged */
    SPEAKER_INSERTED,       /* speaker inserted */

} speaker_event;

/* ---- Constants and Types ---------------------------------------------- */

#define SPEAKER_MAGIC   'h'

#define SPEAKER_CMD_FIRST               0x80
#define SPEAKER_CMD_GET_STATE           0x80
#define SPEAKER_CMD_LAST                0x80

#define SPEAKER_IOCTL_GET_STATE _IOR( SPEAKER_MAGIC, SPEAKER_CMD_GET_STATE, speaker_state )

/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */
/*static*/ int speaker_init( void );

#endif  /* LINUX_SPEAKER_H */
