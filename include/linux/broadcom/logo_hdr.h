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

#if !defined( LOGO_HDR_H )
#define LOGO_HDR_H

#include <linux/broadcom/bcmtypes.h>

/* All fields are to be stored in little-endian order. */

typedef struct
{
    char        signature[ 4 ];     /* 'LOGO' */
    uint32_t    areaSize;           /* Size of area which contains the logo
                                     * areaSize is expected to be a multiple of the flash block size. */

    /* Add required header information here. */

} LOGO_HEADER;

#endif  /* LOGO_HDR_H */

