/*****************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
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
#if !defined( STD_TIME_H )
#define STD_TIME_H

#include <linux/types.h>

struct timespec
{
   time_t   tv_sec;        /* seconds */
   long     tv_nsec;       /* nanoseconds */
};

#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif

#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 0
#endif

#ifdef clock_gettime
#undef clock_gettime
#endif
#define clock_gettime clock_gettime_kernel
int clock_gettime_kernel(clockid_t clk_id, struct timespec *tp);

#endif

