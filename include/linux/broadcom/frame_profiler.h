/*****************************************************************************
* Copyright 2006 - 2011 Broadcom Corporation.  All rights reserved.
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

/*****************************************************************************
*
*  frame_profiler.h
*
*  PURPOSE:
*
*  This file contains the functions for profiling.
*
*  NOTES:
*
*****************************************************************************/

#ifndef FRAME_PROFILER_H
#define FRAME_PROFILER_H

int frame_profiler_init(void);
void FP_startProfiling(void);
void FP_stopProfiling(void);

#endif