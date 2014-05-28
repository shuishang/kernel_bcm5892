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
*  perfcnt.h
*
*  Usage:
*****************************************************************************/
#ifndef LINUX_BROADCOM_PERFCNT_H
#define LINUX_BROADCOM_PERFCNT_H

#ifdef CONFIG_CPU_V6
#include <mach/csp/arm_perf.h>
#else
#error Only ARMv6 CPU can use this file
#endif

#ifdef CONFIG_CPU_V6
typedef ARM_PERF_CNTRS PERFCNT_CNTRS;
#endif

int perfcnt_availability_check(void);
int perfcnt_start(void);
int perfcnt_stop(PERFCNT_CNTRS *cntrs);
int perfcnt_clear(void);
void perfcnt_read(PERFCNT_CNTRS *cntrs);
const char *perfcnt_get_evtstr0(void);
const char *perfcnt_get_evtstr1(void);


#endif /* !defined( LINUX_BROADCOM_PERFCNT_H ) */

