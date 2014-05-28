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
*  @file    gos_priv.h
*
*  @brief   Contains the private GOS definitions.
*
*****************************************************************************/
#if !defined( GOS_PRIV_H )
#define GOS_PRIV_H

/* ---- Include Files ---------------------------------------------------- */
#include <linux/init.h>                      /* For __init, __exit */

/* ---- Constants and Types ---------------------------------------------- */
/* ---- Variable Externs ------------------------------------------------- */

/* Parent procfs directory handle */
struct proc_dir_entry;           /* forward declaration */
extern struct proc_dir_entry     *gGosProcDir;

/* ---- Function Prototypes ---------------------------------------------- */
int __init gosSemInit( void );
void __exit gosSemExit( void );
int __init gosMemInit( void );
void __exit gosMemExit( void );
int __init gosThreadInit( void );
void __exit gosThreadExit( void );

#endif   /* GOS_PRIV_H */
