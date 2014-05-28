/*****************************************************************************
* Copyright 2005 - 2009 Broadcom Corporation.  All rights reserved.
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

#ifndef _OTP_H
#define _OTP_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#endif

#define OTP_MAGIC    'o'

struct otp_data {
	/* enable/disable ECC for data reading */
	unsigned int enable_ecc;

	/* row to read */
	unsigned int row;

	/* store the 32-bit data */
	uint32_t data;
};

#define OTP_CMD_READ_ROW    0x80

/*
 * Read the OTP data from a specific row. Result is stored in the 'data' field
 */
#define OTP_IOCTL_READ_ROW   _IOWR(OTP_MAGIC, OTP_CMD_READ_ROW, struct otp_data)

#endif  /* _OTP_H */
