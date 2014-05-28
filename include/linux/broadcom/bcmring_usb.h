/*****************************************************************************
* Copyright 2004 - 2010 Broadcom Corporation.  All rights reserved.
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

#ifndef BCMRING_USB_H
#define BCMRING_USB_H

extern int bcmring_usb_init(unsigned int port_index, unsigned int is_gadget);
extern int bcmring_usb_term(unsigned int port_index);
extern int bcmring_usb_suspend(unsigned int port_index);
extern int bcmring_usb_resume(unsigned int port_index);

#endif
