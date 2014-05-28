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

#ifndef __REGACCESS_H
#define __REGACCESS_H

#include <mach/bcm5892_sw.h>
#include <linux/string.h>

#ifndef AHB_BIT32
#define AHB_BIT32 (1<<5)
#endif

#ifndef AHB_BIT16
#define AHB_BIT16 (1<<4)
#endif

#ifndef AHB_BIT8
#define AHB_BIT8 (1<<3)
#endif
/* ARM Macros */

#define CPU_READ_SINGLE(addr,size)  ((size == AHB_BIT8) ?  *((volatile uint8_t *)addr) :\
                                      ((size == AHB_BIT16) ? *((volatile uint16_t *)addr) :\
                                      *((volatile uint32_t *)addr)))

#define CPU_READ_BURST(addr, data, size) memcpy((void *)data, (void *)addr, size);

#define CPU_RMW_OR_SINGLE(addr,data,size)  if(size == AHB_BIT8) *((volatile uint8_t *)addr) |= data; \
                                           else if(size == AHB_BIT16) *((volatile uint16_t *)addr) |= data; \
                                           else  *((volatile uint32_t *)addr) |= data


#define CPU_RMW_AND_SINGLE(addr,data,size) if(size == AHB_BIT8) *((volatile uint8_t *)addr) &= data; \
                                           else if(size == AHB_BIT16)  *((volatile uint16_t *)addr) &= data ; \
                                           else  *((volatile uint32_t *)addr) &= data

#define CPU_WRITE_SINGLE(addr,data,size)  if(size == AHB_BIT8) *((volatile uint8_t *)addr) = data ;\
                                          else if(size == AHB_BIT16) *((volatile uint16_t *)addr) = data ; \
                                          else *((volatile uint32_t *)addr) = data

#define CPU_READ_SINGLE(addr,size)        ((size == AHB_BIT8) ?  *((volatile uint8_t *)addr) :\
                                          ((size == AHB_BIT16) ? *((volatile uint16_t *)addr) :\
                                          *((volatile uint32_t *)addr)))

#define CPU_WRITE_BURST(dest_addr, src_addr, byte_length) memcpy((void *)dest_addr, (const void *)src_addr, byte_length)

#define CPU_BZERO(dest_addr, byte_length) memset((void *)dest_addr, 0, byte_length)
#define CPU_RMW_INC_SINGLE(addr,data,size) if (size == AHB_BIT8) *((volatile uint8_t *)addr) += data; \
					   else if (size == AHB_BIT16) *((volatile uint16_t *)addr) += data;\
					   else *((volatile uint32_t *)addr) += data 

#define AHB_WRITE_VERIFY(addr, data) AHB_WRITE_VERIFY_mb(addr, data);
#endif
