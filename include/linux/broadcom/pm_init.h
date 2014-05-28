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

#ifndef _PM_INIT_H_
#define _PM_INIT_H_

#define PM_INIT_DRIVER_NAME "power-mgmt-init"

typedef struct 
{
   char *p_block_name;
   int  gpio_pin;
   int  disable_val;
} POWER_MGMT_BLOCKS;

struct POWER_MGMT_INIT_t
{
   char *p_block_name_0;
   int  gpio_pin_0;
   int  disable_val_0;
   char *p_block_name_1;
   int  gpio_pin_1;
   int  disable_val_1;
   char *p_block_name_2;
   int  gpio_pin_2;
   int  disable_val_2;
   
   POWER_MGMT_BLOCKS *p_power_blocks;
   unsigned int pm_init_cnt;
    //void (*reset_master)(void);
};


#endif    /* _PM_INIT_H_ */

