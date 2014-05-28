/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
 * Description: Public header of the LED Matrix driver shared between various
 * platforms
 */

#ifndef LEDM_H
#define LEDM_H

#include <linux/leds.h>

#define LEDM_MAX_ROWS     6
#define LEDM_MAX_COLS     12

/*
 * Clock frequency divisor
 */
typedef enum {
   LEDM_CLK_DIV_2 = 0,
   LEDM_CLK_DIV_4,
   LEDM_CLK_DIV_8,
   LEDM_CLK_DIV_16,
   LEDM_CLK_DIV_INVALID
} LEDM_CLK_DIV;

/*
 * Operation mode
 */
typedef enum {
   LEDM_MODE_SERIAL,
   LEDM_MODE_PARALLEL
} LEDM_MODE;

typedef struct {
   /* value to drive Y control pins during ON state */
   unsigned int row_on_val;

   /* value to drive X control pins during ON state */
   unsigned int col_on_val;

   /* clock divisor, based on input clock of 166 MHz */
   LEDM_CLK_DIV clk_div;

   /*
    * On time, guard time frequency resolution divisor, based on
    * (res + 1) / 166 MHz
    */
   unsigned int res_div;

   /*
    * ON time of LED during each scan cycle, with unit of 'res_div'
    */
   unsigned int on_time;

  /*
   * Minimum off time between when a X or Y control is last active and when
   * the next X or Y control is activated, with unit of 'res_div'
   */
   unsigned int guard_time;

   /* serial or parallel mode? */
   LEDM_MODE mode;

   /* rows in use */
   unsigned int *row_map;

   /* number of rows */
   unsigned int row_cnt;

   /* columns in use */
   unsigned int *col_map;

   /* number of columns */
   unsigned int col_cnt;

   struct led_platform_data *leds;
} LEDM_CFG;

#endif /* LEDM_H */
