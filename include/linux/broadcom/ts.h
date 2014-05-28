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

#if !defined( LINUX_TS_H )
#define LINUX_TS_H

typedef enum tsc_wire_mode {
   TSC_MODE_4WIRE = 0,
   TSC_MODE_5WIRE
} tsc_wire_mode;

typedef struct tsc_param {
   tsc_wire_mode wire_mode;
   
   /* sampling rate in Hz */
   int sample_rate;

   /* FIFO threshold */
   int fifo_threshold;

   /* debounce timeout in usec (must be multiples of 512) */
   int debounce;

   /*
    * The settling duration (in usec) is the amount of time the touch screen
    * controller waits to allow the voltage to settle after turning on the
    * drivers in detection mode
    */
   int settling;

   /*
    * Number of data samples which are averaged before a final data point is
    * placed into the FIFO
    */
   int data_point_average;

   /* touch timeout in sample counts */
   int touch_timeout;

   /* data value inverted on the X axis */
   int inverted_x;

   /* data value inverted on the Y axis */
   int inverted_y;

   /* X/Y swapped (e.g., screen rotated 90 degree) */
   int swapped_x_y;
} tsc_param;

#endif
