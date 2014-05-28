/*****************************************************************************
* Copyright 2009-2010 Broadcom Corporation.  All rights reserved.
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

#ifndef _I2C_TS_H_
#define _I2C_TS_H_

/* The two I2C slave device addresses the driver supports. */
#define I2C_TS_DRIVER_SLAVE_NUMBER_0x20    0x20
#define I2C_TS_DRIVER_SLAVE_NUMBER_0x38    0x38
#define I2C_TS_DRIVER_NAME                 "cy8ctst120_ts"
#define I2C_TS_DRIVER_MAX_NUM_SLAVES       2


/*  y
 *  ^             +---->x
 *  |      or     |
 *  |             |
 *  +--->x        v
 *                y
 * X_RIGHT_Y_UP  X_RIGHT_Y_DOWN
 */
typedef enum
{
    X_RIGHT_Y_UP,
    X_RIGHT_Y_DOWN,
    X_LEFT_Y_UP,
    X_LEFT_Y_DOWN,
} SCREEN_XY_LAYOUT_e;    

struct I2C_TS_t
{
    int i2c_slave_address;    
    int gpio_irq_pin;
    int timer_wait;
    int x_max_value;
    int y_max_value;
    SCREEN_XY_LAYOUT_e layout;
    int num_bytes_to_read;
    int is_multi_touch;
    int is_resetable;
    int gpio_reset_pin;
    /* The location of bytes in the stream read from the slave. */
    int x1_hi_idx;
    int x1_lo_idx;
    int y1_hi_idx;
    int y1_lo_idx;
    /* Multi touch when supported. */
    int x2_hi_idx;
    int x2_lo_idx;
    int y2_hi_idx;
    int y2_lo_idx;
    int num_fingers_idx;
    int gesture_idx;
    int button_idx;
    int movement_idx;
    int version_idx;    
    int version_val;
    int idle_idx;
    int idle_val;
    int timeout_idx;
    int timeout_val;
    int auto_power_idx;
    int auto_power_val;    
    int min_finger_val;
    int max_finger_val;
    int panel_width;  /* LCD panel width in millimeters */
};

#endif    /* _I2C_TS_H_ */

