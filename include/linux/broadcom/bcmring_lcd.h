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
 * Description: Public header of the LCD driver.
 *
 * The LCD driver implements double buffers internally. The user can update
 * the LCD screen by issuing the update command with a specified Y offset (in
 * pixels)
 *
 * NOTE: The user is responsible for maintaining the synchronization between
 * the two buffers
 */ 

#ifndef _BCMRING_LCD_H
#define _BCMRING_LCD_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#endif

#ifdef __KERNEL__
#define LCD_LOG_ENABLED    0
#if LCD_LOG_ENABLED
#define LCD_LOG            KNLLOG
#else
#define LCD_LOG(c,args...)
#endif
#endif

/*
 * RGB or BGR order
 */
enum lcd_rgb_order {
   LCD_ORDER_RGB,
   LCD_ORDER_BGR,
   LCD_ORDER_INVALID,
};

/*
 * Bits per pixel
 */
enum lcd_bpp {
   LCD_BPP_16_555 = 0, /* RGB555 */
   LCD_BPP_16_565, /* RGB565 */
   LCD_BPP_32, /* RGB888 */
   LCD_BPP_MAX
};

/*
 * LCD power sequence signal source. Can be from the LCD controller or GPIO
 */
enum lcd_pwr_sig_src {
   LCD_PWR_SIG_SRC_LCD,
   LCD_PWR_SIG_SRC_GPIO
};

/*
 * There are 3 different types of signals from the LCD controller: 1) enable,
 * 2) power on/off, 3) backlight
 */
enum lcd_pwr_sig_type {
   LCD_PWR_SIG_ENABLE, /* enable */
   LCD_PWR_SIG_POWER, /* power on/off */
   LCD_PWR_SIG_BACKLIGHT, /* backlight */
};

/*
 * LCD panel power up/down sequence
 */
struct lcd_pwr_param {
   enum lcd_pwr_sig_src sig_src;
   union {
      enum lcd_pwr_sig_type sig_type;
      unsigned int gpio;
   } option;
   unsigned int val; /* value, usually 1 or 0 */
   unsigned int mdelay; /* time delay (in miliseconds) */
};

/*
 * LCD panel parameters
 */
struct lcd_panel_param {
   const char *name; /* panel name */
   unsigned int refresh_rate; /* refresh rate in Hz */
   enum lcd_bpp bpp; /* bits per pixel */
   enum lcd_rgb_order rgb_order; /* RGB or BGR */
   unsigned int xres; /* X resolution in pixels */
   unsigned int yres; /* Y resolution in pixels */
   unsigned int left_margin; /* horizontal back porch */
   unsigned int right_margin; /* horizontal front porch */
   unsigned int upper_margin; /* vertical back porch */
   unsigned int lower_margin; /* vertical front porch */
   unsigned int hsync_len; /* horizontal pulse width */
   unsigned int vsync_len; /* vertical pulse width */
   unsigned int inv_out_enable; /* invert output enable */
   unsigned int inv_pixel_clk; /* invert pixel clock */
   unsigned int inv_hsync; /* invert horizontal sync */
   unsigned int inv_vsync; /* invert certical sync */

   struct lcd_pwr_param *pwr_up; /* panel power up sequence */
   unsigned int num_pwr_up; /* number of entries in the power up sequence */

   struct lcd_pwr_param *pwr_down; /* panel power down sequence */
   unsigned int num_pwr_down; /* number of entries in the power down sequence */
};

/*
 * Data buffer
 */
struct lcd_buf {
   void *ptr; /* virtual address */
   uint32_t addr; /* physical address */
   unsigned int len; /* buffer size in bytes */
};

/*
 * LCD buffer parameters
 */
struct lcd_buf_param {
   unsigned int xres; /* X resolution in pixels */
   unsigned int yres; /* Y resolution in pixels */
   unsigned int virtual_xres; /* X virtual resolution in pixels */
   unsigned int virtual_yres; /* Y virtual resolution in pixels */
   unsigned int bpp; /* bits per pixel */
   struct lcd_buf data; /* data buffer info */
};

#ifdef __KERNEL__
/*
 * Callbacks to be used by lower level panel drivers
 */
struct lcd_panel_operations {
	int (*lcd_panel_enable) (void);
	int (*lcd_panel_disable) (void);
};
#endif

/*
 * IOCTLs for user applications
 */

#define CLCD_MAGIC                  'G'
#define LCD_CMD_PANEL_INFO_GET     0x80
#define LCD_CMD_BUF_INFO_GET       0x81
#define LCD_CMD_PANEL_RESET        0x82
#define LCD_CMD_PANEL_ENABLE       0x83
#define LCD_CMD_PANEL_DISABLE      0x84
#define LCD_CMD_UPDATE             0x85

#define LCD_IOCTL_PANEL_INFO_GET   _IOR(CLCD_MAGIC, LCD_CMD_PANEL_INFO_GET, struct lcd_panel_param)
#define LCD_IOCTL_BUF_INFO_GET     _IOR(CLCD_MAGIC, LCD_CMD_BUF_INFO_GET, struct lcd_buf_param)
#define LCD_IOCTL_PANEL_RESET      _IO(CLCD_MAGIC,LCD_CMD_PANEL_RESET)
#define LCD_IOCTL_PANEL_ENABLE     _IO(CLCD_MAGIC,LCD_CMD_PANEL_ENABLE)
#define LCD_IOCTL_PANEL_DISABLE    _IO(CLCD_MAGIC,LCD_CMD_PANEL_DISABLE)
#define LCD_IOCTL_UPDATE           _IOR(CLCD_MAGIC, LCD_CMD_UPDATE, unsigned int)

#ifndef FBIO_WAITFORVSYNC
#define FBIO_WAITFORVSYNC          _IOW('F', 0x20, __u32)
#endif

#ifdef __KERNEL__
/*
 * Get the LCD panel info
 */
extern int lcd_panel_info_get(struct lcd_panel_param *panel_param);

/*
 * Get the LCD framebuffer info
 */
extern int lcd_buf_info_get(struct lcd_buf_param *buf_param);

/*
 * Reset the LCD panel
 */
extern int lcd_panel_reset(void);

/*
 * Enable the LCD panel
 */
extern int lcd_panel_enable(void);

/*
 * Disable the LCD panel
 */
extern int lcd_panel_disable(void);

/*
 * The LCD driver uses its own internal buffer in normal conditions. However,
 * the user can use this routine to set the LCD controller to use an external
 * buffer (needs to be the same size of the LCD internal buffer). Set bus_addr
 * to zero to make the LCD driver switches back to use its internal buffer
 */
extern int lcd_buf_addr_set(uint32_t bus_addr);

/*
 * Update the LCD screen, starting from the Y offset (in pixels).
 * This routine updates the hardware register and returns immediately.
 * The actual change is done by the hardware during the vertical retrace cycle.
 */
extern int lcd_update(unsigned int y_offset);

/*
 * Wait for a vertical sync. 
 * block waits until the next VSYNC to return. By the time this routine
 * returns the screen has been updated.
 */
extern int lcd_wait_for_vsync(void);

/*
 * Test whether hardware is currently in a vsync.
 */
extern int lcd_is_vsyncing(void);

#endif /* __KERNEL__ */

#endif /* _BCMRING_LCD_H */
