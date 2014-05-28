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
 
/*
 * Public header of the Graphics/Video Renderer (GVR) component.
 *
 * The GVR can be used by multimedia, GUI applications for rendering RGB
 * pixels onto the LCD screen.
 *     
 * The GVR maintains a list of elements. An image can be shared among
 * various elements. When an update request is submitted, the GVR goes
 * through all elements in the list with an ascending order of its associated
 * z-layer number and performs specified operations, then merges all elements
 * to produce the final displayed image.
 */

#ifndef GVR_H
#define GVR_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/module.h>
#include <linux/broadcom/knllog.h>
#else
#include <stdint.h>
#endif

/*
 * Value to indicate no chnage should be made to a parameter
 */
#define GVR_PARAM_NOCHNAGE    -1

typedef int GVR_HDL;
typedef void * GVR_ELEMENT_HDL;

/*
 * To control the LCD panel
 */
typedef enum gvr_lcd_panel_act {
   GVR_LCD_PANEL_DISABLE = 0,
   GVR_LCD_PANEL_ENABLE,
   GVR_LCD_PANEL_RESET,
   GVR_LCD_PANEL_MAX,
} GVR_LCD_PANEL_ACT;

/*
 * Color format
 */
typedef enum gvr_color_format {
   /* 32-bit unpack RGB. Alpha channel is optional */
   GVR_COLOR_FORMAT_URGB888 = 0,

   /* 24-bit packed RGB */
   GVR_COLOR_FORMAT_PRGB888,

   /* 16-bit RGB 565 */
   GVR_COLOR_FORMAT_RGB565,

   /*
    * to be added ... 
    */
   GVR_COLOR_FORMAT_MAX
} GVR_COLOR_FORMAT;

/*
 * Alpha blending options
 */
typedef enum gvr_alpha_opt {
   /* no alpha blending required */
   GVR_ALPHA_OPT_NONE = 0,

   /* use embedded alpha channel in source image */
   GVR_ALPHA_OPT_SRC,

   /* use global alpha */
   GVR_ALPHA_OPT_GLOBAL, 
} GVR_ALPHA_OPT;

/*
 * LCD information
 */
typedef struct gvr_lcd_info {
   /* X resolution in pixels */
   unsigned int xres;

   /* Y resolution in lines */
   unsigned int yres; 

   /* X virtual resolution in pixels */
   unsigned int virtual_xres;

   /* Y virtual resolution in lines */
   unsigned int virtual_yres;

   /* bits per pixel */
   unsigned int bpp;
} GVR_LCD_INFO;

/*
 * Element paramters used when adding/modifying an element
 */
typedef struct gvr_element_param {
   /* color format, required */
   GVR_COLOR_FORMAT color_format;

   /*
    * Optional physical address of the image buffer. The physical address of
    * the image buffer does NOT need to be specified until the first time
    * "gvr_element_update" is being called. An element remains "inactive"
    * until the buffer physical address is specified
    */
   uint32_t buf_addr;

   /* image buffer width in pixels, required */
   unsigned int buf_width;

   /* image buffer height in lines, required */
   unsigned int buf_height;

   /* alpha blending options, required */
   GVR_ALPHA_OPT alpha_opt;

   /*
    * Optional global alpha, 0 - 255. Only used when alpha_opt is set to
    * GVR_ALPHA_OPT_GLOBAL
    */
   uint8_t global_alpha;

   /*
    * Z-order layering, images with higher z-order number are in front,
    * required
    */
   int layer; 

   /* X offset (pixels) from the start of the image buffer, required */
   int sx;

   /* Y offset (lines) from the start of the image buffer, required */
   int sy;

   /* 
    * X offset (pixels) from the start of the destination buffer (LCD),
    * required
    */
   int dx;

   /* 
    * Y offset (lines) from the start of the destination buffer (LCD),
    * required
    */
   int dy; 

   /* width of the element (pixels), required */
   unsigned int width;

   /* height of the element (lines), required */
   unsigned int height; 
} GVR_ELEMENT_PARAM;

/*
 * Paramters to be used with the update request. All of the parameters here
 * are optional.
 */
typedef struct gvr_element_update_param {
   /*
    * Optional physical address of the image buffer. If the user has already
    * specified the physical address when the element was added/modified, the
    * address can be set to zero here if the user does not want it to be
    * changed.
    *
    * If the physical address has not been specified previously, the user has
    * to supply a valid address here, otherwise -EINVAL is returned
    */
   uint32_t buf_addr;

   /*
    * Optional X offset (pixels) from the start of the image buffer. Assign 
    * GVR_PARAM_NOCHANGE if do not want it to be changed 
    */
   int sx;

   /*
    * Optional Y offset (lines) from the start of the image buffer. Assign 
    * GVR_PARAM_NOCHANGE if do not want it to be changed 
    */
   int sy;

   /*
    * Optional X offset (pixels) from the start of the destination buffer
    * (LCD). Assign GVR_PARAM_NOCHANGE if do not want it to be changed 
    */
   int dx;

   /*
    * Optional Y offset (lines) from the start of the destination buffer
    * (LCD). Assign GVR_PARAM_NOCHANGE if do not want it to be changed 
    */
   int dy; 
} GVR_ELEMENT_UPDATE_PARAM;

/*
 * Open a GVR instance. The handle associated with the instance is returned on success
 */
extern int gvr_open(
      GVR_HDL *hdlp);

/*
 * Close a GVR instance. All associated elements will be removed and memories
 * will be freed
 */
extern int gvr_close(
      GVR_HDL hdl);

/*
 * Get the LCD information such as resolution, bits per pixel, etc.
 */
extern int gvr_lcd_info_get(
      GVR_HDL hdl,
      GVR_LCD_INFO *info);

/*
 * Enable, disable, or reset the LCD panels through the GVR
 */
extern int gvr_lcd_panel_set(
      GVR_HDL hdl,
      GVR_LCD_PANEL_ACT act);

/*
 * Add an element. When a success is returned, an element handle is filled in
 * 'handle' that can be used later to modify/remove the element
 *
 * After an element is added, it remains 'inactive' until the first update
 * request (along with the physical address associated with the image buffer
 * used by the element) is submitted and processed. An element needs to be
 * 'active' for it to be seen on the LCD
 */
extern int gvr_element_add(
      GVR_HDL hdl,
      const GVR_ELEMENT_PARAM *param,
      GVR_ELEMENT_HDL *element_hdl);

/*
 * Remove an element using its handle
 */
extern int gvr_element_remove(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL element_hdl);

/*
 * Modify the parameters of an element. For the result to be shown on the LCD,
 * the user needs to call gvr_element_update after the modification
 */
extern int gvr_element_mod(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL element_hdl,
      const GVR_ELEMENT_PARAM *param);

/*
 * Query an element
 */
extern int gvr_element_get(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL element_hdl,
      GVR_ELEMENT_PARAM *param);

/*
 * Set/unset an element to be the 'master'
 *
 * The system can have either 1) no master at all or 2) one and only one
 * master. When the system has no master, update requests from any element
 * will be respected and processed. When the system has a master, only update
 * requests from the master element will be processed and all other update
 * requests will be ignored (pending until an master update is done)
 *
 * If an element is set to be the master:
 * 1) if the system has no master, this element will become the master
 * 2) if the system has another master, this element will become the new
 * master and the previous master element will become a non-master
 *
 * Unset the master from a master element turns the system back into non-master
 * mode
 */
extern int gvr_element_set_master(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL element_hdl,
      unsigned int set);

/*
 * Routine to query the current master. If there's no master in the system,
 * NULL(0) is filled up in the element handle memory provided by the user
 */
extern int gvr_element_get_master(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL *element_hdl);

/*
 * Submit an update request. This is a synchronous call; therefore, upon
 * sucessful return from this routine, the user can be sure the update has
 * finished
 */
extern int gvr_element_update(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL element_hdl,
      const GVR_ELEMENT_UPDATE_PARAM *param);

#endif /* GVR_H */
