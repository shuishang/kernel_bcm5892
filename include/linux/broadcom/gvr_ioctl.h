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
 * The Graphics/Video Renderer (GVR) user IOCTL API definitions
 */

#ifndef GVR_IOCTL_H
#define GVR_IOCTL_H

#include <linux/broadcom/gvr.h>

#define GVR_MAGIC    'D'

typedef struct gvr_ioctl_lcd_info_get_param {
   GVR_LCD_INFO info;
} GVR_IOCTL_LCD_INFO_GET_PARAM;

typedef struct gvr_ioctl_lcd_panel_set_param {
   GVR_LCD_PANEL_ACT act;
} GVR_IOCTL_LCD_PANEL_SET_PARAM;

typedef struct gvr_ioctl_element_add_param {
   GVR_ELEMENT_PARAM element;
   GVR_ELEMENT_HDL element_hdl;
} GVR_IOCTL_ELEMENT_ADD_PARAM;

typedef struct gvr_ioctl_element_remove_param {
   GVR_ELEMENT_HDL element_hdl;
} GVR_IOCTL_ELEMENT_REMOVE_PARAM;

typedef struct gvr_ioctl_element_mod_param {
   GVR_ELEMENT_HDL element_hdl;
   GVR_ELEMENT_PARAM element;
} GVR_IOCTL_ELEMENT_MOD_PARAM;

typedef struct gvr_ioctl_element_get_param {
   GVR_ELEMENT_HDL element_hdl;
   GVR_ELEMENT_PARAM element;
} GVR_IOCTL_ELEMENT_GET_PARAM;

typedef struct gvr_ioctl_element_set_master_param {
   GVR_ELEMENT_HDL element_hdl;
   unsigned int set;
} GVR_IOCTL_ELEMENT_SET_MASTER_PARAM;

typedef struct gvr_ioctl_element_get_master_param {
   GVR_ELEMENT_HDL element_hdl;
} GVR_IOCTL_ELEMENT_GET_MASTER_PARAM;

typedef struct gvr_ioctl_element_update_param {
   GVR_ELEMENT_HDL element_hdl;
   GVR_ELEMENT_UPDATE_PARAM update;
} GVR_IOCTL_ELEMENT_UPDATE_PARAM;

/* IOCTL commands */
typedef enum gvr_cmd
{
   GVR_CMD_LCD_INFO_GET = 0x10,
   GVR_CMD_LCD_PANEL_SET,
   GVR_CMD_ELEMENT_ADD,
   GVR_CMD_ELEMENT_REMOVE,
   GVR_CMD_ELEMENT_MOD,
   GVR_CMD_ELEMENT_GET,
   GVR_CMD_ELEMENT_SET_MASTER,
   GVR_CMD_ELEMENT_GET_MASTER,
   GVR_CMD_ELEMENT_UPDATE,
   GVR_CMD_LAST /* do no delete */
} GVR_CMD;

#define GVR_IOCTL_LCD_INFO_GET        _IOR(GVR_MAGIC, GVR_CMD_LCD_INFO_GET, GVR_IOCTL_LCD_INFO_GET_PARAM)
#define GVR_IOCTL_LCD_PANEL_SET       _IOW(GVR_MAGIC, GVR_CMD_LCD_PANEL_SET, GVR_IOCTL_LCD_PANEL_SET_PARAM)
#define GVR_IOCTL_ELEMENT_ADD         _IOWR(GVR_MAGIC, GVR_CMD_ELEMENT_ADD, GVR_IOCTL_ELEMENT_ADD_PARAM)
#define GVR_IOCTL_ELEMENT_REMOVE      _IOW(GVR_MAGIC, GVR_CMD_ELEMENT_REMOVE, GVR_IOCTL_ELEMENT_REMOVE_PARAM)
#define GVR_IOCTL_ELEMENT_MOD         _IOW(GVR_MAGIC, GVR_CMD_ELEMENT_MOD, GVR_IOCTL_ELEMENT_MOD_PARAM)
#define GVR_IOCTL_ELEMENT_GET         _IOWR(GVR_MAGIC, GVR_CMD_ELEMENT_MOD, GVR_IOCTL_ELEMENT_GET_PARAM)
#define GVR_IOCTL_ELEMENT_SET_MASTER  _IOW(GVR_MAGIC, GVR_CMD_ELEMENT_SET_MASTER, GVR_IOCTL_ELEMENT_SET_MASTER_PARAM)
#define GVR_IOCTL_ELEMENT_GET_MASTER  _IOR(GVR_MAGIC, GVR_CMD_ELEMENT_GET_MASTER, GVR_IOCTL_ELEMENT_GET_MASTER_PARAM)
#define GVR_IOCTL_ELEMENT_UPDATE      _IOW(GVR_MAGIC, GVR_CMD_ELEMENT_UPDATE, GVR_IOCTL_ELEMENT_UPDATE_PARAM)

#endif /* GVR_IOCTL_H */
