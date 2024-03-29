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
 * Description: The Linux framebuffer driver for the BCMRING series chips
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/console.h>
#include <asm/sizes.h>
#include <asm/uaccess.h>
#include <linux/broadcom/lcd.h>
#include <linux/broadcom/bcmring_lcd.h>
#include <linux/broadcom/bcmring_ge.h>
#include <linux/broadcom/bcmring_fb.h>
#include <mach/csp/cap.h>

#define CMAP_SIZE 256

static const char *fb_name = "BCMRING_FB";
static struct bcmring_fb *gFB = NULL;
static atomic_t bus_is_probed;

static unsigned int chan_to_field(struct fb_bitfield *bf, unsigned int val,
      unsigned int bpp)
{
   if (bpp != 16 && bpp != 32)
      return 0;

   val >>= bpp - bf->length;
   return val << bf->offset;
}

static int bcmringfb_check_var(struct fb_var_screeninfo *var,
      struct fb_info *info)
{
   struct bcmring_fb *fb = container_of(info, struct bcmring_fb, fb_info);
   struct fb_var_screeninfo v;

   v = *var;

   if ((v.xres != info->var.xres) || /* cannot be changed */
       (v.yres != info->var.yres) || /*  cannot be changed */
       (v.xres_virtual != info->var.xres_virtual) || /* cannot be changed */
       (v.yres_virtual > fb->buf.virtual_yres) || /* exceeds max allowed value */
       (v.yres_virtual < info->var.yres) ||
       (v.xoffset) ||
       (v.yoffset > v.yres_virtual - v.yres) ||
       (v.bits_per_pixel != info->var.bits_per_pixel)) {
      printk(KERN_ERR "FB: The following settings are not supported or "
            "cannot be changed on the fly\n");
      printk(KERN_ERR "FB: xres=%u yres=%u xres_virtual=%u yres_virtual=%u "
            "xoffset=%u yoffset=%u bits_per_pixel=%u\n",
            v.xres, v.yres, v.xres_virtual, v.yres_virtual, v.xoffset,
            v.yoffset, v.bits_per_pixel);
      return -EINVAL;
   }

   switch (v.bits_per_pixel) {
      case 16: /* RGB565 */
         v.transp.offset = 0;
         v.transp.length = 0;
         v.red.offset = 11;
         v.red.length = 5;
         v.green.offset = 5;
         v.green.length = 6;
         v.blue.offset = 0;
         v.blue.length = 5;
         break;

      case 32: /* RGB888 */
         v.transp.offset = 24;
         v.transp.length = 8;
         v.red.offset = 16;
         v.red.length = 8;
         v.green.offset = 8;
         v.green.length = 8;
         v.blue.offset = 0;
         v.blue.length = 8;
         break;

      default:
         return -EINVAL;
   }

   *var = v;

   return 0;
}

/*
 * Set var parameters
 */
static int bcmringfb_set_par(struct fb_info *info)
{
   info->fix.line_length = info->var.xres_virtual *
      info->var.bits_per_pixel / 8;

   return 0;
}

static int bcmringfb_setcolreg(unsigned int regno, unsigned int red,
      unsigned int green, unsigned int blue, unsigned int transp,
      struct fb_info *info)
{
   struct bcmring_fb *fb = container_of(info, struct bcmring_fb, fb_info);
   unsigned int bpp = info->var.bits_per_pixel;

   switch (info->fix.visual) {
      case FB_VISUAL_TRUECOLOR:
      case FB_VISUAL_DIRECTCOLOR:
         if (regno < BCMRING_FB_PALETTE_SIZE) {
            unsigned int val;

            val = chan_to_field(&fb->fb_info.var.transp, transp, bpp);
            val |= chan_to_field(&fb->fb_info.var.red, red, bpp);
            val |= chan_to_field(&fb->fb_info.var.green, green, bpp);
            val |= chan_to_field(&fb->fb_info.var.blue, blue, bpp);

            fb->palette[regno] = val;

            return 0;
         }

      default:
         return -EINVAL;
   }
}

static int bcmringfb_pan_display(struct fb_var_screeninfo *var,
      struct fb_info *info)
{
   /*
    * The pan display here is to support double buffering (that prevents
    * flickering) by using virtual_yres = 2 * yres.
    *
    * We DO NOT support panning in X direction or YWRAP
    */
   if ((var->vmode & FB_VMODE_YWRAP) || (var->xoffset != 0)) {
      return -EINVAL;
   }

   info->var.yoffset = var->yoffset;

   /* update the LCD */
   if (lcd_update(var->yoffset) != 0) {
      return -EFAULT;
   }

   return 0;
}

static int bcmringfb_blank(int blank_mode,
      struct fb_info *info)
{
   switch (blank_mode) {
      case FB_BLANK_UNBLANK:
         /* turn on panel */
         lcd_panel_enable();
         break;

      case FB_BLANK_NORMAL:
      case FB_BLANK_VSYNC_SUSPEND:
      case FB_BLANK_HSYNC_SUSPEND:
      case FB_BLANK_POWERDOWN:
         /* turn off panel */
         lcd_panel_disable();
         break;

      default:
         return -EINVAL;
   }
   return 0;
}

#if defined(CONFIG_BCMRING_GEV3_MODULE) || defined(CONFIG_BCMRING_GEV3)
static void fb_copyarea(struct fb_info *info,
      const struct fb_copyarea *area)
{
   unsigned int width, height;
   struct ge_param param;

   if (cap_isPresent(CAP_GE, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "FB: Hardware acceleration of copyarea not "
            "supported\n");
      return;
   }

   if (area->width == 0 || area->height == 0 ||
       area->sx > info->var.xres_virtual ||
       area->dx > info->var.xres_virtual ||
       area->sy > info->var.yres_virtual ||
       area->dy > info->var.yres_virtual) {
      return;
   }

   /* chop off some out of bound area if there's any */
   width = area->width;
   height = area->height;
   if (area->sx + width > info->var.xres_virtual) {
      width = info->var.xres_virtual - area->sx;
   }
   if (area->dx + width > info->var.xres_virtual) {
      uint32_t new_width = info->var.xres_virtual - area->dx;
      width = width < new_width ? width : new_width;
   }
   if (area->sy + height > info->var.yres_virtual) {
      height = info->var.yres_virtual - area->sy;
   }
   if (area->dy + height > info->var.yres_virtual) {
      uint32_t new_height = info->var.yres_virtual - area->dy;
      height = height < new_height ? height : new_height;
   }

   memset(&param, 0, sizeof(param));
   param.mode = GE_MODE_SYNC;
   param.operation = GE_OP_RASTER;
   param.option.raster.op = GE_RASTER_S1;
   param.src1_format =
      (info->var.bits_per_pixel == 32) ? GE_COLOR_ARGB888 : GE_COLOR_RRGB565;
   param.src2_dst_format = param.src1_format;
   param.s1x = area->sx;
   param.s1y = area->sy;
   param.dx = area->dx;
   param.dy = area->dy;
   param.width = width;
   param.height = height;
   param.src1_pitch = info->var.xres_virtual;
   param.src2_dst_pitch = info->var.xres_virtual;
   param.src1_addr = info->fix.smem_start;
   param.dst_addr = info->fix.smem_start;

   if (ge_engage(&param) < 0)
      printk(KERN_ERR "FB: ge_engage failed in fb_copyarea\n");
}
#endif

#if defined(CONFIG_BCMRING_GEV3_MODULE) || defined(CONFIG_BCMRING_GEV3)
static void fb_fillrect(struct fb_info *info,
      const struct fb_fillrect *rect)
{
   unsigned int width, height;
   struct ge_param param;

   if (cap_isPresent(CAP_GE, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "FB: Hardware acceleration of fillrect not "
            "supported\n");
      return;
   }

   /* validate parameters */
   if (rect->width == 0 || rect->height == 0 ||
       rect->dx > info->var.xres_virtual ||
       rect->dy > info->var.yres_virtual ||
       rect->color >= BCMRING_FB_PALETTE_SIZE ||
       rect->rop != ROP_COPY) {
      return;
   }

   /* chop off some out of bound area if there's any */
   width = rect->width;
   height = rect->height;
   if (rect->dx + width > info->var.xres_virtual)
      width = info->var.xres_virtual - rect->dx;
   if (rect->dy + height > info->var.yres_virtual)
      height = info->var.yres_virtual - rect->dy;

   memset(&param, 0, sizeof(param));
   param.mode = GE_MODE_SYNC;
   param.operation = GE_OP_FILL_COLOR;
   if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
      /* get color from the palette array */
      param.option.color =
         ((uint32_t *)info->pseudo_palette)[rect->color];
   }
   else {
      /* use color directly */
      param.option.color = rect->color;
   }
   param.src2_dst_format =
      (info->var.bits_per_pixel == 32) ? GE_COLOR_ARGB888 : GE_COLOR_RRGB565;
   param.dx = rect->dx;
   param.dy = rect->dy;
   param.width = width;
   param.height = height;
   param.src2_dst_pitch = info->var.xres_virtual;
   param.dst_addr = info->fix.smem_start;

   if (ge_engage(&param) < 0)
      printk(KERN_ERR "FB: ge_engage failed in fb_fillrect\n");
}
#endif

#if defined(CONFIG_BCMRING_GEV3_MODULE) || defined(CONFIG_BCMRING_GEV3)
static void fillrect_color(struct fb_info *info,
      LCD_FillRectColor_t *rect)
{
   unsigned int width, height;
   struct ge_param param;

   if (cap_isPresent(CAP_GE, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "FB: Hardware acceleration of fillrect not "
            "supported\n");
      return;
   }

   /* validate parameters */
   if (rect->width == 0 || rect->height == 0 ||
       rect->dx > info->var.xres_virtual ||
       rect->dy > info->var.yres_virtual) {
      return;
   }

   /* chop off some out of bound area if there's any */
   width = rect->width;
   height = rect->height;
   if (rect->dx + width > info->var.xres_virtual)
      width = info->var.xres_virtual - rect->dx;
   if (rect->dy + height > info->var.yres_virtual)
      height = info->var.yres_virtual - rect->dy;

   memset(&param, 0, sizeof(param));
   param.mode = GE_MODE_SYNC;
   param.operation = GE_OP_FILL_COLOR;
   param.option.color = rect->rawColor;
   param.src2_dst_format =
      (info->var.bits_per_pixel == 32) ? GE_COLOR_ARGB888 : GE_COLOR_RRGB565;
   param.dx = rect->dx;
   param.dy = rect->dy;
   param.width = width;
   param.height = height;
   param.src2_dst_pitch = info->var.xres_virtual;
   param.dst_addr = info->fix.smem_start;

   if (ge_engage(&param) < 0)
      printk(KERN_ERR "FB: ge_engage failed in fb_fillrect\n");
}
#endif

static void lcd_info_get(LCD_Info_t *lcd_info, struct fb_info *info)
{
   lcd_info->width = info->var.xres;
   lcd_info->height = info->var.yres;
   lcd_info->bitsPerPixel = info->var.bits_per_pixel;
}

/*
 * IOCTL command processing
 */
static int ioctl_handler(struct fb_info *info, unsigned int cmd,
      unsigned long arg)
{
   int rc = 0;

   switch (cmd) {
      case LCD_IOCTL_INFO:
      {
         LCD_Info_t lcd_info;

         lcd_info_get(&lcd_info, info);

         rc = copy_to_user((void *)arg, &lcd_info, sizeof(LCD_Info_t));
         if (rc != 0) {
            return -EFAULT;
         }
         return 0;
      }

      case LCDFB_IOCTL_UPDATE_LCD:
      {
         return 0;
      }
#if defined(CONFIG_BCMRING_GEV3_MODULE) || defined(CONFIG_BCMRING_GEV3)
      case LCD_IOCTL_COPYAREA:
      {
         struct fb_copyarea area;

         if (cap_isPresent(CAP_GE, 0) != CAP_PRESENT) {
            printk(KERN_WARNING "FB: Hardware acceleration of copyarea not "
                  "supported\n");
            return -EFAULT;
         }

         if (copy_from_user(&area, (struct fb_copyarea *)arg,
                  sizeof(area)) != 0) {
            return -EFAULT;
         }

         fb_copyarea(info, &area);
         return 0;
      }


      case LCD_IOCTL_FILLRECT_COLOR:
      {
         LCD_FillRectColor_t rect;

         if (cap_isPresent(CAP_GE, 0) != CAP_PRESENT) {
            printk(KERN_WARNING "FB: Hardware acceleration of fillrect not "
                  "supported\n");
            return -EFAULT;
         }

         if (copy_from_user(&rect, (LCD_FillRectColor_t *)arg,
                    sizeof(rect)) != 0) {
            return -EFAULT;
         }

         fillrect_color(info, &rect);
         return 0;
      }
#endif
      case FBIO_WAITFORVSYNC:
      {
         uint32_t crtc;

         if (get_user(crtc, (uint32_t __user *) arg))
            return -EFAULT;

         if (crtc != 0)
            return -ENODEV;

         return lcd_wait_for_vsync();
      }
      case FBIOGET_VBLANK:
      {
         struct fb_vblank vblank = { 0 };
         vblank.flags = FB_VBLANK_HAVE_VBLANK;

         if (lcd_is_vsyncing())
            vblank.flags |= FB_VBLANK_VBLANKING;

         if (copy_to_user((void *) arg, &vblank, sizeof vblank))
            return -EFAULT;

         return 0;
      }

      default:
         return -EINVAL;
   }
   return 0;
}


static struct fb_ops fb_ops = {
   .owner = THIS_MODULE,
   .fb_check_var = bcmringfb_check_var,
   .fb_set_par = bcmringfb_set_par,
   .fb_setcolreg = bcmringfb_setcolreg,
   .fb_pan_display = bcmringfb_pan_display,
#if defined(CONFIG_BCMRING_GEV3_MODULE) || defined(CONFIG_BCMRING_GEV3)
   .fb_fillrect = fb_fillrect,
   .fb_copyarea = fb_copyarea,
#endif
   .fb_imageblit = cfb_imageblit,
   .fb_ioctl = ioctl_handler,
   .fb_blank = bcmringfb_blank,
};

static int fb_probe(struct platform_device *pdev)
{
   int rval;
   struct bcmring_fb *fb;
   struct fb_info *fb_info;
   struct lcd_panel_param *panel;
   struct lcd_buf_param *buf;

   atomic_set(&bus_is_probed, 0);

   if (cap_isPresent(CAP_CLCD, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "FB: Not supported\n");
      return -ENODEV;
   }

   fb = kzalloc(sizeof(struct bcmring_fb), GFP_KERNEL);
   if (!fb) {
      printk(KERN_ERR "FB: Unable to allocate new bcmring_fb struct\n");
      rval = -ENOMEM;
      goto err_exit;
   }
   gFB = fb;
   fb_info = &fb->fb_info;
   panel = &fb->panel;
   buf = &fb->buf;

   /* get panel info */
   rval = lcd_panel_info_get(panel);
   if (rval < 0) {
      printk(KERN_ERR "FB: Failed to get panel information\n");
      goto err_free_fb;
   }
   
   /* get LCD buffer info */
   rval = lcd_buf_info_get(buf);
   if (rval < 0) {
      printk(KERN_ERR "FB: Failed to get panel buffer information\n");
      goto err_free_fb;
   }

   fb_info->fbops = &fb_ops;
   fb_info->flags = FBINFO_FLAG_DEFAULT | FBINFO_HWACCEL_COPYAREA |
      FBINFO_HWACCEL_FILLRECT;
   fb_info->pseudo_palette = fb->palette;

   strncpy(fb_info->fix.id, fb_name, sizeof(fb_info->fix.id));
   fb_info->fix.type = FB_TYPE_PACKED_PIXELS;
   fb_info->fix.visual = FB_VISUAL_TRUECOLOR;
   fb_info->fix.accel = FB_ACCEL_NONE;
   fb_info->screen_base = buf->data.ptr; /* virtual address */
   fb_info->fix.smem_start = buf->data.addr; /* physical address */
   fb_info->fix.smem_len = buf->data.len; /* lendth of the double buffer */
   fb_info->fix.xpanstep = 0;
   fb_info->fix.ypanstep = 1;
   fb_info->fix.ywrapstep = 0;

   fb_info->var.xres = buf->xres;
   fb_info->var.yres = buf->yres;
   fb_info->var.xres_virtual = buf->virtual_xres;
   fb_info->var.yres_virtual = buf->virtual_yres;
   fb_info->var.bits_per_pixel = buf->bpp;
   fb_info->var.left_margin = panel->left_margin;
   fb_info->var.right_margin = panel->right_margin;
   fb_info->var.upper_margin = panel->upper_margin;
   fb_info->var.lower_margin = panel->lower_margin;
   fb_info->var.hsync_len = panel->hsync_len;
   fb_info->var.vsync_len = panel->vsync_len;
   fb_info->var.vmode = FB_VMODE_NONINTERLACED;
   fb_info->var.activate = FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;
   fb_info->var.height = -1;
   fb_info->var.width = -1;

   fb_info->fix.line_length = fb_info->var.xres_virtual *
      fb_info->var.bits_per_pixel / 8; /* pitch in bytes */

   /* validate the vars and set color bitfields */
   rval = bcmringfb_check_var(&fb_info->var, fb_info);
   if (rval != 0) {
      goto err_free_fb;
   }

   fb_alloc_cmap(&fb_info->cmap, CMAP_SIZE, 0);
   fb_set_var(fb_info, &fb_info->var);
   register_framebuffer(fb_info);
   platform_set_drvdata(pdev, fb);
   atomic_set(&bus_is_probed, 1);

   return 0;

err_free_fb:
   kfree(fb);
err_exit:
   return rval;
}

static int fb_remove(struct platform_device *pdev)
{
   struct bcmring_fb *fb = platform_get_drvdata(pdev);

   atomic_set(&bus_is_probed, 0);
   unregister_framebuffer(&fb->fb_info);
   framebuffer_release(&fb->fb_info);
   kfree(fb);

   return 0;
}

#ifdef CONFIG_PM
/*
 * This routine assumes that by the time it gets executed, lcd_update has
 * completed and will not re-engage until lcd_resume is done
 */
static int fb_suspend(struct platform_device *pdev, pm_message_t mesg)
{
   struct bcmring_fb *fb = platform_get_drvdata(pdev);
   struct fb_info *info = &fb->fb_info;

   /*
    * We have the low-level LCD and GE drivers handle their own suspend, so
    * here we just need to notify the Linux framebuffer subsystem and that's
    * it
    */
   acquire_console_sem();
   fb_set_suspend(info, 1);
   release_console_sem();

   return 0;
}

static int fb_resume(struct platform_device *pdev)
{
   struct bcmring_fb *fb = platform_get_drvdata(pdev);
   struct fb_info *info = &fb->fb_info;

   acquire_console_sem();
   fb_set_suspend(info, 0);
   release_console_sem();

   return 0;
}
#else
#define fb_suspend    NULL
#define fb_resume     NULL
#endif

static struct platform_driver fb_driver = {
   .driver = {
      .name = "bcmring-fb",
      .owner = THIS_MODULE,
   },
   .probe = fb_probe,
   .remove = fb_remove,
   .suspend = fb_suspend,
   .resume = fb_resume,
};

static int __init fb_init(void)
{
   return platform_driver_register(&fb_driver);
}

static void __exit fb_exit(void)
{
   platform_driver_unregister(&fb_driver);
}

module_init(fb_init);
module_exit(fb_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Framebuffer Driver");
MODULE_LICENSE("GPL");
