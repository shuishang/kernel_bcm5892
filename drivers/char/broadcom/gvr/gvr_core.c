/*****************************************************************************
* Copyright 2005 - 2009 Broadcom Corporation.  All rights reserved.
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
 * The GVR core driver
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/ioctl.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include <asm/uaccess.h>
#include <mach/csp/cap.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/mmdma.h>
#include <linux/broadcom/bcmring_ge.h>
#include <linux/broadcom/bcmring_lcd.h>
#include <linux/broadcom/gvr.h>
#include <linux/broadcom/gvr_ioctl.h>

/*TODO: add RGB565 support in GVR */

#define MODULE_NAME                "GVR"

#define MAX_PROC_BUF_SIZE          256
#define PROC_PARENT_DIR            "gvr"
#define PROC_ENTRY_DBG             "dbg"
#define PROC_ENTRY_KNLLOG          "knllog"
#define PROC_ENTRY_STATS           "stats"
#define PROC_ENTRY_ELEMENT_LIST    "elementList"
#define PROC_ENTRY_MASTER          "master"

#define GVR_ERR(fmt, args...) printk(KERN_ERR "%s: " fmt, __FUNCTION__, ## args)
#define GVR_DBG(fmt, args...) do { if (gDbgEnable) printk(KERN_INFO "%s: " fmt, __FUNCTION__ , ## args); } while (0)
#define GVR_LOG(fmt, args...) do { if (gKnllogEnable) KNLLOG(fmt, ## args); } while (0)
#define ELEMENT_GET(update_ptr)    container_of(update_ptr, ELEMENT, update)

union gvr_ioctl_params {
   GVR_IOCTL_LCD_INFO_GET_PARAM lcd_info_get;
   GVR_IOCTL_LCD_PANEL_SET_PARAM lcd_panel_set;
   GVR_IOCTL_ELEMENT_ADD_PARAM element_add;
   GVR_IOCTL_ELEMENT_REMOVE_PARAM element_remove;
   GVR_IOCTL_ELEMENT_MOD_PARAM element_mod;
   GVR_IOCTL_ELEMENT_GET_PARAM element_get;
   GVR_IOCTL_ELEMENT_SET_MASTER_PARAM element_set_master;
   GVR_IOCTL_ELEMENT_GET_MASTER_PARAM element_get_master;
   GVR_IOCTL_ELEMENT_UPDATE_PARAM element_update;
};

/*
 * When an element is active, their changes will be seen on the LCD after an
 * update. An element becomes active after its buffer address is specified
 */
typedef enum element_state {
   ELEMENT_STATE_INACTIVE = 0,
   ELEMENT_STATE_ACTIVE,
} ELEMENT_STATE;

/*
 * An element is idle while no update associated with that element is being
 * processed. The user can only call update when its element is in the idle
 * state
 */
typedef enum element_update_state {
   /*
    * The element is currently idle and there's no current update request
    * pending or being processed associated with this element
    */
   ELEMENT_UPDATE_STATE_IDLE,

   /*
    * There's an onging update request pending or being processed with this
    * element. Any futher update request will be rejected
    */
   ELEMENT_UPDATE_STATE_UPDATING,
} ELEMENT_UPDATE_STATE;

typedef struct update {
   volatile ELEMENT_UPDATE_STATE state;

   /* return value / error code */
   volatile int rval;

   /* to signal the completion of the update */
   struct completion complete;

   struct list_head list;
} UPDATE;

typedef struct element {
   /* element lock */
   struct mutex lock;

   /* user controlled parameters */
   GVR_ELEMENT_PARAM param;
   
   /* active/inactive states */
   volatile ELEMENT_STATE state;

   /* device id, to track which device the element is associated with */
   GVR_HDL dev_id;

   /* the update data structure */
   UPDATE update;

   struct list_head list;
} ELEMENT;

typedef struct element_ctrl {
   /* lock for the element linked list */
   struct mutex list_lock;

   /* total number of elements in the system */
   volatile unsigned int tot_cnt;

   /* linked list of all elements with ascending z-order */
   struct list_head list;
} ELEMENT_CTRL;

typedef struct update_ctrl {
   /* lock for the update linked list */
   struct mutex list_lock;

   /* to control when to wake up / sleep the update processing thread */
   wait_queue_head_t start;
   
   /*
    * The update processing FIFO count. The update processing thread goes to
    * sleep when the FIFO count = 0
    */
   volatile unsigned int fifo_cnt;
   
   /*
    * The total update request count.
    * tot_cnt = fifo_cnt + pending/waiting cnt
    */
   volatile unsigned int tot_cnt;
   
   /* linked list of the update reuqests */
   struct list_head list;
   
   /*
    * Linked list of the update requests that are 'waiting'. Update requests
    * are put to the wait list when there is a master element in the system
    * and the update request does NOT come from the master element
    */
   struct list_head wait_list;
} UPDATE_CTRL;

typedef struct gvr_ctrl {
   /* flag to indicate whether the GVR has been initialized or not */
   int is_initialized;

   /*
    * When the system only has one element and the buffer dimension of the
    * element is equal to that of the LCD internal buffer, the system is set
    * to bypass mode. In bypass mode, the LCD controller uses the external
    * buffer provided by the user instead its internal buffer. This eliminates
    * the data transfer between the user buffer and the LCD internal buffer
    */
   volatile int bypass;

   /* to signal the completion of the kthread shutdown */
   struct completion kthread_stop;

   /*
    * Flag to indicate whether the update processing thread should be shut
    * down or not
    */
   volatile int kthread_shutdown;

   /* update processing kthread */
   struct task_struct *thread;
   

   ELEMENT_CTRL element_ctrl;
   UPDATE_CTRL update_ctrl;

   /* flag to indicate whether the LCD driver supports double buffer or not */
   int has_dbuf;

   /* Y offset of the double buffer (only used when has_dbuf is set) */
   unsigned int yoffset;

   /*
    * Pointer to the master element. If no master element is present, point
    * to NULL obviously
    */
   volatile ELEMENT *master_element;

   /* LCD buffer paramters */
   struct lcd_buf_param lcd;
} GVR_CTRL;

struct proc_dir {
   struct proc_dir_entry *parent_dir;
};

/*
 * Mappings for the GVR color format and bytes per pixel
 */
static const int bytes_per_pixel[GVR_COLOR_FORMAT_MAX] =
   {4, 3};

/*
 * Mappings between the GE/GVR color formats
 */
static const enum ge_color_format ge_color_format[GVR_COLOR_FORMAT_MAX] =
   {GE_COLOR_URGB888, GE_COLOR_PRGB888};

/* enable/disable debug messages */
static volatile int gDbgEnable;

/* enable/disable knllog */
static volatile int gKnllogEnable;

static struct proc_dir gProc;
static GVR_CTRL gGvrCtrl;

/*
 * Given a color format, returns the number of bytes per pixel
 */
static inline int bytes_per_pixel_get(GVR_COLOR_FORMAT format)
{
   if (format < GVR_COLOR_FORMAT_MAX)
      return bytes_per_pixel[format];
   else
      return -EINVAL;
}

/*
 * Given a GVR color format, returns the GE color format
 */
static inline int ge_color_format_get(GVR_COLOR_FORMAT format)
{
   if (format < GVR_COLOR_FORMAT_MAX)
      return ge_color_format[format];
   else
      return -EINVAL;
}

/*
 * Returns 1 after the GVR is initialized, otherwise returns 0
 */
static inline int gvr_is_initialized(void)
{
   GVR_CTRL *gvr = &gGvrCtrl;

   return gvr->is_initialized;
}

/*
 * Convert the element handle to the element pointer
 */
static inline ELEMENT *element_map_ptr(GVR_ELEMENT_HDL element_hdl)
{
   return (ELEMENT *)element_hdl;
}

/*
 * Add a new element to the list. The user should grab the required locks
 */
static inline void element_add_unsafe(ELEMENT *element, int layer)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   ELEMENT_CTRL *ectrl = &gvr->element_ctrl;
   ELEMENT *tmp;
   int added = 0;

   list_for_each_entry(tmp, &ectrl->list, list) {
      if (layer <= tmp->param.layer) {
         __list_add(&element->list, tmp->list.prev, &tmp->list);
         added = 1;
         break;
      }
   }
      
   if (!added) 
      list_add_tail(&element->list, &ectrl->list);
}

/*
 * Examine the element list. If there's only one element in the system and the
 * element's buffer's dimension is the same as that of the LCD internal
 * buffer, set the system to bypass mode. Otherwise bring the system out of the
 * bypass mode
 */
static void bypass_mode_test_and_set_unsafe(ELEMENT_CTRL *ectrl)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   struct lcd_buf_param *lcd = &gvr->lcd;
   ELEMENT *element;
   GVR_ELEMENT_PARAM *param;

   if (list_empty(&ectrl->list))
      return;

   element = list_first_entry(&ectrl->list, ELEMENT, list);
   param = &element->param;
   if (ectrl->tot_cnt == 1 &&
         param->buf_addr &&
         param->buf_width == lcd->virtual_xres &&
         param->buf_height == lcd->virtual_yres &&
         param->width == lcd->xres &&
         param->height == lcd->yres &&
         bytes_per_pixel_get(param->color_format) == (lcd->bpp / 8)) {
      /* if not already in bypass mode, set it to bypass mode */
      if (!gvr->bypass) {
         gvr->bypass = 1;
         lcd_buf_addr_set(param->buf_addr);
      }
   } else {
      /* if already in bypass mode, bring it out of bypass mode */
      if (gvr->bypass) {
         gvr->bypass = 0;
         lcd_buf_addr_set(0);
      }
   }
}

/*
 * Rendering the contents of an element to the LCD buffer
 */
static int frames_render(ELEMENT *element)
{
   int rc;
   GVR_CTRL *gvr = &gGvrCtrl;
   GVR_ELEMENT_PARAM *param = &element->param;
   struct lcd_buf_param *lcd = &gvr->lcd;
   struct ge_param ge;

   /*
    * In the case when there's no alpha blending required and the element
    * has a full width, use DMA to copy data because it's faster than the
    * graphic engine
    */
   if (param->alpha_opt == GVR_ALPHA_OPT_NONE &&
         param->width == param->buf_width &&
         param->width == lcd->virtual_xres &&
         bytes_per_pixel_get(param->color_format) == lcd->bpp / 8) {
      dma_addr_t src_addr, dst_addr;
      unsigned int bytes_per_pixel;
      uint32_t data_len;
            
      bytes_per_pixel = bytes_per_pixel_get(param->color_format);

      src_addr = element->param.buf_addr +
         (param->sy * param->buf_width * bytes_per_pixel) +
         (param->sx * bytes_per_pixel);

      dst_addr = lcd->data.addr +
         (param->dy * lcd->virtual_xres * lcd->bpp / 8) +
         (param->dx * lcd->bpp / 8) + 
         (gvr->yoffset * lcd->virtual_xres * lcd->bpp / 8);

      data_len = param->width * param->height * bytes_per_pixel;
            
      rc = mmdma_transfer(src_addr, dst_addr, data_len);
      if (rc) {
         GVR_ERR("mmdma_transfer failed, err=%d\n", rc);
         return rc;
      }
   } else {
      /* set up GE parameters */
      memset(&ge, 0, sizeof(ge));

      ge.mode = GE_MODE_SYNC;
      ge.src1_endian = GE_ENDIAN_LITTLE;
      ge.src2_dst_endian = GE_ENDIAN_LITTLE;

      ge.src1_format = ge_color_format_get(param->color_format);
      ge.src2_dst_format = GE_COLOR_URGB888;

      ge.s1x = param->sx;
      ge.s1y = param->sy;
      ge.s2x = param->dx;
      ge.s2y = param->dy;
      ge.dx = param->dx;
      ge.dy = param->dy;
      ge.width = param->width;
      ge.height = param->height;
            
      ge.src1_pitch = param->buf_width;
      ge.src2_dst_pitch = lcd->virtual_xres;

      ge.src1_addr = element->param.buf_addr;
      ge.src2_addr = lcd->data.addr +
         (gvr->yoffset * lcd->virtual_xres * lcd->bpp / 8);
      ge.dst_addr = ge.src2_addr;

      /* alpha blending required */
      if (element->param.alpha_opt != GVR_ALPHA_OPT_NONE) {
         ge.operation = GE_OP_ALPHA_BLEND;
         if (element->param.alpha_opt == GVR_ALPHA_OPT_GLOBAL) {
            ge.option.alpha.use_global_alpha = 1;
            ge.option.alpha.global_alpha = param->global_alpha;
         } else { /* in-pixel alpha channel */
            ge.option.alpha.use_global_alpha = 0;
            ge.src1_format = GE_COLOR_ARGB888;
         }
         ge.s2x = ge.dx;
         ge.s2y = ge.dy;
      } else { /* no alpha blending, just copy it over */
         ge.operation = GE_OP_RASTER;
         ge.option.raster.op = GE_RASTER_S1;
      }

      rc = ge_engage(&ge);
      if (rc) {
         GVR_ERR("ge_engage failed, err=%d\n", rc);
         return rc;
      }
   }

   return 0;
}

/*
 * Kernel thread that processes the update requests
 */
static void update_process(void)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   ELEMENT_CTRL *ectrl = &gvr->element_ctrl;
   ELEMENT *element;
   UPDATE_CTRL *uctrl = &gvr->update_ctrl;
   UPDATE *tmp_update, *n;
   struct lcd_buf_param *lcd = &gvr->lcd;
   int rc, require_update;

   for (;;) {
      /* block and wait for an update request to come */
      wait_event(uctrl->start, uctrl->fifo_cnt);

      /* driver shutting down... let's quit the kthread */
      if (gvr->kthread_shutdown)
         break;

      /*
       * The update list should not be empty here unless there's something
       * wrong
       */
      if (list_empty(&uctrl->list)) {
         GVR_ERR("update list is empty but update FIFO count = %d!\n",
               uctrl->fifo_cnt);
         uctrl->fifo_cnt = 0;
         continue;
      }

      mutex_lock(&uctrl->list_lock);
      require_update = 0;
      list_for_each_entry_safe(tmp_update, n, &uctrl->list, list) {
         element = ELEMENT_GET(tmp_update);

         /*
          * Firstly, move the update requests from the main list to the
          * pending list 
          */
         list_del(&tmp_update->list);
         list_add_tail(&tmp_update->list, &uctrl->wait_list);
         uctrl->fifo_cnt--;

         if (gvr->master_element) { /* when there's a freaking master */
            if (gvr->master_element == element) { /* update is from master */
               require_update = 1;
            }
         } else { /* no master, always need to update */
            require_update = 1;
         }
      } /* for loop done */

      /* error check */
      if (uctrl->fifo_cnt) {
         GVR_ERR("Finished processing update list but update FIFO count != 0 (%d)!\n",
               uctrl->fifo_cnt);
      }

      /* If update is not required, go back to sleep */
      if (!require_update) {
         mutex_unlock(&uctrl->list_lock);
         continue;
      }

      mutex_unlock(&uctrl->list_lock);

      if (gvr->bypass) {
         GVR_LOG("tstart [GVR update]");
         
         /* there should be only one element in bypass mode */
         element = list_first_entry(&ectrl->list, ELEMENT, list);
         
         mutex_lock(&element->lock);

         GVR_LOG("tstart [GVR Vsync]");
         rc = lcd_update(element->param.dy);
         GVR_LOG("tstop [GVR Vsync]");
         if (rc)
            GVR_ERR("lcd_update failed! err=%d\n", rc);

         mutex_unlock(&element->lock);
         
         GVR_LOG("tstop [GVR update]");
      } else {
         /*
          * Let's start updating by going through the element list and process
          * all active elements
          */
         mutex_lock(&ectrl->list_lock);
         GVR_LOG("tstart [GVR update]");
         list_for_each_entry(element, &ectrl->list, list) {
            /* skip non-active elements */
            if (element->state != ELEMENT_STATE_ACTIVE)
               continue;

            /* render the frames */
            mutex_lock(&element->lock);
            GVR_LOG("tstart [GVR rendering 0x%x]", element);
            rc = frames_render(element);
            GVR_LOG("tstop [GVR rendering 0x%x]", element);
            if (rc)
               GVR_ERR("frame rendering failed! err=%d\n", rc);
            mutex_unlock(&element->lock);
         } /* for each active element */
         mutex_unlock(&ectrl->list_lock);

         /* update the screen if there's double buffer */
         if (gvr->has_dbuf) {
            GVR_LOG("tstart [GVR Vsync]");
            rc = lcd_update(gvr->yoffset);
            GVR_LOG("tstop [GVR Vsync]");
            if (rc)
               GVR_ERR("lcd_update failed! err=%d\n", rc);

            /* switch to the inactive buffer for the next update */
            if (gvr->yoffset == 0)
               gvr->yoffset = lcd->yres;
            else
               gvr->yoffset = 0;
         }
         GVR_LOG("tstop [GVR update]");
      }

      /*
       * Now we are done with the current update, need to do some maintainance
       * work to the update list 
       */
      mutex_lock(&uctrl->list_lock);

      /*
       * Remove the all updates from the wait list and signal the update
       * completion
       */
      list_for_each_entry_safe(tmp_update, n, &uctrl->wait_list, list) {
         list_del(&tmp_update->list);
         uctrl->tot_cnt--;
         complete(&tmp_update->complete);
      }
      mutex_unlock(&uctrl->list_lock);
   }

   gvr->thread = NULL;
   complete(&gvr->kthread_stop);
}

static int
proc_dbg_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
   int rc, enable;
   unsigned char kbuf[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   rc = copy_from_user(kbuf, buffer, count);
   if (rc) {
      printk(KERN_ERR "copy_from_user failed status=%d", rc);
      return -EFAULT;
   }

   if (sscanf(kbuf, "%d", &enable) != 1) {
      printk(KERN_ERR "echo <enable> > /proc/%s/%s\n",
            PROC_PARENT_DIR, PROC_ENTRY_DBG);
      return count;
   }

   if (enable)
      gDbgEnable = 1;
   else
      gDbgEnable = 0;

   printk(KERN_INFO "GVR debugging mode %s\n",
         (enable ? "enabled" : "disabled"));

   return count;
}

static int
proc_dbg_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "GVR debugging mode is %s\n",
         (gDbgEnable) ? "enabled" : "disabled");

   return len;
}

static int
proc_knllog_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
   int rc, enable;
   unsigned char kbuf[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;

   rc = copy_from_user(kbuf, buffer, count);
   if (rc) {
      printk(KERN_ERR "copy_from_user failed status=%d", rc);
      return -EFAULT;
   }

   if (sscanf(kbuf, "%d", &enable) != 1) {
      printk(KERN_ERR "echo <enable> > /proc/%s/%s\n",
            PROC_PARENT_DIR, PROC_ENTRY_KNLLOG);
      return count;
   }

   if (enable)
      gKnllogEnable = 1;
   else
      gKnllogEnable = 0;

   printk(KERN_INFO "GVR knllog turned %s\n",
         (enable ? "ON" : "OFF"));

   return count;
}

static int
proc_knllog_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "GVR knllog is turned %s\n",
         (gKnllogEnable) ? "ON" : "OFF");

   return len;
}

static int
proc_stats_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   unsigned int len = 0;
   GVR_CTRL *gvr = &gGvrCtrl;
   UPDATE_CTRL *uctrl = &gvr->update_ctrl;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "GVR statistics snapshot:\n");
   len += sprintf(buffer + len, "tot_cnt: %d\n", uctrl->tot_cnt);
   len += sprintf(buffer + len, "fifo_cnt: %d\n", uctrl->fifo_cnt);

   return len;
}

static int
proc_master_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   unsigned int len = 0;
   GVR_CTRL *gvr = &gGvrCtrl;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "GVR master mode is turned %s\n",
         (gvr->master_element) ? "ON" : "OFF");

   return len;
}

static int
proc_element_list_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   int index;
   unsigned int len = 0;
   GVR_CTRL *gvr = &gGvrCtrl;
   ELEMENT_CTRL *ectrl = &gvr->element_ctrl;
   ELEMENT *element, *n;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "GVR list of elements:\n");

   mutex_lock(&ectrl->list_lock);
   index = 0;
   list_for_each_entry_safe(element, n, &ectrl->list, list) {
      GVR_ELEMENT_PARAM *param = &element->param;
      
      len += sprintf(buffer + len, "[%d] %s %s hdl=%08x addr=%08x ",
            index,
            (element->state == ELEMENT_STATE_ACTIVE) ? "active" : "inactive",
            (element == gvr->master_element) ? "master" : "non-master",
            (unsigned int)element,
            element->param.buf_addr);
      len += sprintf(buffer + len, "color_f=%d buf_width=%u buf_height=%u "
            "alpha_opt=%d global_alpha=%u layer=%d sx=%u sy=%u dx=%u dy=%u"
            "width=%u height=%u\n",
            param->color_format, param->buf_width, param->buf_height,
            param->alpha_opt, param->global_alpha, param->layer, param->sx,
            param->sy, param->dx, param->dy, param->width, param->height);

      index++;
   }
   mutex_unlock(&ectrl->list_lock);

   return len;
}

/*
 * Initialize the proc entries
 */
static int proc_init(void)
{
   int rc;
   struct proc_dir_entry *proc_dbg;
   struct proc_dir_entry *proc_knllog;
   struct proc_dir_entry *proc_stats;
   struct proc_dir_entry *proc_element_list;
   struct proc_dir_entry *proc_master;

   gProc.parent_dir = proc_mkdir(PROC_PARENT_DIR, NULL);

   proc_dbg = create_proc_entry(PROC_ENTRY_DBG, 0644, gProc.parent_dir);
   if (proc_dbg == NULL) {
      rc = -ENOMEM;
      goto err_exit;
   }
   proc_dbg->read_proc = proc_dbg_read;
   proc_dbg->write_proc = proc_dbg_write;
   proc_dbg->data = NULL;

   proc_knllog = create_proc_entry(PROC_ENTRY_KNLLOG, 0644, gProc.parent_dir);
   if (proc_knllog == NULL) {
      rc = -ENOMEM;
      goto err_del_dbg;
   }
   proc_knllog->read_proc = proc_knllog_read;
   proc_knllog->write_proc = proc_knllog_write;
   proc_knllog->data = NULL;

   proc_stats = create_proc_entry(PROC_ENTRY_STATS, 0644, gProc.parent_dir);
   if (proc_stats == NULL) {
      rc = -ENOMEM;
      goto err_del_knllog;
   }
   proc_stats->read_proc = proc_stats_read;
   proc_stats->write_proc = NULL;
   proc_stats->data = NULL;

   proc_element_list = create_proc_entry(PROC_ENTRY_ELEMENT_LIST, 0644,
         gProc.parent_dir);
   if (proc_element_list == NULL) {
      rc = -ENOMEM;
      goto err_del_stats;
   }
   proc_element_list->read_proc = proc_element_list_read;
   proc_element_list->write_proc = NULL;
   proc_element_list->data = NULL;

   proc_master = create_proc_entry(PROC_ENTRY_MASTER, 0644, gProc.parent_dir);
   if (proc_master == NULL) {
      rc = -ENOMEM;
      goto err_del_element_list;
   }
   proc_master->read_proc = proc_master_read;
   proc_master->write_proc = NULL;
   proc_master->data = NULL;
   
   return 0;

err_del_element_list:
   remove_proc_entry(PROC_ENTRY_ELEMENT_LIST, gProc.parent_dir);

err_del_stats:
   remove_proc_entry(PROC_ENTRY_STATS, gProc.parent_dir);

err_del_knllog:
   remove_proc_entry(PROC_ENTRY_KNLLOG, gProc.parent_dir);

err_del_dbg:
   remove_proc_entry(PROC_ENTRY_DBG, gProc.parent_dir);

err_exit:
   remove_proc_entry(PROC_PARENT_DIR, NULL);

	return rc;
}

/*
 * Terminate and remove the proc entries
 */
static void proc_term(void)
{
   remove_proc_entry(PROC_ENTRY_MASTER, gProc.parent_dir);
   remove_proc_entry(PROC_ENTRY_ELEMENT_LIST, gProc.parent_dir);
   remove_proc_entry(PROC_ENTRY_STATS, gProc.parent_dir);
   remove_proc_entry(PROC_ENTRY_KNLLOG, gProc.parent_dir);
   remove_proc_entry(PROC_ENTRY_DBG, gProc.parent_dir);
   remove_proc_entry(PROC_PARENT_DIR, NULL);
}

static int gvr_dev_open(struct inode *inode, struct file *filp)
{
   return gvr_open((GVR_HDL *)(&filp->private_data));
}

static int gvr_dev_release(struct inode *inode, struct file *filp)
{
   return gvr_close((GVR_HDL)filp->private_data);
}

static int
gvr_dev_ioctl(struct inode *inode, struct file *filp,
      unsigned int cmd, unsigned long arg)
{
   int rc = 0;
   GVR_HDL dev_id;
   union gvr_ioctl_params param;
   unsigned int cmdnr, data_size;

   dev_id = (GVR_HDL)filp->private_data; 
   cmdnr = _IOC_NR(cmd);
   data_size = _IOC_SIZE(cmd);

   if (data_size > sizeof(param))
      return -EINVAL;

   if (_IOC_DIR(cmd) & _IOC_WRITE) {
      rc = copy_from_user(&param, (void *)arg, data_size);
      if (rc) {
         GVR_ERR("copy_from_user failed, status=%d\n", rc);
         return rc;
      }
   }

   switch (cmd)
   {
      case GVR_IOCTL_LCD_INFO_GET:
      {
         rc = gvr_lcd_info_get(dev_id, &param.lcd_info_get.info);
         if (rc) {
            GVR_ERR("gvr_lcd_info_get failed, status=%d\n", rc);
            return rc;
         }
      }
      break;

      case GVR_IOCTL_LCD_PANEL_SET:
      {
         rc = gvr_lcd_panel_set(dev_id, param.lcd_panel_set.act);
         if (rc) {
            GVR_ERR("gvr_lcd_panel_set failed, status=%d\n", rc);
            return rc;
         }
      }
      break;

      case GVR_IOCTL_ELEMENT_ADD:
      {
         rc = gvr_element_add(dev_id, &param.element_add.element,
               &param.element_add.element_hdl);
         if (rc) {
            GVR_ERR("gvr_element_add failed, status=%d\n", rc);
            return rc;
         }
      }
      break;

      case GVR_IOCTL_ELEMENT_REMOVE:
      {
         rc = gvr_element_remove(dev_id, param.element_remove.element_hdl);
         if (rc) {
            GVR_ERR("gvr_element_remove failed, status=%d\n", rc);
            return rc;
         }
      }
      break;

      case GVR_IOCTL_ELEMENT_MOD:
      {
         rc = gvr_element_mod(dev_id, param.element_mod.element_hdl,
               &param.element_mod.element);
         if (rc) {
            GVR_ERR("gvr_element_mod failed, status=%d\n", rc);
            return rc;
         }
      }
      break;

      case GVR_IOCTL_ELEMENT_GET:
      {
         rc = gvr_element_get(dev_id, param.element_get.element_hdl,
               &param.element_get.element);
         if (rc) {
            GVR_ERR("gvr_element_get failed, status=%d\n", rc);
            return rc;
         }
      }
      break;

      case GVR_IOCTL_ELEMENT_SET_MASTER:
      {
         rc = gvr_element_set_master(dev_id, param.element_set_master.element_hdl,
               param.element_set_master.set);
         if (rc) {
            GVR_ERR("gvr_element_set_master failed, status=%d\n", rc);
            return rc;
         }
      }
      break;

       case GVR_IOCTL_ELEMENT_GET_MASTER:
      {
         rc = gvr_element_get_master(dev_id,
               &param.element_get_master.element_hdl);
         if (rc) {
            GVR_ERR("gvr_element_get_master failed, status=%d\n", rc);
            return rc;
         }
      }
      break;

      case GVR_IOCTL_ELEMENT_UPDATE:
      {
         rc = gvr_element_update(dev_id, param.element_update.element_hdl,
               &param.element_update.update);
         if (rc) {
            GVR_ERR("gvr_element_update failed, status=%d\n", rc);
            return rc;
         }
      }
      break;

      default:
         GVR_ERR("Unknown IOCTL command, cmd=%x\n", cmd);
         rc = -EINVAL;
         break;
   }

   if (_IOC_DIR(cmd) & _IOC_READ) {
      rc = copy_to_user((void *)arg, &param, data_size);
      if (rc) {
         GVR_ERR("copy_to_user failed, status=%d\n", rc);
         return rc;
      }
   }

   return rc;
}

static struct file_operations gvr_fops =
{
   owner: THIS_MODULE,
   open: gvr_dev_open,
   release: gvr_dev_release,
   ioctl: gvr_dev_ioctl
};

static int gvr_probe(struct platform_device *pdev)
{
   int rc;
   GVR_CTRL *gvr = &gGvrCtrl;
   UPDATE_CTRL *uctrl = &gvr->update_ctrl;
   ELEMENT_CTRL *ectrl = &gvr->element_ctrl;

   if (cap_isPresent(CAP_GE, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "GVR: Device GE is required for GVR\n");
      return -ENODEV;
   }

   printk(KERN_INFO "GVR: Configuring the GVR core ...\n");

   gDbgEnable = 0;
   gKnllogEnable = 0;

   memset(gvr, 0, sizeof(*gvr));
   init_completion(&gvr->kthread_stop); /* incomplete */

   /* initialize the update control */
   mutex_init(&uctrl->list_lock);
   init_waitqueue_head(&uctrl->start);
   INIT_LIST_HEAD(&uctrl->list);
   INIT_LIST_HEAD(&uctrl->wait_list);

   /* initialize the element control */
   mutex_init(&ectrl->list_lock);
   INIT_LIST_HEAD(&ectrl->list);

   /* get the LCD framebuffer info */
   rc = lcd_buf_info_get(&gvr->lcd);
   if (rc) {
      GVR_ERR("lcd_buf_info_get failed\n");
      goto err_exit;
   }

   /* check the framebuffer parameters */
   if (gvr->lcd.virtual_yres / gvr->lcd.yres >= 2) {
      gvr->has_dbuf = 1;
   } else {
      gvr->has_dbuf = 0;
   }
   gvr->yoffset = 0;

   rc = register_chrdev(BCM_GVR_MAJOR, "gvr", &gvr_fops);
   if (rc < 0) {
      GVR_ERR("register_chrdev failed major=%d\n", BCM_GVR_MAJOR);
      goto err_exit;
   }

   rc = proc_init();
   if (rc) {
      GVR_ERR("proc_init failed\n");
      goto err_unregister_chrdev;
   }

   /* now, start the kernel thread */
   gvr->thread = kthread_run((void *)update_process, NULL, MODULE_NAME);
   if (IS_ERR(gvr->thread)) {
      GVR_ERR("kthread_run of update_process failed\n");
      rc = -EFAULT;
      goto err_remove_proc;
   }

   gvr->is_initialized = 1;
   printk(KERN_INFO "GVR: GVR core configuration is done\n");

   return 0;

err_remove_proc:
   proc_term();

err_unregister_chrdev:
   unregister_chrdev(BCM_GVR_MAJOR, "gvr");

err_exit:
   return rc;
}

static int gvr_remove(struct platform_device *pdev)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   UPDATE_CTRL *uctrl = &gvr->update_ctrl;
   ELEMENT_CTRL *ectrl = &gvr->element_ctrl;
   ELEMENT *element, *e;

   gvr->is_initialized = 0;

   unregister_chrdev(BCM_GVR_MAJOR, "gvr");

   proc_term();

   if (gvr->thread != NULL) {

      gvr->kthread_shutdown = 1;
      /*
       * Fake an update request in case the update processing thread is still
       * sleeping
       */
      uctrl->fifo_cnt++;
      uctrl->tot_cnt++;
      wake_up(&uctrl->start);

      /* wait for kernel thread to die */
      wait_for_completion(&gvr->kthread_stop);
   }

   /* go through the element list and delete all elements */
   list_for_each_entry_safe(element, e, &ectrl->list, list)
      gvr_element_remove(0, element);

   return 0;
}

static struct platform_driver gvr_driver = {
   .driver = {
      .name = "bcm-gvr",
      .owner = THIS_MODULE,
   },
   .probe = gvr_probe,
   .remove = gvr_remove,
};

static int __init gvr_init(void)
{
   return platform_driver_register(&gvr_driver);
}

static void __exit gvr_exit(void)
{
   platform_driver_unregister(&gvr_driver);
}

module_init(gvr_init);
module_exit(gvr_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Renderer");
MODULE_LICENSE("GPL");

int gvr_open(
      GVR_HDL *hdlp)
{
   *hdlp = (GVR_HDL)kmalloc(sizeof(GVR_HDL), GFP_KERNEL);
   if (*hdlp == 0)
      return -ENOMEM;

   return 0;
}

int gvr_close(GVR_HDL hdl)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   ELEMENT_CTRL *ectrl = &gvr->element_ctrl;
   ELEMENT *element, *e;

   /* go through the element list delete all elements with this handle */
   mutex_lock(&ectrl->list_lock);
   list_for_each_entry_safe(element, e, &ectrl->list, list) {
      if (hdl == element->dev_id) {
         mutex_unlock(&ectrl->list_lock);
         gvr_element_remove(hdl, element);
         mutex_lock(&ectrl->list_lock);
      }
   }
   mutex_unlock(&ectrl->list_lock);

   kfree((void *)hdl);

   return 0;
}

int gvr_lcd_info_get(
      GVR_HDL hdl,
      GVR_LCD_INFO *info)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   struct lcd_buf_param *lcd = &gvr->lcd;

   if (gvr_is_initialized() == 0)
      return -ENODEV;

   if (info == NULL)
      return -EINVAL;

   info->xres = lcd->xres;
   info->yres = lcd->yres;
   info->virtual_xres = lcd->virtual_xres;
   info->virtual_yres = lcd->virtual_yres;
   info->bpp = lcd->bpp;

   return 0;
}

int gvr_lcd_panel_set(
      GVR_HDL hdl,
      GVR_LCD_PANEL_ACT act)
{
   if (gvr_is_initialized() == 0)
      return -ENODEV;

   if (act >= GVR_LCD_PANEL_MAX)
      return -EINVAL;

   switch (act) {
      case GVR_LCD_PANEL_DISABLE:
         lcd_panel_disable();
         break;

      case GVR_LCD_PANEL_ENABLE:
         lcd_panel_enable();
         break;

      case GVR_LCD_PANEL_RESET:
         lcd_panel_reset();
         break;

      default:
         return -EINVAL;
   }

   return 0;
}

int gvr_element_add(
      GVR_HDL hdl,
      const GVR_ELEMENT_PARAM *param,
      GVR_ELEMENT_HDL *element_hdl)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   ELEMENT_CTRL *ectrl = &gvr->element_ctrl;
   ELEMENT *element;
   
   if (gvr_is_initialized() == 0)
      return -ENODEV;

   /* make sure color format can be supported */
   if (param == NULL || param->color_format >= GVR_COLOR_FORMAT_MAX)
      return -EINVAL;

   /* make sure the element source region is within the buffer range */
   if (param->sx + param->width > param->buf_width ||
         param->sy + param->height > param->buf_height)
      return -EINVAL;
         
   /* make sure the element destination region is within the LCD buffer range */
   if (param->dx + param->width > gvr->lcd.xres ||
         param->dy + param->height > gvr->lcd.yres)
      return -EINVAL;

   /* allocate element memory */
   element = kcalloc(1, sizeof(*element), GFP_KERNEL);
   if (element == NULL)
      return -ENOMEM;

   /* initialize and copy parameters into element */
   if (param->buf_addr) {
      /*
       * Make the element active if the physical address of the image buffer
       * is known
       */
      element->state = ELEMENT_STATE_ACTIVE;
   } else {
      element->state = ELEMENT_STATE_INACTIVE;
   }
   mutex_init(&element->lock); /* unlock */
   element->dev_id = hdl;
   memcpy(&element->param, param, sizeof(element->param));
   element->update.state = ELEMENT_UPDATE_STATE_IDLE;
   init_completion(&element->update.complete); /* incomplete */

   /*
    * Go through the element list and add the new element to the list in 
    * ascending z-order
    */
   mutex_lock(&ectrl->list_lock);
   mutex_lock(&element->lock);

   /* add the element to the list */
   element_add_unsafe(element, element->param.layer);
   ectrl->tot_cnt++;

   /* now give the user the handle they want */
   *element_hdl = (GVR_ELEMENT_HDL)element;

   /* check and set/unset the system to bypass mode */
   bypass_mode_test_and_set_unsafe(ectrl);

   mutex_unlock(&element->lock);
   mutex_unlock(&ectrl->list_lock);

   return 0;
}

int gvr_element_remove(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL element_hdl)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   ELEMENT_CTRL *ectrl = &gvr->element_ctrl;
   ELEMENT *element;

   if (gvr_is_initialized() == 0)
      return -ENODEV;

   /* find the element */
   element = element_map_ptr(element_hdl);
   if (!element)
      return -EINVAL;

   mutex_lock(&ectrl->list_lock);
   mutex_lock(&element->lock);
   
   /* if it's a master element, unset it */
   if (gvr->master_element == element) {
      gvr->master_element = NULL;
   }

   /* found element, remove it from the list */
   list_del(&element->list);
   ectrl->tot_cnt--;

   /* check and set/unset the system to bypass mode */
   bypass_mode_test_and_set_unsafe(ectrl);

   mutex_unlock(&element->lock);
   mutex_unlock(&ectrl->list_lock);

   kfree(element);
   return 0;
}

int gvr_element_mod(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL element_hdl,
      const GVR_ELEMENT_PARAM *param)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   ELEMENT_CTRL *ectrl = &gvr->element_ctrl;
   ELEMENT *element;

   if (gvr_is_initialized() == 0)
      return -ENODEV;

   /* make sure color format can be supported */
   if (param == NULL || param->color_format >= GVR_COLOR_FORMAT_MAX)
      return -EINVAL;

   /* make sure the element source region is within the buffer range */
   if (param->sx + param->width > param->buf_width ||
         param->sy + param->height > param->buf_height)
      return -EINVAL;
         
   /* make sure the element destination region is within the LCD buffer range */
   if (param->dx + param->width > gvr->lcd.xres ||
         param->dy + param->height > gvr->lcd.yres)
      return -EINVAL;

   element = element_map_ptr(element_hdl);
   if (!element)
      return -EINVAL;
   
   /*
    * Found it, check to see if the z-order layering number has been
    * changed. If it's changed, we need to re-order it in the list
    */
   mutex_lock(&ectrl->list_lock);
   mutex_lock(&element->lock);

   if (element->param.layer != param->layer) {
      list_del(&element->list);
      element_add_unsafe(element, param->layer);
   }

   /* now modify the user paramters */
   memcpy(&element->param, param, sizeof(element->param));

   if (param->buf_addr) {
      element->state = ELEMENT_STATE_ACTIVE;
   } else {
      element->state = ELEMENT_STATE_INACTIVE;
   }

   /* check and set/unset the system to bypass mode */
   bypass_mode_test_and_set_unsafe(ectrl);
   
   mutex_unlock(&element->lock);
   mutex_unlock(&ectrl->list_lock);

   return 0;
}

int gvr_element_get(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL element_hdl,
      GVR_ELEMENT_PARAM *param)
{
   ELEMENT *element;

   if (gvr_is_initialized() == 0)
      return -ENODEV;

   /* find the element */
   element = element_map_ptr(element_hdl);
   if (!element)
      return -EINVAL;

   /* now copy paramters */
   mutex_lock(&element->lock);
   memcpy(param, &element->param, sizeof(*param)); 
   mutex_unlock(&element->lock);

   return 0;
}

int gvr_element_set_master(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL element_hdl,
      unsigned int set)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   ELEMENT_CTRL *ectrl = &gvr->element_ctrl;
   ELEMENT *element;

   if (gvr_is_initialized() == 0)
      return -ENODEV;

   /* find the element */
   element = element_map_ptr(element_hdl);
   if (!element)
      return -EINVAL;

   mutex_lock(&ectrl->list_lock);
   mutex_lock(&element->lock);

   if (set) {
      gvr->master_element = element;
   } else {
      if (gvr->master_element == element) {
         gvr->master_element = NULL;
      }
   }

   mutex_unlock(&element->lock);
   mutex_unlock(&ectrl->list_lock);

   return 0;
}

int gvr_element_get_master(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL *element_hdl)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   ELEMENT_CTRL *ectrl = &gvr->element_ctrl;

   if (gvr_is_initialized() == 0)
      return -ENODEV;

   mutex_lock(&ectrl->list_lock);
   *element_hdl = (GVR_ELEMENT_HDL)gvr->master_element;
   mutex_unlock(&ectrl->list_lock);

   return 0;
}

int gvr_element_update(
      GVR_HDL hdl,
      GVR_ELEMENT_HDL element_hdl,
      const GVR_ELEMENT_UPDATE_PARAM *param)
{
   GVR_CTRL *gvr = &gGvrCtrl;
   UPDATE_CTRL *uctrl = &gvr->update_ctrl;
   UPDATE *update;
   ELEMENT *element;

   if (gvr_is_initialized() == 0)
      return -ENODEV;

   /* find the element associated with this update */
   element = element_map_ptr(element_hdl);
   if (!element)
      return -EINVAL;

   update = &element->update;
   
   mutex_lock(&element->lock);

   /* now, check if there's an ongoing update with this element */
   if (update->state != ELEMENT_UPDATE_STATE_IDLE) {
      mutex_unlock(&element->lock);
      return -EBUSY;
   }

   /* the buffer address needs to be valid by now */
   if (param->buf_addr == 0 && element->param.buf_addr == 0) {
      mutex_unlock(&element->lock);
      return -EINVAL;
   }

   /* store and initialize the element and update information */
   if (param->buf_addr)
      element->param.buf_addr = param->buf_addr;
   if (param->sx != GVR_PARAM_NOCHNAGE)
      element->param.sx = param->sx;
   if (param->sy != GVR_PARAM_NOCHNAGE)
      element->param.sy = param->sy;
   if (param->dx != GVR_PARAM_NOCHNAGE)
      element->param.dx = param->dx;
   if (param->dy != GVR_PARAM_NOCHNAGE)
      element->param.dy = param->dy;

   element->state = ELEMENT_STATE_ACTIVE;
   update->state = ELEMENT_UPDATE_STATE_UPDATING;
   INIT_COMPLETION(update->complete); /* incomplete */

   mutex_unlock(&element->lock);
  
   /* add the update request to the list */
   mutex_lock(&uctrl->list_lock);
   list_add_tail(&update->list, &uctrl->list);
   uctrl->fifo_cnt++;
   uctrl->tot_cnt++;
   wake_up(&uctrl->start);
   mutex_unlock(&uctrl->list_lock);

   /* block wait until update request is processed */
   wait_for_completion(&update->complete);
   update->state = ELEMENT_UPDATE_STATE_IDLE;

   return 0;
}


EXPORT_SYMBOL(gvr_open);
EXPORT_SYMBOL(gvr_close);
EXPORT_SYMBOL(gvr_lcd_info_get);
EXPORT_SYMBOL(gvr_lcd_panel_set);
EXPORT_SYMBOL(gvr_element_add);
EXPORT_SYMBOL(gvr_element_remove);
EXPORT_SYMBOL(gvr_element_mod);
EXPORT_SYMBOL(gvr_element_get);
EXPORT_SYMBOL(gvr_element_set_master);
EXPORT_SYMBOL(gvr_element_get_master);
EXPORT_SYMBOL(gvr_element_update);
