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

#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/suspend.h>
#include <linux/freezer.h>

#include <linux/semaphore.h>
#include <asm/uaccess.h>

#include <mach/csp/chipcHw_inline.h>
#include <csp/intcHw.h>
#include <csp/gev3Hw.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/mmdma.h>
#include <linux/broadcom/bcmring_ge.h>
#include <mach/csp/cap.h>

/*
 * The current GE does not support copy area in address-decrementing
 * mode. Therefore, in some cases of the copy area, when src and dst overlap
 * with each other, and the bottom-right corner of the src falls into the dst
 * region, data overwrite will occur.
 *
 * The workaround covers those cases and uses a tmp buffer to store the
 * result of the copied area, and move it back to the buffer where the copyarea
 * was originally designated
 */
#define GE_COPYAREA_WORKAROUND

#define MAX_BYTES_PER_PIXEL  4

/* maximum buffer size in bytes */
#define MAX_BUF_SIZE         (GE_MAX_WIDTH * GE_MAX_HEIGHT * \
                              MAX_BYTES_PER_PIXEL)

/* maximum number of queues, need to be a power of 2 */
#define MAX_NUM_QUEUES       64

#define IRQ_ENABLE           1
#define IRQ_DISABLE          0
#define IRQ_CTRL             IRQ_ENABLE

#define assert(expr) \
   if (!(expr)) { \
      printk("Assertion failed! %s,%s,%s,line=%d\n", \
             #expr,__FILE__,__func__,__LINE__); \
   }

/*
 * Mod of the standard wait_event_freezable so when it is interrupted by
 * signals it will wait again
 */
#define wait_event_freezable_only(wq, condition)                 \
({                                                               \
   int rval = 0;                                                 \
   int sig_pended = 0;                                           \
   unsigned long flags;                                          \
   struct task_struct *task = current;                           \
                                                                 \
   while (1) {                                                   \
      rval = wait_event_interruptible(wq,                        \
         (condition) || freezing(current));                      \
      if (freezing(current)) {                                   \
         try_to_freeze();                                        \
      } else if (!rval) {                                        \
         break;                                                  \
      } else {                                                   \
         spin_lock_irqsave(&task->sighand->siglock, flags);      \
         if (test_tsk_thread_flag(task, TIF_SIGPENDING)) {       \
            clear_tsk_thread_flag(task, TIF_SIGPENDING);         \
            sig_pended = 1;                                      \
         }                                                       \
         spin_unlock_irqrestore(&task->sighand->siglock, flags); \
      }                                                          \
   }                                                             \
                                                                 \
   if (sig_pended) {                                             \
      spin_lock_irqsave(&task->sighand->siglock, flags);         \
      set_tsk_thread_flag(task, TIF_SIGPENDING);                 \
      spin_unlock_irqrestore(&task->sighand->siglock, flags);    \
   }                                                             \
})

#define MAX_PROC_BUF_SIZE       256
#define PROC_PARENT_DIR         "ge"
#define PROC_ENTRY_DBG_PRINT    "dbgPrint"

/*
 * Current queue status
 */
enum q_list_status {
   QLIST_RUNNING,
   QLIST_STOPPED
};

/*
 * Queue entry
 */
struct q_entry {
   int handle; /* hanlde needs to be >= 0 */
   volatile int *resp; /* indicate if GE operation succeeds */
   volatile int status; /* current queue status */
   wait_queue_head_t wait_q; /* wait queue for synchronous calls */
   struct work_struct work;
   struct ge_param param; /* GE parameters */
};

/*
 * List of queues. Each GE operation request is queued and processed in a
 * first come first serve order
 */
struct q_list {
   struct semaphore lock; /* lock for queue list */
   volatile int status; /* indicate queue list is running/stopped */
   volatile unsigned int head; /* where to add a new queue */
   volatile unsigned int tail; /* where to remove a processed queue */
   volatile unsigned int q_cnt; /* current queue count */
   struct workqueue_struct *wq; /* work queue that runs in a kernel thread */
   struct q_entry queue[MAX_NUM_QUEUES];
};

/*
 * GE configurations
 */
struct ge_cfg {
   atomic_t is_probed;
   struct semaphore lock; /* master lock to protect the GE operation */
   struct completion complete; /* to signal the completion of GE operation */
#ifdef GE_COPYAREA_WORKAROUND
   struct semaphore copy_lock;
   struct ge_buf copy_buffer;
#endif
   struct notifier_block pm_notifier;
};

struct proc_dir {
   struct proc_dir_entry *parent_dir;
};

static struct proc_dir gProc;
static atomic_t gDbgPrint;
static struct q_list gQlist;
static struct ge_cfg gGE;

#if CONFIG_SYSFS
static struct class * ge_class;
static struct device * ge_dev;
#endif

/*
 * Bytes per pixel lookup table. Only valid for RGB color formats
 */
static const int bytes_per_pixel[GE_COLOR_MAX] = {4, 4, 3, 2, -1, -1, -1};

/* 
 * Mappings for GE CSP color formats
 */
static const int gev3_color_format[GE_COLOR_MAX] =
{
   GEV3HW_COLOR_URGB888,
   GEV3HW_COLOR_ARGB888,
   GEV3HW_COLOR_PRGB888,
   GEV3HW_COLOR_RGB565,
   GEV3HW_COLOR_YUV422I,
   GEV3HW_COLOR_YUV420P,
   GEV3HW_COLOR_YUV420S
};

/*
 * Mappings for GE CSP raster operations
 */
static const int gev3_raster_op[GE_RASTER_MAX] =
{
   GEV3HW_RASTER_BLACK,
   GEV3HW_RASTER_NS_AND_ND,
   GEV3HW_RASTER_NS_AND_D,
   GEV3HW_RASTER_NS,
   GEV3HW_RASTER_S_AND_ND,
   GEV3HW_RASTER_ND,
   GEV3HW_RASTER_S_XOR_D,
   GEV3HW_RASTER_NS_OR_ND,
   GEV3HW_RASTER_S_AND_D,
   GEV3HW_RASTER_S_XNOR_D,
   GEV3HW_RASTER_D,
   GEV3HW_RASTER_NS_OR_D,
   GEV3HW_RASTER_S,
   GEV3HW_RASTER_S_OR_ND,
   GEV3HW_RASTER_S_OR_D,
   GEV3HW_RASTER_WHITE,
};

/*
 * Mappings for GE CSP alpha channel operations
 */
static const int gev3_alpha_ch[GE_DST_ALPHA_MAX] = 
{
   GEV3HW_ALPHA_ROPOUT,
   GEV3HW_ALPHA_UNTOUCH,
   GEV3HW_ALPHA_GLOBAL,
   GEV3HW_ALPHA_WEIGHTED1,
   GEV3HW_ALPHA_WEIGHTED2
};

/*
 * Mappings for GE CSP endianness
 */
static const int gev3_endian[GE_ENDIAN_MAX] = 
{
   GEV3HW_LITTLE_ENDIN,
   GEV3HW_BIG_ENDIN,
   GEV3HW_WINCE_ENDIN
};

/*
 * IRQ hanlder. This routine is invoked when a GE operation completes
 */
static irqreturn_t ge_isr(int irq, void *data)
{
   struct ge_cfg *ge = (struct ge_cfg *)data;

   gev3Hw_clearInterrupt();

   /* signals the completion */
   complete(&ge->complete);

   return IRQ_HANDLED;
}

#ifdef GE_COPYAREA_WORKAROUND
/*
 * Check the source and destination coordinates to see if data overwrite can
 * occur with copyarea. If yes return 1 if no return 0
 */
static inline int copy_area_check_cord(uint32_t sx, uint32_t sy, uint32_t dx,
      uint32_t dy, uint32_t width, uint32_t height)
{
   uint32_t br_x, br_y; /* bottom-right corner of the source area */
   uint32_t left, right, top, bottom;  /* destination area */

   br_x = sx + width - 1;
   br_y = sy + height - 1;

   left = dx;
   right = dx + width - 1;
   top = dy;
   bottom = dy + height - 1;

   /*
    * If the bottom-right corner of the source falls into the destination
    * area, return 1 (data overwritten will occur), else return 0
    */
   if (br_x >= left && br_x <= right && br_y >= top && br_y <= bottom)
      return 1;
   else
      return 0;
}
#endif

/*
 * Perform alpha blending and block wait until it finishes
 */
static int alpha_blend(struct ge_param *param)
{
   int rval = -EFAULT;
   struct ge_cfg *ge = &gGE;
   geHw_ALPHABLENDING_PAR_t csp_param;

   GE_LOG("tstart [GE alpha_blend]\n");

   /* parameters validation for YUV format */
   if (param->src1_format > GE_COLOR_RRGB565 ||
         param->src2_dst_format > GE_COLOR_RRGB565) {
      /* both SRC and DST formats have to match in YUV */
      if (param->src1_format != param->src2_dst_format) {
         rval = -EINVAL;
         goto alpha_blend_exit;
      }

      /* YUV does not support X/Y offset */
      if (param->s1x || param->s1y || param->s2x || param->s2y || param->dx || param->dy) {
         rval = -EINVAL;
         goto alpha_blend_exit;
      }
   }

   if (param->src1_format == GE_COLOR_PRGB888 &&
         param->src2_dst_format == GE_COLOR_RRGB565 &&
         param->width == 0x11) {
      rval = -EINVAL;
      goto alpha_blend_exit;
   }

   if (param->option.alpha.dst_alpha_ch >= GE_DST_ALPHA_MAX) {
      rval = -EINVAL;
      goto alpha_blend_exit;
   }

   if (param->src1_endian >= GE_ENDIAN_MAX ||
         param->src2_dst_endian >= GE_ENDIAN_MAX) {
      rval = -EINVAL;
      goto alpha_blend_exit;
   }

   memset(&csp_param, 0, sizeof(csp_param));

   csp_param.srcformat = gev3_color_format[param->src1_format];
   csp_param.dstformat = gev3_color_format[param->src2_dst_format];

   csp_param.src_endianess = gev3_endian[param->src1_endian];
   csp_param.dst_endianess = gev3_endian[param->src2_dst_endian];

   csp_param.color_fill_en = param->option.alpha.color_fill_enable;  
   csp_param.color_key = param->option.alpha.color;
   
   /*
    * In RGB formats, picth = pixels per line. In YUV formats, pitch = bytes
    * per line
    */
   csp_param.src1Pitch = param->src1_pitch;
   csp_param.dstPitch = param->src2_dst_pitch;

   /*
    * In RGB formats, width/height are in pixels. In YUV formats, width/height
    * are in bytes
    */
   csp_param.width = param->width;
   csp_param.height = param->height;

   /*
    * If the source format is ARGB888, then the global alpha value has NO
    * effect
    */
   csp_param.use_global_alpha = param->option.alpha.use_global_alpha;
   csp_param.alpha = param->option.alpha.global_alpha;
   csp_param.dither_en = param->option.alpha.dither_enable;
   csp_param.round_en = param->option.alpha.round_enable;
   csp_param.src1_premultiplied = param->option.alpha.src1_premult;
   csp_param.src2_premultiplied = param->option.alpha.src2_premult;
   csp_param.dst_alpha_option = gev3_alpha_ch[param->option.alpha.dst_alpha_ch];

   csp_param.src1Addr = (void *)(param->src1_addr +
      (param->s1y * csp_param.src1Pitch + param->s1x) *
      bytes_per_pixel[param->src1_format]);

   csp_param.src2Addr = (void *)(param->src2_addr +
      (param->s2y * csp_param.dstPitch + param->s2x) *
      bytes_per_pixel[param->src2_dst_format]);

   csp_param.dstAddr = (void *)(param->dst_addr +
      (param->dy * csp_param.dstPitch + param->dx) *
      bytes_per_pixel[param->src2_dst_format]);

   csp_param.src1_uAddr = (void *)param->src1_uaddr;
   csp_param.src1_vAddr = (void *)param->src1_vaddr;   
   csp_param.src2_uAddr = (void *)param->src2_uaddr;   
   csp_param.src2_vAddr = (void *)param->src2_vaddr; 
   csp_param.dst_uAddr = (void *)param->dst_uaddr;   
   csp_param.dst_vAddr = (void *)param->dst_vaddr;

   /* mark as incomplete before GE engages */
   INIT_COMPLETION(ge->complete);

   /* engage GE for alpha blending */
   gev3Hw_PowerOn(1);
   
   /* need to reset */
   gev3Hw_Reset(1);
   gev3Hw_Reset(0);

   rval = gev3Hw_alphablend(&csp_param, IRQ_CTRL);
   if (rval != 0) {
      printk(KERN_ERR "GE: gev3Hw_alphablend failed\n");
      goto alpha_blend_exit;
   }

   /* block wait until GE finishes its operation */
   wait_for_completion(&ge->complete);

alpha_blend_exit:
   gev3Hw_PowerOn(0);
   GE_LOG("tstop [GE alpha_blend]\n");
   return rval;
}

/*
 * Perform color fill and block wait until it finishes
 */
static int fill_color(struct ge_param *param)
{
   int rval = -EFAULT;
   struct ge_cfg *ge = &gGE;
   geHw_COLORFILLING_PAR_t csp_param;

   GE_LOG("tstart [GE fill_color]\n");

   /* YUV format not supported */
   if (param->src2_dst_format > GE_COLOR_RRGB565) {
      rval = -EINVAL;
      goto fill_color_exit;
   }

   if (param->src2_dst_endian >= GE_ENDIAN_MAX) {
      rval = -EINVAL;
      goto fill_color_exit;
   }

   memset(&csp_param, 0, sizeof(csp_param));
   csp_param.color_format = gev3_color_format[param->src2_dst_format];
   csp_param.dst_endianess = gev3_endian[param->src2_dst_endian];
   csp_param.color_key = param->option.color;
   csp_param.width = param->width;
   csp_param.height = param->height;

   /* GE uses pitch as pixels per line */
   csp_param.dstPitch = param->src2_dst_pitch;

   csp_param.dstAddr = (void *)(param->dst_addr +
      (param->dy * csp_param.dstPitch + param->dx) *
      bytes_per_pixel[param->src2_dst_format]);

   /* mark as incomplete before GE engages */
   INIT_COMPLETION(ge->complete);

   /* engage GE for fill color */
   gev3Hw_PowerOn(1);

   /* need to reset */
   gev3Hw_Reset(1);
   gev3Hw_Reset(0);
   
   rval = gev3Hw_colorFill(&csp_param, IRQ_CTRL);
   if (rval != 0) {
      printk(KERN_ERR "GE: gev3Hw_colorFill failed\n");
      goto fill_color_exit;
   }

   /* block wait until GE finishes its operation */
   wait_for_completion(&ge->complete);

fill_color_exit:
   gev3Hw_PowerOn(0);
   GE_LOG("tstop [GE fill_color]\n");
   return rval;
}

/*
 * Perform a raster operation and block wait until it finishes
 */
static int raster_op(struct ge_param *param)
{
   int rval = -EFAULT;
   struct ge_cfg *ge = &gGE;
   geHw_ROP_TRANSPARENT_PAR_t csp_param;
#ifdef GE_COPYAREA_WORKAROUND
   /* determine if a copyarea SW workaround is needed */
   int workaround_is_needed;
#endif

   GE_LOG("tstart [GE raster_op]\n");

   /* YUV format not supported */
   if (param->src1_format > GE_COLOR_RRGB565 ||
         param->src2_dst_format > GE_COLOR_RRGB565) {
      rval = -EINVAL;
      goto raster_op_exit;
   }

   if (param->src1_format == GE_COLOR_PRGB888 &&
         param->src2_dst_format == GE_COLOR_RRGB565 &&
         param->width == 0x11) {
      rval = -EINVAL;
      goto raster_op_exit;
   }

   if (param->option.raster.op >= GE_RASTER_MAX) {
      rval = -EINVAL;
      goto raster_op_exit;
   }

   if (param->src1_endian >= GE_ENDIAN_MAX ||
         param->src2_dst_endian >= GE_ENDIAN_MAX) {
      rval = -EINVAL;
      goto raster_op_exit;
   }

#ifdef GE_COPYAREA_WORKAROUND
   /* only cover cases that involve just one source */
   if (param->option.raster.op != GE_RASTER_S1 &&
         param->option.raster.op != GE_RASTER_NS1) {
      workaround_is_needed = 0;
   } else if (param->src1_addr != param->dst_addr) {
      /*
       * If copyarea is performed on two different bitmaps, workaround is not
       * needed since data overwrite will not occur
       */
      workaround_is_needed = 0;
   } else {
      /*
       * copyarea is performed on the same bitmap, check coordinates to see
       * if SW workaround is needed
       */
      workaround_is_needed = copy_area_check_cord(param->s1x, param->s1y,
         param->dx, param->dy, param->width, param->height);
   }
#endif

   memset(&csp_param, 0, sizeof(csp_param));

   csp_param.srcformat = gev3_color_format[param->src1_format];
   csp_param.dstformat = gev3_color_format[param->src2_dst_format];

   csp_param.src_endianess = gev3_endian[param->src1_endian];
   csp_param.dst_endianess = gev3_endian[param->src2_dst_endian];

   csp_param.rop = gev3_raster_op[param->option.raster.op];

   csp_param.color_fill_en = param->option.raster.color_fill_enable;
   csp_param.color_key = param->option.raster.color;

   csp_param.width = param->width;
   csp_param.height = param->height;

   /* GE uses pitch as pixels per line */
   csp_param.src1Pitch = param->src1_pitch;
   csp_param.dstPitch = param->src2_dst_pitch;

   csp_param.dst_alpha_option = param->option.raster.dst_alpha_ch;
   csp_param.use_global_alpha = param->option.raster.use_global_alpha;
   csp_param.alpha = param->option.raster.global_alpha;
   csp_param.dither_en = param->option.raster.dither_enable;
   csp_param.round_en = param->option.raster.round_enable;

   csp_param.transparent_en = param->option.raster.trans_enable;
   csp_param.transparent_alpha_en = param->option.raster.trans_alpha_ch_enable;

   csp_param.src1Addr = (void *) param->src1_addr +
      (param->s1y * csp_param.src1Pitch + param->s1x) *
      bytes_per_pixel[param->src1_format];
   csp_param.src2Addr = (void *) param->src2_addr +
      (param->s2y * csp_param.dstPitch + param->s2x) *
      bytes_per_pixel[param->src2_dst_format];
   csp_param.dstAddr = (void *) param->dst_addr +
      (param->dy * csp_param.dstPitch + param->dx) *
      bytes_per_pixel[param->src2_dst_format];

#ifdef GE_COPYAREA_WORKAROUND
   /*
    * If a copyarea SW workaround is needed, copy the affected (dirty) rows
    * to a tmp buffer. Then perform GE copyarea from the original source onto
    * the tmp buffer. After GE finishes, copy the affected (dirty) rows back
    * from the tmp buffer to the destination buffer
    *
    * The affected (dirty) rows are: dy ----> (dy + height)
    */
   if (workaround_is_needed) {
      uint32_t start_row_addr, len;

      down(&ge->copy_lock);

      start_row_addr = param->dst_addr + (param->dy * csp_param.dstPitch) *
         bytes_per_pixel[param->src2_dst_format];
      len = param->height * csp_param.dstPitch *
         bytes_per_pixel[param->src2_dst_format];
      mmdma_transfer(start_row_addr, ge->copy_buffer.addr, len);

      /* now the copyarea destination needs to be the tmp buffer */
      csp_param.dstAddr = (void *) ge->copy_buffer.addr + param->dx *
         bytes_per_pixel[param->src2_dst_format];

      up(&ge->copy_lock);
   }
#endif

   /* mark as incomplete before GE engages */
   INIT_COMPLETION(ge->complete);

   /* engage GE */
   gev3Hw_PowerOn(1);

   /* need to reset */
   gev3Hw_Reset(1);
   gev3Hw_Reset(0);
   
   rval = gev3Hw_rasterOperation(&csp_param, IRQ_CTRL);
   if (rval != 0) {
      printk(KERN_ERR "GE: gev3Hw_rasterOperation failed\n");
      goto raster_op_exit;
   }

   /* block wait until GE finishes its operation */
   wait_for_completion(&ge->complete);
   gev3Hw_PowerOn(0);

#ifdef GE_COPYAREA_WORKAROUND
   /*
    * Now GE finishes, copy the affected (dirty) rows back from the tmp buffer
    * to the destination buffer
    */
   if (workaround_is_needed) {
      uint32_t start_row_addr, len;

      down(&ge->copy_lock);

      start_row_addr = param->dst_addr + (param->dy * csp_param.dstPitch) *
         bytes_per_pixel[param->src2_dst_format];
      len = param->height * csp_param.dstPitch *
         bytes_per_pixel[param->src2_dst_format];
      mmdma_transfer(ge->copy_buffer.addr, start_row_addr, len);

      up(&ge->copy_lock);
   }
#endif

raster_op_exit:
   gev3Hw_PowerOn(0);
   GE_LOG("tstop [GE raster_op]\n");
   return rval;
}

/*
 * Kicks off the graphic engine
 */
static int run_ge(struct ge_param *param)
{
   int rval;
   struct ge_cfg *ge = &gGE;

   down(&ge->lock);

   if (atomic_read(&gDbgPrint)) {
      printk(KERN_ERR "GE: mode=%d op=%d ", param->mode, param->operation);
      switch (param->operation) {
         case GE_OP_ALPHA_BLEND:
            printk(KERN_ERR "dst_alpha_ch=%d use_global_alpha=%u global_alpha=%u dither_enable=%u round_enable=%u src1_premult=%u src2_premult=%u color_fill_enable=%u color=%u ", param->option.alpha.dst_alpha_ch, param->option.alpha.use_global_alpha, (unsigned int)param->option.alpha.global_alpha, param->option.alpha.dither_enable, param->option.alpha.round_enable, param->option.alpha.src1_premult, param->option.alpha.src2_premult, param->option.alpha.color_fill_enable, param->option.alpha.color);
            break;

         case GE_OP_RASTER:
            printk(KERN_ERR "ge_raster_op=%d color_fill_enable=%u trans_enable=%u trans_alpha_ch_enable=%u color=%u dst_alpha_ch=%d use_global_alpha=%u global_alpha=%u dither_enable=%u round_enable=%u ", param->option.raster.op, param->option.raster.color_fill_enable, param->option.raster.trans_enable, param->option.raster.trans_alpha_ch_enable, param->option.raster.color, param->option.raster.dst_alpha_ch, param->option.raster.use_global_alpha, param->option.raster.global_alpha, param->option.raster.dither_enable, param->option.raster.round_enable);
            break;
            
         case GE_OP_FILL_COLOR:
            printk(KERN_ERR "color=%u ", param->option.color);
            break;

         case GE_OP_NOP:
            break;

         default:
            printk(KERN_ERR "unknown" );
      }

      printk(KERN_ERR "src1_format=%d src2_dst_format=%d src1_endian=%d src2_dst_endian=%d s1x=%u s1y=%u s2x=%u s2y=%u dx=%u dy=%u width=%u height=%u src1_pitch=%u src2_dst_pitch=%u src1_addr=0x%x src2_addr=0x%x dst_addr=0x%x", param->src1_format, param->src2_dst_format, param->src1_endian, param->src2_dst_endian, param->s1x, param->s1y, param->s2x, param->s2y, param->dx, param->dy, param->width, param->height, param->src1_pitch, param->src2_dst_pitch, param->src1_addr, param->src2_addr, param->dst_addr);

      printk(KERN_ERR "src1_uaddr=%u src1_vaddr=%u src2_uaddr=%u src2_vaddr=%u dst_uaddr=%u dst_vaddr=%u\n\n", param->src1_uaddr, param->src1_vaddr, param->src2_uaddr, param->src2_vaddr, param->dst_uaddr, param->dst_vaddr);
   }

   GE_LOG("tstart [GE run_ge] param->operation=%d\n", param->operation);

   switch (param->operation) {
      case GE_OP_ALPHA_BLEND: /* the name is quite obvious */
         rval = alpha_blend(param);
         break;

      case GE_OP_RASTER: /* BitBlt using the raster operator */
         rval = raster_op(param);
         break;

      case GE_OP_FILL_COLOR: /* fill a beautiful color */
         rval = fill_color(param);
         break;

      case GE_OP_NOP:
         rval = 0;
         break;

      default:
         rval = -EINVAL;
         break;
   }

   GE_LOG("tstop [GE run_ge]\n");

   up(&ge->lock);
   return rval;
}

/*
 * Process work that is scheduled in q_schedule. The graphic engine will be
 * engaged
 */
static void q_process(struct work_struct *work)
{
   int rval;
   struct q_list *list = &gQlist;
   struct q_entry *q = container_of(work, struct q_entry, work);

   /* change queue status to RUNNING */
   down(&list->lock);
   assert(list->tail == q->handle);
   q->status = GE_STATUS_RUNNING;
   up(&list->lock);

   /* now run the graphic engine, this call may block */
   rval = run_ge(&q->param);

   down(&list->lock);
   
   /* save return error value in sync mode */
   if (rval < 0 && q->resp && q->param.mode == GE_MODE_SYNC)
      *(q->resp) = rval;

   /* now queue is done, update the list */
   q->status = GE_STATUS_DONE;
   list->tail = (list->tail + 1) & (MAX_NUM_QUEUES - 1);
   list->q_cnt--;

   /* sync mode, signal and wake up the wait queue */
   if (q->param.mode == GE_MODE_SYNC) {
      wake_up(&q->wait_q);
   }
   
   up(&list->lock);
}

/*
 * Add a new work into the queue list and schedule it to run. On an async
 * call, return immediately after it's scheduled. On a sync call, wait until
 * the work is done
 */
static int q_schedule(struct q_list *list, struct ge_param *param)
{
   volatile int rval;
   struct q_entry *q;

   /* invalid arguments */
   if (!list || !param)
      return -EINVAL;

   down(&list->lock);

   /* queue has been stopped, no new work should be scheduled */
   if (list->status == QLIST_STOPPED) {
      up(&list->lock);
      return -EFAULT;
   }

   /* queue is full */
   if (list->q_cnt >= MAX_NUM_QUEUES) {
      up(&list->lock);
      return -EAGAIN;
   }

	/* The q_schedule calls are reentrant on parallel decodes so tstart/tstop doesn't work here */
   /* GE_LOG("tstart [GE q_schedule]\n"); */

   /* locate queue, and change its status to PENDING */
   assert(list->head < MAX_NUM_QUEUES);
   q = &list->queue[list->head];
   assert(q->status == GE_STATUS_DONE);
   q->status = GE_STATUS_PENDING;
   rval = q->handle;
   
   if (param->mode == GE_MODE_SYNC) {
      q->resp = &rval;
   }

   memcpy(&q->param, param, sizeof(q->param));

   /* schedule queue to run */
   queue_work(list->wq, &q->work);

   /* update queue list info */
   list->head = (list->head + 1) & (MAX_NUM_QUEUES - 1);
   list->q_cnt++;
   up(&list->lock);

   /* sync mode, set up wait queue and wait for task to finish */
   if (param->mode == GE_MODE_SYNC) {
      wait_event_freezable_only(q->wait_q, (q->status == GE_STATUS_DONE));
   }

   /* GE_LOG("tstop [GE q_schedule]\n"); */

   return 0;
}

static int q_status_get(struct q_list *list, int handle)
{
   if (handle < 0 || handle >= MAX_NUM_QUEUES || !list)
      return -EINVAL;

   return list->queue[handle].status;
}

/*
 * Initialze GE queues
 */
static int q_init(struct q_list *list)
{
   unsigned int i;
   struct q_entry *q;

   if (!list)
      return -EINVAL;
   
   init_MUTEX(&list->lock); /* unlocked */

   list->status = QLIST_RUNNING;

   list->head = 0;
   list->tail = 0;
   list->q_cnt = 0;

   /* use a dedicated thread */
   list->wq = create_singlethread_workqueue("GE_THREAD");
   if (list->wq == NULL)
      return -EBUSY;

   for (i = 0; i < MAX_NUM_QUEUES; i++) {
      q = &list->queue[i];
      q->handle = i;
      q->resp = NULL;
      q->status = GE_STATUS_DONE;
      INIT_WORK(&q->work, q_process);
      init_waitqueue_head(&q->wait_q);
   }

   return 0;
}

/*
 * Terminate GE queues
 */
static void q_term(struct q_list *list)
{
   if (!list)
      return;

   down(&list->lock);
   list->status = QLIST_STOPPED;
   up(&list->lock);

   destroy_workqueue(list->wq);
}

static int ge_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
      unsigned long arg)
{
   int rval;

   switch (cmd) {
      case GE_IOCTL_INFO_GET:
      {
         struct ge_info info;

         memset(&info, 0, sizeof(info));

         info.version = GE_VERSION_3;

         if (copy_to_user((void *)arg, &info,
                  sizeof(info)) != 0) {
            printk(KERN_ERR "GE: copy_to_user failed for "
                  "GE_IOCTL_INFO_GET\n");
            return -EFAULT;
         }
      }
      break;

      case GE_IOCTL_ENGAGE:
      {
         struct ge_param param;

         if (copy_from_user(&param, (struct ge_param *)arg,
                  sizeof(struct ge_param)) != 0) {
            printk(KERN_ERR "GE: copy_from_user failed for GE_IOCTL_ENGAGE\n");
            return -EFAULT;
         }

         rval = ge_engage(&param);
         if (rval < 0 && rval != -EAGAIN)
            printk(KERN_ERR "GE: ge_engage failed\n");

         return rval;
      }

      case GE_IOCTL_STATUS_GET:
      {
         struct q_list *list = &gQlist;

         rval = q_status_get(list, arg);
         if (rval < 0)
            printk(KERN_ERR "GE: q_status_get failed, handle=%d\n", (int)arg);

         return rval;
      }

      default:
         return -EINVAL;
   }
   return 0;
}

static struct file_operations ge_fops =
{
   owner: THIS_MODULE,
   ioctl: ge_ioctl,
};

static int ge_pm_callback(struct notifier_block *nfb, unsigned long action,
      void *ignored)
{
   struct ge_cfg *ge = &gGE;

   switch (action) {
      case PM_HIBERNATION_PREPARE:
      case PM_SUSPEND_PREPARE:
         down(&ge->lock);
         return NOTIFY_OK;

      case PM_POST_HIBERNATION:
      case PM_POST_SUSPEND:
         up(&ge->lock);
         return NOTIFY_OK;
      }
   
   return NOTIFY_DONE;
}

static int
proc_dbg_print_write(struct file *file, const char __user *buffer,
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
            PROC_PARENT_DIR, PROC_ENTRY_DBG_PRINT);
      return count;
   }

   if (enable)
      atomic_set(&gDbgPrint, 1);
   else
      atomic_set(&gDbgPrint, 0);

   printk(KERN_INFO "Debugging print in the GE driver is now %s\n",
         atomic_read(&gDbgPrint) ? "ENABLED" : "DISABLED");

   return count;
}

static int
proc_dbg_print_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Debugging print in the GE driver is %s\n",
         atomic_read(&gDbgPrint) ? "ENABLED" : "DISABLED");

   return len;
}

static int proc_init(void)
{
   int rc;
   struct proc_dir_entry *proc_dbg_print;

   gProc.parent_dir = proc_mkdir(PROC_PARENT_DIR, NULL);

   proc_dbg_print = create_proc_entry(PROC_ENTRY_DBG_PRINT, 0644, gProc.parent_dir);
   if (proc_dbg_print == NULL) {
      rc = -ENOMEM;
      goto proc_exit;
   }
   proc_dbg_print->read_proc = proc_dbg_print_read;
   proc_dbg_print->write_proc = proc_dbg_print_write;
   proc_dbg_print->data = NULL;

   return 0;

proc_exit:
   remove_proc_entry(PROC_PARENT_DIR, NULL);
   return rc;
}

static void proc_term(void)
{
   remove_proc_entry(PROC_ENTRY_DBG_PRINT, gProc.parent_dir);
   remove_proc_entry(PROC_PARENT_DIR, NULL);
}

static int ge_probe(struct platform_device *pdev)
{
   int rval = -EFAULT;
   struct ge_cfg *ge = &gGE;
   struct q_list *list = &gQlist;

   atomic_set(&ge->is_probed, 0);

   if (cap_isPresent(CAP_GE, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "GE: Not supported\n");
      return -ENODEV;
   }

   init_MUTEX(&ge->lock); /* unlocked */
   init_completion(&ge->complete); /* incomplete */

#ifdef GE_COPYAREA_WORKAROUND
   init_MUTEX(&ge->copy_lock); /* unlocked */
   ge->copy_buffer.ptr = dma_alloc_writecombine(NULL, MAX_BUF_SIZE,
			&ge->copy_buffer.addr, GFP_KERNEL);
	if (!ge->copy_buffer.ptr) {
		printk(KERN_ERR "GE: Unable to allocate memory for the copy "
            "buffer\n");
      rval = -ENOMEM;
      goto err_exit;
	}
	ge->copy_buffer.len = MAX_BUF_SIZE;
#endif

   if (request_irq(IRQ_GE, ge_isr, IRQF_SHARED, "GE", ge) < 0) {
      printk(KERN_ERR "GE: request_irq %u failed\n", IRQ_GE);
      goto err_free_buf;
   }

   if (q_init(list) != 0) {
      printk(KERN_ERR "GE: Queue list init failed\n");
      goto err_free_irq;
   }

   ge->pm_notifier.notifier_call = &ge_pm_callback;
   ge->pm_notifier.priority = 99;
   rval = register_pm_notifier(&ge->pm_notifier);
   if (rval < 0) {
      printk(KERN_ERR "GE: register_pm_notifier failed\n");
      goto err_term_qlist;
   }

   if (gev3Hw_Init(1) < 0) {
      printk(KERN_ERR "GE: gev3Hw_Init failed\n");
      goto err_unregister_pm_notifier;
   }

   atomic_set(&gDbgPrint, 0);
   if (proc_init() != 0) {
      printk(KERN_ERR "GE: proc_init failed\n");
      goto err_ge_exit;
   }

   if (register_chrdev(BCM_GE_MAJOR, "ge", &ge_fops) < 0) {
      printk(KERN_ERR "GE: register_chrdev failed for major %u\n",
            BCM_GE_MAJOR);
      goto err_proc_term;
   }

#if CONFIG_SYSFS
   ge_class = class_create(THIS_MODULE,"bcmring-ge");
   if(IS_ERR(ge_class)){
	   printk(KERN_ERR "GE: Class create failed\n");
	   rval = -EFAULT;
	   goto err_unregister_chrdev;
   }
   
   ge_dev = device_create(ge_class, NULL, MKDEV(BCM_GE_MAJOR,0),NULL,"ge");
   if(IS_ERR(ge_dev)){
	   printk(KERN_ERR "GE: Device create failed\n");
	   rval = -EFAULT;
	   goto err_class_destroy;
   }
#endif
   
   printk(KERN_NOTICE "GE: Driver initialized\n");

   platform_set_drvdata(pdev, ge);
   atomic_set(&ge->is_probed, 1);

   return 0;
   
#if CONFIG_SYSFS
err_class_destroy:
   class_destroy(ge_class);
err_unregister_chrdev:
   unregister_chrdev(BCM_GE_MAJOR, "ge");   
#endif
err_proc_term:
   proc_term();
err_ge_exit:
   gev3Hw_Exit();
err_unregister_pm_notifier:
   unregister_pm_notifier(&ge->pm_notifier);
err_term_qlist:
   q_term(list);
err_free_irq:
   free_irq(IRQ_GE, ge);
err_free_buf:
#ifdef GE_COPYAREA_WORKAROUND
   dma_free_writecombine(NULL, ge->copy_buffer.len, ge->copy_buffer.ptr,
         ge->copy_buffer.addr);
   ge->copy_buffer.ptr = NULL;
   ge->copy_buffer.addr = 0;
#endif
err_exit:
   return rval;
}

static int ge_remove(struct platform_device *pdev)
{
   struct ge_cfg *ge = platform_get_drvdata(pdev);
   struct q_list *list = &gQlist;

#if CONFIG_SYSFS
   device_destroy(ge_class,MKDEV(BCM_GE_MAJOR,0));
   class_destroy(ge_class);
#endif

   atomic_set(&ge->is_probed, 0);
   unregister_chrdev(BCM_GE_MAJOR, "ge");
   proc_term();
   gev3Hw_Exit();
   unregister_pm_notifier(&ge->pm_notifier);
   q_term(list);

   free_irq(IRQ_GE, ge);
#ifdef GE_COPYAREA_WORKAROUND
   if (ge->copy_buffer.ptr != NULL) {
      dma_free_writecombine(NULL, ge->copy_buffer.len, ge->copy_buffer.ptr,
            ge->copy_buffer.addr);
      ge->copy_buffer.ptr = NULL;
      ge->copy_buffer.addr = 0;
   }
#endif
   return 0;
}

static struct platform_driver ge_driver = {
   .driver = {
      .name = "bcmring-ge",
      .owner = THIS_MODULE,
   },
   .probe = ge_probe,
   .remove = ge_remove,
};

static int __init ge_init(void)
{
   return platform_driver_register(&ge_driver);
}

static void __exit ge_exit(void)
{
   platform_driver_unregister(&ge_driver);
}

module_init(ge_init);
module_exit(ge_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Graphic Engine (GE) Driver");
MODULE_LICENSE("GPL");

int ge_engage(struct ge_param *param)
{
   struct ge_cfg *ge = &gGE;
   struct q_list *list = &gQlist;

   if (!atomic_read(&ge->is_probed))
      return -ENODEV;

   return q_schedule(list, param);
}
EXPORT_SYMBOL(ge_engage);
