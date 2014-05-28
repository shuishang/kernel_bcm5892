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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/completion.h>
#include <mach/dma.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/broadcom/mmdma.h>
#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/bootmemheap.h>
#include <linux/broadcom/knllog.h>

#define MAX_PROC_BUF_SIZE     256
#define PROC_PARENT_DIR       "mmdma"
#define PROC_ENTRY_GENERAL    "general"
#define PROC_ENTRY_SNAPSHOT   "snapshot"
#define PROC_ENTRY_HIST       "history"

struct buf_info {
   /* flag to indicate whether the this buffer is in use or not */
   volatile unsigned int in_use;

   /* associated MMDMA handle acquired when an MMDMA instance is opened */
   MMDMA_HDL hdl;

   /* maximum possible length (bytes) of the buffer */
   unsigned int max_len;

   /* core buffer data structure */
   struct mmdma_buf buf;

   /* counter that tracks number of times this buffer has been allocated */
   unsigned int alloc_cnt;

   /* counter that tracks number of times this buffer has been freed */
   unsigned int free_cnt;
};

/*
 * MMDMA configurations
 */
struct dma_cfg {
   DMA_Handle_t handle;

   struct mmdma_hw_cfg hwcfg;

   void *buf_ptr;
   uint32_t buf_addr;
   unsigned int buf_len;

   /* lock for DMA data transfer */
   struct mutex transfer_lock;
   
   /* DMA data transfer completion */
   struct completion transfer_complete;

   /* lock for buffer */
   struct mutex buf_lock;

   /* buffer list */
   struct buf_info *buf_list;
};

struct proc_dir {
   struct proc_dir_entry *parent_dir;
};

#if CONFIG_SYSFS
static struct class *dma_class;
static struct device *dma_dev;
#endif

static struct dma_cfg gDMA;
static struct proc_dir gProc;

/*
 * DMA IRQ hanlder. This routine is invoked when DMA completes
 */
static void dma_isr(DMA_Device_t dev, int reason, void *data)
{
   struct dma_cfg *dma = (struct dma_cfg *)data;
   complete(&dma->transfer_complete);
}

int mmdma_transfer(dma_addr_t src_addr, dma_addr_t dst_addr,
      uint32_t len)
{
   struct dma_cfg *dma = &gDMA;

   mutex_lock(&dma->transfer_lock);

   MMDMA_LOG("tstart [MMDMA mmdma_transfer]\n");

   /*
    * If channel is shared, need to reserve the channel everytime before the
    * transfer
    */
   if (dma_device_is_channel_shared(DMA_DEVICE_CLCD_MEM_TO_MEM)) {
      dma->handle = dma_alloc_channel(DMA_DEVICE_CLCD_MEM_TO_MEM);
      if (dma->handle < 0) {
         MMDMA_LOG("tstop [MMDMA mmdma_transfer]\n");
         printk(KERN_ERR "MMDMA: dma_alloc_channel failed\n");
         mutex_unlock(&dma->transfer_lock);
         return -EFAULT;
      }
   }

   /* mark as incomplete before DMA starts */
   INIT_COMPLETION(dma->transfer_complete);

   if (dma_transfer_mem_to_mem(dma->handle, src_addr, dst_addr, len) != 0) {
      printk(KERN_ERR "MMDMA: dma_transfer_mem_to_mem failed\n");
      goto dma_err_complete;
   }

   /* block wait until DMA completes */
   wait_for_completion(&dma->transfer_complete);

   if (dma_device_is_channel_shared(DMA_DEVICE_CLCD_MEM_TO_MEM)) {
      if (dma_free_channel(dma->handle) != 0)
         printk(KERN_ERR "MMDMA: dma_free_channel failed\n");
   }

   MMDMA_LOG("tstop [MMDMA mmdma_transfer]\n");
   mutex_unlock(&dma->transfer_lock);
   return 0;

dma_err_complete:
   complete(&dma->transfer_complete);
   MMDMA_LOG("tstop [MMDMA mmdma_transfer]\n");
   mutex_unlock(&dma->transfer_lock);
   return -EFAULT;
}
EXPORT_SYMBOL(mmdma_transfer);

static int buf_alloc_safe(MMDMA_HDL hdl, struct mmdma_buf *buf,
      unsigned int len)
{
   unsigned int i;
   struct dma_cfg *dma = &gDMA;
   struct buf_info *buf_info;

   if (!dma->hwcfg.tot_mem)
      return -ENOMEM;

   mutex_lock(&dma->buf_lock);

   /* go through the buffer list and find available buffer to use */
   for (i = 0; i < dma->hwcfg.mem_cnt; i++) {
      buf_info = &dma->buf_list[i];
      if (buf_info->max_len >= len && !buf_info->in_use)
         break;
   }

   /* nothing can be found */
   if (i >= dma->hwcfg.mem_cnt) {
      mutex_unlock(&dma->buf_lock);
      return -ENOMEM;
   }

   buf_info->in_use = 1;
   buf_info->hdl = hdl;
   buf_info->alloc_cnt++;
   buf_info->buf.len = len;
   memcpy(buf, &buf_info->buf, sizeof(*buf));

   mutex_unlock(&dma->buf_lock);

   return 0;
}

static int buf_get_safe(MMDMA_HDL hdl, uint32_t bus_addr, struct mmdma_buf *buf)
{
   unsigned int i;
   struct dma_cfg *dma = &gDMA;
   struct buf_info *buf_info;

   if (!dma->hwcfg.tot_mem)
      return -ENOMEM;

   mutex_lock(&dma->buf_lock);

   /* go through the buffer list and find the buffer using the physical address */
   for (i = 0; i < dma->hwcfg.mem_cnt; i++) {
      buf_info = &dma->buf_list[i];
      if (buf_info->buf.bus_addr == bus_addr)
         break;
   }

   /* nothing can be found */
   if (i >= dma->hwcfg.mem_cnt) {
      mutex_unlock(&dma->buf_lock);
      return -EINVAL;
   }

   memcpy(buf, &buf_info->buf, sizeof(*buf));

   mutex_unlock(&dma->buf_lock);

   return 0;
}

static int buf_free_safe(MMDMA_HDL hdl, uint32_t bus_addr)
{
   unsigned int i;
   struct dma_cfg *dma = &gDMA;
   struct buf_info *buf_info;

   mutex_lock(&dma->buf_lock);

   /* go through the buffer list and find the buffer to free */
   for (i = 0; i < dma->hwcfg.mem_cnt; i++) {
      buf_info = &dma->buf_list[i];
      if (bus_addr == buf_info->buf.bus_addr)
         break;
   }

   /* nothing can be found */
   if (i >= dma->hwcfg.mem_cnt) {
      mutex_unlock(&dma->buf_lock);
      return -EINVAL;
   }

   buf_info->in_use = 0;
   buf_info->hdl = 0;
   buf_info->free_cnt++;

   mutex_unlock(&dma->buf_lock);

   return 0;
}

static int mmdma_open(MMDMA_HDL *hdl)
{
   *hdl = (MMDMA_HDL)kmalloc(sizeof(MMDMA_HDL), GFP_KERNEL);
   if (*hdl == 0)
      return -ENOMEM;

   return 0;
}

static int mmdma_close(MMDMA_HDL hdl)
{
   unsigned int i;
   struct dma_cfg *dma = &gDMA;
   struct buf_info *buf_info;

   /* go through the buffer list and delete all buffers with this handle */
   mutex_lock(&dma->buf_lock);
   for (i = 0; i < dma->hwcfg.mem_cnt; i++) {
      buf_info = &dma->buf_list[i];
      if (buf_info->hdl == hdl) {
         mutex_unlock(&dma->buf_lock);
         buf_free_safe(hdl, buf_info->buf.bus_addr);
         mutex_lock(&dma->buf_lock);
      }
   }
   mutex_unlock(&dma->buf_lock);

   kfree((void *)hdl);
   return 0;
}

static int mmdma_dev_open(struct inode *inode, struct file *filp)
{
   return mmdma_open((MMDMA_HDL *)(&filp->private_data));
}

static int mmdma_dev_release(struct inode *inode, struct file *filp)
{
   return mmdma_close((MMDMA_HDL)filp->private_data);
}

static int mmdma_dev_ioctl(struct inode *inode, struct file *filp,
      unsigned int cmd, unsigned long arg)
{
   int rval;
   MMDMA_HDL hdl;

   hdl = (MMDMA_HDL)filp->private_data;

   switch (cmd) {
      case MMDMA_IOCTL_BUF_ALLOC:
      {
         struct mmdma_ioctl_buf_alloc_param ioparam;

         if (copy_from_user(&ioparam, (void *)arg, sizeof(ioparam)) != 0) {
            printk(KERN_ERR "MMDMA: copy_from_user failed for "
                  "ioctl=MMDMA_IOCTL_BUF_ALLOC\n");
            return -EFAULT;
         }

         rval = buf_alloc_safe(hdl, &ioparam.buf, ioparam.buf.len);
         if (rval != 0) {
            printk(KERN_ERR "MMDMA: buf_alloc_safe failed len=%u rc=%d\n",
                  ioparam.buf.len, rval);
            return rval;
         }

         if (copy_to_user((void *)arg, &ioparam, sizeof(ioparam)) != 0) {
            printk(KERN_ERR "MMDMA: copy_to_user failed for "
                  "ioctl=MMDMA_IOCTL_BUF_ALLOC\n");
            return -EFAULT;
         }

         return 0;
      }

      case MMDMA_IOCTL_BUF_GET:
      {
         struct mmdma_ioctl_buf_get_param ioparam;

         if (copy_from_user(&ioparam, (void *)arg, sizeof(ioparam)) != 0) {
            printk(KERN_ERR "MMDMA: copy_from_user failed for "
                  "ioctl=MMDMA_IOCTL_BUF_GET\n");
            return -EFAULT;
         }

         rval = buf_get_safe(hdl, ioparam.buf.bus_addr, &ioparam.buf);
         if (rval != 0) {
            printk(KERN_ERR "MMDMA: buf_get_safe failed rc=%d\n",
                  rval);
            return rval;
         }

         if (copy_to_user((void *)arg, &ioparam, sizeof(ioparam)) != 0) {
            printk(KERN_ERR "MMDMA: copy_to_user failed for "
                  "ioctl=MMDMA_IOCTL_BUF_GET\n");
            return -EFAULT;
         }

         return 0;
      }

      case MMDMA_IOCTL_BUF_FREE:
      {
         struct mmdma_ioctl_buf_free_param ioparam;

         if (copy_from_user(&ioparam, (void *)arg, sizeof(ioparam)) != 0) {
            printk(KERN_ERR "MMDMA: copy_from_user failed for "
                  "ioctl=MMDMA_IOCTL_BUF_FREE\n");
            return -EFAULT;
         }

         rval = buf_free_safe(hdl, ioparam.bus_addr);
         if (rval != 0) {
            printk(KERN_ERR "MMDMA: buf_free failed rc=%d\n", rval);
            return rval;
         }

         return 0;
      }

      default:
         return -EINVAL;
   }
   return 0;
}

/*
 * Map the memory to user space
 */
static int mmdma_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
   struct dma_cfg *dma = &gDMA;
   struct buf_info *buf_info;
   unsigned int i, len;
   unsigned long start = 0, off;

   if (!dma->hwcfg.tot_mem)
      return -ENOMEM;

   if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
      return -EINVAL;

   start = dma->buf_addr & PAGE_MASK;
   len = PAGE_ALIGN((start & ~PAGE_MASK) + dma->buf_len);

   off = vma->vm_pgoff << PAGE_SHIFT;

   for (i = 0; i < dma->hwcfg.mem_cnt; i++) {
      buf_info = &dma->buf_list[i];
      if (buf_info->buf.offset == off)
         break;
   }

   if (i >= dma->hwcfg.mem_cnt) {
      printk(KERN_ERR "MMDMA: mmap failed, cannot locate buffer\n");
      return -EINVAL;
   }

   if ((vma->vm_end - vma->vm_start + off) > len) {
      printk(KERN_ERR "MMDMA: mmap failed, buffer requested is too large\n");
      return -EINVAL;
   }

   off += start;
   vma->vm_pgoff = off >> PAGE_SHIFT;

   vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
   vma->vm_flags |= VM_IO | VM_RESERVED;

   return io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
                                  vma->vm_end - vma->vm_start,
                                  vma->vm_page_prot);
}

struct file_operations mmdma_fops =
{
   owner: THIS_MODULE,
   open: mmdma_dev_open,
   release: mmdma_dev_release,
   ioctl: mmdma_dev_ioctl,
   mmap: mmdma_dev_mmap,
};

static int
proc_general_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   unsigned int len = 0;
   struct dma_cfg *dma = (struct dma_cfg *)data;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Current MMDMA configuration:\n");
   len += sprintf(buffer + len, "Bootmem is %s\n",
         dma->hwcfg.tot_mem ? "RESERVED" : "NOT RESERVED");
   if (dma->hwcfg.tot_mem) {
      len += sprintf(buffer + len, "MMDMA bootmem data=0x%x addr=0x%x len=%u\n",
            (unsigned int)dma->buf_ptr, dma->buf_addr, dma->buf_len);
   }

   return len;
}

static int
proc_snapshot_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   unsigned int i, len = 0;
   struct dma_cfg *dma = (struct dma_cfg *)data;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Current snapshot of MMDMA buffer usage:\n");

   mutex_lock(&dma->buf_lock);
   for (i = 0; i < dma->hwcfg.mem_cnt; i++) {
      struct buf_info *buf_info = &dma->buf_list[i];

      len += sprintf(buffer + len, "[slot %u] %s hdl=0x%x virt=0x%x phys=0x%x len=%u max_len=%u off=0x%x\n",
               i,
               buf_info->in_use ? "in use" : "no use",
               (unsigned int)buf_info->hdl,
               (unsigned int)buf_info->buf.data,
               buf_info->buf.bus_addr,
               buf_info->buf.len,
               buf_info->max_len,
               buf_info->buf.offset);
   }
   mutex_unlock(&dma->buf_lock);

   return len;
}

static int
proc_hist_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   unsigned int i, len = 0;
   struct dma_cfg *dma = (struct dma_cfg *)data;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "History of MMDMA usage:\n");

   mutex_lock(&dma->buf_lock);
   for (i = 0; i < dma->hwcfg.mem_cnt; i++) {
      struct buf_info *buf_info = &dma->buf_list[i];
      len += sprintf(buffer + len, "[slot %u] alloc_cnt=%u free_cnt=%u\n",
            i,
            buf_info->alloc_cnt,
            buf_info->free_cnt);
   }
   mutex_unlock(&dma->buf_lock);

   return len;
}

static int
proc_hist_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
   int i, rc;
   unsigned int clear;
   struct dma_cfg *dma = (struct dma_cfg *)data;
   unsigned char kbuf[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;
	
   rc = copy_from_user(kbuf, buffer, count);
   if (rc) {
      printk(KERN_ERR "copy_from_user failed status=%d", rc);
      return -EFAULT;
   }
	
   if (sscanf(kbuf, "%u", &clear) != 1) {
      printk(KERN_ERR "echo <clear> > /proc/%s/%s\n",
            PROC_PARENT_DIR, PROC_ENTRY_HIST);
      return count;
   }

   if (clear) {
      mutex_lock(&dma->buf_lock);
      for (i = 0; i < dma->hwcfg.mem_cnt; i++) {
         struct buf_info *buf_info = &dma->buf_list[i];
         buf_info->alloc_cnt = 0;
         buf_info->free_cnt = 0;
      }
      mutex_unlock(&dma->buf_lock);
   }
	
   return count;
}

static void proc_term(void);

/*
 * Initialize the proc entries
 */
static int proc_init(void)
{
   struct proc_dir_entry *proc_general;
   struct proc_dir_entry *proc_snapshot;
   struct proc_dir_entry *proc_hist;

   gProc.parent_dir = proc_mkdir(PROC_PARENT_DIR, NULL);

   proc_general = create_proc_entry(PROC_ENTRY_GENERAL, 0644, gProc.parent_dir);
   if (proc_general == NULL) {
      proc_term();
      return -ENOMEM;
   }
   proc_general->read_proc = proc_general_read;
   proc_general->write_proc = NULL;
   proc_general->data = &gDMA;

   proc_snapshot = create_proc_entry(PROC_ENTRY_SNAPSHOT, 0644, gProc.parent_dir);
   if (proc_snapshot == NULL) {
      proc_term();
      return -ENOMEM;
   }
   proc_snapshot->read_proc = proc_snapshot_read;
   proc_snapshot->write_proc = NULL;
   proc_snapshot->data = &gDMA;

   proc_hist = create_proc_entry(PROC_ENTRY_HIST, 0644, gProc.parent_dir);
   if (proc_hist == NULL) {
      proc_term();
      return -ENOMEM;
   }
   proc_hist->read_proc = proc_hist_read;
   proc_hist->write_proc = proc_hist_write;
   proc_hist->data = &gDMA;

	return 0;
}

/*
 * Terminate and remove the proc entries
 */
static void proc_term(void)
{
   remove_proc_entry(PROC_ENTRY_HIST, gProc.parent_dir);
   remove_proc_entry(PROC_ENTRY_SNAPSHOT, gProc.parent_dir);
   remove_proc_entry(PROC_ENTRY_GENERAL, gProc.parent_dir);
   remove_proc_entry(PROC_PARENT_DIR, NULL);
}

static int __devinit mmdma_probe(struct platform_device *pdev)
{
   int rc;
   unsigned int i;
   struct dma_cfg *dma = &gDMA;
   unsigned long data_sum = 0;
   uint8_t *tmp_ptr;
   uint32_t tmp_addr;

   memset(dma, 0, sizeof(*dma));

   /* get the platform data */
   if (pdev->dev.platform_data == NULL) {
      printk(KERN_ERR "MMDMA: Platform data not set properly\n");
      return -ENODEV;
   }
   memcpy(&dma->hwcfg, pdev->dev.platform_data, sizeof(dma->hwcfg));

   mutex_init(&dma->transfer_lock); /* unlocked */
   mutex_init(&dma->buf_lock); /* unlocked */
   init_completion(&dma->transfer_complete); /* incomplete */

   if (dma_set_device_handler(DMA_DEVICE_CLCD_MEM_TO_MEM, dma_isr,
            dma) != 0) {
      printk(KERN_ERR "MMDMA: dma_set_device_handler failed\n");
      return -EFAULT;
   }

   /* if channel is dedicated, reserve the channel now to save overhead */
   if (dma_device_is_channel_shared(DMA_DEVICE_CLCD_MEM_TO_MEM) == 0) {
      printk(KERN_INFO "MMDMA: Using dedicated DMA channel\n");
      dma->handle = dma_alloc_channel(DMA_DEVICE_CLCD_MEM_TO_MEM);
      if (dma->handle < 0) {
         printk(KERN_ERR "MMDMA: dma_alloc_channel failed\n");
         return -EFAULT;
      }
   } else {
      printk(KERN_INFO "MMDMA: Using shared DMA channel\n");
   }

   if (dma->hwcfg.tot_mem) {
      dma->buf_ptr = bootmemheap_alloc("mmdma", dma->hwcfg.tot_mem);
      if (!dma->buf_ptr) {
         printk(KERN_ERR "MMDMA: Unable to allocate the bootmem reserved memory\n");
         rc = -ENOMEM;
         goto err_free_dma_ch;
      }
      dma->buf_addr = virt_to_phys(dma->buf_ptr);
      dma->buf_len = dma->hwcfg.tot_mem;
      memset(dma->buf_ptr, 0, dma->buf_len);

      dma->buf_list = kcalloc(1, dma->hwcfg.mem_cnt * sizeof(*dma->buf_list),
            GFP_KERNEL);
      if (dma->buf_list == NULL) {
         printk(KERN_ERR "MMDMA: Unable to allocate memory for buffer list\n");
         rc = -ENOMEM;
         goto err_free_dma_ch;
      }

      /* 
       * Validate the data size info from the lookup table. Also, at the same time,
       * initialize the buffer info
       */
      data_sum = 0;
      tmp_ptr = (uint8_t *)dma->buf_ptr;
      tmp_addr = dma->buf_addr;
      for (i = 0; i < dma->hwcfg.mem_cnt; i++) {
         if (dma->hwcfg.mem_table[i] % PAGE_SIZE != 0) {
            printk(KERN_ERR "MMDMA: mem_table[%u]=0x%x NOT page aligned!\n",
                  i, dma->hwcfg.mem_table[i]);
         }
      
         dma->buf_list[i].buf.data = tmp_ptr;
         dma->buf_list[i].buf.bus_addr = tmp_addr;
         dma->buf_list[i].buf.offset = data_sum;
         dma->buf_list[i].buf.len = 0;
         dma->buf_list[i].max_len = dma->hwcfg.mem_table[i];
      

         tmp_ptr += dma->hwcfg.mem_table[i];
         tmp_addr += dma->hwcfg.mem_table[i];
         data_sum += dma->hwcfg.mem_table[i];
      }

      if (data_sum > dma->hwcfg.tot_mem) {
         printk(KERN_ERR "MMDMA: Total data sum from lookup table = %lu bytes > "
               "tot_mem=%u\n", data_sum, dma->hwcfg.tot_mem);
         rc = -ENOMEM;
         goto err_free_buf_list;
      }

      printk(KERN_INFO "MMDMA: Reserved memory: %lu bytes\n", data_sum);
      printk(KERN_INFO "MMDMA: Unused memory: %lu bytes\n",
            dma->hwcfg.tot_mem - data_sum);
   } else {
      printk(KERN_INFO "MMDMA: No memory allocation is allowed!\n");
   }

   rc = register_chrdev(BCM_MMDMA_MAJOR, "mmdma", &mmdma_fops);
   if (rc < 0) {
      printk(KERN_WARNING "MMDMA: register_chrdev failed for major %d\n",
            BCM_MMDMA_MAJOR);
      goto err_free_buf_list;
   }

#if CONFIG_SYSFS
   dma_class = class_create(THIS_MODULE, "bcmring-mm-dma");
   if (IS_ERR(dma_class)) {
	   printk(KERN_ERR "DMA: Class create failed\n");
	   rc = -EFAULT;
	   goto err_unreg_driver;
   }
   
   dma_dev = device_create(dma_class, NULL, MKDEV(BCM_MMDMA_MAJOR,0), NULL,
         "mmdma");
   if (IS_ERR(dma_dev)) {
	   printk(KERN_ERR "DMA: Device create failed\n");
	   rc = -EFAULT;
	   goto err_class_destroy;
   }
#endif

   rc = proc_init();
   if (rc < 0) {
      printk(KERN_WARNING "MMDMA: proc_init failed rc=%d\n", rc);
#if CONFIG_SYSFS
      goto err_device_destroy;
#else
      goto err_unreg_driver;
#endif
   }
   
   printk(KERN_INFO "MMDMA: Module initialized\n");
   return 0;
   
#if CONFIG_SYSFS
err_device_destroy:
   device_destroy(dma_class, MKDEV(BCM_MMDMA_MAJOR, 0));
err_class_destroy:
   class_destroy(dma_class);
#endif
   
err_unreg_driver:
   unregister_chrdev(BCM_MMDMA_MAJOR, "mmdma");

err_free_buf_list:
   kfree(dma->buf_list);

err_free_dma_ch:
   if (dma_device_is_channel_shared(DMA_DEVICE_CLCD_MEM_TO_MEM) == 0) {
      dma_free_channel(dma->handle);
   }

   return rc;
}

static int mmdma_remove(struct platform_device *pdev)
{
   unsigned int i;
   struct dma_cfg *dma = &gDMA;
   struct buf_info *buf_info;

   proc_term();

#if CONFIG_SYSFS
   device_destroy(dma_class, MKDEV(BCM_MMDMA_MAJOR, 0));
   class_destroy(dma_class); 
#endif

   unregister_chrdev(BCM_MMDMA_MAJOR, "mmdma");

   if (dma_device_is_channel_shared(DMA_DEVICE_CLCD_MEM_TO_MEM) == 0) {
      if (dma_free_channel(dma->handle) != 0)
         printk(KERN_ERR "MMDMA: dma_free_channel failed\n");
   }

   for (i = 0; i < dma->hwcfg.mem_cnt; i++) {
      buf_info = &dma->buf_list[i];
      buf_free_safe(0, buf_info->buf.bus_addr);
   }

   kfree(dma->buf_list);

   return 0;
}

static struct platform_driver mmdma_driver = {
   .driver = {
      .name = "bcm-mmdma",
      .owner = THIS_MODULE,
   },
   .probe = mmdma_probe,
   .remove = mmdma_remove,
};

static int __init mmdma_init(void)
{
   return platform_driver_register(&mmdma_driver);
}

static void __exit mmdma_exit(void)
{
   platform_driver_unregister(&mmdma_driver);
}

module_init(mmdma_init);
module_exit(mmdma_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Multimedia (MM) DMA Driver");
MODULE_LICENSE("GPL");
