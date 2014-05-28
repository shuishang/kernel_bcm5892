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
 * Description: The LCD driver.
 *
 * The LCD driver implements double buffers internally. The user can update
 * the LCD screen by issuing the update command with a specified Y offset (in
 * lines)
 *
 * NOTE: The user is responsible for maintaining the synchronization between
 * the two buffers
 */

#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/clk.h>

#include <linux/semaphore.h>
#include <asm/uaccess.h>

#include <mach/csp/chipcHw_inline.h>
#include <mach/csp/gpiomux.h>
#include <csp/clcdHw.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/gpio.h>
#include <linux/broadcom/lcd_backlight.h>
#include <linux/broadcom/bcmring_lcd.h>
#include <linux/broadcom/bcmring_display.h>
#include <linux/broadcom/bootmemheap.h>
#include <linux/broadcom/lcd_backlight.h>

#include <mach/csp/cap.h>
#include <mach/pm.h>

#define MAX_PROC_BUF_SIZE     256
#define PROC_PARENT_DIR       "lcd"
#define PROC_ENTRY_REFRESH    "suspendRefreshRate"

#define MAX_PCD      1026
#define MIN_PCD      2

#define MIN_INPUT_CLK 7000000

/* default refresh rate (Hz) in suspend mode */
#define LOW_REFRESH_RATE 15

struct proc_dir {
   struct proc_dir_entry *parent_dir;
};

/*
 * LCD configurations
 */
struct lcd_cfg {
   atomic_t is_probed; /* to indicate the bus has been probed */
   struct clk *clk; /* clock info */
   unsigned int orig_refresh_rate; 
   struct semaphore lock; /* master lock to protect the LCD operations */
   uint32_t vsync_count; /* Vertical sync interrupt counter */
   wait_queue_head_t vsync_wait; /* Vertical sync interrupt wait queue */
   struct lcd_panel_param panel_param; /* LCD panel parameters */
   struct lcd_buf_param buf_param; /* the double buffer data structure */
   /*
    * Active buffer bus address. It is usually the address of the internal LCD
    * double buffer, but the user can overwritte and set it to another address
    * to aovid data copy between buffers in certain cases
    */
   uint32_t active_bus_addr;
   atomic_t enabled; /* panel enabled or disabled */
#ifdef CONFIG_PM
   int was_enabled; /* panel enabled or disabled before suspending */
#endif
	struct lcd_panel_operations panel_ops; /* panel callbacks */
};

/* list that contains reserved GPIO pins */
struct gpio_list {
   unsigned int gpio;
   struct list_head list;
};

static struct proc_dir gProc;
static volatile int gLowRefreshRate = LOW_REFRESH_RATE;
static volatile int gBacklightLevel = 0;

static LIST_HEAD(gGpioList);
static const int bpp_to_bytes_per_pixel[LCD_BPP_MAX] = {2, 2, 4};

/* mappings for LCD CSP driver BPP */
static const int csp_bpp_map[LCD_BPP_MAX] = {CLCDHW_BPP16, CLCDHW_BPP16_565,
   CLCDHW_BPP24};

static struct lcd_cfg gLCD;
#if CONFIG_SYSFS
static struct class * lcd_class;
static struct device * lcd_dev;
#endif

#ifdef CONFIG_PM
static void bcmring_lcd_systemSleepingPattern(void);
#endif

/*
 * LCD IRQ hanlder
 */
static irqreturn_t lcd_isr(int irq, void *data)
{
   struct lcd_cfg *lcd = (struct lcd_cfg *)data;

   /* LCD update */
   if (ClcdHw_GetMaskIntBaseAddrUpd()) {
      /* clear and disable this interrupt */
      ClcdHw_ClearIntBaseAddrUpd();
      ClcdHw_EnIntBaseAddrUpd(0);
   }

   /* LCD FIFO underrun */
   if (ClcdHw_GetMaskIntFifo()) {
      ClcdHw_ClearIntFifo();
   }

   /* AHB master bus error */
   if (ClcdHw_GetMaskIntMBERR()) {
      ClcdHw_ClearIntMBERR();
      printk(KERN_ERR "LCD: AHB Bus Error!\n");
   }

   /* Vertical compare interrupt, i.e. vsync */
   if (ClcdHw_GetMaskIntVcomp()) {
      ClcdHw_ClearIntVcomp();
      ClcdHw_EnIntVertComp(0); /* one shot */
      lcd->vsync_count++;
      wake_up_interruptible(&lcd->vsync_wait);
   }

   return IRQ_HANDLED;
}

/*
 * LCD panel powering up/down operation
 */
static int panel_power_ctrl(struct lcd_pwr_param *param, unsigned int size)
{
   unsigned int i;

   for (i = 0; i < size; i++) {
      if (param[i].sig_src == LCD_PWR_SIG_SRC_LCD) {
         switch (param[i].option.sig_type) {
            case LCD_PWR_SIG_ENABLE:
               ClcdHw_SetLcdEnable(param[i].val);
               break;

            case LCD_PWR_SIG_POWER:
               ClcdHw_SetLcdPwr(param[i].val);
               break;

            case LCD_PWR_SIG_BACKLIGHT:
               if (param[i].val > 0)
                  lcd_backlight_enable(LCD_BACKLIGHT_FULL_ON);
               else
                  lcd_backlight_enable(LCD_BACKLIGHT_OFF);
               break;

            default:
               return -EINVAL;
         }
      } else { /* GPIOs */
         gpio_set_value(param[i].option.gpio, param[i].val);
      }
      mdelay(param[i].mdelay);
   }
   return 0;
}

static int lcd_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
      unsigned long arg)
{
   int rval;

   switch (cmd) {
      case LCD_IOCTL_PANEL_INFO_GET:
      {
         struct lcd_panel_param panel_param;

         lcd_panel_info_get(&panel_param);

         if (copy_to_user((void *)arg, &panel_param,
                  sizeof(struct lcd_panel_param)) != 0) {
            printk(KERN_ERR "LCD: copy_to_user failed for "
                  "ioctl=LCD_IOCTL_PANEL_INFO_GET\n");
            return -EFAULT;
         }
      }
      break;

      case LCD_IOCTL_BUF_INFO_GET:
      {
         struct lcd_buf_param buf_param;

         lcd_buf_info_get(&buf_param);

         if (copy_to_user((void *)arg, &buf_param,
                  sizeof(struct lcd_buf_param)) != 0) {
            printk(KERN_ERR "LCD: copy_to_user failed for "
                  "ioctl=LCD_IOCTL_BUF_INFO_GET\n");
            return -EFAULT;
         }
      }
      break;

      case LCD_IOCTL_PANEL_RESET:
      {
         rval = lcd_panel_reset();
         if (rval != 0) {
            printk(KERN_ERR "LCD: lcd_panel_reset failed\n");
            return -EFAULT;
         }
      }
      break;

      case LCD_IOCTL_PANEL_ENABLE:
      {
         rval = lcd_panel_enable();
         if (rval != 0) {
            printk(KERN_ERR "LCD: lcd_panel_enable failed\n");
            return -EFAULT;
         }
      }
      break;

      case LCD_IOCTL_PANEL_DISABLE:
      {
         rval = lcd_panel_disable();
         if (rval != 0) {
            printk(KERN_ERR "LCD: lcd_panel_disable failed\n");
            return -EFAULT;
         }
      }
      break;

      case LCD_IOCTL_UPDATE:
      {
         rval = lcd_update(arg);
         if (rval != 0) {
            printk(KERN_ERR "LCD: lcd_update failed, y_offset=%u\n",
                  (unsigned int)arg);
            return -EFAULT;
         }
      }
      break;

      default:
         return -EINVAL;
   }
   return 0;
}

static struct file_operations lcd_fops =
{
   owner: THIS_MODULE,
   ioctl: lcd_ioctl,
};

static int pixel_clk_set(struct lcd_panel_param *param)
{
   int clk, pcd, approx_target, approx_clk, trial_val;
   int approx_pcd = MIN_PCD;

   int max_clk = 0;
   int target_pixel_clk = 0;
   int input_clk = 0;

   if (!param)
      return -EINVAL;

   /* get max allowed LCD input clock */
   max_clk = cap_getMaxLcdSpeedHz();

   /* calculate the target pixel clock */
   target_pixel_clk = param->refresh_rate *
      (param->xres + param->left_margin + param->right_margin + param->hsync_len) *
      (param->yres + param->upper_margin + param->lower_margin + param->vsync_len);

   /* targeted pixel clock CANNOT exceed the max allowed clock */
   if (target_pixel_clk > max_clk)
      return -EINVAL;

   /* when target pixel clock is too high, bypass the clock divider */
   if (target_pixel_clk > (max_clk / MIN_PCD)) {
      input_clk = target_pixel_clk;
      ClcdHw_SetBCD(1);
      return input_clk;
   }

   /* when target pixel clock is too low */
   if (target_pixel_clk < (MIN_INPUT_CLK / MAX_PCD)) {
      return -EINVAL;
   }

   approx_target = max_clk / MIN_PCD;
   approx_clk = max_clk;

   /* Scan through clock and divider combinations to find target pixel clock */
   for (clk = MIN_INPUT_CLK; clk <= max_clk; clk += 1000) {
      for (pcd = MIN_PCD; pcd <= MAX_PCD; pcd++) {
         trial_val = clk / pcd;

         if (trial_val == target_pixel_clk) {
            /* Match found, set values and return */
            input_clk = clk;
            ClcdHw_SetPCD(pcd - MIN_PCD);
            return input_clk;
         } else if (trial_val > target_pixel_clk) {
            if (trial_val < approx_target) {
               approx_target = trial_val;
               approx_clk = clk;
               approx_pcd = pcd;
				}
         } else {
            /* no longer need to trial values lower than target */
            break;
         }
      }
   }

   /* No match found at this point. Use approx targets */
   input_clk = approx_clk;
   ClcdHw_SetPCD(approx_pcd - MIN_PCD);

   return input_clk;
}

static void fill_rect(void *buf, unsigned int color,
      unsigned int xoff, unsigned yoff,
      unsigned int width, unsigned int height,
      unsigned int pitch)
{
   unsigned int x, y;
   uint32_t *data = buf;

   data += yoff * pitch + xoff;

   for (y = 0; y < height; y++) {
      for (x = 0; x < width; x++) {
         data[x] = color;
      }
      data += pitch;
   }
}

static void early_screen_draw(void *buf, unsigned int width,
      unsigned int height, unsigned int bytes_per_pixel)
{
   /* draw some REDs */
   fill_rect(buf, 0xFF0000, 0, 0, width / 2, height / 4, width);

   /* draw some GREENs */
   fill_rect(buf, 0xFF00, width / 2, height / 4, width / 2, height / 4, width);

   /* draw some BLUEs */
   fill_rect(buf, 0xFF, 0, height / 2, width / 2, height / 4, width);

   /* draw some WHITEs */
   fill_rect(buf, 0xFFFFFF, width / 2, height / 4 * 3, width / 2, height / 4,
         width);
}

static int lcd_gpio_reserve(struct lcd_panel_param *panel_param,
      struct list_head *list)
{
   int rval = -EFAULT;
   unsigned int i, match_found = 0;
   struct gpio_list *entry = NULL;
   struct gpio_list *n;
   gpiomux_rc_e gpio_status;
   struct lcd_pwr_param *pwr_up, *pwr_down;

   pwr_up = panel_param->pwr_up;
   pwr_down = panel_param->pwr_down;

   for (i = 0; i < panel_param->num_pwr_up; i++) {
      if (pwr_up[i].sig_src == LCD_PWR_SIG_SRC_GPIO) {
         match_found = 0;
         list_for_each_entry(entry, list, list) {
            if (entry->gpio == pwr_up[i].option.gpio) {
               match_found = 1;
               break;
            }
         }

         if (match_found == 0) {
            entry = kcalloc(1, sizeof(struct gpio_list), GFP_KERNEL);
            if (entry == NULL) {
               printk(KERN_ERR "LCD: Unable to allocate memory for GPIO\n");
               rval = -ENOMEM;
               goto free_gpio_mem;
            }
            entry->gpio = pwr_up[i].option.gpio;
            list_add(&entry->list, list);
         }
      }
   }

   for (i = 0; i < panel_param->num_pwr_down; i++) {
      if (pwr_down[i].sig_src == LCD_PWR_SIG_SRC_GPIO) {
         match_found = 0;
         list_for_each_entry(entry, list, list) {
            if (entry->gpio == pwr_down[i].option.gpio) {
               match_found = 1;
               break;
            }
         }

         if (match_found == 0) {
            entry = kcalloc(1, sizeof(struct gpio_list), GFP_KERNEL);
            if (entry == NULL) {
               printk(KERN_ERR "LCD: Unable to allocate memory for GPIO\n");
               rval = -ENOMEM;
               goto free_gpio_mem;
            }
            entry->gpio = pwr_down[i].option.gpio;
            list_add(&entry->list, list);
         }
      }
   }

   list_for_each_entry(entry, list, list) {
      gpio_status = gpiomux_request(entry->gpio, chipcHw_GPIO_FUNCTION_GPIO,
            "LCD");
      if (gpio_status != gpiomux_rc_SUCCESS) {
         printk(KERN_ERR "LCD: Unable to request GPIO pin %u\n", entry->gpio);
         rval = -EFAULT;
         goto free_gpio;
      }
      /* set GPIO to output mode */
      gpio_direction_output(entry->gpio, 0);
   }

   return 0;

free_gpio:
   list_for_each_entry(entry, list, list) {
      gpiomux_free(entry->gpio);
   }

free_gpio_mem:
   list_for_each_entry_safe(entry, n, list, list) {
      list_del(&entry->list);
      kfree(entry);
   }
   INIT_LIST_HEAD(list);

   return rval;
}

static void lcd_gpio_free(struct list_head *list)
{
   struct gpio_list *entry = NULL;
   struct gpio_list *n;

   list_for_each_entry_safe(entry, n, list, list) {
      gpiomux_free(entry->gpio);
      list_del(&entry->list);
      kfree(entry);
   }
   INIT_LIST_HEAD(list);
}

static int
proc_refresh_read(char *buffer, char **start, off_t off, int count,
		int *eof, void *data)
{
   unsigned int len = 0;

   if (off > 0)
      return 0;

   len += sprintf(buffer + len, "Current refresh rate in suspend mode is set to %d Hz\n",
         gLowRefreshRate);

   return len;
}

static int
proc_refresh_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
   int rc;
   int rate;
   unsigned char kbuf[MAX_PROC_BUF_SIZE];

   if (count > MAX_PROC_BUF_SIZE)
      count = MAX_PROC_BUF_SIZE;
	
   rc = copy_from_user(kbuf, buffer, count);
   if (rc) {
      printk(KERN_ERR "copy_from_user failed status=%d", rc);
      return -EFAULT;
   }
	
   if (sscanf(kbuf, "%d", &rate) != 1) {
      printk(KERN_ERR "echo <refresh_rate> > /proc/%s/%s\n",
            PROC_PARENT_DIR, PROC_ENTRY_REFRESH);
      return count;
   }

   if (rate < 1 || rate > 100) {
      printk(KERN_ERR "Refresh rate of %d Hz in suspend mode is NOT supported\n", rate);
      printk(KERN_ERR "Please set it to 1 - 100 Hz\n");
      return count;
   }

   gLowRefreshRate = rate;
   printk(KERN_INFO "Refresh rate in suspend mode it set to %d Hz\n", gLowRefreshRate);
	
   return count;
}

static int proc_init(void)
{
   struct proc_dir_entry *proc_refresh;

   gProc.parent_dir = proc_mkdir(PROC_PARENT_DIR, NULL);

   proc_refresh = create_proc_entry(PROC_ENTRY_REFRESH, 0644,
         gProc.parent_dir);
   if (proc_refresh == NULL) {
      return -ENOMEM;
   }
   proc_refresh->read_proc = proc_refresh_read;
   proc_refresh->write_proc = proc_refresh_write;
   proc_refresh->data = NULL;

   return 0;
}

static void proc_term(void)
{
   remove_proc_entry(PROC_ENTRY_REFRESH, gProc.parent_dir);
   remove_proc_entry(PROC_PARENT_DIR, NULL);
}

static int lcd_probe(struct platform_device *pdev)
{
   unsigned int buf_size;
   int rval = -EFAULT;
   int clk_rate;
   struct device *dev = &pdev->dev;
   struct lcd_cfg *lcd = &gLCD;
   struct lcd_buf_param *buf_param = &lcd->buf_param;
   struct lcd_panel_param *panel_param;

   atomic_set(&lcd->is_probed, 0);

   if (cap_isPresent(CAP_CLCD, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "LCD: Not supported\n");
      return -ENODEV;
   }

   init_MUTEX(&lcd->lock); /* unlocked */
   init_waitqueue_head(&lcd->vsync_wait); /* empty */
   atomic_set(&lcd->enabled, 0);

   /* get and validate platform data */
   panel_param = dev->platform_data;
   if (panel_param == NULL || panel_param->pwr_up == NULL ||
         panel_param->pwr_down == NULL) {
      printk(KERN_ERR "LCD: Platform data (panel parameters) not set\n");
      rval = -ENODEV;
      goto err_exit;
   }

   memcpy(&lcd->panel_param, panel_param, sizeof(lcd->panel_param));

   /* enable clocks */
   lcd->clk = clk_get(dev, "LCD");
   if (IS_ERR(lcd->clk)) {
      printk(KERN_ERR "LCD: Unable to find LCD clock\n");
      rval = PTR_ERR(lcd->clk);
      goto err_exit;
   }

   rval = clk_enable(lcd->clk);
   if (rval < 0) {
      printk(KERN_ERR "LCD: Unable to enable LCD clock\n");
      goto err_exit;
   }

   clk_rate = pixel_clk_set(&lcd->panel_param);
   if (clk_rate < 0) {
      printk(KERN_ERR "LCD: Pixel clock cannot be set properly\n");
      rval = clk_rate;
      goto err_exit;
   }

   rval = clk_set_rate(lcd->clk, clk_rate);
   if (rval < 0) {
      printk(KERN_ERR "LCD: Unable to set LCD clock rate to %d\n", clk_rate);
      goto err_exit;
   }

   ClcdHw_Init();

   /* figure out GPIOs that need to be reserved and reserve them */
   rval = lcd_gpio_reserve(&lcd->panel_param, &gGpioList);
   if (rval < 0) {
      printk(KERN_ERR "LCD: lcd_gpio_reserve failed\n");
      goto err_free_clk;
   }

   /* allocate a double buffer */
   buf_param->xres = lcd->panel_param.xres;
   buf_param->yres = lcd->panel_param.yres;
   buf_param->virtual_xres = lcd->panel_param.xres;
   buf_param->virtual_yres = lcd->panel_param.yres * 2;
   buf_param->bpp = bpp_to_bytes_per_pixel[lcd->panel_param.bpp] * 8;
   buf_size = buf_param->xres * buf_param->yres * buf_param->bpp / 8;

   buf_param->data.ptr = bootmemheap_alloc("bcmring_lcd", buf_size * 2);
	if (!buf_param->data.ptr) {
		printk(KERN_ERR "LCD: Unable to allocate the buffer memory\n");
      rval = -ENOMEM;
      goto err_free_gpio;
   }
	buf_param->data.addr = virt_to_phys(buf_param->data.ptr);
	buf_param->data.len = buf_size * 2;
   lcd->active_bus_addr = buf_param->data.addr;
   memset(buf_param->data.ptr, 0, buf_param->data.len);
   early_screen_draw(buf_param->data.ptr, buf_param->xres,
         buf_param->yres, buf_param->bpp);

	rval = display_panel_init(&lcd->panel_ops);
	if (rval < 0) {
		printk(KERN_ERR "LCD: display_panel_init failed\n");
      goto err_free_buf;
	}

   rval = lcd_panel_reset();
   if (rval != 0) {
      printk(KERN_ERR "LCD: Panel reset failed\n");
      rval = -EINVAL;
      goto err_term_display;
   }
   rval = lcd_panel_enable();
   if (rval != 0) {
      printk(KERN_ERR "LCD: Panel enable failed\n");
      rval = -EINVAL;
      goto err_term_display;
   }

   /*
    * Unmask FIFO underflow and AHB bus error interrupts. Mask update and
    * VSYNC interrupts for now
    */
   ClcdHw_EnIntFifo(1);
   ClcdHw_EnIntBaseAddrUpd(0);
   ClcdHw_EnIntVertComp(0);
   ClcdHw_EnIntMBERR(1);

   if (request_irq(IRQ_CLCD, lcd_isr, IRQF_SHARED, "LCD", lcd) < 0) {
      printk(KERN_ERR "LCD: request_irq %u failed\n", IRQ_CLCD);
      rval = -EFAULT;
      goto err_disable_panel;
   }

   if (register_chrdev(BCM_CLCD_MAJOR, "lcd", &lcd_fops) < 0) {
      printk(KERN_ERR "LCD: register_chrdev failed for major %u\n",
            BCM_CLCD_MAJOR);
      rval = -EFAULT;
      goto err_free_irq;
   }

#if CONFIG_SYSFS
   lcd_class = class_create(THIS_MODULE,"bcmring-lcd");
   if(IS_ERR(lcd_class)){
	   printk(KERN_ERR "LCD: Class create failed\n");
	   rval = -EFAULT;
	   goto err_unregister_chrdev;
   }

   lcd_dev = device_create(lcd_class, NULL, MKDEV(BCM_CLCD_MAJOR,0),NULL,"lcd");
   if(IS_ERR(lcd_dev)){
	   printk(KERN_ERR "LCD: Device create failed\n");
	   rval = -EFAULT;
	   goto err_class_destroy;
   }
#endif
   lcd_update(0);

#ifdef CONFIG_PM
	pm_lcd_register_callout(bcmring_lcd_systemSleepingPattern);
#endif

   proc_init();

   platform_set_drvdata(pdev, lcd);

   printk(KERN_NOTICE "LCD: Driver initialized\n");
   printk(KERN_NOTICE "LCD: name=%s width=%u height=%u bytes/pixel=%u\n",
         lcd->panel_param.name, lcd->panel_param.xres, lcd->panel_param.yres,
         bpp_to_bytes_per_pixel[lcd->panel_param.bpp]);

   atomic_set(&lcd->is_probed, 1);
   return 0;

#if CONFIG_SYSFS
err_class_destroy:
   class_destroy(lcd_class);
err_unregister_chrdev:
   unregister_chrdev(BCM_CLCD_MAJOR, "lcd");
#endif
err_free_irq:
   free_irq(IRQ_CLCD, lcd);

err_disable_panel:
   lcd_panel_disable();

err_term_display:
   display_panel_term(&lcd->panel_ops);

err_free_buf:
   buf_param->data.ptr = NULL;
   buf_param->data.addr = 0;

err_free_gpio:
   lcd_gpio_free(&gGpioList);

err_free_clk:
   ClcdHw_Exit();
   clk_disable(lcd->clk);
   clk_put(lcd->clk);

err_exit:
   return rval;
}

static int lcd_remove(struct platform_device *pdev)
{
   struct lcd_cfg *lcd = platform_get_drvdata(pdev);
   struct lcd_buf_param *buf_param = &lcd->buf_param;

   proc_term();

#if CONFIG_SYSFS
   device_destroy(lcd_class,MKDEV(BCM_CLCD_MAJOR,0));
   class_destroy(lcd_class);
#endif
   atomic_set(&lcd->is_probed, 0);
   unregister_chrdev(BCM_CLCD_MAJOR, "lcd");
   free_irq(IRQ_CLCD, lcd);
   lcd_panel_disable();
	display_panel_term(&lcd->panel_ops);
   buf_param->data.ptr = NULL;
   buf_param->data.addr = 0;

   lcd_gpio_free(&gGpioList);

   ClcdHw_Exit();
   clk_disable(lcd->clk);
   clk_put(lcd->clk);

   return 0;
}

#ifdef CONFIG_PM
/*
 * This routine assumes that by the time it gets executed, lcd_update has
 * completed and will not re-engage until lcd_resume is done
 */
static int lcd_suspend(struct platform_device *pdev, pm_message_t mesg)
{
   int rval = -EFAULT;
   int clk_rate;
   struct lcd_cfg *lcd = platform_get_drvdata(pdev);
   
   /* keep the LCD clock alive but lower the refresh rate */
   if (pm_lcd_get_state() == pm_state_alive) { 
      printk(KERN_INFO "LCD: Entering low refresh rate suspend mode...\n");

      /* record the original refresh rate */
      lcd->orig_refresh_rate = lcd->panel_param.refresh_rate;
      lcd->panel_param.refresh_rate = gLowRefreshRate;
      clk_rate = pixel_clk_set(&lcd->panel_param);
      if (clk_rate < 0) {
         printk(KERN_ERR "LCD: Pixel clock cannot be set properly during suspend refresh_rate=%u\n",
               lcd->panel_param.refresh_rate);
         /* error! we have no choice, need to fall back to the original rate and quit */
         lcd->panel_param.refresh_rate = lcd->orig_refresh_rate;
         pixel_clk_set(&lcd->panel_param);
         return clk_rate;
      }

      rval = clk_set_rate(lcd->clk, clk_rate);
      if (rval < 0) {
         printk(KERN_ERR "LCD: Unable to set LCD clock rate to %d during suspend\n",
              clk_rate);
         /* error! we have no choice, need to fall back to the original rate and quit */
         lcd->panel_param.refresh_rate = lcd->orig_refresh_rate;
         clk_rate = pixel_clk_set(&lcd->panel_param);
         clk_set_rate(lcd->clk, clk_rate);
         return rval;
      }
   } else { /* need to disable the LCD clock in this case */
      /* unmask all LCD interrupts */
      ClcdHw_EnIntFifo(0);
      ClcdHw_EnIntBaseAddrUpd(0);
      ClcdHw_EnIntVertComp(0);
      ClcdHw_EnIntMBERR(0);

      /* disable the LCD panel and stop the clock */
      if (atomic_read(&lcd->enabled))
      {
         lcd->was_enabled = 1;
         gBacklightLevel = lcd_backlight_curr_level();
         rval = lcd_panel_disable();
         if (rval < 0)
            return rval;
      }
      else
      {
         lcd->was_enabled = 0;
      }

      chipcHw_deactivatePifLcdInterface();
      clk_disable(lcd->clk);

      /* Fake a vsync to unblock any tasks waiting for it. */
      lcd->vsync_count++;
      wake_up_interruptible(&lcd->vsync_wait);
   }

   return 0;
}

static int lcd_resume(struct platform_device *pdev)
{
   int rval = -EFAULT;
   int clk_rate;
   struct lcd_cfg *lcd = platform_get_drvdata(pdev);

   if (pm_lcd_get_state() == pm_state_alive) {
      printk(KERN_INFO "LCD: Resuming from low refresh rate suspend mode...\n");
      lcd->panel_param.refresh_rate = lcd->orig_refresh_rate;
      clk_rate = pixel_clk_set(&lcd->panel_param);
      if (clk_rate < 0) {
         printk(KERN_ERR "LCD: Pixel clock cannot be set properly during resume refresh_rate=%u\n",
               lcd->panel_param.refresh_rate);
         return clk_rate;
      }
      rval = clk_set_rate(lcd->clk, clk_rate);
      if (rval < 0) {
         printk(KERN_ERR "LCD: Unable to set LCD clock rate to %d during resume\n",
              clk_rate);
         return rval;
      }
   } else {
      /* enable the LCD clock and panel */
      rval = clk_enable(lcd->clk);
      if (rval < 0)
         return rval;

      chipcHw_activateLcdInterface();

      if ( lcd->was_enabled ) {
         rval = lcd_panel_enable();
         if (rval < 0)
            return rval;
         lcd_backlight_enable(gBacklightLevel);
      }

      /* unmask interrupts */
      ClcdHw_ClearIntFifo();
      ClcdHw_EnIntFifo(1);
      ClcdHw_ClearIntMBERR();
      ClcdHw_EnIntMBERR(1);
   }

   return 0;
}
#else
#define lcd_suspend    NULL
#define lcd_resume     NULL
#endif

static struct platform_driver lcd_driver = {
   .driver = {
      .name = "bcmring-lcd",
      .owner = THIS_MODULE,
   },
   .probe = lcd_probe,
   .remove = lcd_remove,
   .suspend = lcd_suspend,
   .resume = lcd_resume,
};

static int __init lcd_init(void)
{
   return platform_driver_register(&lcd_driver);
}

static void __exit lcd_exit(void)
{
   /*class_destroy(lcd_class); */
   platform_driver_unregister(&lcd_driver);
}

module_init(lcd_init);
module_exit(lcd_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("LCD Driver");
MODULE_LICENSE("GPL");

int lcd_panel_info_get(struct lcd_panel_param *panel_param)
{
   struct lcd_cfg *lcd = &gLCD;

   if (!atomic_read(&lcd->is_probed))
      return -ENODEV;

   memcpy(panel_param, &lcd->panel_param, sizeof(*panel_param));

   return 0;
}
EXPORT_SYMBOL(lcd_panel_info_get);

int lcd_buf_info_get(struct lcd_buf_param *buf_param)
{
   struct lcd_cfg *lcd = &gLCD;

   if (!atomic_read(&lcd->is_probed))
      return -ENODEV;

   memcpy(buf_param, &lcd->buf_param, sizeof(*buf_param));

   return 0;
}
EXPORT_SYMBOL(lcd_buf_info_get);

int lcd_panel_reset(void)
{
   struct lcd_cfg *lcd = &gLCD;
   struct lcd_panel_param *param = &lcd->panel_param;

   down(&lcd->lock);

   ClcdHw_SetPPL(param->xres);
   /* dual panel NOT supported */
   ClcdHw_SetLPP(param->yres, 0);

   ClcdHw_SetHrzBackPorch(param->left_margin);
   ClcdHw_SetHrzFrontPorch(param->right_margin);
   ClcdHw_SetVertBackPorch(param->upper_margin);
   ClcdHw_SetVertFrontPorch(param->lower_margin);

   ClcdHw_SetHrzSynchWidth(param->hsync_len);
   ClcdHw_SetVertSynchWidth(param->vsync_len);

   ClcdHw_SetCPL(param->xres, CLCDHW_LCDTFT);

   ClcdHw_SetIOE(param->inv_out_enable);
   ClcdHw_SetIPC(param->inv_pixel_clk);
   ClcdHw_SetIHS(param->inv_hsync);
   ClcdHw_SetIVS(param->inv_vsync);

   /* NO AC frequency bias */
   ClcdHw_SetACB(0);
   /* NO external clock */
   ClcdHw_SetClkSel(0);
   ClcdHw_SetLineEndEn(0);
   ClcdHw_SetSigDelay(0);
   ClcdHw_SetWatermarkSig(1);
   /* Vertical interrupt on front porch, to have maximum time */
   ClcdHw_SetLcdVComp(3);
   ClcdHw_SetBEPO(0);
   ClcdHw_SetBEBO(0);

   /* NOTE: RGB order in our LCD controller is revered */
   if (param->rgb_order == LCD_ORDER_RGB)
      ClcdHw_SetBGR(1);
   else
      ClcdHw_SetBGR(0);

   ClcdHw_SetMonoLcdInterface(0);
   /* assume it's always TFT panel */
   ClcdHw_SetLcdTFT(1);
   /* who uses black & white nowadays? */
   ClcdHw_SetLcdBW(0);
   ClcdHw_SetLcdBpp(csp_bpp_map[param->bpp]);

   up(&lcd->lock);
   return 0;
}
EXPORT_SYMBOL(lcd_panel_reset);

int lcd_panel_enable(void)
{
   int rval = -EFAULT;
   struct lcd_cfg *lcd = &gLCD;
   struct lcd_pwr_param *param = lcd->panel_param.pwr_up;
   unsigned int num_entries = lcd->panel_param.num_pwr_up;

   if (atomic_read(&lcd->enabled))
      return 0;

   down(&lcd->lock);

   atomic_inc(&lcd->enabled);
   rval = panel_power_ctrl(param, num_entries);

	if (lcd->panel_ops.lcd_panel_enable)
		lcd->panel_ops.lcd_panel_enable();

   up(&lcd->lock);

   return rval;
}
EXPORT_SYMBOL(lcd_panel_enable);

int lcd_panel_disable(void)
{
   int rval = -EFAULT;
   struct lcd_cfg *lcd = &gLCD;
   struct lcd_pwr_param *param = lcd->panel_param.pwr_down;
   unsigned int num_entries = lcd->panel_param.num_pwr_down;

   if (atomic_read(&lcd->enabled) == 0)
      return 0;

   down(&lcd->lock);

   atomic_dec(&lcd->enabled);
   rval = panel_power_ctrl(param, num_entries);

	if (lcd->panel_ops.lcd_panel_disable)
		lcd->panel_ops.lcd_panel_disable();

   up(&lcd->lock);

   return rval;
}
EXPORT_SYMBOL(lcd_panel_disable);

int lcd_buf_addr_set(uint32_t bus_addr)
{
   struct lcd_cfg *lcd = &gLCD;
   struct lcd_buf_param *buf_param = &lcd->buf_param;

   down(&lcd->lock);
   if (bus_addr) {
      lcd->active_bus_addr = bus_addr;
   } else {
      lcd->active_bus_addr = buf_param->data.addr;
   }
   up(&lcd->lock);

   /* issue a LCD update after the buffer address is changed */
   lcd_update(0);

   return 0;
}
EXPORT_SYMBOL(lcd_buf_addr_set);

int lcd_update(unsigned int y_offset)
{
   struct lcd_cfg *lcd = &gLCD;
   struct lcd_buf_param *buf_param = &lcd->buf_param;
   struct lcd_panel_param *panel_param = &lcd->panel_param;

   if (atomic_read(&lcd->enabled) == 0)
      return 0;

   if (y_offset > buf_param->virtual_yres / 2)
      return -EINVAL;

   down(&lcd->lock);

   /* update the LCD */
   if (ClcdHw_SetUpperBaseAddr(lcd->active_bus_addr +
         (y_offset * panel_param->xres * buf_param->bpp / 8)) != 0) {
      printk(KERN_ERR "LCD: ClcdHw_SetUpperBaseAddr failed, most likely due "
            "to address alignment issue addr=0x%08x\n", lcd->active_bus_addr +
         (y_offset * panel_param->xres * buf_param->bpp / 8));
   }

   up(&lcd->lock);

   return 0;
}
EXPORT_SYMBOL(lcd_update);

int lcd_wait_for_vsync(void)
{
   struct lcd_cfg *lcd = &gLCD;
   uint32_t vc = lcd->vsync_count; /* Sample the current count first */
   unsigned prev_refresh_rate = lcd->panel_param.refresh_rate;
   long timeout;
   int wait_result;

again:

   /* If the LCD is not enabled, we just return immediately as if
      the vsync did occur. */
   if (atomic_read(&lcd->enabled) == 0)
      return 0;

   /* Set up interrupt. */
   ClcdHw_ClearIntVcomp();
   ClcdHw_EnIntVertComp(1);


   /* The timeout is the screen refresh period rounded up to the
      nearest jiffy plus one.  A two-jiffy wait could be as short
      as nearly one jiffy, since the next tick can be coming very soon!
      E.g. HZ == 100, refresh == 60:   (100 + 60 + 59 / 60) == 3
      We must sleep 3 jiffies at a HZ 100 to be guaranteed that
      we have waited past 1/60th of a second. */
   timeout = (HZ + 2 * prev_refresh_rate - 1) / prev_refresh_rate;

   wait_result = wait_event_interruptible_timeout(lcd->vsync_wait,
                                                  lcd->vsync_count != vc,
                                                  timeout);

   /* Time remaining, so the event occured. */
   if (wait_result > 0)
      return 0;

   /* Interruption: let's force syscall restart. */
   if (wait_result < 0)
      return -ERESTARTNOINTR;

   /* Now the zero value which indicates a timeout does not
      imply that the event did also not happen! We must re-evaluate
      the condition to find out.  If the vsync event happened,
      but the long wait time has also passed, it likely means
      that the task woke up too late due to a heavy scheduling
      load. The vsync it was waiting for is long gone. We still
      return zero in order not to upset the user space application. */
   if (lcd->vsync_count != vc)
      return 0;

   /* We may have a false timeout if the refresh rate is being manipulated.
      E.g. a thread goes to sleep when a fast refresh rate is in effect,
      but then the refresh rate is slowed down while it waits. */
   if (lcd->panel_param.refresh_rate != prev_refresh_rate) {
      prev_refresh_rate = lcd->panel_param.refresh_rate;
      goto again;
   }

   /* The goal is never to return this value except in a case
      where there is a problem with the hardware not generating the interrupt.
      I.e not under heavy load, or due to cases arising from
      power management transitions. */
   return -ETIMEDOUT;
}
EXPORT_SYMBOL(lcd_wait_for_vsync);

/*
 * Note that the hardware does not have a register to indicate this directly.
 * What the hardware has is a register (or a pair of registers, for dual-panel
 * LCD support) which indicate the approximate current DMA address. If we take
 * two samples of this address one microsecond apart, then we should see a
 * change in the value. (Assumption: the pixel clock will never be down to the
 * range of 1Mhz).
 */
int lcd_is_vsyncing(void)
{
   struct lcd_cfg *lcd = &gLCD;

   if (atomic_read(&lcd->enabled) == 0) {
      return 0;
   } else {
      uint32_t cur_dma_sample_0 = ClcdHw_GetUpperCurAddr();
      uint32_t cur_dma_sample_1 = (udelay(1), ClcdHw_GetUpperCurAddr());
      return cur_dma_sample_0 == cur_dma_sample_1;
   }
}
EXPORT_SYMBOL(lcd_is_vsyncing);

/*
 * The code below is an example reference implementation of how we would
 * show a system sleeping pattern on the LCD while the rest of the system
 * is actually sleeping.
 */




#ifdef CONFIG_PM
/****************************************************************************
*
*  Calculates an rgb value from individual components.
*
*  This function assumes that R G & B values are specified in the range
*  0-255 and will scale appropriately for non 8-bit pixels.
*
***************************************************************************/

static unsigned rgb_val( unsigned red, unsigned green, unsigned blue, unsigned bits_per_pixel )
{
   if ( bits_per_pixel == 16 )
   {
      /* 16 bpp implies 565 format */

      return (( red & 0xf8 ) << 8 ) | (( green & 0xfc ) << 3 ) | (( blue & 0xf8 ) >> 3 );
   }

   /*
    * Otherwise assume 32 bpp, which is ARGB (0xff in the transparency means
    * 100% opaque)
    */

   return 0xff000000 | (( red & 0xff ) << 16 ) | (( green & 0xff ) << 8 ) | ( blue & 0xff );

} /* rgb_val */

/****************************************************************************
*
*   Calculates the location in the frame buffer for a given xy position
*
***************************************************************************/

static void *setPixels( void * fbVirtPtr, unsigned x, unsigned y, unsigned numPixels, unsigned color, unsigned xres, unsigned bits_per_pixel )
{
    if ( bits_per_pixel == 16 )
    {
        uint16_t *fbPtr = fbVirtPtr;

        fbPtr += (( y * xres ) + x );

        while ( numPixels-- > 0 )
        {
            *fbPtr++ = color;
        }
    }
    else
    if ( bits_per_pixel == 24 )
    {
        uint8_t* ptr = (uint8_t*)fbVirtPtr;
        uint8_t* colptr = (uint8_t*)&color;

        ptr += (( y * xres ) + x * 3 );

        while ( numPixels-- > 0 )
        {
            ptr[0]= colptr[0];
            ptr[1]= colptr[1];
            ptr[2]= colptr[2];
            ptr += 3;
        }
    }
    else
    if ( bits_per_pixel == 32 )
    {
        uint32_t *fbPtr = fbVirtPtr;

        fbPtr += (( y * xres ) + x );

        while ( numPixels-- > 0 )
        {
            *fbPtr++ = color;
        }
    }

    return 0;

} /* setPixels */

/****************************************************************************
*
*  ColorBars - Draws some colorbars on the screen
*
***************************************************************************/
static void ColorBars( void *fbPtr, int num, int xres, int yres, int bits_per_pixel )
{
    #define NUM_INCREMENTS       32 /* RGB is 565 format, 2^5 = 32 */

    #define R   1
    #define G   2
    #define B   4

    int pattern[7] = { R, G, B, R | G, R | B, G | B, R | G | B };
    int numPatterns = sizeof( pattern ) / sizeof( pattern[ 0 ] );

    unsigned int    y;
    int patternIdx;
    int numRowsPerPattern = yres / numPatterns;
    int incrWidth = xres / ( NUM_INCREMENTS + 1 );
    int border = ( xres - ( incrWidth * ( NUM_INCREMENTS + 1 ))) / 2;
    int fbSize = xres * yres * bits_per_pixel / 8;

    /* Start off by clearing the whole frame buffer to black */

    memset( fbPtr, 0, fbSize );

    y = 0;
    for ( patternIdx = 0; patternIdx < numPatterns; patternIdx++ )
    {
        int r;
        int rgb = pattern[ ( patternIdx + num) % 7 ];

        for ( r = 0; r < numRowsPerPattern; r++, y++ )
        {
            unsigned x = border;
            int      col1;

            for ( col1 = NUM_INCREMENTS; col1 >= 0; col1-- )
            {
                unsigned component = col1 * 255 / NUM_INCREMENTS;
                unsigned red       = (( rgb & R ) != 0 ) * component;
                unsigned green     = (( rgb & G ) != 0 ) * component;
                unsigned blue      = (( rgb & B ) != 0 ) * component;

                setPixels( fbPtr, x, y, incrWidth, rgb_val( red, green, blue, bits_per_pixel ), xres, bits_per_pixel);

                x += incrWidth;
            }
        }
    }
}

/*
 * Note that you CANNOT msleep in this callout.
 */
static void bcmring_lcd_systemSleepingPattern(void)
{
	static struct lcd_buf_param param;
	static int idx = 0;
	static int gotInfo = 0;
	if (!gotInfo)
	{
		if (lcd_buf_info_get(&param) == 0)
		{
			printk("xres=%u, yres=%u, virtual_xres=%u, virtual_yres=%u, bpp=%u\n", param.xres, param.yres, param.virtual_xres, param.virtual_yres, param.bpp);
			printk("ptr=%p, physaddr=%x,len=%u\n", param.data.ptr, param.data.addr, param.data.len);
			gotInfo = 1;
		}
		else
		{
			printk("lcd_buf_info_get failed\n");
			return;
		}
	}
   ColorBars(param.data.ptr, idx++, param.xres, param.yres, param.bpp);
}
#endif
