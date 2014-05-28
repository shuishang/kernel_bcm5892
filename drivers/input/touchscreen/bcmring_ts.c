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
 * BCMRING Touchscreen Driver
 *
 * The driver uses the Linux input subsystem. User can access the touchscreen
 * data through the /dev/input/eventX node
 *
 * It is recommended to use tslib for user space programs to access the
 * touchscreen raw data. tslib can also be used to calibrate the touchscreen
 */

#include <linux/version.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <linux/platform_device.h>

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/clk.h>

#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <csp/tscHw.h>
#include <linux/broadcom/ts.h>

#include <mach/csp/cap.h>

#define BCM_TS_MAX_X         0xFFF
#define BCM_TS_MAX_Y         0xFFF

#define TSC_CLOCK_RATE_HZ    2000000

#if !defined(HW_CFG_TSC_SWAP_X_Y)
#define HW_CFG_TSC_SWAP_X_Y 0
#endif

#define DEBUG 0
#if DEBUG
#define TS_DEBUG(fmt, args...) printk(KERN_ERR fmt, ## args)
#else
#define TS_DEBUG(fmt, args...)
#endif

/* use a data FIFO threshold of 2 */
#define DATA_THRES 2

/*
 * The touch screen state enumeration is used to report whether the user is
 * currently touching the screen (down) or not (up)
 */
typedef enum
{
   TSC_TOUCH_DOWN = 0, /* down state */
   TSC_TOUCH_UP /* up state/event */
} touch_state;

/*
 * The touch screen event structure is used to report a touch screen event.
 * When an event occurs, X and Y coordinates are reported, along with the touch
 * screen state
 */
struct touch_event {
   int xData; /* X coordinate */
   int yData; /* Y coordinate */
   int pressure; /* pressure (not supported in the current driver and device) */
   touch_state state; /* touch screen state */
};

struct ts_dev {
   struct input_dev *input;
   struct clk *clk;
   struct touch_event touch_data;
   int data_available;
   int pen_transition;
   tsc_param hw_param;
};

static char banner[] __initdata = KERN_INFO "Broadcom Touchscreen Driver\n";

static int mod_param_samp_rate = -1;
module_param(mod_param_samp_rate, int, 0);

static int mod_param_debounce = -1;
module_param(mod_param_debounce, int, 0);

static int mod_param_touch_timeout = -1;
module_param(mod_param_touch_timeout, int, 0);

static int mod_param_settling = -1;
module_param(mod_param_settling, int, 0);

static int mod_param_data_point_average = -1;
module_param(mod_param_data_point_average, int, 0);

static irqreturn_t ts_isr(int irq, void *data)
{
   uint32_t touch_data[2] = {0, 0};
   uint32_t read_words = 0;

   struct ts_dev *ts = data;
   tsc_param *hw_param = &ts->hw_param;

   /*
    * Check the types of interrupt and clear them as early as possible
    */
   if (tscHw_IsIrqPen()) {
      ts->pen_transition = 1;
      tscHw_IrqPenClear();
   }
   if (tscHw_IsIrqData()) {
      /*
       * Don't clear the data interrupt yet, until data has been read from
       * the FIFO
       */
      ts->data_available = 1;
   }

   /* pan up/down interrupt */
   if (ts->pen_transition) {
      /* check the pen status */
      if (tscHw_IsPenDown()) {
         ts->touch_data.state = TSC_TOUCH_DOWN;
         /* artifical pressure data */
         ts->touch_data.pressure = 0xFFF;
      } else {
         ts->touch_data.state = TSC_TOUCH_UP;
         ts->touch_data.pressure = 0;
      }
   }

   /* XY coordinates interrupt */
   if (ts->data_available) {
      /* read raw data (in words) from the touch screen controller */
      read_words = tscHw_Read(touch_data, 2);

      /* clear the interrupt after reading from the FIFO */
      tscHw_IrqDataClear();

      if (read_words != 2) {
         goto isr_done;
      }

      /* ignore invalid values */
      if (touch_data[0] == 0xFFFFFFFF || touch_data[1] == 0xFFFFFFFF)
         goto isr_done;

      /* only report pan down events along with data coordinates */
      if (ts->touch_data.state == TSC_TOUCH_DOWN) {
         /*
          * NOTE: Only use 12-bit values from X and Y, the default 16-bit value
          * can potentially cause a overflow issue in some applications.
          * 2^12 = 4096 is more than enough for the LCD panels that we are
          * using
          */

         if (hw_param->swapped_x_y) {
			   /* X and Y data are wired backwards, swap	*/

            /* Y data located in upper 16 bits, but hardware hooked up backwards, so swap */
            ts->touch_data.xData = (touch_data[0] >> 16) >> 4;
            /* X data located in lower 16 bits, but hardware hooked up backwards, so swap */
            ts->touch_data.yData = (touch_data[0] & 0xFFFF) >> 4;
         } else {
            /* Y data located in upper 16 bits */
            ts->touch_data.yData = (touch_data[0] >> 16) >> 4;
            /* X data located in lower 16 bits */
            ts->touch_data.xData = (touch_data[0] & 0xFFFF) >> 4;
         }

         /* report x y data to the Linux input subsystem */
         if (hw_param->inverted_x)
            input_report_abs(ts->input, ABS_X, BCM_TS_MAX_X - ts->touch_data.xData);
         else
            input_report_abs(ts->input, ABS_X, ts->touch_data.xData);

         if (hw_param->inverted_y)
            input_report_abs(ts->input, ABS_Y, BCM_TS_MAX_Y - ts->touch_data.yData);
         else
            input_report_abs(ts->input, ABS_Y, ts->touch_data.yData);
         
         input_report_abs(ts->input, ABS_PRESSURE, ts->touch_data.pressure);
         input_report_key(ts->input, BTN_TOUCH, 1);
         input_sync(ts->input);

         TS_DEBUG("TS: X: %x Y: %x Pen Down\n", ts->touch_data.xData,
               ts->touch_data.yData, );
      }
   }

   if (ts->touch_data.state == TSC_TOUCH_UP) {
      input_report_abs(ts->input, ABS_PRESSURE, ts->touch_data.pressure);
      input_report_key(ts->input, BTN_TOUCH, 0);
      input_sync(ts->input);

      TS_DEBUG("TS: Pen Up\n");
   }

isr_done:
   /* TODO: probably do not need these two anymore */
   ts->data_available = 0;
   ts->pen_transition = 0;

   return IRQ_HANDLED;
}

static int __init ts_probe(struct platform_device *pdev)
{
   struct ts_dev *ts;
   struct input_dev *input;
   struct tsc_param *hw_param;
   int err;

   if (cap_isPresent(CAP_TSC, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "TSC: Not supported\n");
      return -ENODEV;
   }

   ts = kzalloc(sizeof(*ts), GFP_KERNEL);
   if (!ts) {
      printk(KERN_ERR "TSC: TS data structure allocation failed\n");
      return -ENOMEM;
   }

   input = input_allocate_device();
   if (!input) {
      printk(KERN_ERR "TSC: Input data structure allocation failed\n");
      err = -ENOMEM;
      goto err_free_ts;
   }

   ts->clk = clk_get(NULL, "TSC");
   if (IS_ERR(ts->clk)) {
      printk(KERN_ERR "TSC: Unable to find TSC clock\n");
      err = PTR_ERR(ts->clk);
      goto err_free_input;
   }

   err = clk_set_rate(ts->clk, TSC_CLOCK_RATE_HZ);
   if (err < 0) {
      printk(KERN_ERR "TSC: Unable to set TSC clock\n");
      goto err_free_input;
   }

   err = clk_enable(ts->clk);
   if (err < 0) {
      printk(KERN_ERR "TSC: Unable to enable TSC clock\n");
      goto err_free_input;
   }

   /* init the touchscreen controller block */
   tscHw_Init();

   platform_set_drvdata(pdev, ts);
	ts->input = input;

   memcpy(&ts->hw_param, pdev->dev.platform_data, sizeof(ts->hw_param));
   hw_param = &ts->hw_param;

   /*
    * Set input device info
    */
   input->name = "bcmring-touchscreen";
   input->phys = "ts/input0";
   input->id.bustype = BUS_HOST;
   input->id.vendor = 0x0001;
   input->id.product = 0x1103;
   input->id.version = 0x0000;
   input->dev.parent = &pdev->dev;

   /* Enable event bits */
	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
   input_set_abs_params(input, ABS_X, 0, BCM_TS_MAX_X, 0, 0);
   input_set_abs_params(input, ABS_Y, 0, BCM_TS_MAX_Y, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 0xFFF, 0, 0);

	input_register_device(ts->input);

   /* module params can override default settings */
   if (mod_param_samp_rate != -1)
      hw_param->sample_rate = mod_param_samp_rate;
   if (mod_param_debounce != -1)
      hw_param->debounce = mod_param_debounce;
   if (mod_param_touch_timeout != -1)
      hw_param->touch_timeout = mod_param_touch_timeout;
   if (mod_param_settling != -1)
      hw_param->settling = mod_param_settling;
   if (mod_param_data_point_average != -1)
      hw_param->data_point_average = mod_param_data_point_average;

   /* now set these params in hardware */
   tscHw_SampleRateSet(hw_param->sample_rate);
   tscHw_DataThresholdSet(hw_param->fifo_threshold);
   tscHw_DebounceSet(hw_param->debounce);
   tscHw_TouchTimeoutSet(hw_param->touch_timeout);
   tscHw_SettlingSet(hw_param->settling);
   tscHw_DataPointAverageSet(hw_param->data_point_average);
   tscHw_ModeSet(hw_param->wire_mode == TSC_MODE_5WIRE ?
         TSCHW_MODE_5WIRE : TSCHW_MODE_4WIRE);

   printk(KERN_INFO "TSC: Sample Rate: %u\n", tscHw_SampleRateGet());
   printk(KERN_INFO "TSC: Fifo Threshold: %u\n", tscHw_DataThresholdGet());
   printk(KERN_INFO "TSC: Debounce: %u\n", tscHw_DebounceGet());
   printk(KERN_INFO "TSC: Touch Timeout: %u\n", tscHw_TouchTimeoutGet());
   printk(KERN_INFO "TSC: Settling: %u\n", tscHw_SettlingGet());
   printk(KERN_INFO "TSC: Data Point Average: %u\n",
         tscHw_DataPointAverageGet());
   printk(KERN_INFO "TSC: Mode: %s\n", tscHw_ModeGet() == TSCHW_MODE_5WIRE ?
         "5 wire" : "4 wire");

   /* reserve IRQ line and install the handler */
   if (request_irq(IRQ_TSC, ts_isr, 0, "Touchscreen", ts) != 0) {
      printk(KERN_ERR "TSC: Unable to allocate IRQ %d\n", IRQ_TSC);
      err = -EBUSY;
      goto fail;
   }

   tscHw_IrqPenEnable();
   tscHw_IrqDataEnable();

   tscHw_Enable();

	return 0;

fail:
   input_unregister_device(input);
   tscHw_Exit();
   clk_disable(ts->clk);
   clk_put(ts->clk);
err_free_input:
   input_free_device(input);
err_free_ts:
   kfree(ts);
	return err;
}

static int ts_remove(struct platform_device *pdev)
{
   struct ts_dev *ts = platform_get_drvdata(pdev);

   tscHw_Disable();
   tscHw_IrqPenDisable();
   tscHw_IrqDataDisable();

   free_irq(IRQ_TSC, ts);

   input_unregister_device(ts->input);
   
   tscHw_Exit();
   clk_disable(ts->clk);
   clk_put(ts->clk);

   input_free_device(ts->input);
	kfree(ts);

   return 0;
}

static struct platform_driver ts_driver = {
   .probe = ts_probe,
   .remove = ts_remove,
   .driver = {
      .name = "bcmring-touchscreen",
   }
};

static int __init ts_init(void)
{
   printk(banner);
	return platform_driver_register(&ts_driver);
}

static void __exit ts_exit(void)
{
   platform_driver_unregister(&ts_driver);
}

module_init(ts_init);
module_exit(ts_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Touchscreen Driver");
MODULE_LICENSE("GPL");
