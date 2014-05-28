/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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
#include <linux/bitops.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>

#include <linux/broadcom/keymap.h>
#include <asm/gpio.h>
#include <mach/irqs.h>
#include <mach/csp/chipcHw_inline.h>
#include <linux/broadcom/bcmring/gpio_defs.h>
#include <mach/csp/gpiomux.h>
#include <csp/kpdHw.h>

#include <mach/csp/cap.h>

static KEYPAD_DATA gKeypadData;

#define DEV_NAME		    "keypad"

#define ROW_MASK         0xF
#define MAX_ROWS         (ROW_MASK + 1)
#define COL_MASK         0xF
#define MAX_COLS         (COL_MASK + 1)
#define MAX_SCANCODES    (MAX_ROWS * MAX_COLS)
#define SCANCODE_ROW(s)  (((s) >> 4) & ROW_MASK)
#define SCANCODE_COL(s)  ((s) & COL_MASK)

/*
 * Theoretically even size of 1 is good enough since the system resumes in ms
 * and typical debounce time is 32 ~ 64 ms
 */
#define MAX_KFIFO_SIZE 8

#define KEYOUT_START   52
#define KEYIN_START    43
#define GPIO_KEY_MAX   18

#define DEFAULT_SAMPLEFREQ (9 * 1000)

#define ACTIVE_DEBOUNCE (200)
#define SUSPEND_DEBOUNCE (15)

typedef unsigned int scancode_t;
typedef unsigned char keycode_t;
typedef unsigned short KBD_ROW;
int gpio_keyreserve(unsigned int *reserveKeys);

/*
 * To queue key events when the system is in suspend mode
 */
typedef struct
{
   KEYMAP keymap;
   unsigned int down;
} KQUEUE;

/*
 * Key FIFO control used during system suspend/resume
 */
typedef struct
{
   /* current FIFO size */
   volatile unsigned int q_cnt;

   KQUEUE queue[MAX_KFIFO_SIZE];
} KFIFO_CTRL;

typedef struct
{
   struct input_dev *input_dev;

   unsigned num_row;
   unsigned num_col;
   unsigned num_scancode;

   /* flag to indicate if the system is suspended or not */
   atomic_t suspended;

   /* key FIFO control used during system suspend/resume */
   KFIFO_CTRL kfifo_ctrl;
   

   /* use scancode as an index to look up the corresponding keycode */
   keycode_t keycode[MAX_SCANCODES];

   /*
    * The following bitmask contains one bit for each scancode that was down
    * the last time we were called
    */
   unsigned long prevDown[(MAX_SCANCODES + BITS_PER_LONG - 1) / BITS_PER_LONG];

   /* lock that protects against access to the prevDown data structure */
   spinlock_t status_lock;
} CBLK;

static const __devinitconst char gBanner[] = KERN_INFO "BCMRING Keypad Input Driver: 1.00\n";
static CBLK gCblk;

static unsigned int *gPower_off_row_scancode_set;
static unsigned int *gPower_off_col_scancode_set;
static int gPower_off_enabled = 0;

static int gPanicked = 0;

static int keypad_panic(struct notifier_block *this, unsigned long event, void *ptr)
{
	gPanicked = 1;
	return NOTIFY_DONE;
}
static struct notifier_block panic_block = {
	.notifier_call = keypad_panic,
};

/* Holds the keypad matrix values */
static uint16_t kpdMatrix[kpdHw_MATRIX_STATE_SIZE];

static void keypad_scan(CBLK *cblkp, short *matrixp)
{
   keycode_t keycode;
   scancode_t scancode;

   for (scancode = 0; scancode < MAX_SCANCODES; scancode++) {
      if ((keycode = cblkp->keycode[scancode]) > 0) {
         int r = SCANCODE_ROW(scancode);
         int c = SCANCODE_COL(scancode);
         int down = kpdHw_IsKeyDown(matrixp, r, c);

         /*
          * System is in suspend mode. This interrupt will wake up the
          * system, but the input susbsystem is not ready to process key
          * events at this point. Queue the key events and process it at the
          * resume function
          */
         if (atomic_read(&cblkp->suspended))
         {
            KFIFO_CTRL *kfifo = &cblkp->kfifo_ctrl;

            if (kfifo->q_cnt < MAX_KFIFO_SIZE)
            {
               kfifo->queue[kfifo->q_cnt].keymap.scancode = scancode;
               kfifo->queue[kfifo->q_cnt].keymap.keycode = keycode;
               kfifo->queue[kfifo->q_cnt].down = down;
               kfifo->q_cnt++;
            }
         }
         else /* normal operation, report the key to the input subsystem */
         {
            spin_lock(&cblkp->status_lock);

            /* if state has changed */
            if (test_bit(scancode, cblkp->prevDown) != down) {
               /* report it to the Linux subsystem */
               input_report_key(cblkp->input_dev, keycode, down);

               /* update the prevDown */
               change_bit(scancode, cblkp->prevDown);
            }

            spin_unlock(&cblkp->status_lock);
         }
      }
   }
}

#include <linux/reboot.h>
static inline void keypad_check_power_off(short *matrixp)
{
   if (gPower_off_enabled)
   {
      int i;
      for (i = 0; i < gKeypadData.pwroff_cnt; i++)
      {
         if (!kpdHw_IsKeyDown(matrixp, gPower_off_row_scancode_set[i], gPower_off_col_scancode_set[i]))
         {
            return;
         }
      }
      printk(KERN_ERR"Keypad: Power-off requested\n");

      /* Shutdown system if not already halted.
       * Reboot system if already halted.
       */
      if (gPanicked)
      {
         emergency_restart();
      }
      else if ((system_state != SYSTEM_HALT) && (system_state != SYSTEM_POWER_OFF))
      {
         orderly_poweroff(1);
      }
      else
      {
         kernel_restart(NULL);
      }
   }
}

static irqreturn_t keypad_interrupt(int irq, void *dev_id)
{
   CBLK *cblkp = (CBLK *)dev_id;
   kpdHw_DisableIrq();
   kpdHw_GetMatrixState(kpdMatrix);
   keypad_check_power_off(kpdMatrix);
   keypad_scan(cblkp, kpdMatrix);
   kpdHw_EnableIrq();

   return IRQ_HANDLED;
}

static int keypad_event(struct input_dev *dev, unsigned int type,
      unsigned int code, int value)
{
   /* TODO: future LED events */
   return 0;
}

/* ISR to handle interrupt from keyin GPIOs */
static irqreturn_t gpio_key_isr(int irq, void *data)
{
   int gpio_val;
   GPIOMAP *gpiomap;

   gpiomap = data;
	gpio_val = gpio_get_value( gpiomap->gpio );

	if (gpiomap->invert)
	{
		gpio_val = !gpio_val;
	}
	input_report_key( gCblk.input_dev, gpiomap->keycode , gpio_val );

   return IRQ_HANDLED;
}

/* process the keymap_settings and calculate which gpio lines are reuired. */
int gpio_keyreserve(unsigned int *reserveKeys)
{
   int i;
   unsigned int gpioInOutPool[gKeypadData.keymap_cnt * 2];
   for (i = 0; i < gKeypadData.keymap_cnt; i++) {
      /*
       * traverse keymap settings array; calculate and record the gpio number
       * for each element. (SYNTAX: 0X[KEYOUT][KEYIN])
       */
      gpioInOutPool[2 * i] = (gKeypadData.keymap[i].scancode & 0xF) + KEYIN_START;
      gpioInOutPool[2 * i + 1] = ((gKeypadData.keymap[i].scancode >>4)& 0xF) +
         KEYOUT_START;
   }
   for (i = 0; i < gKeypadData.keymap_cnt * 2; i++) {
      /*
       * reserveKeys is a 18 elements array. The index of non-zero elements
       * plus KEYIN_START will be the gpio lines required
       */
      reserveKeys[gpioInOutPool[i] - KEYIN_START] = 1;
   }
   return 0;
}

static int __devinit keypad_probe(struct platform_device *pdev)
{
   int i, rc;
   gpiomux_rc_e status;
   CBLK *cblkp = &gCblk;

   if (cap_isPresent(CAP_KEYPAD, 0) != CAP_PRESENT) {
      printk(KERN_WARNING "Keypad: Not supported\n");
      return -ENODEV;
   }

   printk(gBanner);

   /* get the platform data */
   if (pdev->dev.platform_data == NULL) {
      printk(KERN_ERR "Keypad: Platform data (KEYPAD_DATA) not set properly\n");
      return -ENODEV;
   }
   memcpy(&gKeypadData, pdev->dev.platform_data, sizeof(gKeypadData));
   memset(cblkp, 0, sizeof(CBLK));
   spin_lock_init(&cblkp->status_lock);

   cblkp->input_dev = input_allocate_device();
   if (cblkp->input_dev == NULL) {
      printk(KERN_ERR "Keypad: input_allocate_device failed\n");
      return -ENOMEM;
   }

   set_bit(EV_KEY, cblkp->input_dev->evbit);

	cblkp->input_dev->name = "keypad";
	cblkp->input_dev->phys = "keypad/input0";
	cblkp->input_dev->id.bustype = BUS_HOST;
	cblkp->input_dev->id.vendor = 0x0001;
	cblkp->input_dev->id.product = 0x0001;
	cblkp->input_dev->id.version = 0x0100;

   cblkp->input_dev->keycode = cblkp->keycode;
   cblkp->input_dev->keycodesize = sizeof(cblkp->keycode[0]);
   cblkp->input_dev->keycodemax = sizeof(cblkp->keycode) /
      sizeof(cblkp->keycode[0]);

   cblkp->input_dev->event = keypad_event;

   rc = input_register_device(cblkp->input_dev);
   if (rc != 0) {
      printk(KERN_ERR "Keypad: input_register_device failed\n");
      goto free_dev;
   }

   /* tell the input subsystem about GPIO based keys */
   if (gKeypadData.gpiomap) {
      for (i = 0; i < gKeypadData.gpiomap_cnt; i++)
      {
         /* Handle fake GPIO key */
         if ( gKeypadData.gpiomap[i].gpio == KEYMAP_FAKE_GPIO )
         {
            /* Register the keycode into the input subsystem to it is reported */
            input_set_capability (cblkp->input_dev, EV_KEY, gKeypadData.gpiomap[i].keycode);
            continue;
         }

         status = gpio_request( gKeypadData.gpiomap[i].gpio, "GPIO KEY" );
         if (status != gpiomux_rc_SUCCESS)
         {
            printk(KERN_ERR "Keypad: GPIO request failed at pin=%d", gKeypadData.gpiomap[i].gpio);
         }
         else
         {
            /* Register the keycode into the input subsystem to it is reported */
            input_set_capability (cblkp->input_dev, EV_KEY, gKeypadData.gpiomap[i].keycode);

            /* Set direction to input */
            gpio_direction_input( gKeypadData.gpiomap[i].gpio );

            /* Enable interrupt */
            GpioHw_IrqEnable( gKeypadData.gpiomap[i].gpio );

            if (request_irq(gpio_to_irq( gKeypadData.gpiomap[i].gpio ), gpio_key_isr, (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING), "GPIO KEY", &gKeypadData.gpiomap[i]) < 0)
            {
            printk(KERN_ERR "Keypad: request_irq %u failed\n", gpio_to_irq( gKeypadData.gpiomap[i].gpio ));
            }
         }
      }
   }

   /* tell the input subsystem about our default keymap */
   if (gKeypadData.keymap != NULL) {
      int  i;
      for (i = 0; i < gKeypadData.keymap_cnt; i++) {
         cblkp->input_dev->setkeycode(cblkp->input_dev,
               gKeypadData.keymap[i].scancode, gKeypadData.keymap[i].keycode);
      }
   }

   if (gKeypadData.pwroff) {
      /* lookup scancodes */
      if (gKeypadData.pwroff_cnt)
      {
         int i, j;
         gPower_off_enabled = 1;
         gPower_off_row_scancode_set =
            kmalloc(gKeypadData.pwroff_cnt * sizeof(*gPower_off_row_scancode_set), GFP_KERNEL);
         gPower_off_col_scancode_set =
            kmalloc(gKeypadData.pwroff_cnt * sizeof(*gPower_off_col_scancode_set), GFP_KERNEL);

         if (gPower_off_row_scancode_set == NULL || gPower_off_col_scancode_set == NULL) {
            rc = -ENOMEM;
            goto free_kdata;
         }

         for (i = 0; i < gKeypadData.pwroff_cnt; i++)
         {
            for (j = 0; j < gKeypadData.keymap_cnt; j++)
            {
               if (gKeypadData.keymap[j].keycode == gKeypadData.pwroff[i])
               {
                  gPower_off_row_scancode_set[i] = SCANCODE_ROW(gKeypadData.keymap[j].scancode);
                  gPower_off_col_scancode_set[i] = SCANCODE_COL(gKeypadData.keymap[j].scancode);
                  break;
               }
            }
            if (j >= gKeypadData.keymap_cnt)
            {
               printk(KERN_ERR "Keypad: lookup scancode for (%c) failed, power-off keyset disabled\n",
                     gKeypadData.pwroff[i]);
               gPower_off_enabled = 0;
               break;
            }
         }
      }
      if (gPower_off_enabled)
      {
         char power_off_keys_str[gKeypadData.pwroff_cnt * 2];
         int i;
         for (i = 0; i < gKeypadData.pwroff_cnt; i++)
         {
            power_off_keys_str[i * 2] = gKeypadData.pwroff[i];
            power_off_keys_str[i * 2 + 1] = '-';
         }
         power_off_keys_str[gKeypadData.pwroff_cnt * 2 - 1] = '\0';
         printk(KERN_INFO "Keypad: power-off keyset: %s\n", power_off_keys_str);

         atomic_notifier_chain_register(&panic_notifier_list, &panic_block);
      }
   }

   /* register an interrupt handler to deal with the keypad events */
   rc = request_irq(IRQ_KEYC, keypad_interrupt, 0, DEV_NAME, (void *)cblkp);
   if (rc != 0) {
      printk(KERN_ERR "Keypad: request_irq for IRQ_KEYC(%d) failed: %d\n",
            IRQ_KEYC, rc);
      goto unregister_dev;
   }

   /* get numbers of rows, cols, and scancodes */
   cblkp->num_row = kpdHw_GetMatrixRows();
   cblkp->num_col = kpdHw_GetMatrixCols();
   cblkp->num_scancode = cblkp->num_row * cblkp->num_col;

   /* reserver GPIOs */
   {
      unsigned int gpioInOutReserve[GPIO_KEY_MAX];
      memset(gpioInOutReserve, 0, sizeof(gpioInOutReserve));
      /*call gpio_keyreserve to determine which gpio lines are needed by this keymap. */
      gpio_keyreserve(gpioInOutReserve);
      /*request gpio lines */
      for (i = 0; i < GPIO_KEY_MAX; i++) {
         if (gpioInOutReserve[i]==1){
            status = gpiomux_request(i+KEYIN_START, chipcHw_GPIO_FUNCTION_KEYPAD,"Keypad");
            if (status != gpiomux_rc_SUCCESS) {
               printk(KERN_ERR "Keypad: GPIO MUX request failed at pin=%d", i);
            }
         }
      }
   }

   chipcHw_busInterfaceClockEnable(chipcHw_REG_BUS_CLOCK_LDK);

   kpdHw_DisableScan();
   kpdHw_SetMode(kpdHw_MODE_DELTA);
   kpdHw_SetKeyoutMode(kpdHw_KEYOUT_OPENDRAIN);
   kpdHw_SetDebounce(ACTIVE_DEBOUNCE);
   kpdHw_SetSampleFreq(DEFAULT_SAMPLEFREQ);
   kpdHw_EnableScan();
   kpdHw_EnableIrq();

   platform_set_drvdata(pdev, cblkp);

   printk(KERN_NOTICE "Keypad: rows=%u cols=%u scancode=%u Debounce count=%u "
         "Sampling freq=%u\n", cblkp->num_row, cblkp->num_col,
         cblkp->num_scancode, kpdHw_GetDebounce(), kpdHw_GetSampleFreq());

   return 0;

free_kdata:
   kfree(gPower_off_row_scancode_set);
   kfree(gPower_off_col_scancode_set);

unregister_dev:
   input_unregister_device(cblkp->input_dev);

free_dev:
   input_free_device(cblkp->input_dev);

   return rc;
}

static int keypad_remove(struct platform_device *pdev)
{
   unsigned int i;
   CBLK *cblkp = platform_get_drvdata(pdev);

   kpdHw_DisableIrq();
   kpdHw_DisableScan();

   disable_irq(IRQ_KEYC);
   free_irq(IRQ_KEYC, cblkp);

   /* free GPIOs */
   {
      unsigned int gpioInOutReserve[GPIO_KEY_MAX];
      /*call gpio_keyreserve to determine which gpio lines are needed by this keymap. */
      gpio_keyreserve(gpioInOutReserve);
      /*free gpio lines*/
      for (i = 0; i < GPIO_KEY_MAX; i++) {
         if (gpioInOutReserve[i]==1) {
            gpiomux_free(i+KEYIN_START);
         }
      }
   }

   kfree(gPower_off_row_scancode_set);
   kfree(gPower_off_col_scancode_set);

   input_unregister_device(cblkp->input_dev);
   input_free_device(cblkp->input_dev);

   platform_set_drvdata(pdev, NULL);

   return 0;
}

#ifdef CONFIG_PM
static int keypad_suspend(struct platform_device *pdev, pm_message_t state)
{
   CBLK *cblkp = platform_get_drvdata(pdev);

   atomic_set(&cblkp->suspended, 1);
   
   /* Switch to shorter deboune value to maintain fast key response during suspend.
    * The Keypad controller sampling clock is derived from ARM clock, and during
    * suspend, the ARM clock may be slowed down.
    */
   kpdHw_SetDebounce(SUSPEND_DEBOUNCE);
   return 0;
}

static int keypad_resume(struct platform_device *pdev)
{
   unsigned int i;
   unsigned long flags;
   CBLK *cblkp = platform_get_drvdata(pdev);
   KFIFO_CTRL *kfifo = &cblkp->kfifo_ctrl;

   /* Restore keypad debounce value.
    */
   kpdHw_SetDebounce(ACTIVE_DEBOUNCE);
   
   /*
    * Need to protect the prev_status bitmask as it's also modified in the
    * ISR
    */
   spin_lock_irqsave(&cblkp->status_lock, flags);

   atomic_set(&cblkp->suspended, 0);
   for (i = 0; i < kfifo->q_cnt; i++)
   {
      KQUEUE *q = &kfifo->queue[i];

      if (test_bit(q->keymap.scancode, cblkp->prevDown) != q->down)
      {
         /* report it to the Linux subsystem */
         input_report_key(cblkp->input_dev, q->keymap.keycode, q->down);
         
         /* update the prevDown */
         change_bit(q->keymap.scancode, cblkp->prevDown);
      }
   }
   spin_unlock_irqrestore(&cblkp->status_lock, flags);

   kfifo->q_cnt = 0;
   
   return 0;
}
#else
#define keypad_suspend   NULL
#define keypad_resume    NULL
#endif

static struct platform_driver keypad_driver = {
   .driver = {
      .name = "bcmring-keypad",
      .owner = THIS_MODULE,
   },
   .probe = keypad_probe,
   .remove = keypad_remove,
   .suspend = keypad_suspend,
   .resume = keypad_resume,
};

static int __init keypad_init(void)
{
   return platform_driver_register(&keypad_driver);
}

static void __exit keypad_exit(void)
{
   platform_driver_unregister(&keypad_driver);
}

module_init(keypad_init);
module_exit(keypad_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom BCMRING Keypad Input Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
