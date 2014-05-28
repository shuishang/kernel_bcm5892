/*****************************************************************************
* Copyright 2009-2010 Broadcom Corporation.  All rights reserved.
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
 * I2C Touchscreen Driver
 *
 * The touchscreen controller is a slave on the I2C bus and is assigned an 
 * address. This driver sets up the SOC as a I2C master and reads the slave 
 * address to obtain touch information.
 *
 * The driver uses the Linux input subsystem. User can access the touchscreen
 * data through the /dev/input/eventX node
 *
 */

/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/broadcom/gpio_irq.h>
#include <asm/gpio.h>
#include <linux/wait.h>
#include <linux/broadcom/timer.h>
#include <linux/signal.h>
#include <linux/kthread.h>
#include <linux/syscalls.h>

#include <linux/hrtimer.h>
#include <asm/io.h>

#include <linux/broadcom/i2c_ts.h>

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
typedef struct
{
   int x1;
   int y1;
   int x2;
   int y2;
   int gesture;
   int num_fingers;
   int version;
   int idle;    
   int timeout;
} t_i2c_touch_data;    

typedef struct
{
   struct input_dev        *p_input;
   struct t_i2c_touch_data *p_data;
} t_i2c_input;   

struct i2c_priv_data 
{
   struct i2c_client *p_i2c_client;
};   

/* Driver upgrade changes ... */
struct i2c_state 
{
   struct i2c_client *p_i2c_client;
};

typedef enum
{
   TSC_TOUCH_DOWN = 0, /* down state */
   TSC_TOUCH_UP        /* up state/event */ 
} touch_state;

/* Error checking ... */
typedef enum
{
   kErrorX1         = 0x1,
   kErrorY1         = 0x2,
   kErrorX2         = 0x4 ,
   kErrorY2         = 0x8,
   kErrorNumFingers = 0x10,
   kErrorTouchState = 0x20,
   kErrorVersion    = 0x40,
   kErrorTimeout    = 0x80,
   kErrorIdle       = 0x100,
} touch_errors;

#define GPIO_I2C_RESET_DELAY_MSECS      10
#define GPIO_RESET_PIN                  16
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
#define LEGACY_DRIVER                   0
#else
#define LEGACY_DRIVER                   0
#endif
#define INPUT_EVENT_PRESSURE            0xff
#define INPUT_EVENT_NO_PRESSURE         0
#define I2C_TS_DRIVER_DO_RESET_CNTRLR   1
#define I2C_TS_DRIVER_DONT_RESET_CNTRLR 0
#define MAX_NUMBER_READ_ERRORS          5

#define USE_ECLAIR_MULTITOUCH           1

/* The size of the finger contact area in millimeters. */
#define I2C_TS_DRIVER_BLOB_SIZE         8

/* Input filtering */
/* Must see more than COUNT_THRESH points within DISTANCE_THRESH pixels of each other, 
 * within TIME_THRESH milliseconds, for an input event to be generated.
 */
#if CONFIG_TOUCHSCREEN_CY8CTST120_I2C_FILTER
#define ENABLE_FILTER                   1
#define DISTANCE_THRESH                 75
#define TIME_THRESH                     500
#define COUNT_THRESH                    1
#endif  

/* Debugging ... */
#define I2C_TS_DRIVER_SHOW_ALL_EVENTS   0
#define I2C_TS_DRIVER_SHOW_ERR_EVENTS   0
#define I2C_TS_DRIVER_SHOW_OK_EVENTS    0
#define I2C_TS_DRIVER_SHOW_RAW_EVENTS   0
#define I2C_TS_DRIVER_SHOW_INPUT_EVENTS 0
#define I2C_TS_DRIVER_SHOW_EVENT_COUNT  0

#define ENABLE_QA_TESTING                  1

/* ---- Private Variables ------------------------------------------------ */
atomic_t g_atomic_irqs_rxd = ATOMIC_INIT(0);

const struct I2C_TS_t        *gp_i2c_ts        = NULL;

static   int g_kthread_pid    = 0;

static const unsigned short normal_i2c[] = 
{
   I2C_TS_DRIVER_SLAVE_NUMBER_0x20, I2C_TS_DRIVER_SLAVE_NUMBER_0x38, I2C_CLIENT_END
};

static unsigned short ignore[2] = {I2C_CLIENT_END, I2C_CLIENT_END};
static struct i2c_client_address_data addr_data = 
{
	normal_i2c,
	ignore,
	ignore,
};

static touch_state g_touch_state                     = TSC_TOUCH_UP;
static int g_last_x                                  = 0;
static int g_last_y                                  = 0;
static timer_tick_count_t g_last_timestamp           = 0;
static char        *gp_buffer                        = NULL; 
static struct      i2c_priv_data *gp_i2c_driver_priv = NULL;
static struct      input_dev     *gp_input_dev       = NULL;

static DECLARE_WAIT_QUEUE_HEAD(g_event_waitqueue);

static t_i2c_touch_data g_curr_touch_data;
static t_i2c_touch_data g_prev_touch_data; 

static unsigned long g_num_good_events   = 0;
static unsigned long g_num_bad_events    = 0;
static unsigned long g_num_gen_up_events = 0;
static int           g_default_reset_pin = GPIO_RESET_PIN;
static int           g_num_read_errors   = 0;
static int           g_num_driver_errors = 0;
static int           g_timeout           = 0;
static int           g_idle              = 0;
static int           g_low_power         = 0;
static int           g_timeout_changed   = 0;
static int           g_idle_changed      = 0;
static int           g_low_power_changed = 0;

static int           g_found_slave_addr  = 0;

static int g_num_good_events_per_touch = 0;
static int g_num_bad_events_per_touch  = 0;

/* Needed for multitouch.                                          *
 * The surface X coordinate of the center of the touching ellipse  */
static int           g_blob_size         = 0;

#if CONFIG_TOUCHSCREEN_CY8CTST120_I2C_FILTER
static int g_last_count                              = 0;

static int mod_param_enable_filter = ENABLE_FILTER;
module_param(mod_param_enable_filter, int, 0644);

static int mod_param_distance_thresh = DISTANCE_THRESH;
module_param(mod_param_distance_thresh, int, 0644);

static int mod_param_time_thresh = TIME_THRESH;
module_param(mod_param_time_thresh, int, 0644);

static int mod_param_count_thresh = COUNT_THRESH;
module_param(mod_param_count_thresh, int, 0644);
#endif

static int mod_param_debug = (I2C_TS_DRIVER_SHOW_RAW_EVENTS << 1) & I2C_TS_DRIVER_SHOW_INPUT_EVENTS;
module_param(mod_param_debug, int, 0644);

/* The period to wait before entering low power mode. */
static int mod_param_timeout = 0;
module_param(mod_param_timeout, int, 0644);

/* The duration the controller is idle when in low power mode. */
static int mod_param_idle = 0;
module_param(mod_param_idle, int, 0644);

/* Allows the controller to enter low power mode when timeout expires. */
static int mod_param_low_power = 0;
module_param(mod_param_low_power, int, 0644);

#if ENABLE_QA_TESTING
static int mod_param_test_enable = 0;
module_param(mod_param_test_enable, int, 0644);

static int mod_param_test_xy = 0;
module_param(mod_param_test_xy, int, 0644);

static int mod_param_test_num_touches = 5;
module_param(mod_param_test_num_touches, int, 0644);

static int mod_param_test_delay = 50;
module_param(mod_param_test_delay, int, 0644);
#endif

/* ---- Private Function Prototypes -------------------------------------- */
static int  i2c_ts_driver_read                   (void);
static int  i2c_ts_driver_check_touch_info       (void);
static void i2c_ts_driver_send_touch_info        (void);
static void i2c_ts_driver_send_multitouch_info   (void);
static void i2c_ts_driver_show_events            (int err_no);
static int  i2c_ts_driver_reset_slave            (int is_default_reset);
static void i2c_ts_driver_remap_layout           (void);
static int  i2c_ts_driver_kthread                (void *unused);
static void i2c_ts_driver_handle_i2c_error       (int rc);
static int  i2c_ts_driver_write                  (int length);
static int  i2c_ts_driver_check_mod_params       (void);
#if ENABLE_TESTING
static void i2c_ts_driver_test_mode              (void);
#endif

/* ---- Functions -------------------------------------------------------- */

#ifdef CONFIG_PM
/* In preparation for implementing PM_SUSPEND_STANDBY. */
static int i2c_ts_suspend_driver(struct i2c_client *p_client, pm_message_t mesg)
{  /* Can put it into deep sleep only if the slave can be reset to bring it out. */
   int rc = 0;
   if (gp_i2c_ts->is_resetable)
   {
      gp_buffer[0] = gp_i2c_ts->timeout_idx;
      gp_buffer[1] = 0;   
      rc = i2c_ts_driver_write(2);
   }   
   disable_irq(gpio_to_irq(gp_i2c_ts->gpio_irq_pin));
   return 0;
}

static int i2c_ts_resume_driver(struct i2c_client *p_client)
{
   if (gp_i2c_ts->is_resetable)
   {
      i2c_ts_driver_reset_slave(0);
   }   
   enable_irq(gpio_to_irq(gp_i2c_ts->gpio_irq_pin));
   return 0;
}
#endif

#if ENABLE_QA_TESTING
/* QA wants to generate touch events by entering console input.
 * 1. Go to driver directory
 *    "cd /sys/module/cy8ctst120_i2c_ts/parameters"
 * 2. Set mod_param_test_enable to 1 
 *    "echo 1 > mod_param_test_enable"
 * 3. Enter a touch location 0xXXXYYY
 *    "echo 0x36d661 > mod_param_test_xy"
 * (Optional)
 * 4. Change number of touches to 8
 *    "echo 8 > mod_param_test_num_touches"
 * 5. Change delay between touches to 60 mSecs
 *    "echo 60 > mod_param_test_delay"
 */
static void i2c_ts_driver_test_mode(void)
{  
   int x;
   int y;
   int i;
   
   if (mod_param_test_enable == 0 || 
       mod_param_test_xy     == 0 || 
       mod_param_test_delay  == 0 ||
       mod_param_test_num_touches == 0)
{
      return;
   }

   x = (mod_param_test_xy & 0xfff000) >> 12;
   y = mod_param_test_xy & 0xfff;
   
   if (mod_param_debug > 0)
   {
      printk("test x: 0x%x y: 0x%x number touches: %d delay %d(mSecs)\n", 
             x, y, mod_param_test_num_touches, mod_param_test_delay);
   }   
   g_touch_state = TSC_TOUCH_DOWN;
   g_curr_touch_data.x1 = x;
   g_curr_touch_data.y1 = y;
   g_curr_touch_data.num_fingers = 1;
   
   for (i = 0; i < mod_param_test_num_touches; i++)
   {
      i2c_ts_driver_send_touch_info();
      msleep(mod_param_test_delay);       
   }
   
   g_curr_touch_data.num_fingers = 0;
   g_touch_state = TSC_TOUCH_UP;
   i2c_ts_driver_send_touch_info();
   
   /* Clear the test point so it does not continue to run. */
   mod_param_test_xy = 0;         
}
#endif


static int i2c_ts_driver_check_mod_params(void)
{    
   int rc = 0;
   
   if (gp_i2c_ts == NULL)
   {
      return -1;
   }
   
   if (g_timeout != mod_param_timeout)
   {
      g_timeout_changed = 1;
   }

   if (g_idle != mod_param_idle)
   {
      g_idle_changed = 1;
   }
   
   if (g_low_power != mod_param_low_power)
   {
      g_low_power_changed = 1;
   }
   
   if (g_timeout_changed > 0)
   {  /* User wants to change the timeout period. */ 
      g_timeout_changed = 0;
      if (mod_param_timeout < 0 || mod_param_timeout > 0xff ||
          gp_i2c_ts->timeout_idx == 0)
      {
         printk("%s: timeout out of bounds or cannot be changed\n", I2C_TS_DRIVER_NAME);
         mod_param_timeout = g_timeout;
      }
      else      
      {
         g_timeout = mod_param_timeout;
         gp_buffer[0] = gp_i2c_ts->timeout_idx;
         gp_buffer[1] = g_timeout;
   
         if (((rc = i2c_ts_driver_write(2)) > 0) && mod_param_debug)
         {
            printk(KERN_INFO "%s set timeout to 0x%x\n", 
                   I2C_TS_DRIVER_NAME, gp_buffer[1]);
         }         
      }   
   }

   if (g_idle_changed > 0)
   {  /* User wants to change the idle period. */ 
      g_idle_changed = 0;
      if (mod_param_idle < 0 || mod_param_idle > 0xff ||
          gp_i2c_ts->idle_idx == 0)
      {
         printk("%s: idle out of bounds or cannot be changed\n", I2C_TS_DRIVER_NAME);
         mod_param_idle = g_idle;
      }      
      else
      {
         g_idle = mod_param_idle;
         gp_buffer[0] = gp_i2c_ts->idle_idx;
         gp_buffer[1] = g_idle;
   
         if (((rc = i2c_ts_driver_write(2)) > 0) && mod_param_debug)
         {
            printk(KERN_INFO "%s set idle to 0x%x\n", 
                   I2C_TS_DRIVER_NAME, gp_buffer[1]);
         }      
      }   
   }

   if (g_low_power_changed > 0)
   {  /* User wants to change the auto low power mode.  This is entered after *
       * the timeout expires.                                                 */ 
      g_low_power_changed = 0;
      if (mod_param_low_power < 0 || mod_param_low_power > 1 ||
          gp_i2c_ts->auto_power_idx == 0)
      {
         printk("%s: low_power out of bounds or cannot be changed\n", I2C_TS_DRIVER_NAME);
         mod_param_low_power = g_low_power;
      }      
      else
      {
         g_low_power = mod_param_low_power;
         gp_buffer[0] = gp_i2c_ts->auto_power_idx;
         gp_buffer[1] = g_low_power;
   
         if (((rc = i2c_ts_driver_write(2)) > 0) && mod_param_debug)
         {
            printk(KERN_INFO "%s set auto low power to 0x%x\n", 
                   I2C_TS_DRIVER_NAME, gp_buffer[1]);
      }   
   }   
}

   if (rc < 0)
   {
      printk("%s error detected when writing slave settings %d\n",
             I2C_TS_DRIVER_NAME, rc);
      return rc;
   }
   else
   {
      return 0;
   }   
}

static irqreturn_t i2c_ts_driver_isr(int irq, void *dev_id)
{
   /* The touch screen controller is connected to the mainboard via the I2C bus
    * and two GPIO lines. Taking GPIO 16 low resets the controller while GPIO 35
    * is used an active low interrupt to inform the main CPU that a touch event
    * has occurred. Reading the touch controller via the I2C bus causes the
    * interrupt line to return to the high state.
    * 
    * Cannot read the I2C from the ISR so it signals the thread that it is time
    * to read the I2C bus to obtain the touch information.
    */              
   atomic_inc(&g_atomic_irqs_rxd);
   
   if (atomic_read(&g_atomic_irqs_rxd) == 1)
   {
      wake_up(&g_event_waitqueue);
   }   
   return IRQ_HANDLED;
}

/*
 * This thread cannot simply wait for an interrupt and kick off a read of the
 * I2C bus. Too often touch up events are missed and the controller is left 
 * thinking a finger is still in contact with the touch screen. It regularly
 * wakes up and checks that if the touched state is still down it will send a
 * touch up event.
 */
static int i2c_ts_driver_kthread(void *unused)
{
   int rc = 0;
   long unsigned int my_jiffies, timeout_jiffies;
   wait_queue_head_t wait_queue;
   init_waitqueue_head (&wait_queue);

   daemonize("i2c-ts-driver");
    
   /* Request delivery of SIGKILL */
   allow_signal(SIGKILL);

   timeout_jiffies = msecs_to_jiffies(gp_i2c_ts->timer_wait);  

   for (;;) 
   {
      /* Relinquish the processor until the event occurs */      
      set_current_state(TASK_INTERRUPTIBLE);
      
      if (signal_pending(current)) 
      {  /* Die if I receive SIGKILL */
         printk("%s driver received SIGKILL\n", I2C_TS_DRIVER_NAME); 
         break;
      }         
            
      if (atomic_read(&g_atomic_irqs_rxd) == 0)
      {  /* Nothing to read, wait a while ... */
         my_jiffies = wait_event_timeout(g_event_waitqueue,       /* the waitqueue to wait on */
                                         atomic_read(&g_atomic_irqs_rxd), /* condition to check */
                                         timeout_jiffies);        /* timeout in jiffies */
#if ENABLE_QA_TESTING                                                   
         i2c_ts_driver_test_mode();
#endif         
         if (my_jiffies > 0)    /* timeout, in jiffies */
         { /* Woken before timer expired. */
               
            if (i2c_ts_driver_read() > 0)            
            {
               i2c_ts_driver_check_touch_info();
            }   

            if (atomic_read(&g_atomic_irqs_rxd) > 0)
            {   
               atomic_dec(&g_atomic_irqs_rxd);            
            }  
         } 
         else if (my_jiffies < 0)
         {         
            printk("i2c-driver kernel thread ended!\n");
            break;
         }
         else 
         {  /* Timed out, check if touch state changed. */       
            if (g_touch_state != TSC_TOUCH_UP)
            {  /* Timed out and controller thinks finger is still on - generate 
                * finger up event. 
                */
               if (mod_param_debug & 0x1 || I2C_TS_DRIVER_SHOW_ALL_EVENTS || I2C_TS_DRIVER_SHOW_OK_EVENTS)
               {
                  printk("generate touch up event (finger off)\n");
               }   
               g_num_gen_up_events++;
               g_touch_state = TSC_TOUCH_UP;   
               i2c_ts_driver_send_touch_info();
            }
            
         }
      }
      else       
      {  /* Perform a read immediately. */
         
         if (i2c_ts_driver_read() > 0)            
         {
            i2c_ts_driver_check_touch_info();
         }   

         if (atomic_read(&g_atomic_irqs_rxd) > 0)
         {   
            atomic_dec(&g_atomic_irqs_rxd);            
         }   
      }
       
    }

    return rc;    
}

int i2c_ts_driver_read(void)
{
   int rc = 0;
   int i  = 0;
   
   if (gp_i2c_driver_priv == NULL ||
       gp_i2c_driver_priv->p_i2c_client == NULL)
   {
      printk("i2c_driver_read() gp_i2c_driver_priv->p_i2c_client == NULL\n");
      return -1;
   }

   rc = i2c_master_recv(gp_i2c_driver_priv->p_i2c_client, 
                        gp_buffer, 
                        gp_i2c_ts->num_bytes_to_read);

   if (rc != gp_i2c_ts->num_bytes_to_read)                           
   {
      printk("%s i2c_ts_driver_read() failed %d\n", I2C_TS_DRIVER_NAME, rc);
      g_num_read_errors++;
      i2c_ts_driver_handle_i2c_error(rc);   
      return rc;
   }
    
   g_num_read_errors = 0;  
   
   g_curr_touch_data.x1 = (gp_buffer[gp_i2c_ts->x1_hi_idx] << 8) | gp_buffer[gp_i2c_ts->x1_lo_idx];
   g_curr_touch_data.y1 = (gp_buffer[gp_i2c_ts->y1_hi_idx] << 8) | gp_buffer[gp_i2c_ts->y1_lo_idx];    
   
   if (gp_i2c_ts->is_multi_touch)
   {
      if (gp_i2c_ts->x2_hi_idx >= 0 && gp_i2c_ts->x2_lo_idx >= 0)
      {
         g_curr_touch_data.x2 = (gp_buffer[gp_i2c_ts->x2_hi_idx] << 8) | 
                                gp_buffer[gp_i2c_ts->x2_lo_idx];
      }         
      else
      {
         g_curr_touch_data.x2 = -1;
      }   
         
      if (gp_i2c_ts->y2_hi_idx >= 0 && gp_i2c_ts->y2_lo_idx >= 0)
      {
         g_curr_touch_data.y2 = (gp_buffer[gp_i2c_ts->y2_hi_idx] << 8) |
                                gp_buffer[gp_i2c_ts->y2_lo_idx];    
      }                          
      else
      {
         g_curr_touch_data.y2 = -1;
      }   
   }   
   
   g_curr_touch_data.num_fingers = 0x3 & gp_buffer[gp_i2c_ts->num_fingers_idx];   
   
   /* Not used but you never know ... */
   if (gp_i2c_ts->gesture_idx >= 0)
   {
      g_curr_touch_data.gesture = 0xf & gp_buffer[gp_i2c_ts->gesture_idx];
   }   
   
   if (gp_i2c_ts->version_idx >= 0)
   {
      g_curr_touch_data.version = 0xf & gp_buffer[gp_i2c_ts->version_idx]; 
   }   

   if (gp_i2c_ts->idle_idx >= 0 &&
       gp_i2c_ts->num_bytes_to_read >= (gp_i2c_ts->idle_idx + 1))
   {
      g_curr_touch_data.idle = gp_buffer[gp_i2c_ts->idle_idx];
   }

   if (gp_i2c_ts->timeout_idx >= 0 &&
       gp_i2c_ts->num_bytes_to_read >= (gp_i2c_ts->timeout_idx + 1))
   {
      g_curr_touch_data.timeout = gp_buffer[gp_i2c_ts->timeout_idx];
   }
   
   if (mod_param_debug & 0x20)
   {
      for (i = 0; i < gp_i2c_ts->num_bytes_to_read; i++)
      {
         printk("%2x ", gp_buffer[i]);
      }            
      printk("\n");
   }

   /* Check before going to sleep if the user changed any parameters. *
    * Needs to be here. It will not work if device is idle.           *
    */
   i2c_ts_driver_check_mod_params();          
   
   return rc;
}   

int  i2c_ts_driver_write(int length)
{
   int rc;
   
   rc = i2c_master_send(gp_i2c_driver_priv->p_i2c_client,
                        gp_buffer,
                        length);
   return rc;
}

void i2c_ts_driver_handle_i2c_error(int rc)
{
   if (mod_param_debug > 0)
   {
      printk("%s I2C error, rc %d # read errors %d # known driver errors %d\n",
             I2C_TS_DRIVER_NAME, rc, 
             g_num_read_errors,
             g_num_driver_errors);
   }
   
   if (rc != 0)
   {  /* Was called by i2c_ts_driver_read(). */
      if (g_num_read_errors < MAX_NUMBER_READ_ERRORS)
      {
         printk("%s I2C read error %d, error %d\n", 
                I2C_TS_DRIVER_NAME, g_num_read_errors, rc);
      }
      else if (g_num_read_errors == MAX_NUMBER_READ_ERRORS)
      {
         printk("%s maximum # I2C read errors reached %d, error %d\n", 
                I2C_TS_DRIVER_NAME, g_num_read_errors, rc);
      }
      else
      {
         return;
      }
   }
   else
   {
      g_num_driver_errors++;
   }   
         
   if (gp_i2c_ts->is_resetable)
   {
      printk("%s i2c_ts_driver_handle_i2c_error() resetting I2C slave at 0x%x\n", 
             I2C_TS_DRIVER_NAME, g_found_slave_addr);
      msleep(50);       
      i2c_ts_driver_reset_slave(0);  
      msleep(50);       
   }
   else
   {
      printk("%s I2C bus has problems but cannot reset slave at 0x%x\n", 
             I2C_TS_DRIVER_NAME, g_found_slave_addr);
   }         
      
   if (rc == -EREMOTEIO)
   {  /* Indicates a problem with the bus. Reset the I2C master controller. */
   
      if (0)
      {  // Need to reset the master if the message below appears.
      printk("%s resetting I2C bus master\n", I2C_TS_DRIVER_NAME);      
         //reset_i2c_master();
      msleep(200);       
   }
      else
      {
         printk("%s detected remote IO problem but cannot reset I2C bus master\n", 
                I2C_TS_DRIVER_NAME);            
      }      
   }
}

/* Have to ensure that the values read over the I2C bus are in range. */   
int i2c_ts_driver_check_touch_info(void)
{
   int rc = 0;
   int touch_state_changed = 0;
   
   if (g_curr_touch_data.x1 > gp_i2c_ts->x_max_value && 
       g_curr_touch_data.num_fingers > 0)
   {  /* The number of fingers have to be checked because when the number of
       * fingers is zero, i.e. no fingers are in contact, the x and y may be
       * out of range.
       */
      rc = kErrorX1;
   }
   
   if (g_curr_touch_data.y1 > gp_i2c_ts->y_max_value && 
       g_curr_touch_data.num_fingers > 0)
   {
      rc |= kErrorY1;
   }
   
   // Version 2 checking
   if (g_curr_touch_data.num_fingers == 0)
   {
      if (g_curr_touch_data.x1 < gp_i2c_ts->x_max_value ||
          g_curr_touch_data.y1 < gp_i2c_ts->y_max_value)
      {
         rc |= kErrorNumFingers;
      }      
   }
   
   if (gp_i2c_ts->is_multi_touch && g_curr_touch_data.num_fingers > 1)
   {
      if (g_curr_touch_data.x2 > gp_i2c_ts->x_max_value)
      {
         rc |= kErrorX2;
      }
   
      if (g_curr_touch_data.y2 > gp_i2c_ts->y_max_value )
      {
         rc |= kErrorY2;
      }
   }
   
   if (g_curr_touch_data.num_fingers > gp_i2c_ts->max_finger_val)
   {  
      rc |= kErrorNumFingers;
   }   
   
   if (g_curr_touch_data.num_fingers == 0 && g_touch_state == TSC_TOUCH_UP)
   {
      if (g_curr_touch_data.x1 != gp_i2c_ts->x_max_value || 
          g_curr_touch_data.y1 != gp_i2c_ts->y_max_value)
      {  /* A second finger up event when x and y are at max is ok. */  
         rc |= kErrorTouchState;
      }   
   }   

   if (gp_i2c_ts->version_idx >= 0 &&
       g_curr_touch_data.version != gp_i2c_ts->version_val)
   {
      rc |= kErrorVersion;
   }
   
   if (gp_i2c_ts->num_bytes_to_read >= (gp_i2c_ts->idle_idx + 1) &&
            g_curr_touch_data.idle != g_idle)
   {
      rc |= kErrorIdle;
   }
   
   if (gp_i2c_ts->num_bytes_to_read >= (gp_i2c_ts->timeout_idx + 1) &&
            g_curr_touch_data.timeout != g_timeout)
   {
      rc |= kErrorTimeout;
   }   

   if (rc == 0)
   {  /* No error so far, check the touch state and number of fingers. */
      if (g_curr_touch_data.num_fingers > 0 &&
          g_curr_touch_data.num_fingers <= gp_i2c_ts->max_finger_val)
      {               
         if (g_touch_state == TSC_TOUCH_UP)
         {  /* At least one finger is touching the screen, change the state. */
            g_touch_state = TSC_TOUCH_DOWN;
            touch_state_changed = 1;
         }      
      }  
   
      if (g_curr_touch_data.num_fingers == 0)
      {           
         if (g_touch_state == TSC_TOUCH_DOWN)
         {  /* No fingers are touching the screen, change the state. */
            g_touch_state = TSC_TOUCH_UP;
            touch_state_changed = 1;
         }
         else
         {
            rc = kErrorTouchState;
         }
      }
      
      g_num_good_events_per_touch++;

   }
   
   if (rc != 0)
   {
   
      if (mod_param_debug & 0x2)
      {
         printk("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x E:0x%x irqs:%d\n",
               gp_buffer[0],gp_buffer[1],gp_buffer[2],gp_buffer[3],
               gp_buffer[4],gp_buffer[5],gp_buffer[6],gp_buffer[7],
               gp_buffer[8],gp_buffer[9],gp_buffer[10],gp_buffer[11],               
               rc, atomic_read(&g_atomic_irqs_rxd));
      }         
             
      i2c_ts_driver_show_events(rc);
      g_num_bad_events++;
      g_num_bad_events_per_touch++;
      return rc;       
   }   
   
   if (mod_param_debug & 0x2)
   {
      printk("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %d\n",
             gp_buffer[0],gp_buffer[1],gp_buffer[2],gp_buffer[3],
             gp_buffer[4],gp_buffer[5],gp_buffer[6],gp_buffer[7],
             gp_buffer[8],gp_buffer[9],gp_buffer[10],gp_buffer[11],
             atomic_read(&g_atomic_irqs_rxd));
   }      
      
   g_prev_touch_data = g_curr_touch_data;
   i2c_ts_driver_send_touch_info();   
   
   i2c_ts_driver_show_events(rc);   
   
   g_num_good_events++;
   
   if ((g_num_good_events % 100) == 0 && 
       (I2C_TS_DRIVER_SHOW_ALL_EVENTS || I2C_TS_DRIVER_SHOW_EVENT_COUNT))
   {
      printk("# good events %lu # bad events %lu gen up events %lu\n", 
             g_num_good_events, g_num_bad_events, g_num_gen_up_events);
   }
   
   return rc;
}    

void i2c_ts_driver_remap_layout(void)
{   
   if (gp_i2c_ts->layout == X_RIGHT_Y_UP)
   {        
      g_curr_touch_data.y1 = gp_i2c_ts->y_max_value - g_curr_touch_data.y1;
      g_curr_touch_data.y2 = gp_i2c_ts->y_max_value - g_curr_touch_data.y2;      
   }
}

void i2c_ts_driver_show_events(int err_no)
{
   if (err_no)
   {
      if (I2C_TS_DRIVER_SHOW_ALL_EVENTS || I2C_TS_DRIVER_SHOW_ERR_EVENTS)
      {

         printk("x1:%5d y1:%5d x2:%5d y2:%5d f:%d v:%2d i:%3d t:%3d ERR! 0x%x\n", 
                g_curr_touch_data.x1, 
                g_curr_touch_data.y1, 
                g_curr_touch_data.x2, 
                g_curr_touch_data.y2, 
                g_curr_touch_data.num_fingers,
                g_curr_touch_data.version,
                g_curr_touch_data.idle,    
                g_curr_touch_data.timeout,
                err_no);         
      }         
      return;
   }   

   if (I2C_TS_DRIVER_SHOW_ALL_EVENTS == 0 && I2C_TS_DRIVER_SHOW_OK_EVENTS == 0)   
   {
      return;
   }   
   
   if (g_touch_state == TSC_TOUCH_DOWN)
   {
      printk("x1:%4d y1:%4d x2:%4d y2:%4d f:%d v:%2d i:%3d t:%3d(on)\n", 
             g_curr_touch_data.x1, 
             g_curr_touch_data.y1, 
             g_curr_touch_data.x2, 
             g_curr_touch_data.y2, 
             g_curr_touch_data.num_fingers,
             g_curr_touch_data.version,
             g_curr_touch_data.idle,    
             g_curr_touch_data.timeout);         
   }
   else
   {
      printk("x1:%4d y1:%4d x2:%4d y2:%4d f:%d v:%2d i:%3d t:%3d(off)\n", 
             g_curr_touch_data.x1, 
             g_curr_touch_data.y1, 
             g_curr_touch_data.x2, 
             g_curr_touch_data.y2, 
             g_curr_touch_data.num_fingers,
             g_curr_touch_data.version,
             g_curr_touch_data.idle,    
             g_curr_touch_data.timeout);         
   }   
}

void i2c_ts_driver_send_touch_info(void)
{
   if (g_touch_state == TSC_TOUCH_UP)
   {
      if (mod_param_debug & 0x10)
      {
         printk("finger up, # good %d # bad %d\n", 
                g_num_good_events_per_touch, g_num_bad_events_per_touch);
      }   
      else if (mod_param_debug & 0x1)
      {
         printk("finger up\n");
      }   
      
      g_num_good_events_per_touch = 0;
      g_num_bad_events_per_touch  = 0;
      
      input_report_abs(gp_input_dev, ABS_PRESSURE, INPUT_EVENT_NO_PRESSURE);        
      input_report_key(gp_input_dev, BTN_TOUCH, 0);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)      
      if (gp_i2c_ts->is_multi_touch)
      {
         input_report_abs(gp_input_dev, ABS_MT_TOUCH_MAJOR, 0);
         input_report_abs(gp_input_dev, ABS_MT_POSITION_X, 0);
         input_report_abs(gp_input_dev, ABS_MT_POSITION_Y, 0);
         input_mt_sync(gp_input_dev);
      }
#endif      
      input_sync(gp_input_dev);
      return;
   }
   
   if (gp_i2c_ts->layout != X_RIGHT_Y_DOWN)
   {  /* The x and y values have to be changed so x goes from left to right and
       * y from top to bottom.
       */
      i2c_ts_driver_remap_layout();
   }   
   
   if (gp_i2c_ts->is_multi_touch)
   {  /* Handles single or multi touches. */
      i2c_ts_driver_send_multitouch_info();
   }   
   else   
   {
      /* x and y values have already been checked that they in range and have
       *  been put into the proper layout. 
       */      
      if (mod_param_debug & 0x1)
      {         
         printk("finger down, x: %4d y: %4d\n", 
                 g_curr_touch_data.x1, g_curr_touch_data.y1);
      }   
      
      /* Good data, send it off */
      g_last_x = g_curr_touch_data.x1;
      g_last_y = g_curr_touch_data.y1;
      g_last_timestamp = timer_get_tick_count();
            
      input_report_abs(gp_input_dev, ABS_PRESSURE, INPUT_EVENT_PRESSURE);        
      input_report_abs(gp_input_dev, ABS_X, g_curr_touch_data.x1);   
      input_report_abs(gp_input_dev, ABS_Y, g_curr_touch_data.y1);   
      input_report_key(gp_input_dev, BTN_TOUCH, 1);
      input_sync(gp_input_dev);      
   }
}

/*
 * Here is what a minimal event sequence for a two-finger touch would look
 * like:
 *   ABS_MT_TOUCH_MAJOR
 *   ABS_MT_POSITION_X
 *   ABS_MT_POSITION_Y
 *   SYN_MT_REPORT
 *   ABS_MT_TOUCH_MAJOR
 *   ABS_MT_POSITION_X
 *   ABS_MT_POSITION_Y
 *   SYN_MT_REPORT
 *   SYN_REPORT
 *
 * Example 
 * x 0 - 800, y 0 - 240, finger size 20
 *
 * +--------------------------------------------------------------------------+
 * |                                             blob/ellipse 1 in contact    |
 * |                                                         -   (640,30)     |
 * |            +-------------------------------------------| |               |
 * |            |                                            -                |
 * |            |                                            |                |
 * |            |                                            |                |
 * |            |                                            |                |
 * |            |                                            |                |
 * |            |                                            |                |
 * |            |                                            |                |
 * |            |                                            |                |
 * |            |                                            |                |
 * |            |                                            |                |
 * |            -                                            |                |
 * |           | |-------------------------------------------+                |
 * |            -                                                             |
 * |   blob/ellipse 2 in contact                                                   |
 * |      (130,150)                                                           |
 * |            ABS_MT_TOUCH_MAJOR = 20 (ellipse in contact with screen)      |
 * |            ABS_MT_POSITION_X 1 = 640 ABS_MT_POSITION_X 2 = 130           |
 * |            ABS_MT_POSITION_Y 1 = 30  ABS_MT_POSITION_Y 2 = 150           |
 * +--------------------------------------------------------------------------+
 */
void i2c_ts_driver_send_multitouch_info(void)   
{
   int finger2_pressed = g_curr_touch_data.num_fingers > 1;
   
   if (mod_param_debug & 0x1)
   {         
      printk(" %d fingers x1: %4d y1: %4d x2: %4d y2: %4d\n", 
             g_curr_touch_data.num_fingers,   
             g_curr_touch_data.x1, g_curr_touch_data.y1, 
             g_curr_touch_data.x2, g_curr_touch_data.y2);
   }   
   
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
   input_report_abs(gp_input_dev, ABS_X, g_curr_touch_data.x1);
   input_report_abs(gp_input_dev, ABS_Y, g_curr_touch_data.y1);
   
   input_report_abs(gp_input_dev, ABS_PRESSURE, INPUT_EVENT_PRESSURE);     
   input_report_abs(gp_input_dev, ABS_TOOL_WIDTH, g_blob_size);
   input_report_abs(gp_input_dev, BTN_TOUCH, g_curr_touch_data.num_fingers);
   
   if (g_curr_touch_data.num_fingers > 1)
   {
      finger2_pressed = 1;
   }
   
   input_report_abs(gp_input_dev, BTN_2, finger2_pressed);

   /* Step 1: ABS_MT_TOUCH_MAJOR                                      */
   /* The length of the major axis of the contact.                    */
   /* Assume to be circular so _MINOR is not set.                     */
   input_report_abs(gp_input_dev, ABS_MT_TOUCH_MAJOR, g_blob_size);   

   /* Step 2: ABS_MT_POSITION_X                                       */
   /* The surface X coordinate of the center of the touching ellipse. */
   input_report_abs(gp_input_dev, ABS_MT_POSITION_X, g_curr_touch_data.x1);      
   /* Step 3: ABS_MT_POSITION_Y                                       */
   /* The surface Y coordinate of the center of the touching ellipse. */
   input_report_abs(gp_input_dev, ABS_MT_POSITION_Y, g_curr_touch_data.y1);  
      
   /* Step 4: SYN_MT_REPORT                                           */
   input_mt_sync(gp_input_dev);
   
   if (g_curr_touch_data.num_fingers > 1)
   {
      /* Step 5: ABS_MT_TOUCH_MAJOR                                      */
      /* The length of the major axis of the contact.                    */   
      input_report_abs(gp_input_dev, ABS_MT_TOUCH_MAJOR, g_blob_size);   
      /* Step 6: ABS_MT_POSITION_X                                       */
      /* The surface X coordinate of the center of the touching ellipse. */
      input_report_abs(gp_input_dev, ABS_MT_POSITION_X, g_curr_touch_data.x2);   
      /* Step 7: ABS_MT_POSITION_Y                                       */
      /* The surface Y coordinate of the center of the touching ellipse. */
      input_report_abs(gp_input_dev, ABS_MT_POSITION_Y, g_curr_touch_data.y2);   
      /* Step 8: SYN_MT_REPORT                                           */
      input_mt_sync(gp_input_dev);
   }
   
   /* Step 9: SYN_REPORT                                              */
   input_sync(gp_input_dev);
#endif   
}


/*
 * Have to reset the I2C slave but do not have enough information about what
 * reset pin to use so use the default. It will be changed later if necessary.
 */
int setup_gpio(int is_default_reset)
{
   int rc;
   int ret = 0;

   if (is_default_reset > 0)
   {  /* The I2C slave device has to be reset so it will be detected. */ 
      if ((rc = gpio_request(g_default_reset_pin, "i2c-driver reset")) != 0)
      {
         printk("setup_gpio() gpio_request(pin %d) failed, rc = %d\n", 
                g_default_reset_pin, rc);      
         ret = rc;
      }     

      if ((rc = gpio_direction_output(g_default_reset_pin, I2C_TS_DRIVER_DONT_RESET_CNTRLR)) != 0)
      {
         printk("gpio_direction_output(%d, I2C_TS_DRIVER_DONT_RESET_CNTRLR) error %d\n", 
                g_default_reset_pin, rc);
         ret = rc;
      }   
      
      return rc;
   }
   
   if ((rc = gpio_request(gp_i2c_ts->gpio_irq_pin, "i2c touch screen driver")) != 0)
   {
      printk("setup_gpio() gpio_request(%d) failed, rc = %d\n", 
             gp_i2c_ts->gpio_irq_pin, rc);      
      ret = rc;
   }     
   
   if ((rc = request_irq(gpio_to_irq(gp_i2c_ts->gpio_irq_pin), 
                         i2c_ts_driver_isr, 
                         (IRQF_TRIGGER_FALLING),
                         "GPIO cap touch screen irq", 
                         gp_i2c_driver_priv)) < 0)
   {
      printk("setup_gpio() request_irq(%d) failed, rc = %d\n", 
             gp_i2c_ts->gpio_irq_pin, rc);            
      ret = rc;
   }                             

   if (!gp_i2c_ts->is_resetable || gp_i2c_ts->gpio_reset_pin != g_default_reset_pin)
   { /* Release the reset pin if it is the wrong one or it is not needed. */
      gpio_free(g_default_reset_pin);
   }   
   
   if (gp_i2c_ts->is_resetable)
   {
      if (gp_i2c_ts->gpio_reset_pin != g_default_reset_pin)
      {      
         if ((rc = gpio_request(gp_i2c_ts->gpio_reset_pin, "i2c-driver reset")) != 0)
         {
            printk("setup_gpio() gpio_request() failed, rc = %d\n", rc);      
            ret = rc;
         }     

         if ((rc = gpio_direction_output(gp_i2c_ts->gpio_reset_pin, I2C_TS_DRIVER_DONT_RESET_CNTRLR)) != 0)
         {
            printk("gpio_direction_output(%d, I2C_TS_DRIVER_DONT_RESET_CNTRLR) error %d\n", 
                  gp_i2c_ts->gpio_reset_pin, rc);
            ret = rc;
         }   
      }
   }   

   return ret;
}

static int i2c_ts_driver_reset_slave(int is_default_reset)
{
   int rc = 0;
   
   if (is_default_reset == 0 && gp_i2c_ts->is_resetable == 0)
   {  /* Slave does not have a reset pin. */
      return rc;
   }
   
   if (is_default_reset)
   {
      gpio_set_value(g_default_reset_pin, I2C_TS_DRIVER_DO_RESET_CNTRLR);
      mdelay(GPIO_I2C_RESET_DELAY_MSECS);      
      gpio_set_value(g_default_reset_pin, I2C_TS_DRIVER_DONT_RESET_CNTRLR);
      mdelay(GPIO_I2C_RESET_DELAY_MSECS);      
   }
   else
   {
      gpio_set_value(gp_i2c_ts->gpio_reset_pin, I2C_TS_DRIVER_DO_RESET_CNTRLR);
      mdelay(GPIO_I2C_RESET_DELAY_MSECS);      
      gpio_set_value(gp_i2c_ts->gpio_reset_pin, I2C_TS_DRIVER_DONT_RESET_CNTRLR);
      mdelay(GPIO_I2C_RESET_DELAY_MSECS);      
      
      /* Rewrite these settings following reset. */
      g_timeout_changed   = 1;
      g_idle_changed      = 1;
      g_low_power_changed = 1;
      rc = i2c_ts_driver_check_mod_params();   
   }

   return rc;
   } 
   

static int i2c_ts_driver_probe(struct i2c_client *p_i2c_client,
                               const struct i2c_device_id *id)
{
   int rc = 0;
   int i;
   struct i2c_state *p_state;
   struct device *dev = &p_i2c_client->dev;
   
   struct I2C_TS_t *p_data = i2c_get_clientdata(p_i2c_client);
   
   if (p_i2c_client == NULL)
   {
      printk(KERN_ERR "%s i2c_ts_driver_probe() p_i2c_client == NULL\n", I2C_TS_DRIVER_NAME);      
      return -1;
   }

   if (p_i2c_client->dev.platform_data == NULL)
   {
      printk(KERN_ERR "%s i2c_ts_driver_probe() p_i2c_client->dev.platform_data == NULL\n",
             I2C_TS_DRIVER_NAME);      
      return -1;
   }
   
   if (!i2c_check_functionality(p_i2c_client->adapter, 
                                I2C_FUNC_SMBUS_BYTE_DATA |I2C_FUNC_SMBUS_WORD_DATA))
   {                             
      printk("%s: i2c_ts_driver_probe() i2c_check_functionality() failed %d\n", 
             I2C_TS_DRIVER_NAME, -ENODEV);
      return -ENODEV;
   } 
   
   if (g_found_slave_addr > 0)
   {
      printk(KERN_ERR "%s i2c_ts_driver_probe() i2c slave already found at 0x%x\n",
             I2C_TS_DRIVER_NAME, g_found_slave_addr);      
      return -1;
   }

   /* Get the I2C information compiled in for this platform. */   
   gp_i2c_ts = (struct I2C_TS_t *)p_i2c_client->dev.platform_data;   
   
   if (gp_i2c_ts == NULL)
   {  /* Cannot access platform data. */   
      printk("%s:%s Cannot access platform data for I2C slave address %d\n", 
             I2C_TS_DRIVER_NAME, __FUNCTION__, p_i2c_client->addr);
      return -1;
   }    
      
   printk("%s: slave address 0x%x\n", I2C_TS_DRIVER_NAME, p_i2c_client->addr);
   printk("%s: max x         0x%x\n", I2C_TS_DRIVER_NAME, gp_i2c_ts->x_max_value);
   printk("%s: max y         0x%x\n", I2C_TS_DRIVER_NAME, gp_i2c_ts->y_max_value);
   printk("%s: is multitouch %d\n", I2C_TS_DRIVER_NAME, gp_i2c_ts->is_multi_touch);

   /* Assign the finger touch size. */
   g_blob_size = I2C_TS_DRIVER_BLOB_SIZE*gp_i2c_ts->x_max_value/gp_i2c_ts->panel_width;
   
   if (gp_i2c_ts->is_multi_touch)
   {  /* Blob is the size of the finger contact. */
      printk("%s: blob size     %d\n", I2C_TS_DRIVER_NAME, g_blob_size);   
   }   
   
   p_state = kzalloc(sizeof(struct i2c_state), GFP_KERNEL);
   
   if (p_state == NULL) 
   {
      dev_err(dev, "failed to create our state\n");
     return -ENOMEM;
   }
 
   p_state->p_i2c_client = p_i2c_client;
   gp_i2c_driver_priv->p_i2c_client = p_i2c_client;

   i2c_set_clientdata(p_i2c_client, p_state);
 
   /* Rest of the initialisation goes here. */
   
   /* Create some space to store the I2C bytes read from the slave. */
   gp_buffer = kzalloc(gp_i2c_ts->num_bytes_to_read, GFP_KERNEL);

	if (!gp_buffer) 
   {
      printk("i2c_ts_driver_probe() kzalloc() returned NULL\n");
      kfree(p_i2c_client);
      return -ENOMEM;
   }       

   /* Retrieve the idle, timeout and auto low power settings from configuration file. */
   g_idle = gp_i2c_ts->idle_val;
   mod_param_idle = g_idle;

   g_timeout = gp_i2c_ts->timeout_val;
   mod_param_timeout = g_timeout;
   
   g_low_power = gp_i2c_ts->auto_power_val;
   mod_param_low_power = g_low_power;
      
      g_timeout_changed   = 1;
      g_idle_changed      = 1;
      g_low_power_changed = 1;
   
   /* This sets values on the slave. If the slave is not there it will fail *
    * ensuring the slave address is valid.                                  */
   rc = i2c_ts_driver_check_mod_params();         
   
   if (rc < 0)
   {  /* This also ensures that the slave is actually there! */
      printk("%s i2c_ts_driver_probe() failed to write to slave, rc = %d\n", 
             I2C_TS_DRIVER_NAME, rc);
      kfree(gp_buffer);
      return rc;    
   }      
   else
   {
      rc = 0;
   }

   /* Try to use the gpio pin to reset the I2C slave device prior to being probed. */
   
   /* 
    *  Setup the gpio for handling interrupt requests and the reset pin if used
    *  based on platform_data. 
    */
   setup_gpio(0);      
   
   g_kthread_pid = kernel_thread(i2c_ts_driver_kthread,               /* pointer to function */
                      NULL,                                /* argument            */
                      CLONE_FS|CLONE_FILES|CLONE_SIGHAND); /* |SIGCHILD); flags */
   
   if (g_kthread_pid == 0)
   {
      printk("%s i2c_ts_driver_probe() kernel thread not created, pid = %d\n", 
             I2C_TS_DRIVER_NAME, g_kthread_pid);
      return -1;       
   }

   /* The following code initializes the input system so events can be passed 
    * from the touch controller up to Android.   
    */
   gp_input_dev = input_allocate_device();

   if (gp_input_dev == NULL) {
      printk(KERN_ERR "%s i2c_ts_driver_probe() input_allocate_device() allocation failed\n",
             I2C_TS_DRIVER_NAME);
      kfree(p_i2c_client);
      kfree(gp_buffer);
      return -ENOMEM;
   }
             
   /* Set input device info. */
   gp_input_dev->name       = I2C_TS_DRIVER_NAME;
   gp_input_dev->phys       = "ts/input1";
   gp_input_dev->id.bustype = BUS_I2C;
   gp_input_dev->id.vendor  = 0x0001;
   gp_input_dev->id.product = 0x1103;
   gp_input_dev->id.version = 0x0000;

   /* Enable event bits. */
   set_bit(EV_SYN, gp_input_dev->evbit);
   set_bit(EV_KEY, gp_input_dev->evbit);
   set_bit(BTN_TOUCH, gp_input_dev->keybit);
   set_bit(EV_ABS, gp_input_dev->evbit);
   
   input_set_abs_params(gp_input_dev, ABS_X, 0, gp_i2c_ts->x_max_value, 0, 0);
   input_set_abs_params(gp_input_dev, ABS_Y, 0, gp_i2c_ts->y_max_value, 0, 0);
   /* Is pressure necessary .... yes! */
   input_set_abs_params(gp_input_dev, ABS_PRESSURE, 0, INPUT_EVENT_PRESSURE, 0, 0);
   input_set_abs_params(gp_input_dev, ABS_TOOL_WIDTH, 0, g_blob_size, 0, 0);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
   input_set_abs_params(gp_input_dev, ABS_MT_POSITION_X, 0, gp_i2c_ts->x_max_value, 0, 0);
   input_set_abs_params(gp_input_dev, ABS_MT_POSITION_Y, 0, gp_i2c_ts->y_max_value, 0, 0);
   input_set_abs_params(gp_input_dev, ABS_MT_TOUCH_MAJOR, 0, g_blob_size, 0, 0);
#endif   
   
   input_register_device(gp_input_dev);

   g_found_slave_addr = p_i2c_client->addr;

   return rc;
}
 
static int __devexit i2c_ts_driver_remove(struct i2c_client *client)
{
   struct i2c_state *state = i2c_get_clientdata(client);
   kfree(state);

   if (gp_i2c_ts->is_resetable)
   {
   gpio_free(gp_i2c_ts->gpio_reset_pin);
   }   

   if (g_kthread_pid > 0)
   {  /* Have to kill thread. */
      sys_kill(g_kthread_pid, SIGKILL);
   }   

   free_irq(gp_i2c_ts->gpio_irq_pin, gp_i2c_driver_priv);
   
   /* Free all the memory that was allocated. */   
   if (gp_i2c_driver_priv->p_i2c_client != NULL)
   {
      kfree(gp_i2c_driver_priv->p_i2c_client);
   }
   
   if (gp_i2c_driver_priv != NULL)
   {
      kfree(gp_i2c_driver_priv);
   }

   if (gp_input_dev != NULL)
   {
      input_free_device(gp_input_dev);
   }

   if (gp_buffer != NULL)
   {
      kfree(gp_buffer);
   }
   
   return 0;
}

/* End of if using .probe in i2c_driver. */ 

static struct i2c_device_id cy8ctst120_i2c_idtable[] = {
   { I2C_TS_DRIVER_NAME, 0 },
   { }
};

static struct i2c_driver cy8ctst120_i2c_driver = {
   .driver = {
      .name  = I2C_TS_DRIVER_NAME,
   },   
   .id_table       = cy8ctst120_i2c_idtable,
   .class          = I2C_CLASS_TOUCHSCREEN,
   .probe          = i2c_ts_driver_probe,
   .remove         = __devexit_p(i2c_ts_driver_remove),
#ifdef CONFIG_PM  
   .suspend        = i2c_ts_suspend_driver,
   .resume         = i2c_ts_resume_driver,
#endif   
   .address_data   = &addr_data,   
};

int __init i2c_ts_driver_init(void)
{
   int rc;
   
   gp_i2c_driver_priv = kmalloc(sizeof(struct i2c_priv_data), GFP_KERNEL);
   
   printk("%s: i2c_ts_driver_init() entering ...\n", I2C_TS_DRIVER_NAME);

   if (gp_i2c_driver_priv == NULL)
   {
      printk("i2c_ts_driver_init(): memory allocation failed for gp_i2c_driver_priv!\n");
      return -ENOMEM;
   }       
   
   /* Try to reset the I2C slave so it will be to detected by the master (us). 
    * Do not have any information about the slave device so use the default
    * reset ping.
    */
   setup_gpio(1);
   i2c_ts_driver_reset_slave(1);
   
   rc = i2c_add_driver(&cy8ctst120_i2c_driver);
   
   if (rc != 0) 
   {
      printk("%s i2c_ts_driver_init(): i2c_add_driver() failed, errno is %d\n", I2C_TS_DRIVER_NAME, rc);
      return rc;
   }

   return rc;
}

static void __exit i2c_ts_driver_exit(void)
{
	i2c_del_driver(&cy8ctst120_i2c_driver);
}

MODULE_DESCRIPTION("I2C Touchscreen driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");

module_init(i2c_ts_driver_init);
module_exit(i2c_ts_driver_exit);
