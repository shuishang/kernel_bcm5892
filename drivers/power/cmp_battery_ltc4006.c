/*****************************************************************************
* Copyright 2006 - 2010 Broadcom Corporation.  All rights reserved.
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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/broadcom/battery_settings.h>
#include <linux/gpio.h>

#include <linux/signal.h>
#include <linux/kthread.h>
#include <linux/syscalls.h>

#include <csp/tscHw.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
#endif

#ifndef CMP_USB_POWER
#define CMP_USB_POWER 0
#endif

#ifndef CMP_POWER_BUTTON_TURNS_OFF
#define CMP_POWER_BUTTON_TURNS_OFF 0
#endif

#ifdef HW_BATTERY_MAX_VOLTAGE
#define BATTERY_MAX HW_BATTERY_MAX_VOLTAGE
#define BATTERY_MIN HW_BATTERY_MIN_VOLTAGE
#else
#define BATTERY_MAX 4310000
#define BATTERY_MIN 2000000 
#endif

#define MIN_VOLTAGE_MV 5000

/* A concern was raised about the use of timers. The recommended behavior
 * is that the timer function should be atomic. This behavior may not obtainable
 * when the AC is connected or disconnected due to the querying of the GPIO
 * level for AC power.
 */
#define USE_TIMER 0

/* Need to poll all the time whether using interrupts or not to keep the
 * battery voltages readings current. If interrupts are used the polling
 * period can be much longer.
 */
#define POLL_ONLY 0

#if POLL_ONLY
#define POLL_INTERVAL_MILLISECONDS 5000
#else
/* Poll more often at start to inform Android at start of battery condition. */
#define INITIAL_POLL_INTERVAL      5000
#define POLL_INTERVAL_MILLISECONDS 120000 
#endif

/* When setting up interrupts on the 11181 need to a short delay between 
 * request_irq() calls.
 */
#define INT_SETUP_SLEEP_MSECS      50

#define USE_CHARGER_INTERRUPT      0  

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_HAS_WAKELOCK)
#define USE_WAKELOCK               1
#else
#define USE_WAKELOCK               0
#endif

#define NUM_DEBOUNCE_TRIES         5
#define DEBOUNCE_WAIT_MSECS        50 

#define USE_HIGH_LOW_IRQ           0

#include <linux/timer.h>
#include <linux/jiffies.h>

/* ---- Private Variables ------------------------------------------------ */

static const __devinitconst char gBanner[] =
    KERN_INFO "CMP LTC4006 Battery Driver: 1.01\n";

#if USE_TIMER    
static struct work_struct bat_work;
static struct work_struct pow_work;
static struct work_struct isr_work;
static struct timer_list polling_timer;
#else
static DECLARE_WAIT_QUEUE_HEAD(g_event_waitqueue);
static struct task_struct *pg_kernel_task = NULL;
atomic_t g_atomic_irqs_rxd = ATOMIC_INIT(0);
#endif

struct power {
   int online;
   struct power_supply psy;

   struct mutex work_lock;   /* protects data */

   int gpio_dok_b;
   int gpio_power_b;
};

struct battery {
   int status;
   struct power_supply psy;
   int full_chrg;

   struct mutex work_lock;   /* protects data */

   bool(*is_present) (struct battery * bat);
   int gpio_charging_b;

   int technology;

   int adc_bat;
   int bat_max;
   int bat_min;

   int adc_temp;
};
static struct battery battery_main;

static int mod_param_show_voltage = 0x0;   
module_param(mod_param_show_voltage, int, 0644);

#if USE_WAKELOCK
static struct wake_lock wakelock;
const char   *wake_lock_name = "ltc4006_battery_driver";
#define WAKE_LOCK_TIMEOUT_DURATION 6*HZ
#define USE_WAKELOCK_TIMEOUT       0
#endif

static struct power power_dc;
static void bcmring_power_off(void);

static char *work_callers[] = 
{  
   "reset", 
   "ISR",
   "poll",
   "resume",
   "external power changed",
};

typedef enum
{
   enum_caller_reset,
   enum_caller_isr,
   enum_caller_poll,
   enum_caller_resume,
   enum_caller_external_power_changed
} work_caller_enum;  

static work_caller_enum battery_work_caller = 0; 
static work_caller_enum power_work_caller   = 0; 
static int              test_isr_count      = 0;

#if POLL_ONLY
static long             poll_interval       = POLL_INTERVAL_MILLISECONDS;
#else
static long             poll_interval       = INITIAL_POLL_INTERVAL;
#endif

/* ---- Private Function Prototypes -------------------------------------- */
static int  battery_driver_kthread (void *unused);
static void power_update           (struct power *pow);
static void battery_update         (struct battery *bat);
static void isr_handler_work       (struct work_struct *work);
#if USE_TIMER
static void power_work             (struct work_struct *work);
#endif


/* 
 * Use the following values to convert the ADC output into the measured
 * battery voltage.
 * #define HW_BATTERY_MAX_VOLTAGE   
 * #define HW_BATTERY_MAXV_R1         
 * #define HW_BATTERY_MAXV_R2         
 * #define HW_BATTERY_REF_VOLTAGE   
 * #define HW_BATTERY_RV_R1         
 * #define HW_BATTERY_RV_R2         
 * #define HW_BATTERY_CCT_OFFSET    
 */
#ifdef HW_BATTERY_MAX_VOLTAGE   
static int compute_battery_voltage(void)
{
   int battery_voltage_mV;
   int resistor1 = HW_BATTERY_MAXV_R1;
   int resistor2 = HW_BATTERY_MAXV_R2;   
   /* The reference voltage for the TSC_NEGIN pin. */
   int ref_voltage_mv = HW_BATTERY_REF_VOLTAGE * (HW_BATTERY_RV_R2)/
                        (HW_BATTERY_RV_R1 + HW_BATTERY_RV_R2);                    
    
   int adc_measured_mV;
   u32 adc_output = tscHw_ReadAuxIn();
        
   adc_measured_mV = ((adc_output*TSCHW_AUXIN_VOLTAGE_RANGE)/TSCHW_AUXIN_STAT_MASK)+ref_voltage_mv;    
   battery_voltage_mV = (((resistor1+resistor2)*1000/resistor2)*adc_measured_mV)/1000 + HW_BATTERY_CCT_OFFSET;
          
 //For debugging purposes          
   if (mod_param_show_voltage >= 3) 
   {
      printk("%s() ref_volt_mv: %d adc_measured_mV: %d bat_volt_mV: %d\n", 
             __FUNCTION__, ref_voltage_mv, adc_measured_mV, battery_voltage_mV);
   } 
   
   if (battery_voltage_mV < MIN_VOLTAGE_MV)
   {  /* No battery. */
      if (mod_param_show_voltage > 0)
      {
          printk("%s() no battery installed?\n", __FUNCTION__);
      }   
      return BATTERY_MAX;
   }
   
   return battery_voltage_mV;
}
#endif    

static int power_get_property(struct power_supply *psy,
               enum power_supply_property psp,
               union power_supply_propval *val)
{
   struct power *pow = container_of(psy, struct power, psy);
   
   switch (psp) {
   case POWER_SUPPLY_PROP_ONLINE:   
      val->intval = pow->online;
      break;

   default:
      return -EINVAL;
   }

   return 0;
}

#if USE_TIMER == 0
static int battery_driver_kthread(void *unused)
{
   long unsigned int my_jiffies = 0, timeout_jiffies;
   wait_queue_head_t wait_queue;
   init_waitqueue_head (&wait_queue);

   timeout_jiffies = msecs_to_jiffies(POLL_INTERVAL_MILLISECONDS);  
   
   while(!kthread_should_stop())
   {
      /* Relinquish the processor until the event occurs */      
      set_current_state(TASK_INTERRUPTIBLE);
      
      if (atomic_read(&g_atomic_irqs_rxd) == 0)
      {  
         my_jiffies = wait_event_timeout(g_event_waitqueue,       /* the waitqueue to wait on */
                                         atomic_read(&g_atomic_irqs_rxd), /* condition to check */
                                         timeout_jiffies);        /* timeout in jiffies */
      }                                   
              
      if (my_jiffies > 0)    /* timeout, in jiffies */
      {  /* Woken before timer expired. */
            
         /* Calls power_update(). */
         isr_handler_work(NULL);
      }
      else
      {  /* Timer expired. */
 
         power_update(&power_dc);      
         battery_update(&battery_main);
      }

      if (atomic_read(&g_atomic_irqs_rxd) > 0)
      {   
         atomic_dec(&g_atomic_irqs_rxd);            
      }        
      
   }
   
   return 0;
}
#endif

#if !POLL_ONLY
/* This is called when a rising or falling edge on the GPIO monitoring the AC
 * power is detected. There is some bouncing when the AC power is connected
 * or disconnected so this may be called more than once for a given action.
 */  
static irqreturn_t power_gpio_isr(int irq, void *data)
{
   int ac_power_gpio_val = gpio_get_value(irq_to_gpio(irq));
   test_isr_count++;
   /* pr_info("power_gpio_isr: gpio val %d\n", ac_power_gpio_val); */
   
#if USE_HIGH_LOW_IRQ   
   /* Switch the interrupt so the tablet is not woken again for the current level. */
   set_irq_type(irq, 
                ac_power_gpio_val? IRQF_TRIGGER_LOW:IRQF_TRIGGER_HIGH);
#endif   

#if USE_WAKELOCK   
#if USE_WAKELOCK_TIMEOUT
      wake_lock_timeout(&wakelock, WAKE_LOCK_TIMEOUT_DURATION);
#else      
      wake_lock(&wakelock);
#endif      
              
#endif   

   power_work_caller = enum_caller_isr;

#if USE_TIMER == 1      
   /* Cannot call schedule_work(&pow_work) becaue there is some bounce in the 
    * the AC power GPIO value. isr_work will call pow_work once the signal
    * has been debounced.
    */
   schedule_work(&isr_work);
#else   
   if (atomic_read(&g_atomic_irqs_rxd) == 0)
   {
      atomic_inc(&g_atomic_irqs_rxd);
   }
  
   
   if (atomic_read(&g_atomic_irqs_rxd) > 0)
   {
      wake_up(&g_event_waitqueue);
   }   
#endif   

   return IRQ_HANDLED;
}
#endif

static void power_update(struct power *pow)
{
   int old_power_online;  
   struct power_supply *psy = &pow->psy;

   mutex_lock(&pow->work_lock); 
   old_power_online = pow->online;
   pow->online = gpio_get_value(pow->gpio_dok_b);                  
   power_supply_changed(psy);      
   
#if CMP_POWER_BUTTON_TURNS_OFF
   if (pow->gpio_power_b > 0 && gpio_get_value(pow->gpio_power_b) != 1)
   {
      bcmring_power_off();
   }
   
#endif
   mutex_unlock(&pow->work_lock);
}

#if USE_TIMER
static void power_work(struct work_struct *work)
{
   if (mod_param_show_voltage > 2)                       
   {
      printk("running %s() called by: %s isr count: %d\n", 
             __FUNCTION__, work_callers[power_work_caller], test_isr_count);
   }          
   power_update(&power_dc);
   power_work_caller = enum_caller_reset;
}
#endif

static void isr_handler_work(struct work_struct *work)
{
   int i;
   int real_gpio_val = gpio_get_value(HW_BATTERY_ACP_SHDN);
   int gpio_val;
   
   /* While the ISR is triggered on the rising or falling edge the signal has 
    * to be debounced before it be known for sure what really happened.
    */   
      
   for (i = 0; i < NUM_DEBOUNCE_TRIES; i++)
   {
      msleep(DEBOUNCE_WAIT_MSECS);
      gpio_val = gpio_get_value(HW_BATTERY_ACP_SHDN);
      
      if (gpio_val != real_gpio_val)
      {      
         real_gpio_val = gpio_val;      
      }
   }
   
   if (real_gpio_val)
   {  /* The tablet is plugged into an external power supply. */
#if 0   
      /* Grabbing the wakelock in the ISR instead. */       
#if USE_WAKELOCK_TIMEOUT     
      wake_lock_timeout(&wakelock, WAKE_LOCK_TIMEOUT_DURATION);
#else      
      wake_lock(&wakelock);
#endif      
        
#endif   
   }
   else
   {  /* The tablet has been unplugged from external power supply. */
#if USE_WAKELOCK   
      wake_unlock(&wakelock);        
#endif   
   }
               
   power_update(&power_dc);
   power_work_caller = enum_caller_reset;
}

static enum power_supply_property power_props[] = {
   POWER_SUPPLY_PROP_ONLINE,

};

#if CMP_USB_POWER
static int usb_get_property(struct power_supply *psy,
             enum power_supply_property psp,
             union power_supply_propval *val)
{
   switch (psp) {
   case POWER_SUPPLY_PROP_ONLINE:
      val->intval = 0;
      break;

   default:
      return -EINVAL;
   }

   return 0;
}

static enum power_supply_property usb_props[] = {
   POWER_SUPPLY_PROP_ONLINE,
};
#endif

static unsigned long read_bat(struct battery *bat)
{
   unsigned long value = 0;

#if 0
   if (bat->adc_bat < 0)
      return 0;
#else
   switch (bat->status) {
   case POWER_SUPPLY_STATUS_UNKNOWN:
      value = 0;
      break;
   case POWER_SUPPLY_STATUS_CHARGING:      
#ifdef HW_BATTERY_MAX_VOLTAGE      
      value = compute_battery_voltage()*1000;
#else
      value = 4100000;
#endif      
      break;
   case POWER_SUPPLY_STATUS_DISCHARGING:
#ifdef HW_BATTERY_MAX_VOLTAGE      
      value = compute_battery_voltage()*1000;
#else
      value = 4000000;
#endif      
      break;
   case POWER_SUPPLY_STATUS_FULL:
#ifdef HW_BATTERY_MAX_VOLTAGE      
      value = compute_battery_voltage()*1000;
#else
      value = 4200000; 
#endif      
      break;
   case POWER_SUPPLY_STATUS_NOT_CHARGING:
#ifdef HW_BATTERY_MAX_VOLTAGE      
      value = compute_battery_voltage()*1000;
#endif      
      break;
   default:
      value = 4200001;
      break;
   }
#endif

   return value;
}

static unsigned long read_temp(struct battery *bat)
{
   unsigned long value = 0;

   if (bat->adc_temp < 0)
      return 20;

   return value;
}

static int battery_get_property(struct power_supply *psy,
            enum power_supply_property psp,
            union power_supply_propval *val)
{
   int ret = 0;
   u32 current_voltage, max_diff_voltage;
   struct battery *bat = container_of(psy, struct battery, psy);

   if (bat->is_present && !bat->is_present(bat)
       && psp != POWER_SUPPLY_PROP_PRESENT) {
      return -ENODEV;
   }
   
   switch (psp) {
   case POWER_SUPPLY_PROP_STATUS:
      val->intval = bat->status;
      break;
   case POWER_SUPPLY_PROP_TECHNOLOGY:
      val->intval = bat->technology;
      break;
   case POWER_SUPPLY_PROP_VOLTAGE_NOW:
      val->intval = read_bat(bat);
      break;
   case POWER_SUPPLY_PROP_VOLTAGE_MAX:
      val->intval = bat->full_chrg;
      break;
   case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
      val->intval = bat->bat_max;
      break;
   case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
      val->intval = bat->bat_min;
      break;
   case POWER_SUPPLY_PROP_HEALTH:
      val->intval = POWER_SUPPLY_HEALTH_GOOD;
      break;

   case POWER_SUPPLY_PROP_CAPACITY:   
#ifdef HW_BATTERY_MAX_VOLTAGE      
      current_voltage = compute_battery_voltage();
      max_diff_voltage = HW_BATTERY_MAX_VOLTAGE - HW_BATTERY_MIN_VOLTAGE;
      
      if (current_voltage >= HW_BATTERY_MAX_VOLTAGE)
      {
         val->intval = 100;
      }
      else if (current_voltage < HW_BATTERY_MIN_VOLTAGE)
      {
         val->intval = 0;
      }
      else
      {
         val->intval = ((max_diff_voltage - 
                        (HW_BATTERY_MAX_VOLTAGE - current_voltage))*100)
                        /max_diff_voltage;
      }
#else      
      val->intval = 100;
#endif      
      break;
   case POWER_SUPPLY_PROP_TEMP:
      val->intval = read_temp(bat);
      break;
   case POWER_SUPPLY_PROP_PRESENT:
      val->intval = bat->is_present ? bat->is_present(bat) : 1;
      break;
   default:
      ret = -EINVAL;
      break;
   }
   return ret;
}

static bool batttery_is_present(struct battery *bat)
{
   (void)bat;
   return 1;
}

static void battery_external_power_changed(struct power_supply *psy)
{   
   battery_work_caller = enum_caller_external_power_changed;
#if USE_TIMER == 1
   schedule_work(&bat_work);
#else
   battery_update(&battery_main);
#endif 
}

static char *status_text[] = {
   [POWER_SUPPLY_STATUS_UNKNOWN] = "Unknown",
   [POWER_SUPPLY_STATUS_CHARGING] = "Charging",
   [POWER_SUPPLY_STATUS_DISCHARGING] = "Discharging",
   [POWER_SUPPLY_STATUS_FULL] = "Full",
   [POWER_SUPPLY_STATUS_NOT_CHARGING] = "Not Charging",
};

#if USE_TIMER
static void polling_timer_func(unsigned long unused)
{
   power_work_caller = enum_caller_poll;
   schedule_work(&pow_work);
   battery_work_caller = enum_caller_poll;
   schedule_work(&bat_work);
   mod_timer(&polling_timer, jiffies + msecs_to_jiffies(poll_interval));
}
#endif

#if !POLL_ONLY
#if USE_CHARGER_INTERRUPT 
static irqreturn_t battery_gpio_isr(int irq, void *data)
{
   pr_info("battery_gpio irq: %d\n", gpio_get_value(irq_to_gpio(irq)));
   battery_work_caller = enum_caller_isr;
#if USE_TIMER == 1 
   schedule_work(&bat_work);
#else
#endif 
   return IRQ_HANDLED;
}
#endif
#endif

static void battery_update(struct battery *bat)
{
   int old_battery_status;
   struct power_supply *psy = &bat->psy;
   
   mutex_lock(&bat->work_lock);

   old_battery_status = bat->status;

   if (bat->is_present && !bat->is_present(bat)) 
   {
      printk(KERN_NOTICE "%s not present\n", psy->name);
      bat->status = POWER_SUPPLY_STATUS_UNKNOWN;
   } 
   else if (power_supply_am_i_supplied(psy)) 
   {
      if (gpio_get_value(bat->gpio_charging_b)) 
      {
         bat->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
      } 
      else 
      {
         bat->status = POWER_SUPPLY_STATUS_CHARGING;
      }
   } 
   else 
   {
      bat->status = POWER_SUPPLY_STATUS_DISCHARGING;
   }

   if (old_battery_status != bat->status) 
   {
      printk(KERN_INFO "%s %s -> %s\n", psy->name,
          status_text[old_battery_status], status_text[bat->status]);
   }

   power_supply_changed(psy);

   mutex_unlock(&bat->work_lock);
}

#if USE_TIMER
static void battery_work(struct work_struct *work)
{
   if (mod_param_show_voltage >= 2)                       
   {
       printk("%s() run by: %s\n", __FUNCTION__, work_callers[battery_work_caller]);
   }    
   
   if (poll_interval != POLL_INTERVAL_MILLISECONDS &&
       battery_work_caller == enum_caller_external_power_changed)
   {  /* Android has asked for info at least once, change to longer poll      *
       * interval.                                                            */
      poll_interval = POLL_INTERVAL_MILLISECONDS;
   }   
   
   battery_update(&battery_main);
   battery_work_caller = enum_caller_reset;
}
#endif

static enum power_supply_property battery_main_props[] = {
   POWER_SUPPLY_PROP_STATUS,
   POWER_SUPPLY_PROP_TECHNOLOGY,
   POWER_SUPPLY_PROP_VOLTAGE_NOW,
   POWER_SUPPLY_PROP_VOLTAGE_MAX,
   POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
   POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
   POWER_SUPPLY_PROP_TEMP,
   POWER_SUPPLY_PROP_HEALTH,
   POWER_SUPPLY_PROP_CAPACITY,
   POWER_SUPPLY_PROP_PRESENT,
};

static char *power_supplied_to[] = {
   "battery",
};

static struct power power_dc = {
   .online = -1,
   .psy = {
      .name = "dc",
      .type = POWER_SUPPLY_TYPE_MAINS,
      .supplied_to = power_supplied_to,
      .num_supplicants = ARRAY_SIZE(power_supplied_to),
      .properties = power_props,
      .num_properties = ARRAY_SIZE(power_props),
      .get_property = power_get_property,
      },

   .gpio_dok_b = HW_BATTERY_ACP_SHDN,
   .gpio_power_b = HW_BATTERY_POWER_BTN,
};

#if CMP_USB_POWER
static struct power_supply psy_usb = {
   .name = "usb",
   .type = POWER_SUPPLY_TYPE_USB,
   .supplied_to = power_supplied_to,
   .num_supplicants = ARRAY_SIZE(power_supplied_to),
   .properties = usb_props,
   .num_properties = ARRAY_SIZE(usb_props),
   .get_property = usb_get_property,
};
#endif

static struct battery battery_main = {
   .status = POWER_SUPPLY_STATUS_DISCHARGING,
   .full_chrg = BATTERY_MAX,
   .psy = {
      .name = "battery",
      .type = POWER_SUPPLY_TYPE_BATTERY,
      .properties = battery_main_props,
      .num_properties = ARRAY_SIZE(battery_main_props),
      .get_property = battery_get_property,
      .external_power_changed = battery_external_power_changed,
      .use_for_apm = 1,
      },
   .is_present = batttery_is_present,
   .gpio_charging_b = HW_BATTERY_CHG,

   .technology = POWER_SUPPLY_TECHNOLOGY_LION,

   .adc_bat = -1,
   .bat_max = BATTERY_MAX,
   .bat_min = BATTERY_MIN,

   .adc_temp = -1,
};

static void bcmring_power_off(void)
{
   gpio_set_value(HW_BATTERY_CTL_PWR, 0);
}

static struct {
   int gpio;
   char *name;
   bool output;
   int value;
} gpios[] = {
   {HW_BATTERY_CTL_PWR, "switch off", 1, 1},

   /* These pins are normally inputs, but can be set as outputs */
   {HW_BATTERY_CHG, "battery charging", 0, 0}, 
   {HW_BATTERY_ACP_SHDN, "ac present/shutdown", 0, 0},
};

static int battery_irq_register(void)
{
   int ret = 0;
#if !POLL_ONLY
   /* Setup the interrupt to detect if external power is plugged in or not. 
    * IRQF_TRIGGER_RISING  - external power is connected
    * IRQF_TRIGGER_FALLING - external power is disconnected 
    * Unfortunately the signal bounces up and down before settling in either
    * case.
    */
   msleep(INT_SETUP_SLEEP_MSECS);
   /* Hi AC power applied, Low AC power removed. */
   ret = request_irq(gpio_to_irq(HW_BATTERY_ACP_SHDN),
                     power_gpio_isr,
#if USE_HIGH_LOW_IRQ   
    /* It would be nice to use this but does not appear possible.
    * It hangs when tablet starts when AC is connected.
    * It continues to retrigger as long as pin is high. */
    
    /* Set the level to the opposite of what the AC power indicatior is currently. */
    IRQF_DISABLED | ac_power_gpio_level? IRQF_TRIGGER_LOW: IRQF_TRIGGER_HIGH,                  
#else           
    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif           
                     "dc detect", &power_dc);
   if (ret)
   {
      printk("%s():  request_irq(request_irq(gpio_to_irq(HW_BATTERY_ACP_SHDN)) returned %d\n",
             __FUNCTION__, ret);  
   }          

#if USE_CHARGER_INTERRUPT   
   /* Setup the interrupt to detect when the battery changes from charging to * 
    * discharging or vice versa                                               */
   msleep(INT_SETUP_SLEEP_MSECS);
   ret = request_irq(gpio_to_irq(HW_BATTERY_CHG),   
           battery_gpio_isr,
           IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
           "main chg", &battery_main);
   if (ret)
   {
      printk("%s():  request_irq(request_irq(gpio_to_irq(HW_BATTERY_CHG)) returned %d\n",
             __FUNCTION__, ret);        
   }   
#endif
#endif // !POLL_ONLY
   return ret;
}

static void battery_irq_unregister(void)
{
#if !POLL_ONLY
   free_irq(gpio_to_irq(HW_BATTERY_ACP_SHDN), &power_dc);
#if USE_CHARGER_INTERRUPT   
   free_irq(gpio_to_irq(HW_BATTERY_CHG), &battery_main);
#endif   
#endif
}

#ifdef CONFIG_PM
static int battery_suspend(struct platform_device *dev, pm_message_t state)
{    
   if (HW_BATTERY_ACP_SHDN >= VC_0_GPIO_PIN_OFFSET)
   {  /* The AC power indication is connected to a video core GPIO. 
       * If the unregister and register is done in the resume the video core
       * will hang.
       */
      battery_irq_unregister();
   }  
   
   /* flush all pending status updates */
#if USE_TIMER  
   flush_scheduled_work();   
#endif    
   return 0;
}

static int battery_resume(struct platform_device *dev)
{
   /* Since video core GPIO is not able to handle the interrupt behavior after
    * suspend/resume. We need to free the irq and request_irq again on wakeup
    */
   int ret;
   int ac_power_gpio_val;
   
   /* things may have changed while we were away */
   
   if (HW_BATTERY_ACP_SHDN < VC_0_GPIO_PIN_OFFSET)
   {  /* The AC power indication is connected to a GPIO on the main processor. */
      /* Plugging in AC wakes the tablet and grabs the wakelock.               */
      battery_irq_unregister();
   }  
   else
   {  /* The AC power indication is connected to a GPIO on a video core. */
      /* Grab the wakelock if the AC is plugged in.                      */
      ac_power_gpio_val = gpio_get_value(HW_BATTERY_ACP_SHDN);      
      if (ac_power_gpio_val)
      {
#if USE_WAKELOCK   
#if USE_WAKELOCK_TIMEOUT
         wake_lock_timeout(&wakelock, WAKE_LOCK_TIMEOUT_DURATION);
#else         
         wake_lock(&wakelock);
#endif         
#endif   

#if !USE_TIMER
      atomic_inc(&g_atomic_irqs_rxd);
      wake_up(&g_event_waitqueue);   
#endif   
      }
   }   
   
   ret = battery_irq_register();
   if (ret)
   {
       printk("%s battery_resume cannot assign irq %d ret %d\n", 
              gBanner, gpio_to_irq(HW_BATTERY_ACP_SHDN), ret);
   }
   power_work_caller = enum_caller_resume;
#if USE_TIMER == 1 
   schedule_work(&pow_work);
#else
#endif 
   battery_work_caller = enum_caller_resume;
#if USE_TIMER == 1   
   schedule_work(&bat_work);
#else
#endif     
   return 0;
}
#endif

static int __devinit battery_probe(struct platform_device *dev)
{
   int ret;
   int i;
   int ac_power_gpio_level = 0;

   printk("%s AC power GPIO: %d\n", gBanner, HW_BATTERY_ACP_SHDN);
   
   for (i = 0; i < ARRAY_SIZE(gpios); i++) {
      ret = gpio_request(gpios[i].gpio, gpios[i].name);
      if (ret) {
         i--;
         goto err_gpio;
      }

      if (gpios[i].output)
         ret = gpio_direction_output(gpios[i].gpio,
                      gpios[i].value);
      else
         ret = gpio_direction_input(gpios[i].gpio);

      if (ret)
         goto err_gpio;
   }
   
   /* Is the tablet plugged or not? Need to know for setting the level. */
   ac_power_gpio_level = gpio_get_value(HW_BATTERY_ACP_SHDN);

   pm_power_off = bcmring_power_off;

   mutex_init(&battery_main.work_lock);
   mutex_init(&power_dc.work_lock);

#if USE_TIMER == 1  
   INIT_WORK(&bat_work, battery_work);
   INIT_WORK(&pow_work, power_work);
   INIT_WORK(&isr_work, isr_handler_work);
#endif

   ret = power_supply_register(&dev->dev, &power_dc.psy);
   if (ret)
      goto err_psy_reg_dc;
#if CMP_USB_POWER
   ret = power_supply_register(&dev->dev, &psy_usb);
   if (ret)
      goto err_psy_reg_usb;
#endif
   ret = power_supply_register(&dev->dev, &battery_main.psy);
   if (ret)
      goto err_psy_reg_main;

#if USE_TIMER == 1 
   setup_timer(&polling_timer, polling_timer_func, 0);
   mod_timer(&polling_timer, jiffies + msecs_to_jiffies(poll_interval));
#endif

   ret = battery_irq_register();
   if (ret)
      goto err_req_main;
#if !POLL_ONLY
#if USE_WAKELOCK
   /* Use a wake lock to keep the tablet awake. */
   wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, wake_lock_name);
#endif
#endif /* POLL_ONLY */   

   if (!ret) 
   {
#if USE_WAKELOCK   
      if (ac_power_gpio_level)
      {
#if USE_WAKELOCK_TIMEOUT
         wake_lock_timeout(&wakelock, WAKE_LOCK_TIMEOUT_DURATION);
#else         
         wake_lock(&wakelock);
#endif         
      }   
               
#endif   
   
#if USE_TIMER == 1
      schedule_work(&pow_work);
      schedule_work(&bat_work);
#else  
      pg_kernel_task = kthread_run(battery_driver_kthread,              /* pointer to function */
                                   NULL,                                /* argument            */
                                  "ltc4006-kernel-thread"); 
   
      if (pg_kernel_task == NULL)
      {
         printk("%s %s kernel thread not created\n", 
                gBanner, __FUNCTION__);
         goto probe_error;       
      }
      
      /* Update the power and battery for the first time. */
      
#endif
      return 0;
   }

probe_error:  
#if !POLL_ONLY

#if USE_WAKELOCK
   wake_lock_destroy(&wakelock);
#endif
   free_irq(gpio_to_irq(HW_BATTERY_CHG), &battery_main);
      err_req_main:
#endif
   power_supply_unregister(&battery_main.psy);
      err_psy_reg_main:
#if CMP_USB_POWER
   power_supply_unregister(&psy_usb);
      err_psy_reg_usb:
#endif
   power_supply_unregister(&power_dc.psy);
      err_psy_reg_dc:

   /* see comment in battery_remove */
   flush_scheduled_work();

   i--;
      err_gpio:
   pm_power_off = NULL;

   for (; i >= 0; i--)
      gpio_free(gpios[i].gpio);

   printk("%s(): return %d\n", __FUNCTION__, ret);
   return ret;
}

static int __devexit battery_remove(struct platform_device *dev)
{
   int i;


   battery_irq_unregister();
   power_supply_unregister(&battery_main.psy);
#if CMP_USB_POWER
   power_supply_unregister(&psy_usb);
#endif
   power_supply_unregister(&power_dc.psy);

#if USE_TIMER == 1  
   del_timer_sync(&polling_timer);
   /*
    * now flush all pending work.
    * we won't get any more schedules, since all
    * sources (isr and external_power_changed)
    * are unregistered now.
    */
   flush_scheduled_work();
   pm_power_off = NULL;
#else
   if (pg_kernel_task != NULL)
   {  /* Have to kill thread. */
      kthread_stop(pg_kernel_task);
   }      
#endif

   for (i = ARRAY_SIZE(gpios) - 1; i >= 0; i--)
      gpio_free(gpios[i].gpio);

#if USE_WAKELOCK
   wake_lock_destroy(&wakelock);
#endif
      
   return 0;
}

static struct platform_driver battery_driver = {
   .driver.name = "cmp-battery",
   .driver.owner = THIS_MODULE,
   .probe = battery_probe,
   .remove = __devexit_p(battery_remove),
#ifdef CONFIG_PM
   .suspend = battery_suspend,
   .resume = battery_resume,
#endif
};

static struct platform_device *battery_platform_device = NULL;

static int __init battery_init(void)
{
   int ret;

   battery_platform_device = platform_device_alloc("cmp-battery", -1);
   if (!battery_platform_device) {
      printk("%s(): platform_device_alloc failed\n", __FUNCTION__);
      return -ENOMEM;
   }

   ret = platform_driver_register(&battery_driver);
   if (ret) {
      printk("%s(): platform_driver_register failed %d\n", __FUNCTION__,
             ret);
      goto err_platform_driver_register;
   }

   ret = platform_device_add(battery_platform_device);
   if (ret) {
      printk("%s(): platform_device_add failed %d\n", __FUNCTION__,
             ret);
      goto err_platform_device_add;
   }

   return 0;

      err_platform_device_add:
   platform_driver_unregister(&battery_driver);
      err_platform_driver_register:
   platform_device_del(battery_platform_device);
   platform_device_put(battery_platform_device);
   
   return ret;
}

static void __exit battery_exit(void)
{
   if (battery_platform_device) {
      platform_device_del(battery_platform_device);
      platform_device_put(battery_platform_device);
   }
   platform_driver_unregister(&battery_driver);
}

module_init(battery_init);
module_exit(battery_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom CMP LTC4006 Battery Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.01");
