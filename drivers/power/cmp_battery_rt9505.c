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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/broadcom/battery_settings.h>
#include <linux/gpio.h>

#include <csp/tscHw.h>

#ifndef CMP_USB_POWER
#define CMP_USB_POWER 0
#endif

#ifndef HW_BATTERY_POWER_BTN_NORMAL
/* Assume power button input is normally high */
#define HW_BATTERY_POWER_BTN_NORMAL 1
#endif

#ifdef HW_BATTERY_MAX_VOLTAGE
#define BATTERY_MAX HW_BATTERY_MAX_VOLTAGE
#define BATTERY_MIN HW_BATTERY_MIN_VOLTAGE
#else
#define BATTERY_MAX 4310000
#define BATTERY_MIN 2000000 
#endif

#define MIN_VOLTAGE_MV 5000

#define NEED_TO_POLL 1
#if NEED_TO_POLL
#include <linux/timer.h>
#include <linux/jiffies.h>
static struct timer_list polling_timer;
#endif

static const __devinitconst char gBanner[] =
    KERN_INFO "CMP RT9505 Battery Driver: 1.00\n";
static struct work_struct bat_work;
static struct work_struct pow_work;

struct power {
	int online;
	struct power_supply psy;

	struct mutex work_lock;	/* protects data */

	int gpio_dok_b;
	int gpio_power_b;
    int gpio_power_btn_norm;
};
static struct power power_dc;
static void bcmring_power_off(void);

static int mod_param_show_voltage = 0;
module_param(mod_param_show_voltage, int, 0644);

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
    
   if (0)
   {
      printk("tscHw_ReadAuxIn(): 0x%lx \n", adc_output);
   }       

   if (mod_param_show_voltage > 0)
   {
      printk("cmp_battery: ref_voltage_mv: %d adc_measured_mV: %d battery_voltage_mV: %d\n", 
             ref_voltage_mv, adc_measured_mV, battery_voltage_mV);
   }
   
   if (battery_voltage_mV < MIN_VOLTAGE_MV)
   {  /* No battery. */
      if (mod_param_show_voltage > 0)
      {
         printk("cmp_battery: no battery installed?\n");
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

#if !NEED_TO_POLL
static irqreturn_t power_gpio_isr(int irq, void *data)
{
	pr_info("power_gpio_isr: %d\n", gpio_get_value(irq_to_gpio(irq)));
	schedule_work(&pow_work);
	return IRQ_HANDLED;
}
#endif

static void power_update(struct power *pow)
{
	int old;
	struct power_supply *psy = &pow->psy;

	mutex_lock(&pow->work_lock);

	old = pow->online;

	pow->online = gpio_get_value(pow->gpio_dok_b);

#ifndef HW_BATTERY_MAX_VOLTAGE  
	if (old != pow->online) {
		power_supply_changed(psy);
	}   
#else
   /* Update all the time rather than only when the online value changes. */
   power_supply_changed(psy);
#endif   

	if (pow->gpio_power_b > 0 && 
		gpio_get_value(pow->gpio_power_b) != HW_BATTERY_POWER_BTN_NORMAL)
	{
		bcmring_power_off();
	}
	mutex_unlock(&pow->work_lock);
}

static void power_work(struct work_struct *work)
{
	power_update(&power_dc);
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

struct battery {
	int status;
	struct power_supply psy;
	int full_chrg;

	struct mutex work_lock;	/* protects data */

	bool(*is_present) (struct battery * bat);
	int gpio_charging_b;

	int technology;

	int adc_bat;
	int bat_max;
	int bat_min;

	int adc_temp;
};
static struct battery battery_main;

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
	schedule_work(&bat_work);
}

static char *status_text[] = {
	[POWER_SUPPLY_STATUS_UNKNOWN] = "Unknown",
	[POWER_SUPPLY_STATUS_CHARGING] = "Charging",
	[POWER_SUPPLY_STATUS_DISCHARGING] = "Discharging",
	[POWER_SUPPLY_STATUS_FULL] = "Full",
	[POWER_SUPPLY_STATUS_NOT_CHARGING] = "Not Charging",
};

#if NEED_TO_POLL
static void polling_timer_func(unsigned long unused)
{
	schedule_work(&pow_work);
	schedule_work(&bat_work);
	mod_timer(&polling_timer, jiffies + msecs_to_jiffies(5000));
}
#else
static irqreturn_t battery_gpio_isr(int irq, void *data)
{
	pr_info("battery_gpio irq: %d\n", gpio_get_value(irq_to_gpio(irq)));
	schedule_work(&bat_work);
	return IRQ_HANDLED;
}
#endif

static void battery_update(struct battery *bat)
{
	int old;
	struct power_supply *psy = &bat->psy;

	mutex_lock(&bat->work_lock);

	old = bat->status;

	if (bat->is_present && !bat->is_present(bat)) {
		printk(KERN_NOTICE "%s not present\n", psy->name);
		bat->status = POWER_SUPPLY_STATUS_UNKNOWN;
	} else if (power_supply_am_i_supplied(psy)) {
		if (gpio_get_value(bat->gpio_charging_b)) {
			bat->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else {
			bat->status = POWER_SUPPLY_STATUS_CHARGING;
		}
	} else {
		bat->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (old != bat->status) {
		printk(KERN_INFO "%s %s -> %s\n", psy->name,
			 status_text[old], status_text[bat->status]);
		power_supply_changed(psy);
	}

	mutex_unlock(&bat->work_lock);
}

static void battery_work(struct work_struct *work)
{
	battery_update(&battery_main);
}

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
	.gpio_power_btn_norm = HW_BATTERY_POWER_BTN_NORMAL,
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
	{HW_BATTERY_CHG_EN, "charging enable", 1, 0},
	
	{HW_BATTERY_CHG, "battery charging", 0, 0}, 
	{HW_BATTERY_ACP_SHDN, "ac present/shutdown", 0, 0},
};

#ifdef CONFIG_PM
static int battery_suspend(struct platform_device *dev, pm_message_t state)
{
	/* flush all pending status updates */
	flush_scheduled_work();
	return 0;
}

static int battery_resume(struct platform_device *dev)
{
	/* things may have changed while we were away */
	schedule_work(&pow_work);
	schedule_work(&bat_work);
	return 0;
}
#endif

static int __devinit battery_probe(struct platform_device *dev)
{
	int ret;
	int i;

	printk(gBanner);

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

	pm_power_off = bcmring_power_off;

	mutex_init(&battery_main.work_lock);
	mutex_init(&power_dc.work_lock);

	INIT_WORK(&bat_work, battery_work);
	INIT_WORK(&pow_work, power_work);

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

#if NEED_TO_POLL
	setup_timer(&polling_timer, polling_timer_func, 0);
	mod_timer(&polling_timer, jiffies + msecs_to_jiffies(5000));
#else
	ret = request_irq(gpio_to_irq(HW_BATTERY_CHG),
			  battery_gpio_isr,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "main chg", &battery_main);
	if (ret)
		goto err_req_main;

	ret = request_irq(gpio_to_irq(HW_BATTERY_ACP_SHDN),
			  power_gpio_isr,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "dc detect", &power_dc);

	ret = request_irq(gpio_to_irq(HW_BATTERY_POWER_BTN),
			  power_gpio_isr,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "power button", &power_dc);

#endif
	if (!ret) {
		schedule_work(&pow_work);
		schedule_work(&bat_work);
		return 0;
	}
#if !NEED_TO_POLL
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

	printk("%s: return %d\n", __FUNCTION__, ret);
	return ret;
}

static int __devexit battery_remove(struct platform_device *dev)
{
	int i;

#if NEED_TO_POLL
	del_timer_sync(&polling_timer);
#else
	free_irq(gpio_to_irq(HW_BATTERY_ACP_SHDN), &power_dc);
	free_irq(gpio_to_irq(HW_BATTERY_CHG), &battery_main);
#endif

	power_supply_unregister(&battery_main.psy);
#if CMP_USB_POWER
	power_supply_unregister(&psy_usb);
#endif
	power_supply_unregister(&power_dc.psy);

	/*
	 * now flush all pending work.
	 * we won't get any more schedules, since all
	 * sources (isr and external_power_changed)
	 * are unregistered now.
	 */
	flush_scheduled_work();
	pm_power_off = NULL;

	for (i = ARRAY_SIZE(gpios) - 1; i >= 0; i--)
		gpio_free(gpios[i].gpio);

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
		printk("%s: platform_device_alloc failed\n", __FUNCTION__);
		return -ENOMEM;
	}

	ret = platform_driver_register(&battery_driver);
	if (ret) {
		printk("%s: platform_driver_register failed %d\n", __FUNCTION__,
		       ret);
		goto err_platform_driver_register;
	}

	ret = platform_device_add(battery_platform_device);
	if (ret) {
		printk("%s: platform_device_add failed %d\n", __FUNCTION__,
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
MODULE_VERSION("1.0");
