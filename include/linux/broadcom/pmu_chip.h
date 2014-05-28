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




/*
*
*****************************************************************************
*
*  pmu_chip.h
*
*  PURPOSE:
*
*  This file defines the common interface to a PMU chip.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( BCM_PMU_CHIP_H )
#define BCM_PMU_CHIP_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/ioctl.h>
#include <linux/fs.h>

#ifdef __KERNEL__ 
#include <linux/i2c.h>
#include <linux/interrupt.h>
#endif

/* ---- Constants and Types ---------------------------------------------- */

#define BCM_PMU_MAGIC   'P'

#define BCM_PMU_CMD_FIRST               0x80

#define BCM_PMU_CMD_ENABLE_INTS         0x80
#define BCM_PMU_CMD_DISABLE_INTS        0x81
#define BCM_PMU_CMD_READ_REG            0x83
#define BCM_PMU_CMD_WRITE_REG           0x84
#define BCM_PMU_CMD_ACTIVATESIM         0x85
#define BCM_PMU_CMD_DEACTIVATESIM       0x86
#define BCM_PMU_CMD_GET_REGULATOR_STATE 0x87
#define BCM_PMU_CMD_SET_REGULATOR_STATE 0x88
#define BCM_PMU_CMD_SET_PWM_LED_CTRL    0x89
#define BCM_PMU_CMD_SET_PWM_HI_PER      0x8a
#define BCM_PMU_CMD_SET_PWM_LO_PER      0x8b
#define BCM_PMU_CMD_SET_PWM_PWR_CTRL    0x8c
#define BCM_PMU_CMD_GET_VOLTAGE		0x8d
#define BCM_PMU_CMD_SET_VOLTAGE		0x8e

#define BCM_PMU_CMD_LAST                0x8e

#define BCM_PMU_IOCTL_ENABLE_INTS       _IO(  BCM_PMU_MAGIC, BCM_PMU_CMD_ENABLE_INTS )       /* arg is unused */
#define BCM_PMU_IOCTL_DISABLE_INTS      _IO(  BCM_PMU_MAGIC, BCM_PMU_CMD_DISABLE_INTS )      /* arg is unused */
#define BCM_PMU_IOCTL_READ_REG          _IOWR(BCM_PMU_MAGIC, BCM_PMU_CMD_READ_REG, BCM_PMU_Reg_t )
#define BCM_PMU_IOCTL_WRITE_REG         _IOW( BCM_PMU_MAGIC, BCM_PMU_CMD_WRITE_REG, BCM_PMU_Reg_t )
#define BCM_PMU_IOCTL_ACTIVATESIM       _IOW( BCM_PMU_MAGIC, BCM_PMU_CMD_ACTIVATESIM, PM_SimVoltage_t )
#define BCM_PMU_IOCTL_DEACTIVATESIM 	_IO( BCM_PMU_MAGIC, BCM_PMU_CMD_DEACTIVATESIM )  /* arg is unused */
#define BCM_PMU_IOCTL_GET_REGULATOR_STATE _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_REGULATOR_STATE, BCM_PMU_Regulator_t)
#define BCM_PMU_IOCTL_SET_REGULATOR_STATE _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_REGULATOR_STATE, BCM_PMU_Regulator_t)
#define BCM_PMU_IOCTL_GET_VOLTAGE	 _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_VOLTAGE, BCM_PMU_Regulator_Volt_t)
#define BCM_PMU_IOCTL_SET_VOLTAGE	 _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_VOLTAGE, BCM_PMU_Regulator_Volt_t)
#define BCM_PMU_IOCTL_SET_PWM_LED_CTRL    _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_LED_CTRL, BCM_PMU_PWM_ctrl_t)
#define BCM_PMU_IOCTL_SET_PWM_HI_PER      _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_HI_PER, BCM_PMU_PWM_hi_per_t)
#define BCM_PMU_IOCTL_SET_PWM_LO_PER      _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_LO_PER, BCM_PMU_PWM_lo_per_t)
#define BCM_PMU_IOCTL_SET_PWM_PWR_CTRL    _IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_PWR_CTRL, BCM_PMU_PWM_pwr_ctrl_t)

typedef enum
{
    SIM_3POINT0VOLT = 0,
    SIM_2POINT5VOLT,
    SIM_3POINT1VOLT,
    SIM_1POINT8VOLT,
    SIM_MAX_VOLTAGE    
} PM_SimVoltage_t;

typedef struct
{
    unsigned char  reg;
    unsigned char  val;
} BCM_PMU_Reg_t;

typedef struct
{
   int regulatorID;
   int voltage;
   int min;
   int max;
   int step;
} BCM_PMU_Regulator_Volt_t;

typedef enum
{
   PMU_PCF50603 = 0,
   PMU_PCF50611,
   PMU_BCM59001,
   PMU_BCM59035,
   PMU_NONE,
   PMU_NUM_CHIPS,
} BCM_PMU_Chip_t;

typedef enum
{
   PMU_Regulator_Off,
   PMU_Regulator_On,
   PMU_Regulator_Eco
} BCM_PMU_Regulator_State_t;

typedef struct
{
   int regulatorID;
   BCM_PMU_Regulator_State_t state;
} BCM_PMU_Regulator_t;

typedef struct 
{
    unsigned int pwmled_ctrl ;
    unsigned int pwmdiv ; /* divider value. fsys/x value.     */
} BCM_PMU_PWM_ctrl_t ;

typedef struct 
{
    unsigned int hi_per ;
} BCM_PMU_PWM_hi_per_t ;

typedef struct 
{
    unsigned int lo_per ;
} BCM_PMU_PWM_lo_per_t ;

typedef struct 
{
    unsigned int pwr_ctrl ;
} BCM_PMU_PWM_pwr_ctrl_t ;

typedef enum
{
   PMU_Power_On_By_On_Button,      /* on button */
   PMU_Power_On_By_Charger,        /* charger insertion and no on button */
   PMU_Power_On_By_Restart         /* re-started while power on */
} BCM_PMU_Power_On_State_t;

typedef enum                     /* If you change this, please update PM_EventTable as well */
{
   PMU_EVENT_ATTACHED = 0,

   PMU_EVENT_BATTERY_LOW,
   PMU_EVENT_BATTERY_FULL,

   PMU_EVENT_BATTERY_TEMPERATURE_FAULT,
   PMU_EVENT_BATTERY_TEMPERATURE_OK,

   PMU_EVENT_ONKEY_RISE,
   PMU_EVENT_ONKEY_FALL,
   PMU_EVENT_ONKEY_1S_HOLD,

   PMU_EVENT_HIGH_TEMPERATURE,

   PMU_EVENT_CHARGER_INSERT,
   PMU_EVENT_CHARGER_REMOVE,
   PMU_EVENT_CHARGER_ERROR,

   PMU_NUM_EVENTS,
} BCM_PMU_Event_t;


/* ---- Variable Externs ------------------------------------------------- */

#ifdef __KERNEL__
extern u32 pmu_max_isr_clk;      /* ISR profiling counter */
#endif

/* ---- Function Prototypes ---------------------------------------------- */

#ifdef __KERNEL__

#if defined( CONFIG_BCM_PMU )

/* Initialization function */
typedef int (*pmu_init_fnc)(void);
#define PMU_init(pmu) ((pmu)->init?(pmu)->init():0)

/* Interrupt service routine */
typedef irqreturn_t (*pmu_isr_fnc)(void *dev_id);
#define PMU_isr(pmu,dev_id) ((pmu)->isr?(pmu)->isr(dev_id):IRQ_NONE)

/* Get power on condition */
typedef BCM_PMU_Power_On_State_t (*pmu_get_power_on_state_fnc)(void);
#define PMU_get_power_on_state(pmu) ((pmu)->get_power_on_state?(pmu)->get_power_on_state():PMU_Power_On_By_On_Button)

/* IOCTL handler */
typedef int (*pmu_ioctl_fnc) (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
#define PMU_ioctl(pmu,inode,file,cmd,arg) ((pmu)->ioctl?(pmu)->ioctl(inode,file,cmd,arg):-EINVAL)
 
/* Power off function */
typedef void (*pmu_poweroff_fnc)(void);
#define PMU_poweroff(pmu) ((pmu)->poweroff?(pmu)->poweroff():0)

/* Platform specific timed run function */
typedef void (*pmu_run_fnc)(void);
#define PMU_run(pmu) ((pmu)->run?(pmu)->run():0)

/* Platform specific logLevel setting */
typedef void (*pmu_logLevel_fnc)(int level);
#define PMU_logLevel(pmu, level) ((pmu)->logLevel?(pmu)->logLevel(level):0)

/* Power regulator control */
typedef int (*pmu_regulator_set_state_fnc)(int regulatorID, BCM_PMU_Regulator_State_t state);
#define PMU_regulator_set_state(pmu,regulatorID,state) \
   ((pmu)->regulator.set_state?(pmu)->regulator.set_state(regulatorID,state):0)

typedef BCM_PMU_Regulator_State_t (*pmu_regulator_get_state_fnc)(int regulatorID);
#define PMU_regulator_get_state(pmu,regulatorID) \
   ((pmu)->regulator.get_state?(pmu)->regulator.get_state(regulatorID,state):PMU_Regulator_On)

typedef int (*pmu_regulator_set_voltage_fnc)(int regulatorID, u32 mV);
#define PMU_regulator_set_voltage(pmu,regulatorID,mV) \
   ((pmu)->regulator.set_voltage?(pmu)->regulator.set_voltage(regulatorID,mV):0)

typedef u32 (*pmu_regulator_get_voltage_fnc)(int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step);
#define PMU_regulator_get_voltage(pmu,regulatorID,min_mV,max_mV,mV_step) \
   ((pmu)->regulator.get_voltage?(pmu)->regulator.get_voltage(regulatorID,min_mV,max_mV,mV_step):0)


/* Charger control */
typedef void (*pmu_charger_start_fnc)(int chargerID);
#define PMU_charger_start(pmu,chargerID) \
   ((pmu)->charger.start?(pmu)->charger.start(chargerID):0)

typedef void (*pmu_charger_stop_fnc)(int chargerID);
#define PMU_charger_stop(pmu,chargerID) \
   ((pmu)->charger.stop?(pmu)->charger.stop(chargerID):0)

typedef int (*pmu_charger_is_inserted_fnc)(int *chargerID);
#define PMU_charger_is_inserted(pmu,chargerID) \
   ((pmu)->charger.is_inserted?(pmu)->charger.is_inserted(chargerID):0)


typedef struct
{
   pmu_init_fnc init;
   pmu_isr_fnc isr;
   pmu_get_power_on_state_fnc get_power_on_state;
   pmu_ioctl_fnc ioctl;
   pmu_poweroff_fnc poweroff;
   pmu_run_fnc run;
   pmu_logLevel_fnc logLevel;
   struct
   {
      pmu_regulator_set_state_fnc set_state;
      pmu_regulator_get_state_fnc get_state;
      pmu_regulator_set_voltage_fnc set_voltage;
      pmu_regulator_get_voltage_fnc get_voltage;
   } regulator;
   struct
   {
      pmu_charger_start_fnc start;
      pmu_charger_stop_fnc stop;
      pmu_charger_is_inserted_fnc is_inserted;
   } charger;
   struct i2c_client_address_data *i2c_data;
   char driver_name[ 20 ];
   struct i2c_driver driver;
} BCM_PMU_Operations_t;

int pmu_register_device(BCM_PMU_Chip_t chip, BCM_PMU_Operations_t *pmu_ops);

int pmu_i2c_read(u8 regAddr);
int pmu_i2c_write(u8 regAddr, u8 value);

int pmu_i2c_read_bytes(u8 regAddr, u8 *values, int num);

void pmu_enable_ints( void );
void pmu_disable_ints( void );

void pmu_event_notify(BCM_PMU_Chip_t chip, BCM_PMU_Event_t event, void *data);

int pmu_chip_init(BCM_PMU_Chip_t chip);
void pmu_chip_exit( void );

void pmu_start_null_pmu( void );

#else

/* Allow PMU calls to compile but not do anything */

#define pmu_register_device(chip, pmu_ops, i2c_data) ((void)(chip), (void)(pmu_ops), (void)(i2c_data), -1)
#define pmu_i2c_read(regAddr)        ((void)(regAddr), -1)
#define pmu_i2c_write(regAddr, val)  ((void)(regAddr), (void)(val), -1)
#define pmu_enable_ints() (-1)
#define pmu_disable_ints() (-1)
#define pmu_event_notify(chip, event, data) ((void)(chip), (void)(event), (void)(data), -1)
#define pmu_chip_init(chip) ((void)(chip), -1)
#define pmu_chip_exit() (-1)



#endif

#endif

#endif  /* BCM_PMU_CHIP_H */

