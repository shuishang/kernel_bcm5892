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
*  pmu_chip.c
*
*  PURPOSE:
*
*     This implements support for multiple PMU chip drivers.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/sysctl.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>

#include <linux/broadcom/hw_cfg.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/PowerManager.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/gpio.h>

#if defined(CONFIG_ARM)
#include <asm/irq.h>
#endif

/* ---- Public Variables ------------------------------------------------- */

/* structure to hold driver state */
typedef struct
{
#if defined( HW_GPIO_PMU_IRQ_PIN )
    int g_irqs;
    int g_isr;
#endif
    int g_level;
    BCM_PMU_Chip_t chip;
} PMU_DATA;

PMU_DATA gPmuData;

u32 pmu_max_isr_clk = 0;

/* ---- Private Constants and Types -------------------------------------- */

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

#define DBG_DEFAULT_LEVEL  (DBG_ERROR)
/*#define DBG_DEFAULT_LEVEL  (DBG_ERROR | DBG_INFO | DBG_TRACE) */

#if DEBUG
#	define PMU_DEBUG(level,fmt,args...) do { if (level & gPmuData.g_level) printk( "%s: " fmt, __FUNCTION__, ##args ); } while (0)
#else
#	define PMU_DEBUG(level,fmt,args...)
#endif

/* I2C */
#define IF_NAME			"pmui2c"
#define I2C_DRIVERID_PMU	0xF001

/* ---- Private Variables ------------------------------------------------ */

static char banner[] = KERN_INFO "PMU Chips Support: 1.00 (built on "__DATE__" "__TIME__")\n";

/* sysctl */
static  struct ctl_table_header    *gSysCtlHeader;

static struct ctl_table gSysCtlPmu[] = {
   {
      .ctl_name      = BCM_SYSCTL_PMU_LEVEL,
      .procname      = "level",
      .data          = &gPmuData.g_level,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
#if defined( HW_GPIO_PMU_IRQ_PIN )
   {
      .ctl_name      = BCM_SYSCTL_PMU_IRQS,
      .procname      = "irqs",
      .data          = &gPmuData.g_irqs,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
#endif
   {
      .ctl_name      = 100,
      .procname      = "max_isr_clk",
      .data          = &pmu_max_isr_clk,
      .maxlen        = sizeof( int ),
      .mode          = 0644,
      .proc_handler  = &proc_dointvec
   },
   {}
};
static struct ctl_table gSysCtl[] = {
   {
      .ctl_name      = CTL_BCM_PMU,
      .procname      = "pmu",
      .child         = gSysCtlPmu,
      .mode          = 0555
   },
   {}
};

#if defined( CONFIG_I2C )

struct pmu_i2c_info
{
    struct i2c_client client;
};
static struct pmu_i2c_info *pmu_i2c_datap;

#endif

/* ---- Private Function Prototypes -------------------------------------- */

#if defined( HW_GPIO_PMU_IRQ_PIN )
static irqreturn_t pmu_isr(int irq, void *dev_id);
#endif

static void pmu_chip_cleanup(void);

#if defined( CONFIG_I2C )
static int pmu_i2c_attach(struct i2c_adapter *adap, int addr, int kind);
static int pmu_i2c_attach_adapter(struct i2c_adapter *adap);
static int pmu_i2c_detach_client(struct i2c_client *device);
static int pmu_i2c_command(struct i2c_client *device, unsigned int cmd, void *arg);

struct i2c_driver i2c_driver_pmu =
{
	.driver = {
   .name           = IF_NAME,
   },
   .id             = I2C_DRIVERID_PMU,
   .attach_adapter = pmu_i2c_attach_adapter,
   .detach_client  = pmu_i2c_detach_client,
   .command        = pmu_i2c_command,
};
#endif

static BCM_PMU_Operations_t *pmus[PMU_NUM_CHIPS] = {
   NULL,
   NULL,
   NULL,
   NULL
};

static BCM_PMU_Power_On_State_t null_get_power_on_state( void )
{
   return PMU_Power_On_By_Restart;
}

/* PMU device operations */
static BCM_PMU_Operations_t pmu_none_ops =
{
   init:                NULL,
   isr:                 NULL,
   get_power_on_state:  null_get_power_on_state,
   ioctl:               NULL,
   poweroff:            NULL,
   regulator:
   {
      set_state:        NULL,
      get_state:        NULL,
      set_voltage:      NULL,
      get_voltage:      NULL,
   },
   charger:
   {
      start:            NULL,
      stop:             NULL,
      is_inserted:      NULL,
   },
   i2c_data:            NULL
};


/* static */ BCM_PMU_Operations_t *current_pmu = NULL;

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  pmu_register_device
*
***************************************************************************/
int pmu_register_device(BCM_PMU_Chip_t chip,
                        BCM_PMU_Operations_t *pmu_ops)
{
   int rc;
   /* Validate chip ID range */
   if ((chip < 0) || (chip >= PMU_NUM_CHIPS))
   {
      printk("PMU: pmu_register_device: chip ID %d exceeds range\n", chip);
      return -1;
   }

   /* Check if previous registration exists */
   if (pmus[chip])
   {
      printk("PMU: pmu_register_device: chip ID %d already registered\n", chip);
      return -2;
   }

   /* Save data for PMU chip */
   pmus[chip] = pmu_ops;

   if (chip == gPmuData.chip)
   {
      printk("PMU: pmu_register_device: chip ID %d selected\n", chip);
      current_pmu = pmus[chip];
   }

   if ( chip != PMU_NONE )
   {
#if defined( CONFIG_I2C )
      /* set up the I2C */
      pmus[chip]->driver = i2c_driver_pmu;
      pmus[chip]->driver.driver.name = pmus[chip]->driver_name;
      snprintf( pmus[chip]->driver_name, sizeof(pmus[chip]->driver_name), "pmui2c-%d", (int)chip );
      rc = i2c_add_driver(&pmus[chip]->driver);
#else
      rc = -ENODEV;
#endif
      if (rc != 0)
      {
          PMU_DEBUG(DBG_ERROR, "PMU - Failed to initialize I2C\n");
          goto fail;
      }
   }

   return 0;

fail:
	pmu_chip_cleanup();
	return rc;

}

/****************************************************************************
*
*  pmu_isr
*
***************************************************************************/

#if defined( HW_GPIO_PMU_IRQ_PIN )

static irqreturn_t pmu_isr( int irq, void *dev_id )
{
	(void)dev_id;
    (void)irq;

	PMU_DEBUG(DBG_TRACE2, "called\n");

   gPmuData.g_irqs++;

   if (current_pmu)
   {
      PMU_isr(current_pmu, dev_id);
   }


   return IRQ_HANDLED;
}
#endif

/****************************************************************************
*
*  pmu_chip_cleanup
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
static void pmu_chip_cleanup( void )
{
	PMU_DEBUG(DBG_TRACE, "called\n" );

	/* Unregister sysctl */
   if ( gSysCtlHeader != NULL )
   {
      unregister_sysctl_table( gSysCtlHeader );
   }
   gSysCtlHeader = NULL;

#if defined( HW_GPIO_PMU_IRQ_PIN )

   /* Unregister interrupt handler */
   if ( gPmuData.g_isr )
   {
       free_irq (GPIO_TO_IRQ(HW_GPIO_PMU_IRQ_PIN), NULL);
      gPmuData.g_isr--;
   }
#endif
}

/****************************************************************************
*
*  pmu_open
*
***************************************************************************/

static int pmu_open( struct inode *inode, struct file *file )
{
   PMU_DEBUG(DBG_TRACE, "called\n" );
   return 0;

} /* pmu_open */

/****************************************************************************
*
*  pmu_read
*
***************************************************************************/

static ssize_t pmu_read( struct file *file, char *buffer, size_t count, loff_t *ppos )
{
   PMU_DEBUG(DBG_TRACE, "major = %d, minor = %d\n", MAJOR( file->f_dentry->d_inode->i_rdev ), MINOR( file->f_dentry->d_inode->i_rdev ));
   return -EINVAL;

} /* pmu_read */

/****************************************************************************
*
*  pmu_release
*
***************************************************************************/

static int pmu_release( struct inode *inode, struct file *file )
{
   PMU_DEBUG(DBG_TRACE, "called\n" );
   return 0;

} /* pmu_release */

/****************************************************************************
*
*  pmu_ioctl
*
***************************************************************************/

static int pmu_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
    PMU_DEBUG(DBG_TRACE, "type: '%c' cmd: 0x%x\n", _IOC_TYPE( cmd ), _IOC_NR( cmd ));

    /* Ignore all IOCTLs if no PMU chip is present */
    if (current_pmu == pmus[PMU_NONE])
    {
       PMU_DEBUG(DBG_ERROR, "ioctl not supported on NULL PMU: '0x%x'\n", cmd );
       return -ENOTTY;
    }

    switch ( cmd )
    {
        case BCM_PMU_IOCTL_ENABLE_INTS:
        {
            pmu_enable_ints();
            break;
        }

        case BCM_PMU_IOCTL_DISABLE_INTS:
        {
            pmu_disable_ints();
            break;
        }

#if defined( CONFIG_I2C )
        case BCM_PMU_IOCTL_READ_REG:
        {
            BCM_PMU_Reg_t reg;
            int rc;

            if ( copy_from_user( &reg, (BCM_PMU_Reg_t *)arg, sizeof( reg )) != 0 )
            {
                return -EFAULT;
            }
            rc = pmu_i2c_read( reg.reg );
            if ( rc == -1 )
            {
                return -EFAULT;
            }
            reg.val = (u8)rc;
            if ( copy_to_user( (BCM_PMU_Reg_t *)arg, &reg, sizeof( reg )) != 0 )
            {
                return -EFAULT;
            }
            break;
        }

        case BCM_PMU_IOCTL_WRITE_REG:
        {
            BCM_PMU_Reg_t reg;

            if ( copy_from_user( &reg, (BCM_PMU_Reg_t *)arg, sizeof( reg )) != 0 )
            {
                return -EFAULT;
            }

            pmu_i2c_write( reg.reg, reg.val );

            break;
        }
#endif
        case BCM_PMU_IOCTL_ACTIVATESIM :
        case BCM_PMU_IOCTL_DEACTIVATESIM :
        case BCM_PMU_IOCTL_GET_REGULATOR_STATE :
        case BCM_PMU_IOCTL_SET_REGULATOR_STATE :
        case BCM_PMU_IOCTL_GET_VOLTAGE:
        case BCM_PMU_IOCTL_SET_VOLTAGE:
	case BCM_PMU_IOCTL_SET_PWM_LED_CTRL :
	case BCM_PMU_IOCTL_SET_PWM_HI_PER :
	case BCM_PMU_IOCTL_SET_PWM_LO_PER :
	case BCM_PMU_IOCTL_SET_PWM_PWR_CTRL :
        {
           int rc = PMU_ioctl(current_pmu, inode, file, cmd, arg);
           if (rc != 0)
           {
              PMU_DEBUG(DBG_ERROR, "IOCTL failed: cmd [0x%0x] ret[0x%x]\n", cmd, rc );
              return (rc);
           }
        }
        break;

        default:
        {
            PMU_DEBUG(DBG_ERROR, "Unrecognized ioctl: '0x%x'\n", cmd );
            return -ENOTTY;
        }
    }

    return 0;

} /* pmu_ioctl */

/****************************************************************************
*
*  pmu_write
*
***************************************************************************/

static ssize_t pmu_write( struct file *file, const char *buffer, size_t count, loff_t *ppos )
{
    return -EINVAL;

} /* pmu_write */

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations pmu_fops =
{
    owner:      THIS_MODULE,
    open:       pmu_open,
    release:    pmu_release,
    ioctl:      pmu_ioctl,
    read:       pmu_read,
    write:      pmu_write,
};

/****************************************************************************
*
*  pmu_chip_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

int pmu_chip_init( BCM_PMU_Chip_t chip )
{
   int rc;

   PMU_DEBUG(DBG_TRACE, "called\n" );

    printk( banner );
    if (( rc = register_chrdev( BCM_PMU_MAJOR, "pmu", &pmu_fops )) < 0 )
    {
       printk( KERN_WARNING "pmu: register_chrdev failed for major %d\n", BCM_PMU_MAJOR );
       return rc;
    }

    gPmuData.chip = chip;

    if ( chip == PMU_NONE )
    {
       pmu_register_device(PMU_NONE, &pmu_none_ops);
    }


	/* Initialize control structure */
	gPmuData.g_level = DBG_DEFAULT_LEVEL;


#if defined( HW_GPIO_PMU_IRQ_PIN )
   gPmuData.g_irqs = 0;
   gPmuData.g_isr = 0;
#endif

   gSysCtlHeader = register_sysctl_table( gSysCtl );

   return 0;

} /* pmu_chip_init */

/****************************************************************************
*
*  pmu_start_null_pmu
*
*       Called to kick off a fake poweron for the Null PMU
*
*     This couldn't be done at the end of the pmu_chip_init because the
*     power manager's work queues hadn't been initialized yet.
*
***************************************************************************/

void pmu_start_null_pmu( void )
{
#if defined( CONFIG_BCM_POWER_MANAGER )
   pm_submit_event(PM_EVENT_PMU_CHIP, PMU_EVENT_ATTACHED, (uint32_t)current_pmu, 0);
#endif
}

/****************************************************************************
*
*  pmu_chip_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

void pmu_chip_exit( void )
{
   PMU_DEBUG(DBG_TRACE, "called\n" );

   pmu_chip_cleanup();

} /* pmu_chip_exit */

/****************************************************************************
*
*  pmu_enable_ints
*
***************************************************************************/
void pmu_enable_ints( void )
{
#if defined( HW_GPIO_PMU_IRQ_PIN )
    enable_irq( GPIO_TO_IRQ( HW_GPIO_PMU_IRQ_PIN ));
#endif
}

/****************************************************************************
*
*  pmu_disable_ints
*
***************************************************************************/
void pmu_disable_ints( void )
{
#if defined( HW_GPIO_PMU_IRQ_PIN )
    disable_irq( GPIO_TO_IRQ( HW_GPIO_PMU_IRQ_PIN ));
#endif
}

/****************************************************************************
*
*  pmu_event_notify
*
***************************************************************************/
void pmu_event_notify(BCM_PMU_Chip_t chip, BCM_PMU_Event_t event, void *data)
{
   if (current_pmu != pmus[chip])
   {
      printk ("PMU: received event from wrong PMU device.\n");
      return;
   }

   /* Queue event to Power Manager */
   PMU_DEBUG(DBG_INFO, "Queue event %d(%d) to Power Manager\n", (int)event, (int)data);
#if defined( CONFIG_BCM_POWER_MANAGER )
   pm_submit_event(PM_EVENT_PMU_CHIP, (uint32_t)event, (uint32_t)data, 0);
#endif
}

#if defined( CONFIG_I2C )
/****************************************************************************
*
*  pmu_i2c_read
*
*  reg: address of register to read
*
*  returns: data read (8 bits) or -1 on error
*
***************************************************************************/
int pmu_i2c_read(u8 reg)
{
    if ( pmu_i2c_datap == NULL )
    {
        panic( "BCM i2c bus not configured for PMU\n" );
    }
    return i2c_smbus_read_byte_data(&pmu_i2c_datap->client, reg);
}

/****************************************************************************
*
*  pmu_i2c_write
*
*  reg: address of register to write
*  value: value to be written
*
*  returns: 0 on success, -1 on error
*
***************************************************************************/
int pmu_i2c_write(u8 reg, u8 value)
{
    if ( pmu_i2c_datap == NULL )
    {
        panic( "BCM i2c bus not configured for PMU\n" );
    }
    return i2c_smbus_write_byte_data(&pmu_i2c_datap->client, reg, value);
}

/****************************************************************************
*
*  pmu_i2c_read_bytes
*
*  reg: address of register to read
*  values: address of data buffer
*  num: number of bytes to read
*
*  returns: number of bytes or < 0 on error
*
***************************************************************************/
int pmu_i2c_read_bytes(u8 reg, u8 *values, int num)
{
    struct i2c_msg msg[2];
    unsigned char msgbuf0[1];

    if ( pmu_i2c_datap == NULL )
    {
        panic( "BCM i2c bus not configured for PMU\n" );
    }

    /* setup I2C messages for reading a number of bytes */
    msg[0].addr = pmu_i2c_datap->client.addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = msgbuf0;
    msgbuf0[0] = reg;

    msg[1].addr = pmu_i2c_datap->client.addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = num;
    msg[1].buf = values;

    return i2c_transfer(pmu_i2c_datap->client.adapter, msg, 2);
}

/****************************************************************************
*
*  pmu_i2c_attach
*
***************************************************************************/
static int pmu_i2c_attach(struct i2c_adapter *adap, int addr, int kind)
{
   struct i2c_client       *client;
   int rc;

   pmu_i2c_datap = kmalloc( sizeof( *pmu_i2c_datap ), GFP_KERNEL );
   memset( pmu_i2c_datap, 0, sizeof( *pmu_i2c_datap ));

   client = &pmu_i2c_datap->client;
   client->adapter = adap;
   client->addr = addr;
   client->driver = &current_pmu->driver;
   sprintf(client->name, "%s-%x", IF_NAME, addr);
   i2c_set_clientdata( client, pmu_i2c_datap );

   if ((rc = i2c_attach_client(client)) < 0)
   {
      kfree( pmu_i2c_datap );
      pmu_i2c_datap = NULL;
      return rc;
   }

#if defined( HW_GPIO_PMU_IRQ_PIN )
    rc = request_irq( GPIO_TO_IRQ( HW_GPIO_PMU_IRQ_PIN ), pmu_isr, ( IRQF_DISABLED  | IRQF_TRIGGER_FALLING ), "PMU_irq", NULL);
   if (rc != 0)
   {
      PMU_DEBUG(DBG_ERROR, "PMU - Failed to register ISR on GPIO pin %d.\n", HW_GPIO_PMU_IRQ_PIN);
      return rc;
   }
   gPmuData.g_isr++;
#endif

   /* Disable interrupts */
   pmu_disable_ints();

   /* Initialize PMU chip */
   PMU_init(current_pmu);

   /* Enable interrupts */
   pmu_enable_ints();

   /* Let Power Manager know the PMU chip is up */
#if defined( CONFIG_BCM_POWER_MANAGER )
   pm_submit_event(PM_EVENT_PMU_CHIP, PMU_EVENT_ATTACHED, (uint32_t)current_pmu, 0);
#endif

   return 0;
}

/****************************************************************************
*
*  pmu_i2c_attach_adapter
*
***************************************************************************/
static int pmu_i2c_attach_adapter(struct i2c_adapter *adap)
{
   /* Look for this device on the given adapter (bus) */
   if (adap->id != I2C_HW_B_BCM11XX)
       return -1;

   if (current_pmu && current_pmu->i2c_data)
   {
      return i2c_probe(adap, current_pmu->i2c_data, &pmu_i2c_attach);
   }
   else
   {
      return -1;
   }
}

/****************************************************************************
*
*  pmu_i2c_detach_client
*
***************************************************************************/
static int pmu_i2c_detach_client(struct i2c_client *device)
{
   int rc = 0;

   if ((rc = i2c_detach_client(&pmu_i2c_datap->client)) != 0) {
       printk(IF_NAME "detach failed: %d\n", rc);
   } else {
       kfree(pmu_i2c_datap);
       pmu_i2c_datap = NULL;
   }
   return rc;
}

/****************************************************************************
*
*  pmu_i2c_command
*
***************************************************************************/
static int pmu_i2c_command(struct i2c_client *device, unsigned int cmd, void *arg)
{
    return 0;
}

#endif
