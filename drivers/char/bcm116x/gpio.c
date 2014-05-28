/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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
*  gpio.c
*
*  PURPOSE:
*
*     This implements the gpio driver.
*
*  NOTES:
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#include <mach/hardware.h>
#include <mach/reg_gpio.h>
#include <mach/reg_irq.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/gpio_irq.h>
#include <asm/gpio.h>
#include <asm/mach/irq.h>

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

#define  USE_NEW_IRQ 1

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

#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO)

#if DEBUG
#	define GPIO_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define GPIO_DEBUG(level,x)
#endif


static char banner[] __initdata = KERN_INFO "GPIO Control Driver: 1.00 (built on "__DATE__" "__TIME__")\n";
static int gLevel = DBG_DEFAULT_LEVEL;

/* ---- Private Variables ------------------------------------------------ */

/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  gpio_irq_ack
*
*     Called by the interrupt handler to acknowledge (i.e. clear)
*     the interrupt.
*
***************************************************************************/

static void gpio_irq_ack( unsigned irq )
{
   /*
    * Since this function is ONLY called with interrupts disabled, we don't 
    * need to disable irqs around the following 
    */

   REG_GPIO_GPICR( IRQ_TO_GPIO(irq)) |= REG_GPIO_GPICR_MASK( IRQ_TO_GPIO(irq));
}

/****************************************************************************
*
*  gpio_irq_mask
*
*     Called to mask (i.e. disable) an interrupt.
*
***************************************************************************/

static void gpio_irq_mask( unsigned irq )
{
   /*
    * Since this function is ONLY called with interrupts disabled, we don't 
    * need to disable irqs around the following 
    */

   REG_GPIO_GPIMR( IRQ_TO_GPIO(irq)) &= ~REG_GPIO_GPIMR_MASK( IRQ_TO_GPIO(irq));
}

/****************************************************************************
*
*  gpio_irq_unmask
*
*     Called to unmask (i.e. enable) an interrupt.
*
***************************************************************************/

static void gpio_irq_unmask( unsigned irq )
{
   /*
    * Since this function is ONLY called with interrupts disabled, we don't 
    * need to disable irqs around the following 
    */

   REG_GPIO_GPIMR( IRQ_TO_GPIO(irq)) |= REG_GPIO_GPIMR_MASK( IRQ_TO_GPIO(irq));
}

/****************************************************************************
*
*  gpio_irq_type
*
*     Sets the type of the GPIO irq.
*
***************************************************************************/

static int gpio_irq_set_type( unsigned irq, unsigned type )
{
   int   gpio;

   gpio = IRQ_TO_GPIO( irq );

   /*
    * Since this function is ONLY called with interrupts disabled, we don't 
    * need to disable irqs around the following 
    */

   if ( type == IRQ_TYPE_PROBE )
   {
      /* Don't mess GPIOs which already have interrupt handlers registered. */

      if ( reg_gpio_is_interrupt_enable( gpio ))
      {
         return 0;
      }

      if ( reg_gpio_itr_get_interrupt_type( gpio ) != GPIO_NO_INTERRUPT )
      {
         return 0;
      }

      type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;
   }

   printk( KERN_INFO "IRQ%d (gpio%d): ", irq, gpio );

   reg_gpio_disable_interrupt( gpio );
   reg_gpio_iotr_set_pin_type( gpio, GPIO_PIN_TYPE_INPUT_WITH_INTERRUPT );
   reg_gpio_clear_interrupt( gpio );

   if ( type & IRQ_TYPE_EDGE_RISING )
   {
      if ( type & IRQ_TYPE_EDGE_FALLING )
      {
         printk( "both " );
         reg_gpio_itr_set_interrupt_type( gpio, GPIO_BOTH_EDGE_INTERRUPT_TRIGGER );
      }
      else
      {
         printk( "rising " );
         reg_gpio_itr_set_interrupt_type( gpio, GPIO_RISING_EDGE_INTERRUPT_TRIGGER );
      }
   }
   else
   if ( type & IRQ_TYPE_EDGE_FALLING )
   {
      printk( "falling " );
      reg_gpio_itr_set_interrupt_type( gpio, GPIO_FALLING_EDGE_INTERRUPT_TRIGGER );
   }
   else
   {
      printk( "no " );
      reg_gpio_itr_set_interrupt_type( gpio, GPIO_NO_INTERRUPT );
   }
   printk( "edges\n" );

   return 0;

} /* gpio_irq_set_type */

/****************************************************************************
*
*  gpio_isr_handler
*
*     Figures out which GPIO caused the interrupt and calls the register
*     handler to deal with it.
*
*     The handler function will in all likelyhood be do_edge_IRQ.
*
***************************************************************************/

void gpio_isr_handler( unsigned int irq, struct irq_desc *desc )
{
   unsigned mask;
   int loop;

   /* Clear the IRQ_GPIO interrupt */

   REG_IRQ_ICR = REG_IRQ_ICR_GPIOCLR;

   do
   {
      loop = 0;

      mask = REG_GPIO_GPISR0;
      if ( mask )
      {
         /* Clear the interrupts */

         REG_GPIO_GPICR0 = mask;

         irq = GPIO_TO_IRQ( 0 );
         desc = irq_desc + irq;
         do
         {
            if ( mask & 1 )
            {
               desc->handle_irq( irq, desc );
            }
            irq++;
            desc++;
            mask >>= 1;

         } while ( mask );

         loop = 1;
      }

      mask = REG_GPIO_GPISR1;
      if ( mask )
      {
         /* Clear the interrupts */

         REG_GPIO_GPICR1 = mask;

         irq = GPIO_TO_IRQ( 32 );
         desc = irq_desc + irq;
         do
         {
            if ( mask & 1 )
            {
               desc->handle_irq( irq, desc );
            }
            irq++;
            desc++;
            mask >>= 1;

         } while ( mask );

         loop = 1;
      }

   } while ( loop );

} /* gpio_isr_handler */

#ifdef CONFIG_HAVE_GPIO_LIB
/****************************************************************************
*
*  Configure a GPIO pin as an input pin
*
*****************************************************************************/

static int gpio_116x_direction_input( struct gpio_chip *chip, unsigned offset )
{
    (void)chip;

    reg_gpio_iotr_set_pin_type( offset, GPIO_PIN_TYPE_INPUT_WITH_INTERRUPT);
    return 1;

} /* gpio_116x_direction_input */

/****************************************************************************
*
*  Configure a GPIO pin as an output pin and sets its initial value.
*
*****************************************************************************/

static int gpio_116x_direction_output( struct gpio_chip *chip, unsigned offset, int value )
{
    (void)chip;

    reg_gpio_iotr_set_pin_type( offset, GPIO_PIN_TYPE_OUTPUT );
    reg_gpio_set_pin( offset, value );
    return 1;

} /* gpio_116x_direction_output */

/****************************************************************************
*
*  Retrieve the value of a GPIO pin. Note that this returns zero or the raw
*   value.
*
*****************************************************************************/

static int gpio_116x_get( struct gpio_chip *chip, unsigned offset )
{
    (void)chip;

    return reg_gpio_get_pin( offset );

} /* gpio_116x_get */

/****************************************************************************
*
*  Set the value of a GPIO pin
*
*****************************************************************************/

static void gpio_116x_set( struct gpio_chip *chip, unsigned offset, int value )
{
    (void)chip;

    reg_gpio_set_pin( offset, value );

} /* gpio_116x_set */

/****************************************************************************
*
*  gpiolib chip description
*
*****************************************************************************/

/*
 * Note: If you need to add a field, like a register base, then create a new 
 *       structure which includes the gpio_chip as the first element and has 
 *       the custom fields after. See asm-arm/arch-pxa/gpio.c for an example. 
 */

static struct gpio_chip gpio_116x_chip =
{
    .label              = "116x",
    .direction_input    = gpio_116x_direction_input,
    .direction_output   = gpio_116x_direction_output,
    .get                = gpio_116x_get,
    .set                = gpio_116x_set,
    .base               = 0,
    .ngpio              = NUM_GPIO_IRQS,
};

/****************************************************************************
*
*  brcm_init_gpio
*
*   Sets up gpiolib so that it's aware of how to manipulate our GPIOs
*
*****************************************************************************/

void  bcm_init_gpio( void )
{
    gpiochip_add( &gpio_116x_chip );

} /* bcm_init_gpio */
#endif /*#ifdef CONFIG_HAVE_GPIO_LIB */
/****************************************************************************
*
*  gpio_chip data structure.
*
***************************************************************************/

static struct irq_chip gpio_chip =
{
   .ack     = gpio_irq_ack,
   .mask    = gpio_irq_mask,
   .unmask  = gpio_irq_unmask,
   .disable = gpio_irq_mask,
   .enable  = gpio_irq_unmask,
   .set_type    = gpio_irq_set_type,
};

/****************************************************************************
*
*  gpio_cleanup
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
static void gpio_cleanup( void )
{
	GPIO_DEBUG(DBG_TRACE,("COMCTL - gpio_cleanup()\n"));

	/* unregister sysctl table */
}

/****************************************************************************
*
*  gpio_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static int __init gpio_init( void )
{
	/* Initialize debug level */
	gLevel = DBG_DEFAULT_LEVEL;

	GPIO_DEBUG(DBG_INFO,( "gpio_init called\n" ));

	printk( banner );
   {
      int   irq;

      for ( irq = GPIO_TO_IRQ( 0 ); irq <= GPIO_TO_IRQ( NUM_GPIO_IRQS-1 ); irq++ )
      {
         set_irq_chip( irq, &gpio_chip );
         set_irq_handler( irq, handle_edge_irq );
         set_irq_flags( irq, IRQF_VALID | IRQF_PROBE );
      }
      set_irq_chained_handler( IRQ_GPIO, gpio_isr_handler );
   }

   return 0;
} /* gpio_init */

/****************************************************************************
*
*  gpio_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit gpio_exit( void )
{
	GPIO_DEBUG(DBG_INFO,( "gpio_exit called\n" ));

	gpio_cleanup();
} /* gpio_exit */

#if 1

/*
 * The following code needs to stay here, even though its deprecated. 
 * the vc02 driver uses it. When the gpio irq mechanism is integrated into 
 * the MIPS driver, then we'll be able to switch the vc02 driver to using 
 * the standard linux API, and then we can remove the following 
 */

static irqreturn_t (*gHandler[REG_GPIO_NUM_GPIO])( void *);
static void *gDevId[REG_GPIO_NUM_GPIO];

/****************************************************************************
*
*  gpio_request_irq
*
*       Called to register ISR on GPIO pin, irq is enbaled by default.
*
***************************************************************************/

irqreturn_t gpio_old_handler_proxy( int irq, void *dev_id )
{
   irqreturn_t retval;

   retval = (*(gHandler[ IRQ_TO_GPIO( irq ) ]))( dev_id );

   return retval;
}

int gpio_request_irq( int pin,
                      GPIO_INTERRUPT_TYPE interruptType,
                      irqreturn_t (*handler)( void *),
                      void *dev_id )
{
   int   rc;
   int   irqType = 0;
   int   irq = GPIO_TO_IRQ( pin );

   if ( interruptType == GPIO_RISING_EDGE_INTERRUPT_TRIGGER )
   {
      irqType |= IRQ_TYPE_EDGE_RISING;
   }
   else if ( interruptType == GPIO_FALLING_EDGE_INTERRUPT_TRIGGER )
   {
      irqType |= IRQ_TYPE_EDGE_FALLING;
   }
   else if ( interruptType == GPIO_BOTH_EDGE_INTERRUPT_TRIGGER )
   {
      irqType |= ( IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING );
   }
   else
   {
      return -EINVAL;
   }

   /* The gpio_request_irq API has interrupts enabled by default */

   if ( irq >= NR_IRQS )
   {
      return -EINVAL;
   }

   set_irq_flags( irq, IRQF_VALID );
   set_irq_type( irq, irqType );

   gHandler[pin] = handler;
   gDevId[pin] = dev_id;

   rc = request_irq( irq, gpio_old_handler_proxy, IRQF_DISABLED, "gpio-old", dev_id );

   return rc;
}



/****************************************************************************
*
*  gpio_free_irq
*
*       Called to free ISR on GPIO pin
*
***************************************************************************/
int gpio_free_irq( int pin )
{

    free_irq( GPIO_TO_IRQ( pin ), gDevId[pin]);

    gHandler[pin] = NULL;
    gDevId[pin] = NULL;


    return 0;
}

/****************************************************************************
*
*  gpio_enable_irq
*
*       Called to enable ISR on GPIO pin
*
***************************************************************************/
int gpio_enable_irq( int pin )
{
    enable_irq( GPIO_TO_IRQ( pin ));
    return 0;
}

/****************************************************************************
*
*  gpio_disable_irq
*
*       Called to disable ISR on GPIO pin
*
***************************************************************************/
int gpio_disable_irq( int pin )
{
   disable_irq( GPIO_TO_IRQ( pin ));
   return 0;
}
EXPORT_SYMBOL (gpio_request_irq);
EXPORT_SYMBOL (gpio_free_irq);
EXPORT_SYMBOL (gpio_enable_irq);
EXPORT_SYMBOL (gpio_disable_irq);

#endif

/* Changed from module_init to fs_initcall so that GPIO driver
 * is loaded before the any of the PMU drivers is loaded. PMU drivers
 * were also changed to fs_initcall so that they are loaded before
 * VC02 and USB drivers are loaded. THis was done because the host has to
 * read the PMU interrupts in time (< 8sec) or else the PMU
 * timeout timer (of 8sec) could expire causing the phone to shut off.
 * This was observed in cases where a battery was removed and then re inserted.
 * This action would cause a LOWBAT interrupt generated and the host has 8sec
 * to clear it before PMU goes into standby mode. If VC02 driver was loaded
 * before PMU driver, the PMU driver was getting loaded well past 8sec window
 */

fs_initcall(gpio_init);
module_exit(gpio_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("GPIO Control Driver");
MODULE_LICENSE("GPL v2");



